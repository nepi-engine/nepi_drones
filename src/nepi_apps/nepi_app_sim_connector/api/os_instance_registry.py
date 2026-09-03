#!/usr/bin/env python3
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License",
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#

# Additive multi-OS-instance deploy-target registry -- see
# docs/SIM_OS_INSTANCES_PLAN.md (nepi_drones) for the full design.
#
# simulator_launch_targets.yaml's five targets are all hardcoded to one
# developer's one VM (host/ssh_user/ssh_port baked into every target entry).
# This module adds a small, separately-persisted registry of OTHER machines
# ("OS instances") and one integration point -- select() -- that overwrites
# every target's host/ssh_user/ssh_port in a loaded SimulatorLauncher's
# in-memory config to match whichever instance is currently selected.
# Nothing in simulator_launcher.py itself changes: launch()/stop()/
# is_installed()/install()/_ssh_cmd() already read those three fields
# per-target, so once select() runs, every existing code path is already
# instance-aware.
#
# Deliberately zero nepi_sdk/rospy dependency, same reasoning
# simulator_launcher.py already establishes: plain SSH orchestration plus
# flat per-instance YAML files, testable standalone before
# sim_connector_app_node.py ever imports it.
#
# No credentials are ever stored here -- SSH key resolution duplicates (does
# not import) simulator_launcher.py's own _ssh_key_candidates logic, so this
# module keeps working even on a deployment that has no
# simulator_launch_targets.yaml at all (the two features are independently
# optional).

import os
import re
import socket
import subprocess

import yaml

from nepi_api.simulator_launcher import LauncherError

# Mirrors sim_connector_app_node.py's own ROBOT_CONFIGS_STORAGE_DIR convention
# (one YAML file per entry, under this app's database tree) rather than
# simulator_launch_targets.yaml's own $NEPI_CONFIG/hand-edited-file convention:
# instances are created/removed through the RUI, not hand-authored, so they
# belong in the read/write database tree, not the dev-only checked-in config
# tree.
OS_INSTANCES_STORAGE_DIR = '/mnt/nepi_storage/databases/nepi_app_sim_connector/os_instances'

# The existing single-VM default every launch target hardcodes is 12222 --
# allocation starts one above it so a freshly-registered instance can never
# collide with that pre-existing default.
FIRST_ALLOCATED_SSH_PORT = 12223

# Id of the pseudo-instance representing whatever connection
# simulator_launch_targets.yaml itself hardcodes -- see ensure_baseline's own
# docstring. Never operator-created or removable, always present whenever a
# launcher config is loaded at all, so the RUI's picker always has a real,
# named, already-verified entry to show instead of a generic "default"
# placeholder -- reported live: the picker should "name the name of the one
# it's currently connected to," never say "Default."
BASELINE_INSTANCE_ID = 'baseline'

SSH_CONNECT_TIMEOUT_SEC = 8

DEFAULT_SSH_KEY = os.path.expanduser("~/.ssh/nepi_default_ssh_key")

# The platform's own authoritative device-IP config, already present on
# every real deployment (nepi_setup's docker_config_setup.sh writes it) --
# see _guess_device_ip's own docstring for why this, not a network-routing
# guess, is the right primary source.
NEPI_SYSTEM_CONFIG_PATH = '/opt/nepi/etc/nepi_system_config.yaml'


def _guess_device_ip():
  """Best-effort discovery of THIS device's own LAN-reachable IP, so the
  generated setup commands can show a real, working example instead of an
  abstract placeholder -- reported live: "give the example and also put
  that as the example in the RUI."

  Tries NEPI_SYSTEM_CONFIG_PATH's own NEPI_STATIC_IP first -- the
  platform's own authoritative, intentionally-configured device address --
  and only falls back to a local-socket outbound-routing guess if that
  file is missing/unreadable. This order matters, confirmed the hard way:
  a real device can be multi-homed (this one has a static `eth0` at
  192.168.179.103 -- the address every other machine in this whole
  session actually uses to reach it -- alongside a DHCP `wlan0` used only
  for the device's own internet uplink), and a naive "connect to 8.8.8.8,
  see which interface answers" guess picks whichever has the DEFAULT
  ROUTE, which is commonly the wifi/internet-facing one, not the one other
  machines should dial in on. The socket fallback (no packet is actually
  sent -- UDP connect() just asks the kernel to pick a route) stays purely
  as a safety net for a deployment that genuinely has no system config
  file to read, not the primary source of truth. Both paths stay zero-ROS-
  dependency, like the rest of this module (no network_mgr/ip_addr_query
  service call). Returns None if neither works, in which case callers fall
  back to a plain placeholder."""
  try:
    with open(NEPI_SYSTEM_CONFIG_PATH, 'r') as f:
      config = yaml.safe_load(f)
    static_ip = (config or {}).get('NEPI_STATIC_IP', '')
    if static_ip:
      return str(static_ip).split('/')[0]
  except (OSError, yaml.YAMLError):
    pass
  try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
      s.connect(('8.8.8.8', 80))
      return s.getsockname()[0]
    finally:
      s.close()
  except OSError:
    return None


def _sanitize_display_name(name):
  name = str(name).strip()
  return name[:64] if name else 'Unnamed'


def _instance_id_from_name(name, existing_ids):
  """Derives a filesystem/topic-safe id from the display name -- same
  sanitize-then-suffix-on-collision approach sim_connector_app_node.py's own
  sanitizeRobotConfigKey uses for saved robot configs. 'os_' prefixed so it
  can never collide with any other key this app persists."""
  safe = re.sub(r'[^A-Za-z0-9_-]', '_', name)[:48].strip('_').lower() or 'instance'
  candidate = 'os_' + safe
  if candidate not in existing_ids:
    return candidate
  n = 2
  while (candidate + '_' + str(n)) in existing_ids:
    n += 1
  return candidate + '_' + str(n)


class OsInstanceRegistry(object):
  """Loads/persists registered OS instances and applies the selected one onto
  a SimulatorLauncher's in-memory config. One process-wide instance, owned by
  sim_connector_app_node.py, constructed alongside self.launcher there."""

  def __init__(self, storage_dir=OS_INSTANCES_STORAGE_DIR):
    self.storage_dir = storage_dir
    self.instances = {}
    self.selected_instance_id = ''
    self._load_all()

  #**********************
  # Persistence

  def _instance_path(self, instance_id):
    return os.path.join(self.storage_dir, instance_id + '.yaml')

  def _load_all(self):
    try:
      filenames = os.listdir(self.storage_dir)
    except OSError:
      return
    for filename in filenames:
      if not filename.endswith('.yaml'):
        continue
      instance_id = filename[:-len('.yaml')]
      path = os.path.join(self.storage_dir, filename)
      try:
        with open(path, 'r') as f:
          entry = yaml.safe_load(f)
      except (OSError, yaml.YAMLError):
        continue
      if not isinstance(entry, dict):
        continue
      self.instances[instance_id] = entry
      if entry.get('selected', False):
        self.selected_instance_id = instance_id

  def _persist(self, instance_id):
    entry = dict(self.instances[instance_id])
    try:
      os.makedirs(self.storage_dir, exist_ok=True)
      with open(self._instance_path(instance_id), 'w') as f:
        yaml.safe_dump(entry, f, default_flow_style=False, sort_keys=False)
    except OSError as e:
      raise LauncherError("Failed to persist OS instance '" + instance_id + "': " + str(e))

  #**********************
  # Port allocation

  def _next_ssh_port(self):
    used = set()
    for entry in self.instances.values():
      port = entry.get('ssh_port')
      if isinstance(port, int):
        used.add(port)
    port = FIRST_ALLOCATED_SSH_PORT
    while port in used:
      port += 1
    return port

  #**********************
  # SSH -- mirrors simulator_launcher.py's own _ssh_key_candidates/_ssh_cmd
  # shape (duplicated, not imported -- see module docstring for why)

  def _ssh_key_candidates(self):
    candidates = []
    env_key_path = os.environ.get('NEPI_SSH_KEY_PATH', '')
    if env_key_path:
      candidates.append(env_key_path)
    env_key = os.environ.get('NEPI_SSH_KEY', '')
    if env_key:
      candidates.append(env_key)
      ssh_folder = os.environ.get('NEPI_SSH_FOLDER', '')
      if ssh_folder and os.path.basename(env_key) == env_key:
        candidates.append(os.path.join(ssh_folder, env_key))
    candidates.append(DEFAULT_SSH_KEY)
    return [os.path.expanduser(c) for c in candidates]

  def _ssh_key(self):
    candidates = self._ssh_key_candidates()
    for candidate in candidates:
      if os.path.isfile(candidate):
        return candidate
    raise LauncherError("No usable SSH key found. Tried: " + ", ".join(candidates))

  def _probe_connection(self, host, ssh_user, ssh_port):
    ssh_key = self._ssh_key()
    ssh_cmd = [
        "ssh",
        "-i", ssh_key,
        "-p", str(int(ssh_port)),
        "-o", "StrictHostKeyChecking=no",
        "-o", "ConnectTimeout=" + str(SSH_CONNECT_TIMEOUT_SEC),
        ssh_user + "@" + host,
        "echo ok",
    ]
    try:
      result = subprocess.run(ssh_cmd, capture_output=True, text=True,
                               timeout=SSH_CONNECT_TIMEOUT_SEC + 2)
    except subprocess.TimeoutExpired:
      return False, "SSH connection timed out after " + str(SSH_CONNECT_TIMEOUT_SEC) + "s"
    if result.returncode != 0 or 'ok' not in (result.stdout or ''):
      return False, (result.stderr or result.stdout or "SSH connection failed").strip()
    return True, ''

  #**********************
  # Setup-command generation

  def build_setup_commands(self, instance):
    """The exact copy-paste block the RUI shows for a freshly-registered
    instance -- mirrors docs/SIM_VM_CONNECTION_SETUP.md's own two documented
    tunnel paths (systemd, and the plain-autossh fallback needed on a
    machine like this dev VM's own WSL environment, per that doc's own WSL
    callout), with the real allocated port substituted in rather than a
    human hand-editing port numbers out of a markdown file.

    Restructured (reported live: "the commands are kind of hard to
    understand ... give the right places to go properly") as a plain,
    numbered walkthrough with an explicit WHERE line on every step, rather
    than a wall of shell-comment-prefixed lines -- the earlier shape read as
    one long script when it's actually two machines and two alternative
    paths interleaved.

    No SSH-key-generation step -- reported live: "most people will already
    have the normal OS-to-NEPI connection working, so there's no reason to
    have the ssh pub/priv thing there." The overwhelming common case is a
    machine that's already been through NEPI's own Remote Setup, which
    already created ~/.ssh/nepi_default_ssh_key and already got its public
    half trusted by the device -- generating a SECOND keypair here would be
    redundant at best, and once (see docs/SIM_OS_INSTANCES_PLAN.md's
    incident writeup) actually destructive: running it against a machine
    that already had a WORKING nepi_default_ssh_key overwrote that working
    key, breaking the very connection the device already trusted. The
    tunnel commands below reuse whatever key is already there; a machine
    that has genuinely never done Remote Setup at all needs that setup
    first (see NEPI_REMOTE_SETUP.md), not a key generated in isolation
    here.

    device_host below is filled in with this device's own real, currently-
    detected IP (see _guess_device_ip) rather than a bare placeholder --
    reported live: "give the example and also put that as the example in
    the RUI." user/port are NOT placeholders needing a real value filled
    in -- 'nepi'/2222 is the actual, tested default this whole mechanism
    (`nepi_tunnel()`/`nepi-tunnel.service`, see SIM_VM_CONNECTION_SETUP.md's
    own Step 2) already uses, confirmed directly against the real device
    this session: the host-level 'nepi' account exists but has a
    /sbin/nologin shell (cannot SSH in at all), so port 2222 (the
    container's own sshd, where 'nepi' has a real shell) is the only
    combination that actually works -- not a placeholder value to replace,
    unlike device_host which genuinely differs per deployment."""
    port = instance['ssh_port']
    iid = instance['instance_id']
    # No angle brackets in this fallback placeholder (rare -- only when
    # _guess_device_ip finds neither a config file nor a route) -- reported
    # live: pasting a bracketed placeholder verbatim into a real shell
    # command breaks it outright, since '<' is redirection syntax, rather
    # than failing obviously (e.g. "no such host").
    device_host = _guess_device_ip() or "YOUR_NEPI_DEVICE_IP_OR_HOSTNAME"
    return (
        "STEP 1 of 2 -- Open a reverse tunnel back to the NEPI device\n"
        "Where: on the NEW machine (" + instance['display_name'] + ")\n"
        "\n"
        "This assumes this machine already has NEPI Remote Setup done (the\n"
        "normal, already-working OS-to-device connection) -- it already has\n"
        "~/.ssh/nepi_default_ssh_key, and the device already trusts it. This\n"
        "step just opens a tunnel with that existing key; it does not create\n"
        "or change any key. (If this machine has never done Remote Setup at\n"
        "all, do that first -- see NEPI_REMOTE_SETUP.md -- rather than running\n"
        "anything below.)\n"
        "\n"
        "This NEPI device's own current address is " + device_host + " --\n"
        "already filled in below. user (nepi) and port (2222) are this\n"
        "platform's own fixed convention, not placeholders -- leave them as\n"
        "written.\n"
        "\n"
        "Both options below need autossh -- install it first if you don't\n"
        "already have it (reported live: a fresh WSL machine didn't):\n"
        "\n"
        "    command -v autossh >/dev/null || sudo apt-get install -y autossh\n"
        "\n"
        "Pick ONE of the two options below -- not both.\n"
        "\n"
        "  Option A -- recommended: restarts itself automatically on reboot.\n"
        "\n"
        "    mkdir -p ~/.config/systemd/user ~/.config\n"
        "    cp sim_container/systemd/nepi-tunnel.service "
        "~/.config/systemd/user/nepi-tunnel-" + iid + ".service\n"
        "    cat > ~/.config/nepi-tunnel-" + iid + ".env <<'EOF'\n"
        "    DEVICE_SSH_HOST=" + device_host + "\n"
        "    DEVICE_SSH_USER=nepi\n"
        "    DEVICE_SSH_PORT=2222\n"
        "    TUNNEL_SSH_PORT=" + str(port) + "\n"
        "    EOF\n"
        "    systemctl --user daemon-reload\n"
        "    systemctl --user enable --now nepi-tunnel-" + iid + ".service\n"
        "    loginctl enable-linger $(id -un)\n"
        "\n"
        "  Option B -- one-off command, no systemd required (use this on WSL\n"
        "  unless you've already turned systemd on in /etc/wsl.conf):\n"
        "\n"
        "    autossh -M 0 -f -N -R " + str(port) + ":127.0.0.1:22 -p 2222 \\\n"
        "      -i ~/.ssh/nepi_default_ssh_key nepi@" + device_host + "\n"
        "\n"
        "\n"
        "STEP 2 of 2 -- Verify it worked\n"
        "Where: back in the RUI (NOT the new machine, and NOT a command\n"
        "you need to run at all in the normal case)\n"
        "\n"
        "Enter your username on the NEW machine in the field below, then\n"
        "click \"Test Connection\". That button runs the real check from the\n"
        "NEPI DEVICE's own side -- which is the only place it means anything,\n"
        "since the tunnel makes the DEVICE listen on port " + str(port) + " and\n"
        "forward back to THIS machine's own sshd.\n"
        "\n"
        "(Only if that fails and you want to double-check by hand: the\n"
        "equivalent manual command has to run ON THE NEPI DEVICE ITSELF --\n"
        "e.g. over `sshnh`/`sshn` if you have direct access to it -- never on\n"
        "the new machine. Fill in a real username before running it (don't\n"
        "paste the placeholder text below verbatim -- the < character is\n"
        "shell redirection syntax and will error):\n"
        "\n"
        "    ssh -p " + str(port) + " YOUR_USERNAME_ON_THE_NEW_MACHINE@localhost echo ok\n"
        "\n"
        "Running this ON THE NEW MACHINE INSTEAD will always fail with\n"
        "\"Connection refused\" -- there's nothing listening on that port\n"
        "there; the forwarded listener only exists on the device side.)"
    )

  #**********************
  # Public API, called from sim_connector_app_node.py

  def list_instances(self):
    return dict(self.instances)

  def get_instance(self, instance_id):
    if instance_id not in self.instances:
      raise LauncherError("Unknown OS instance: " + str(instance_id))
    return self.instances[instance_id]

  def register(self, display_name):
    """Creates a new, unverified instance and returns (instance_id,
    setup_commands). host/ssh_user stay blank until verify() fills them in
    -- nothing here can guess a real reachable address up front."""
    display_name = _sanitize_display_name(display_name)
    instance_id = _instance_id_from_name(display_name, self.instances.keys())
    instance = {
        'instance_id': instance_id,
        'display_name': display_name,
        'host': '',
        'ssh_user': '',
        'ssh_port': self._next_ssh_port(),
        'status': 'pending',
    }
    self.instances[instance_id] = instance
    self._persist(instance_id)
    return instance_id, self.build_setup_commands(instance)

  def verify(self, instance_id, host=None, ssh_user=None):
    """Runs the real SSH probe and updates + persists the instance's status.
    host defaults to 127.0.0.1 (the reverse-tunnel convention every existing
    launch target already uses) the first time, when not given explicitly --
    a caller reaching the machine directly on a routable LAN address passes
    host itself instead. Raises LauncherError (with the real SSH failure
    text) on a failed probe, after still recording/persisting 'unreachable'."""
    instance = self.get_instance(instance_id)
    if host:
      instance['host'] = host
    elif not instance.get('host'):
      instance['host'] = '127.0.0.1'
    if ssh_user:
      instance['ssh_user'] = ssh_user
    if not instance.get('ssh_user'):
      raise LauncherError("An SSH username is required before an instance can be verified")
    ok, error = self._probe_connection(instance['host'], instance['ssh_user'], instance['ssh_port'])
    instance['status'] = 'verified' if ok else 'unreachable'
    self._persist(instance_id)
    if not ok:
      raise LauncherError("Connection test failed for '" + instance['display_name'] + "': " + error)
    return instance

  def ensure_baseline(self, display_name, host, ssh_user, ssh_port):
    """Registers (or refreshes) the pseudo-instance representing whatever
    connection simulator_launch_targets.yaml itself hardcodes -- called once
    at startup with the values the launcher's own config just loaded (its
    first target's host/ssh_user/ssh_port; all targets share the same
    connection by convention). This is what lets the RUI's picker always
    show a real, named, already-'verified' entry -- reported live: there
    should be no generic "Default" placeholder, the picker should "name the
    name of the one it's currently connected to."

    host/ssh_user/ssh_port/display_name are refreshed every call (every app
    startup) since they're derived from the yaml, not operator-set -- an
    edit to simulator_launch_targets.yaml followed by a restart is reflected
    here too. selected_instance_id is left alone if the operator already
    chose a real, different instance on a previous run; only defaults to
    this baseline when nothing has ever been selected (a genuinely first
    boot, or after a fresh install)."""
    instance = self.instances.get(BASELINE_INSTANCE_ID, {})
    instance.update({
        'instance_id': BASELINE_INSTANCE_ID,
        'display_name': display_name,
        'host': host,
        'ssh_user': ssh_user,
        'ssh_port': ssh_port,
        'status': 'verified',
    })
    self.instances[BASELINE_INSTANCE_ID] = instance
    self._persist(BASELINE_INSTANCE_ID)
    if not self.selected_instance_id:
      instance['selected'] = True
      self._persist(BASELINE_INSTANCE_ID)
      self.selected_instance_id = BASELINE_INSTANCE_ID

  def remove(self, instance_id):
    if instance_id == BASELINE_INSTANCE_ID:
      raise LauncherError("Cannot remove the default NEPI-device connection")
    if instance_id not in self.instances:
      return
    try:
      path = self._instance_path(instance_id)
      if os.path.exists(path):
        os.remove(path)
    except OSError:
      pass
    del self.instances[instance_id]
    if self.selected_instance_id == instance_id:
      self.selected_instance_id = ''

  def select(self, instance_id, launcher):
    """The one integration point with the existing launch machinery: applies
    the given instance's host/ssh_user/ssh_port onto EVERY target in
    launcher.config['launch_targets'] (a plain in-memory dict -- confirmed by
    reading simulator_launcher.py), in place. Every existing code path
    (launch/stop/is_installed/install/_ssh_cmd) already reads those three
    fields per-target, so nothing else needs to change for this to take
    effect immediately on the next launch/install/check.

    Raises if the instance isn't 'verified' yet (Test Connection first) --
    selecting an unconfirmed instance would silently point every target at
    an address that was never actually shown to accept the SSH key, which
    would surface later as a confusing launch failure instead of here, where
    the real cause is obvious. launcher may be None (auto-launch not
    configured on this deployment at all) -- selection is still recorded so
    the RUI's picker reflects it, just with nothing to actually apply."""
    instance = self.get_instance(instance_id)
    if instance.get('status') != 'verified':
      raise LauncherError("Cannot select '" + instance['display_name'] +
                           "': not yet verified (Test Connection first)")
    if launcher is not None:
      for target in launcher.config.get('launch_targets', {}).values():
        if not target:
          continue
        target['host'] = instance['host']
        target['ssh_user'] = instance['ssh_user']
        target['ssh_port'] = instance['ssh_port']
    for iid, entry in self.instances.items():
      if entry.get('selected'):
        entry.pop('selected', None)
        self._persist(iid)
    instance['selected'] = True
    self._persist(instance_id)
    self.selected_instance_id = instance_id
