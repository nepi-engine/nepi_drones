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

SSH_CONNECT_TIMEOUT_SEC = 8

DEFAULT_SSH_KEY = os.path.expanduser("~/.ssh/nepi_default_ssh_key")


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
    paths (systemd, and the plain-autossh fallback needed on a machine like
    this dev VM's own WSL environment, per that doc's own WSL callout), with
    the real allocated port substituted in rather than a human hand-editing
    port numbers out of a markdown file."""
    port = instance['ssh_port']
    iid = instance['instance_id']
    return (
        "# 1) SSH key -- reuse ~/.ssh/nepi_default_ssh_key if you already set one up\n"
        "#    for an earlier instance; skip straight to step 2 if so.\n"
        "ssh-keygen -t ed25519 -f ~/.ssh/nepi_default_ssh_key -N \"\"\n"
        "# Copy the PUBLIC half (nepi_default_ssh_key.pub) into THIS machine's own\n"
        "# ~/.ssh/authorized_keys -- the NEPI device already trusts this key.\n"
        "\n"
        "# 2) Reverse tunnel back to the NEPI device, forwarding port " + str(port) + "\n"
        "#    to this machine's own sshd -- pick ONE of the two options below.\n"
        "\n"
        "# Option A -- systemd (recommended, survives reboots):\n"
        "mkdir -p ~/.config/systemd/user ~/.config\n"
        "cp sim_container/systemd/nepi-tunnel.service "
        "~/.config/systemd/user/nepi-tunnel-" + iid + ".service\n"
        "cat > ~/.config/nepi-tunnel-" + iid + ".env <<'EOF'\n"
        "DEVICE_SSH_HOST=<your NEPI device's real IP or hostname>\n"
        "DEVICE_SSH_USER=nepi\n"
        "DEVICE_SSH_PORT=22\n"
        "TUNNEL_SSH_PORT=" + str(port) + "\n"
        "EOF\n"
        "systemctl --user daemon-reload\n"
        "systemctl --user enable --now nepi-tunnel-" + iid + ".service\n"
        "loginctl enable-linger $(id -un)\n"
        "\n"
        "# Option B -- plain autossh, no systemd needed (e.g. WSL without\n"
        "#    systemd=true in /etc/wsl.conf):\n"
        "autossh -M 0 -f -N -R " + str(port) + ":127.0.0.1:22 -p 22 \\\n"
        "  -i ~/.ssh/nepi_default_ssh_key <your NEPI device user>@<your NEPI device host>\n"
        "\n"
        "# 3) Verify, then click \"Test Connection\" in the RUI:\n"
        "ssh -p " + str(port) + " <your username>@localhost echo ok"
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

  def remove(self, instance_id):
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
