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

import json
import os
import re
import socket
import subprocess
import time

import yaml

from nepi_api.simulator_launcher import LauncherError

# Mirrors sim_connector_app_node.py's own ROBOT_CONFIGS_STORAGE_DIR convention
# (one YAML file per entry, under this app's database tree) rather than
# simulator_launch_targets.yaml's own $NEPI_CONFIG/hand-edited-file convention:
# instances are created/removed through the RUI, not hand-authored, so they
# belong in the read/write database tree, not the dev-only checked-in config
# tree.
OS_INSTANCES_STORAGE_DIR = '/mnt/nepi_storage/databases/nepi_app_sim_connector/os_instances'

# Mailbox root for 'shared_storage'-mode instances (see CONNECTION_MODES'
# own comment) -- one subfolder per instance_id, matching
# vm_command_watcher.py's own convention exactly (that script builds this
# same path from a --storage-root it's given; this is the device's own
# local view of the identical nepi_storage tree, so no coordination beyond
# "the operator mounted the same share" is needed). Deliberately a sibling
# of OS_INSTANCES_STORAGE_DIR under the same app database tree, not nested
# under it -- these are message-passing scratch files, not persisted
# instance records.
VM_COMMANDS_STORAGE_DIR = '/mnt/nepi_storage/databases/nepi_app_sim_connector/vm_commands'

# Requested live (2026-09-04): reverse SSH into an operator's own laptop is
# a real security concern (a trusted key that can run ANY remote command).
# 'shared_storage' is an additive alternative -- nepi_storage is already a
# shared SMB drive both the device and the operator's machine can read/
# write, so a small file-drop protocol (see vm_command_watcher.py's own
# module docstring for the full command/status file protocol) replaces
# "SSH in and run a command" with "write a command file, a local watcher
# picks it up and runs a pre-defined command locally, writes a status file
# back" -- no listening port anywhere, and the watcher can only ever do
# what it's coded to do (run this app's own authored launch_command/
# stop_command/install_command/check_installed_command text), not arbitrary
# commands. 'ssh' stays the default and is unaffected -- this is a second
# choice per instance, not a replacement (confirmed: keep both, additive).
CONNECTION_MODES = ('ssh', 'shared_storage')
DEFAULT_CONNECTION_MODE = 'ssh'

# A shared_storage watcher writes watcher_heartbeat.json on this cadence
# (see vm_command_watcher.py's own HEARTBEAT_INTERVAL_SEC, kept in sync
# manually since this module deliberately doesn't import that script --
# it's meant to run standalone on a machine with no NEPI code installed).
# verify() treats a heartbeat older than
# WATCHER_HEARTBEAT_STALE_AFTER_SEC as "no watcher actually running here",
# not just "briefly between polls" -- several multiples of the write
# cadence so one slow tick under load doesn't read as unreachable.
WATCHER_HEARTBEAT_INTERVAL_SEC = 5
WATCHER_HEARTBEAT_STALE_AFTER_SEC = WATCHER_HEARTBEAT_INTERVAL_SEC * 4
WATCHER_VERIFY_TIMEOUT_SEC = 10

# The existing single-VM default every launch target hardcodes is 12222 --
# allocation starts one above it so a freshly-registered instance can never
# collide with that pre-existing default.
FIRST_ALLOCATED_SSH_PORT = 12223

# Id of the pseudo-instance representing whatever connection
# simulator_launch_targets.yaml itself hardcodes -- see ensure_baseline's own
# docstring. Never operator-created, but removable like any other instance
# (see remove()/ensure_baseline()) -- always present on a genuinely fresh
# install (nothing else registered yet) so the RUI's picker always has a
# real, named, already-verified entry to show instead of a generic "default"
# placeholder -- reported live: the picker should "name the name of the one
# it's currently connected to," never say "Default."
BASELINE_INSTANCE_ID = 'baseline'

# The fixed sim-utility ports every launch target's own launch_command
# hardcodes (heartbeat/bridge/camera listeners, MAVLink) -- forwarded back to
# the device by BOTH the original single-VM nepi_tunnel()/nepi-tunnel.service
# (docs/SIM_VM_CONNECTION_SETUP.md) and every OS instance's own generated
# tunnel below. Only the SSH control-leg port varies per instance (see
# build_setup_commands) -- these stay the same numbers regardless of which
# instance is selected, since only one simulator ever runs at a time (see
# SIM_OS_INSTANCES_PLAN.md's own "explicitly not doing" section).
#
# Reported live (2026-09-03): "it doesn't seem like any of the robot controls
# work for the robot -- i don't think that tcp is getting detected." Root
# cause: rbx_sim_discovery.py runs ON THE DEVICE and probes these ports on
# its OWN 127.0.0.1 (confirmed by reading it -- sim_addr_list = ['127.0.0.1'])
# -- it only ever finds anything because nepi_tunnel()'s reverse tunnel
# forwards them there. This module's own generated tunnel (STEP 2 of
# build_setup_commands) originally forwarded ONLY the SSH control-leg port
# (needed for simulator_launcher.py's _ssh_cmd to reach the target machine at
# all) -- once a launch actually started on a newly-registered instance, its
# heartbeat/bridge processes had nothing forwarding them back to the device,
# so discovery never saw them, silently, with no error anywhere (the launch
# itself reports "running" -- ready_check_command runs "on" the target
# machine and doesn't need the tunnel at all -- only cross-machine discovery
# was broken). Fixed by forwarding this same fixed list from every OS
# instance's own tunnel too, automatically, with no additional operator step
# -- reported requirement: "this should automatically happen for any
# connected vm without me prompting."
#
# Registering and tunneling a SECOND instance while the first's tunnel is
# still up (explicitly supported -- see SIM_OS_INSTANCES_PLAN.md: "multiple
# instances can be registered/reachable simultaneously") surfaced a second
# bug the same day: these ports are identical across every instance's
# tunnel on purpose (only one simulator ever runs at a time, so there's
# only ever one real listener to reach), so the SECOND tunnel's `-R`
# requests for them are always rejected by sshd -- the first tunnel already
# holds them. With `-o ExitOnForwardFailure=yes` (needed on the SSH
# control-leg port so a genuinely dead connection is detected -- see that
# same flag's own history above), a single rejected forward killed the
# WHOLE ssh session, including the control-leg port that instance actually
# owns -- confirmed live: a second registered+tunneled instance crash-
# looped forever ("remote port forwarding failed for listen port 5760"),
# unreachable even for its own install/verify/launch despite never being
# selected. Fixed in build_setup_commands by putting these shared ports on
# their own ssh invocation, separate from the control-leg port, WITHOUT
# ExitOnForwardFailure -- a rejected shared-port bind now just logs a
# harmless warning and the session stays up, while the control-leg port
# (which has no such conflict, unique per instance) keeps its own strict
# connection, in its own ssh invocation, where ExitOnForwardFailure still
# means what it says.
SIM_UTILITY_TUNNEL_PORTS = [
    5760, 5771,
    9021, 9022, 9023, 9024, 9025, 9026, 9027, 9028, 9029,
    9041, 9042, 9046, 9047,
]

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
    """Reported live (2026-09-03): "robot controls don't work... TCP isn't
    getting detected" traced back to selected_instance_id silently reverting
    to 'baseline' -- baseline.yaml had 'selected: true' persisted while the
    operator's actually-chosen instance's own file had no 'selected' key at
    all, even though select() always clears every other instance's flag
    before setting the new one's. The likely cause: os.listdir() order is
    filesystem-dependent, not alphabetical or creation-order -- if two
    files ever transiently both carried 'selected: true' (e.g. a restart
    landing between select()'s clear-old and persist-new writes), which one
    "won" here depended on that arbitrary order, silently and
    non-deterministically, across restarts. Sorted filenames below remove
    that nondeterminism; the dedup pass after the loop makes a second
    'selected: true' self-healing instead of a silent, unpredictable
    reversion -- and prefers a real, operator-chosen instance over baseline
    (see BASELINE_INSTANCE_ID's own comment: baseline is a fallback-of-last-
    resort, never something worth silently preferring over a real choice)."""
    try:
      filenames = sorted(os.listdir(self.storage_dir))
    except OSError:
      return
    selected_ids = []
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
        selected_ids.append(instance_id)
    if len(selected_ids) > 1:
      non_baseline = [iid for iid in selected_ids if iid != BASELINE_INSTANCE_ID]
      winner = non_baseline[0] if non_baseline else selected_ids[0]
      for iid in selected_ids:
        if iid != winner:
          self.instances[iid].pop('selected', None)
          self._persist(iid)
      self.selected_instance_id = winner
    elif selected_ids:
      self.selected_instance_id = selected_ids[0]

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

  def _device_public_key(self):
    """Reads this device's own SSH PUBLIC key -- the .pub half of whatever
    _ssh_key() would use as the private key -- so the wizard can show the
    exact line to append to the new machine's authorized_keys, instead of a
    vague "make sure it's trusted" instruction with nothing to copy.

    Confirmed live this session (a real end-to-end test, register through
    select): a working reverse tunnel is NOT enough on its own. The tunnel
    only lets the device dial back into the new machine's sshd -- it still
    needs a key the new machine actually trusts to complete that login,
    same as this app's own pre-existing single-VM feature already requires
    (see SIM_VM_CONNECTION_SETUP.md's original Step 1, "On the device...
    whose PUBLIC half is in the VM user's authorized_keys" -- a requirement
    that doc got right and the 2-step wizard lost when the keygen step was
    removed). Returns None if no candidate's matching .pub file is
    readable, in which case callers fall back to a plain instruction."""
    for candidate in self._ssh_key_candidates():
      try:
        with open(candidate + '.pub', 'r') as f:
          content = f.read().strip()
        if content:
          return content
      except OSError:
        continue
    return None

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
  # shared_storage transport -- see CONNECTION_MODES' own comment and
  # vm_command_watcher.py's module docstring for the full design/protocol.

  def _mailbox_dir(self, instance_id):
    return os.path.join(VM_COMMANDS_STORAGE_DIR, instance_id)

  def _heartbeat_path(self, instance_id):
    return os.path.join(self._mailbox_dir(instance_id), 'watcher_heartbeat.json')

  def _shared_storage_setup_commands(self, instance):
    """The copy-paste block for a 'shared_storage' instance -- no SSH key,
    no reverse tunnel, no listening port. The operator only needs the same
    nepi_storage SMB share already mounted (see this module's own
    CONNECTION_MODES comment for why that's the whole point) and a copy of
    vm_command_watcher.py, which ships in this repo's own
    sim_container/scripts/ -- the exact same file this device's own
    checkout has, since the protocol is a strict file contract, not a
    version-sensitive API.

    MOUNT_PATH_PLACEHOLDER is deliberately left for the operator to fill
    in: unlike build_setup_commands' device_host (a real, guessable value
    on the DEVICE side), this script has no way to know what local path an
    arbitrary laptop mounted the SAME share at (a drive letter on Windows,
    an arbitrary mountpoint on Linux/WSL) -- see _guess_device_ip's own
    docstring for the same reasoning applied to the SSH path's device_host,
    which faces the opposite direction (that one IS guessable, this one
    genuinely is not)."""
    iid = instance['instance_id']
    return (
        "Run this on the NEW machine (" + instance['display_name'] + ").\n"
        "No SSH key, tunnel, or listening port needed for this connection\n"
        "type -- it talks to the device only through the nepi_storage share\n"
        "you already have mounted.\n"
        "\n"
        "1. Confirm nepi_storage is mounted locally and you can see:\n"
        "     <your mount path>/databases/nepi_app_sim_connector/\n"
        "   (ask your NEPI admin for the mount details if you don't already\n"
        "   have this share mounted -- it's the same share nepi setup itself\n"
        "   uses, nothing new to provision.)\n"
        "\n"
        "2. Copy vm_command_watcher.py (from this device's own checkout,\n"
        "   nepi_drones/sim_container/scripts/vm_command_watcher.py) to this\n"
        "   machine, then start it -- re-run this command any time the\n"
        "   watcher needs restarting; wrap it in a systemd/scheduled-task\n"
        "   unit yourself if you want it to survive a reboot, that's not\n"
        "   required for it to work:\n"
        "\n"
        "     python3 vm_command_watcher.py --storage-root <your mount path> "
        "--instance-id " + iid + "\n"
        "\n"
        "Then back here in the RUI: click \"Test Connection.\""
    )

  def _verify_shared_storage(self, instance):
    """'Test Connection' for a shared_storage instance -- there is no SSH
    endpoint to probe, so this checks for a live watcher_heartbeat.json
    instead (see WATCHER_HEARTBEAT_STALE_AFTER_SEC's own comment for why
    'exists but stale' still counts as unreachable, not just 'missing').
    Returns (ok, error_message)."""
    path = self._heartbeat_path(instance['instance_id'])
    try:
      with open(path, 'r') as f:
        heartbeat = json.load(f)
    except (OSError, ValueError):
      return False, ("No watcher_heartbeat.json found yet at " + path +
                      " -- has vm_command_watcher.py been started on that "
                      "machine, pointed at the right --instance-id and a "
                      "nepi_storage mount that's actually the same share "
                      "this device uses?")
    alive_at = heartbeat.get('alive_at')
    if not isinstance(alive_at, (int, float)):
      return False, "watcher_heartbeat.json is malformed (no numeric 'alive_at')"
    age_sec = time.time() - alive_at
    if age_sec > WATCHER_HEARTBEAT_STALE_AFTER_SEC:
      return False, ("Last watcher heartbeat was " + str(int(age_sec)) +
                      "s ago (stale) -- the watcher process on that machine "
                      "appears to have stopped")
    return True, ''

  #**********************
  # Setup-command generation

  def build_setup_commands(self, instance):
    """The exact copy-paste block the RUI shows for a freshly-registered
    instance. Dispatches on connection_mode first (see CONNECTION_MODES'
    own comment) -- shared_storage's own build_setup_commands equivalent,
    _shared_storage_setup_commands, has none of this SSH-specific history
    below to inherit at all. History: went through three earlier shapes this same week
    (a numbered STEP 1/2/3 walkthrough with lettered sub-steps and two
    parallel "Option A/B" tunnel paths each with several paragraphs of
    prose) as each individually-reported gap got fixed -- see
    docs/SIM_OS_INSTANCES_PLAN.md's own dated entries for the full history
    of what each of those fixed and why (missing SSH server, missing
    device-pubkey trust, missing passwordless sudo, a broken `cp` assuming
    nepi_drones was checked out on the target, missing SSH keepalives,
    invalid heredoc indentation, a second-instance port-forwarding
    collision). Collapsed here (2026-09-03) into ONE paste-once block --
    reported live: "way too many steps and too complex... simplify this and
    combine it into smaller commands." Every fix above is still present in
    the commands themselves; only the surrounding prose walkthrough (three
    numbered steps, two lettered sub-steps, two fully-spelled-out parallel
    options) was cut. The systemd path is the only one shown by default now
    (WSL-without-systemd gets a two-line fallback instead of an equally
    prose-heavy parallel "Option B") since it is the one that survives a
    reboot and self-heals, which is what most people want; docs/
    SIM_VM_CONNECTION_SETUP.md remains the fuller reference doc for anyone
    who wants the two-tunnel design explained end to end.

    No SSH-key-generation step, ever (see the git history for why one used
    to exist and why it was removed -- it once overwrote a machine's own
    already-working key). Assumes NEPI Remote Setup already ran here, which
    is the overwhelmingly common case; a machine that has genuinely never
    done that needs Remote Setup first, not a key generated in isolation.

    device_host is this device's own real, currently-detected IP (see
    _guess_device_ip), not a placeholder. user (nepi) and port (2222) are
    this platform's fixed convention (confirmed against a real device: the
    host-level 'nepi' account can't log in at all, only the container's
    port 2222 can), not placeholders either -- only device_host and the
    operator's own username (entered in the RUI at verify time) actually
    vary per deployment.

    Still forwards SIM_UTILITY_TUNNEL_PORTS as a SEPARATE tunnel from the
    per-instance SSH control-leg port -- see that constant's own comment
    for why (a second registered instance's copy would otherwise crash-loop
    fighting the first for the same shared ports)."""
    if instance.get('connection_mode', DEFAULT_CONNECTION_MODE) == 'shared_storage':
      return self._shared_storage_setup_commands(instance)
    port = instance['ssh_port']
    iid = instance['instance_id']
    # One -R flag per fixed sim-utility port, same numbers for every
    # instance (see SIM_UTILITY_TUNNEL_PORTS) -- built once, shared by the
    # systemd unit and its WSL-fallback one-liner below.
    shared_port_args = " ".join(
        "-R " + str(p) + ":127.0.0.1:" + str(p) for p in SIM_UTILITY_TUNNEL_PORTS)
    # No angle brackets in this fallback placeholder (rare -- only when
    # _guess_device_ip finds neither a config file nor a route) -- reported
    # live: pasting a bracketed placeholder verbatim into a real shell
    # command breaks it outright, since '<' is redirection syntax, rather
    # than failing obviously (e.g. "no such host").
    device_host = _guess_device_ip() or "YOUR_NEPI_DEVICE_IP_OR_HOSTNAME"
    device_pubkey = self._device_public_key()
    device_pubkey_line = (
        device_pubkey if device_pubkey else
        "PASTE_THIS_DEVICE'S_OWN_PUBLIC_KEY_HERE  # could not read it automatically"
    )
    return (
        "Run this whole block on the NEW machine (" + instance['display_name'] + ").\n"
        "Assumes NEPI Remote Setup already ran here (so ~/.ssh/nepi_default_ssh_key\n"
        "already exists and the device already trusts it) -- if not, do that\n"
        "first: NEPI_REMOTE_SETUP.md.\n"
        "\n"
        "    command -v sshd >/dev/null || sudo apt-get install -y openssh-server\n"
        "    sudo systemctl enable --now ssh\n"
        "    echo \"" + device_pubkey_line + "\" >> ~/.ssh/authorized_keys\n"
        "    echo \"$USER ALL=(ALL) NOPASSWD:ALL\" | sudo tee /etc/sudoers.d/nepi-sim-connector\n"
        "    sudo chmod 0440 /etc/sudoers.d/nepi-sim-connector\n"
        "    command -v autossh >/dev/null || sudo apt-get install -y autossh\n"
        "    mkdir -p ~/.config/systemd/user\n"
        "    cat > ~/.config/systemd/user/nepi-tunnel-" + iid + ".service <<'EOF'\n"
        "[Unit]\n"
        "Description=NEPI reverse SSH tunnel to device (OS instance " + iid + ", control leg)\n"
        "After=network-online.target\n"
        "Wants=network-online.target\n"
        "StartLimitIntervalSec=0\n"
        "\n"
        "[Service]\n"
        "Type=simple\n"
        "ExecStart=/usr/bin/autossh -M 0 -N -R " + str(port) + ":127.0.0.1:22 -p 2222 "
        "-o ServerAliveInterval=15 -o ServerAliveCountMax=3 "
        "-o ExitOnForwardFailure=yes -o ConnectTimeout=5 "
        "-i %h/.ssh/nepi_default_ssh_key nepi@" + device_host + "\n"
        "Environment=AUTOSSH_GATETIME=0\n"
        "Restart=always\n"
        "RestartSec=10\n"
        "\n"
        "[Install]\n"
        "WantedBy=default.target\n"
        "EOF\n"
        "    cat > ~/.config/systemd/user/nepi-tunnel-" + iid + "-shared.service <<'EOF'\n"
        "[Unit]\n"
        "Description=NEPI reverse SSH tunnel to device (OS instance " + iid + ", shared sim-utility ports)\n"
        "After=network-online.target\n"
        "Wants=network-online.target\n"
        "StartLimitIntervalSec=0\n"
        "\n"
        "[Service]\n"
        "Type=simple\n"
        "ExecStart=/usr/bin/autossh -M 0 -N " + shared_port_args + " -p 2222 "
        "-o ServerAliveInterval=15 -o ServerAliveCountMax=3 "
        "-o ConnectTimeout=5 "
        "-i %h/.ssh/nepi_default_ssh_key nepi@" + device_host + "\n"
        "Environment=AUTOSSH_GATETIME=0\n"
        "Restart=always\n"
        "RestartSec=10\n"
        "\n"
        "[Install]\n"
        "WantedBy=default.target\n"
        "EOF\n"
        "    systemctl --user daemon-reload\n"
        "    systemctl --user enable --now nepi-tunnel-" + iid + ".service nepi-tunnel-" + iid + "-shared.service\n"
        "    loginctl enable-linger $(id -un)\n"
        "\n"
        "On WSL without systemd enabled: skip the systemd block above and run\n"
        "these two lines instead (re-run them if the tunnel ever needs\n"
        "restarting -- they won't survive a reboot on their own):\n"
        "\n"
        "    autossh -M 0 -f -N -R " + str(port) + ":127.0.0.1:22 -p 2222 -o ServerAliveInterval=15 -o ServerAliveCountMax=3 -o ExitOnForwardFailure=yes -o ConnectTimeout=5 -i ~/.ssh/nepi_default_ssh_key nepi@" + device_host + "\n"
        "    autossh -M 0 -f -N " + shared_port_args + " -p 2222 -o ServerAliveInterval=15 -o ServerAliveCountMax=3 -o ConnectTimeout=5 -i ~/.ssh/nepi_default_ssh_key nepi@" + device_host + "\n"
        "\n"
        "Then back here in the RUI: enter your username on this machine below\n"
        "and click \"Test Connection.\""
    )

  #**********************
  # Public API, called from sim_connector_app_node.py

  def list_instances(self):
    return dict(self.instances)

  def get_instance(self, instance_id):
    if instance_id not in self.instances:
      raise LauncherError("Unknown OS instance: " + str(instance_id))
    return self.instances[instance_id]

  def register(self, display_name, connection_mode=DEFAULT_CONNECTION_MODE):
    """Creates a new, unverified instance and returns (instance_id,
    setup_commands). host/ssh_user stay blank until verify() fills them in
    -- nothing here can guess a real reachable address up front (ssh mode
    only -- shared_storage mode never uses them at all, see
    CONNECTION_MODES' own comment). ssh_port is still allocated even for a
    shared_storage instance: cheap, harmless if unused, and means switching
    an existing instance's mode later (not currently exposed, but plausible)
    never has to worry about a port collision that was never checked."""
    if connection_mode not in CONNECTION_MODES:
      raise LauncherError("Unknown connection_mode: " + str(connection_mode) +
                           " (expected one of " + ", ".join(CONNECTION_MODES) + ")")
    display_name = _sanitize_display_name(display_name)
    instance_id = _instance_id_from_name(display_name, self.instances.keys())
    instance = {
        'instance_id': instance_id,
        'display_name': display_name,
        'connection_mode': connection_mode,
        'host': '',
        'ssh_user': '',
        'ssh_port': self._next_ssh_port(),
        'status': 'pending',
    }
    self.instances[instance_id] = instance
    self._persist(instance_id)
    return instance_id, self.build_setup_commands(instance)

  def verify(self, instance_id, host=None, ssh_user=None):
    """Runs the real reachability probe and updates + persists the
    instance's status. For a shared_storage instance this means checking
    for a live watcher_heartbeat.json (see _verify_shared_storage) instead
    of an SSH probe -- host/ssh_user are ignored entirely in that mode,
    there's nothing to fill in. For an ssh instance (default, unchanged):
    host defaults to 127.0.0.1 (the reverse-tunnel convention every existing
    launch target already uses) the first time, when not given explicitly --
    a caller reaching the machine directly on a routable LAN address passes
    host itself instead. Raises LauncherError (with the real failure text)
    on a failed probe, after still recording/persisting 'unreachable'."""
    instance = self.get_instance(instance_id)
    if instance.get('connection_mode', DEFAULT_CONNECTION_MODE) == 'shared_storage':
      ok, error = self._verify_shared_storage(instance)
      instance['status'] = 'verified' if ok else 'unreachable'
      self._persist(instance_id)
      if not ok:
        raise LauncherError("Connection test failed for '" + instance['display_name'] + "': " + error)
      return instance
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
    boot, or after a fresh install).

    Deliberately does NOT recreate baseline if it was explicitly removed
    (see remove()) AND at least one other instance is still registered --
    reported live: "the previous vm that was originally set up is not
    removable... make sure any vm is removable." baseline used to be
    unconditionally (re)created here on every single app startup, which
    made remove()'s own baseline guard below moot even if it were lifted --
    the very next restart would just bring it back. Still recreated when
    self.instances is completely empty, matching this method's original
    purpose (never leave the picker with zero options) -- that case is
    indistinguishable from "never registered anything yet" and a genuinely
    fresh install still needs a starting point."""
    if BASELINE_INSTANCE_ID not in self.instances and len(self.instances) > 0:
      return
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
    # Every instance is removable, baseline included -- see ensure_baseline's
    # own comment for how a removed baseline is kept from silently coming
    # back on the next app restart. If this was the only instance left,
    # ensure_baseline will recreate it next boot regardless (never leave the
    # picker with zero options); that's the one case removing it doesn't
    # stick, and it's the same "genuinely nothing registered" case a fresh
    # install starts from anyway.
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
    the given instance's connection info onto EVERY target in
    launcher.config['launch_targets'] (a plain in-memory dict -- confirmed by
    reading simulator_launcher.py), in place. For an ssh instance (default,
    unchanged) that means host/ssh_user/ssh_port -- every existing code path
    (launch/stop/is_installed/install/_ssh_cmd) already reads those three
    fields per-target. For a shared_storage instance it means
    connection_mode + os_instance_id, which simulator_launcher.py's own
    dispatch (see its own comment on _dispatch_shared_storage) reads instead
    of ever building an SSH command at all -- so nothing else needs to
    change for either mode to take effect immediately on the next
    launch/install/check.

    Raises if the instance isn't 'verified' yet (Test Connection first) --
    selecting an unconfirmed instance would silently point every target at
    a connection that was never actually shown to work (an SSH key never
    tested, or a watcher never confirmed alive), which would surface later
    as a confusing launch failure instead of here, where the real cause is
    obvious. launcher may be None (auto-launch not configured on this
    deployment at all) -- selection is still recorded so the RUI's picker
    reflects it, just with nothing to actually apply."""
    instance = self.get_instance(instance_id)
    if instance.get('status') != 'verified':
      raise LauncherError("Cannot select '" + instance['display_name'] +
                           "': not yet verified (Test Connection first)")
    connection_mode = instance.get('connection_mode', DEFAULT_CONNECTION_MODE)
    if launcher is not None:
      for target in launcher.config.get('launch_targets', {}).values():
        if not target:
          continue
        target['connection_mode'] = connection_mode
        if connection_mode == 'shared_storage':
          target['os_instance_id'] = instance_id
        else:
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
