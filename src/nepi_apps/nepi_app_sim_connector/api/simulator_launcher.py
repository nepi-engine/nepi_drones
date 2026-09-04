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

# Additive simulator auto-launch helper -- see
# docs/SIMULATOR_AUTO_LAUNCH_PLAN.md (nepi_drones) for the full design.
#
# Deliberately zero nepi_sdk/rospy dependency, same reasoning
# demo_bridge_client.py already establishes: this is plain SSH orchestration,
# testable standalone before sim_connector_app_node.py ever imports it.
#
# No credentials are ever read from simulator_launch_targets.yaml -- the SSH
# private key path comes from the NEPI_SSH_KEY environment variable, same
# convention every deploy_*.sh script in this repo already uses.

import json
import os
import re
import subprocess
import time

import yaml

# Where the app looks for launch targets, in order. The env var is the explicit
# override; the NEPI_CONFIG path is the discoverable default, since apps_mgr
# spawns app nodes without any per-app env vars, so an env-var-only opt-in
# could never work for the real, apps_mgr-launched process (confirmed the hard
# way: the production instance reported an empty launch-target list while an
# env-var-launched test instance reported the Gazebo target fine).
# NEPI_CONFIG is already in every app node's inherited environment.
#
# Absence of the file is still the safe default that disables auto-launch
# entirely -- a real deployed device has no reason to have one, and no
# credentials live in it either way (see the SSH key note below).
LAUNCH_TARGETS_CONFIG_ENV_VAR = 'NEPI_SIM_LAUNCH_TARGETS_CONFIG'
LAUNCH_TARGETS_CONFIG_BASENAME = 'simulator_launch_targets.yaml'
FALLBACK_NEPI_CONFIG_DIR = '/mnt/nepi_config'

DEFAULT_SSH_KEY = os.path.expanduser("~/.ssh/nepi_default_ssh_key")
SSH_CONNECT_TIMEOUT_SEC = 8
# How long to give the initial launch a chance to fail fast (bad host, auth
# rejected, remote command exits immediately) before treating the ssh
# connection as successfully holding the sim open in the background.
LAUNCH_STARTUP_GRACE_SEC = 5
READY_CHECK_ATTEMPTS = 6
READY_CHECK_INTERVAL_SEC = 3
# An install can mean anything from a pip install to a multi-package apt
# transaction with a slow mirror -- generous on purpose; this blocks the
# caller's dedicated install thread, never the main status/launch path.
# Raised 600 -> 3600 (2026-09-02) once gazebo_quadcopter got a real
# install_command: a from-source ArduPilot build plus ros-noetic-desktop-full
# plus the ardupilot_gazebo bridge's own cmake build can easily run past
# 10 minutes on modest hardware -- 600s was sized for the package-manager
# one-liners every other target uses, not a build like this one.
INSTALL_TIMEOUT_SEC = 3600

# Shared-storage transport (additive alongside SSH -- see
# os_instance_registry.py's CONNECTION_MODES and vm_command_watcher.py's own
# module docstring for the full design/protocol). Duplicated, not imported,
# from os_instance_registry.py's own identical constant -- same reasoning
# that module already gives for duplicating THIS file's SSH-key logic
# instead of importing it: os_instance_registry.py imports LauncherError
# from here, so importing back would be circular, and the two modules are
# meant to stay independently optional.
VM_COMMANDS_STORAGE_DIR = '/mnt/nepi_storage/databases/nepi_app_sim_connector/vm_commands'
# Extra slack added on top of a dispatched command's own timeout_sec before
# giving up on ever seeing its status file at all -- covers the watcher's
# own poll-interval granularity (it only checks for new command files once
# per tick) plus this process's own poll granularity below, so a command
# that itself finishes right at its deadline doesn't lose a race against
# this side's own polling loop.
SHARED_STORAGE_POLL_GRACE_SEC = 5
SHARED_STORAGE_POLL_INTERVAL_SEC = 1.0
# Mirrors os_instance_registry.py's own WATCHER_HEARTBEAT_INTERVAL_SEC/
# WATCHER_HEARTBEAT_STALE_AFTER_SEC (duplicated, not imported -- same reason
# this file's own module docstring gives for why these two modules don't
# import each other). Used by _shared_storage_watcher_alive below, the same
# "is a watcher actually alive here" test _verify_shared_storage does for an
# operator-initiated Test Connection, just reused here for an AUTOMATIC,
# no-operator-action fallback check.
WATCHER_HEARTBEAT_STALE_AFTER_SEC = 20
# Mirrors vm_command_watcher.py's own DEPLOY_STATE_FILENAME (duplicated, not
# imported -- same reasoning as every other constant this file already
# shares with that module's own docstring). This is the REAL deploy/kill
# transport, not a fallback -- see launch()/stop()/is_ready()'s own shared_
# storage branches. Requested live (2026-09-04): a real deployment gives the
# device no network path to the VM at all (no forward SSH, no reverse
# tunnel -- "all that the user will have is ssh from a linux os to the nepi
# device"), so deploy/kill has to go through nepi_storage as a plain flag,
# not any request/response protocol that assumes a channel back.
DEPLOY_STATE_FILENAME = 'deploy_state.yaml'


class LauncherError(Exception):
  """manual_fallback_commands, when set, overrides whatever per-target
  install fallback sim_connector_app_node.py's publishLauncherStatus would
  otherwise attach -- used for a reverse-tunnel connectivity diagnosis
  (see _classify_connection_failure), where the actual fix has nothing to
  do with that target's own install_command."""
  def __init__(self, message, manual_fallback_commands=None):
    super().__init__(message)
    self.manual_fallback_commands = manual_fallback_commands


# Every default target's host is "127.0.0.1" by design -- the device SSHes
# to a LOCAL port that only means anything because nepi_tunnel()/
# nepi-tunnel.service's autossh, running FROM the sim VM, forwards it back
# from the VM's own sshd (see docs/SIM_VM_CONNECTION_SETUP.md). A
# connection-level failure against that specific host therefore has one
# most-likely explanation worth naming outright: nothing is listening
# there because that reverse tunnel isn't running, not a generic "can't
# reach the VM" -- which is what made the very first report of this
# ("Timed out waiting for the simulator to become ready") so hard to act
# on: the real cause was two hops away from the symptom. Covers both
# hosting setups this has been reported from: a VirtualBox VM (systemd
# --user works out of the box) and WSL/WSL2 (systemd is opt-in via
# /etc/wsl.conf, so a manual nepi_tunnel fallback is offered too).
REVERSE_TUNNEL_FALLBACK_COMMANDS = """VirtualBox VM:
  systemctl --user status nepi-tunnel.service
  systemctl --user enable --now nepi-tunnel.service   # if inactive

WSL / WSL2 (systemd is not enabled by default):
  echo -e "[boot]\\nsystemd=true" | sudo tee -a /etc/wsl.conf
  # From PowerShell: wsl --shutdown, then reopen your WSL terminal, then:
  systemctl --user enable --now nepi-tunnel.service
  # Or, without enabling systemd, start it manually each session instead:
  nepi_tunnel &

Full setup (SSH keys, env var overrides for a non-default device/VM
username): nepi_drones/docs/SIM_VM_CONNECTION_SETUP.md"""

# Reported live (2026-09-02): every install_command needing apt-get failed
# on a genuinely fresh VM with "sudo: a terminal is required to read the
# password" -- _ssh_cmd() never allocates a pty (no -t), so a remote sudo
# call has nothing to prompt on. The original developer's own VM never hit
# this because it already had passwordless sudo configured by hand, long
# before this feature existed. There is no fix that doesn't require the
# operator to configure this once, interactively: piping a password over a
# non-interactive SSH session would mean storing/transmitting a sudo
# password, which this whole file's own design explicitly avoids for SSH
# keys already (see the module docstring) and shouldn't start doing for
# sudo either.
SUDO_NOPASSWD_FALLBACK_COMMANDS = """Run this ONCE on the sim VM (interactively, in a real terminal, so sudo
can prompt for your password this one time):

  echo "$USER ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/nepi-sim-connector
  sudo chmod 0440 /etc/sudoers.d/nepi-sim-connector

Every future Install click works without prompting after this -- Install
runs apt-get over a non-interactive SSH connection with no terminal for
sudo to read a password from, so without passwordless sudo configured,
every target's install_command fails the same way, not just the one you
just tried."""


def find_config_path():
  """Returns the launch-targets config path to use, or "" if this deployment
  has none (the normal case for a real device -- auto-launch simply stays
  unavailable). Env var wins so a dev can point at an alternate file without
  moving anything."""
  env_path = os.environ.get(LAUNCH_TARGETS_CONFIG_ENV_VAR, '')
  if env_path:
    return env_path
  config_dir = os.environ.get('NEPI_CONFIG', FALLBACK_NEPI_CONFIG_DIR)
  candidate = os.path.join(config_dir, LAUNCH_TARGETS_CONFIG_BASENAME)
  if os.path.isfile(candidate):
    return candidate
  return ""


class SimulatorLauncher(object):
  """Loads simulator_launch_targets.yaml and drives per-target launch/stop/
  readiness over SSH. One instance per app node; construction is cheap (just
  a yaml read), so re-instantiate on config changes rather than caching."""

  def __init__(self, config_path):
    self.config_path = config_path
    self.config = self._load_config(config_path)
    self._config_mtime = self._config_mtime_now()
    # target_key -> Popen holding that target's launch ssh connection open.
    # Its lifetime IS the sim's lifetime (see launch_command's own comments
    # in simulator_launch_targets.yaml for why) -- must not be garbage
    # collected or waited on while the sim should still be running.
    self._launch_procs = {}
    # Monotonic counter for _dispatch_shared_storage's own request_id
    # generation -- see that method's own comment for why timestamp alone
    # isn't quite enough.
    self._shared_storage_seq = 0

  def _load_config(self, config_path):
    if not os.path.isfile(config_path):
      raise LauncherError("Config file not found: " + config_path)
    with open(config_path, "r") as f:
      config = yaml.safe_load(f)
    if config is None or "launch_targets" not in config:
      raise LauncherError("Config missing top-level 'launch_targets': " + config_path)
    return config

  def _config_mtime_now(self):
    try:
      return os.path.getmtime(self.config_path)
    except OSError:
      return None

  def reload_if_changed(self):
    """Re-reads the config when the file has changed on disk, so adding or
    editing a launch target takes effect without restarting the app. Returns
    True only when a reload actually happened (the caller republishes status
    so a client's target list refreshes).

    Worth having rather than documenting "restart to apply": a stale in-memory
    config is invisible and actively misleading -- a freshly-deployed
    refuse-to-launch guard silently didn't fire during testing because the
    node still held the previous config.

    A malformed edit leaves the previous good config in place (and returns
    False) rather than dropping every target mid-session; the caller has no
    better recovery, and an editor mid-save shouldn't break a running app."""
    mtime = self._config_mtime_now()
    if mtime is None or mtime == self._config_mtime:
      return False
    try:
      config = self._load_config(self.config_path)
    except LauncherError:
      # Record the mtime anyway, so one bad save isn't retried every tick.
      self._config_mtime = mtime
      return False
    self.config = config
    self._config_mtime = mtime
    return True

  def get_available_targets(self):
    """Returns (keys, display_names) for every non-empty, offered entry in
    launch_targets -- placeholder entries ({}) are not offered, matching
    how an unconfigured target has nothing this class could actually run.
    An entry with hidden_from_selector: true (the same flag/meaning a
    robot_configs entry uses) is reachable only through another target's
    own launch_target_overrides via resolve_launch_target below, never
    picked directly -- e.g. gazebo_rover's own rover world/bridge can't
    serve a flight profile, so it redirects to the hidden gazebo_quadcopter
    target instead. Use list_all_target_keys() when a hidden target's own
    dependency state still needs tracking (the background check sweep)."""
    keys = []
    names = []
    for key, entry in self.config["launch_targets"].items():
      if entry and not entry.get("hidden_from_selector", False):
        keys.append(key)
        names.append(entry.get("display_name", key))
    return keys, names

  def list_all_target_keys(self):
    """Every non-empty entry's key, hidden_from_selector or not -- for the
    background dependency-check sweep, which tracks every real target
    whether or not it's directly offered (mirrors how sim_connector_app_node.py
    treats a hidden_from_selector robot_configs entry: fully valid and
    checked, just not offered in the selector list)."""
    return [key for key, entry in self.config["launch_targets"].items() if entry]

  def get_target(self, target_key):
    targets = self.config["launch_targets"]
    if target_key not in targets or not targets[target_key]:
      raise LauncherError("Unknown or unconfigured launch target: " + str(target_key))
    return targets[target_key]

  def get_default_robot_config(self, target_key):
    return self.get_target(target_key).get("default_robot_config", "")

  def resolve_robot_config(self, target_key, generic_config):
    """Maps a plain, offered Robot Config choice (e.g. "ground_robot_2_wheel",
    what the RUI's selector shows as "2-Wheel Rover") to whatever profile this
    specific target actually needs -- most targets need no mapping at all
    (Gazebo's own configs already match the generic keys 1:1), but a target
    whose simulator can't fully match a generic profile (WPILib's HAL sim has
    no camera or environment concept) points its
    own robot_config_overrides at the specialized, hidden_from_selector
    profile instead. This is what makes "pick 2-Wheel Rover, deploy to
    WPILib" apply the right profile automatically -- there is no separate
    "2-Wheel Rover (WPILib)" selector entry to remember to pick instead."""
    overrides = self.get_target(target_key).get("robot_config_overrides", {})
    return overrides.get(generic_config, generic_config)

  def resolve_launch_target(self, target_key, robot_config):
    """Inverse of resolve_robot_config: maps a plain, offered Simulator
    choice (e.g. "gazebo_rover", shown as "Gazebo") plus whichever robot
    config is currently selected to whichever target must actually run for
    that combination. Most combinations need no redirection at all (a
    target's own launch_command already serves every robot config it
    reasonably can), but a robot config a target's world/bridge
    fundamentally cannot drive (a 4-motor flight profile has no meaning to
    a wheeled rover's diff-drive bridge) points at a different,
    hidden_from_selector target instead, via this target's own
    launch_target_overrides -- gazebo_rover maps flight_robot_4_motor to
    gazebo_quadcopter's ArduCopter SITL + iris model: a different world,
    bridge, and install requirement entirely. This is what makes "pick
    Quadcopter, deploy to Gazebo" launch the right thing automatically --
    there is no separate "Gazebo (Quadcopter)" selector entry to remember
    to pick instead."""
    overrides = self.get_target(target_key).get("launch_target_overrides", {})
    return overrides.get(robot_config, target_key)

  def _ssh_key_candidates(self):
    """Candidate SSH private key PATHS, best first -- paths only, never a key
    itself and never anything secret.

    More than one candidate because NEPI's own env vars disagree about what
    "the ssh key" means, and both meanings are real:
      - On a device, the platform exports NEPI_SSH_KEY_PATH as a full path
        while NEPI_SSH_KEY is only a FILENAME ("nepi_default_ssh_key"), which
        is why it's joined with NEPI_SSH_FOLDER below rather than used alone.
      - In every deploy_*.sh script, NEPI_SSH_KEY *is* a full path
        (NEPI_SSH_KEY=${NEPI_SSH_KEY:-~/.ssh/nepi_default_ssh_key}).
    Trying both, and picking the first candidate that actually exists, is what
    makes the launcher work unchanged in both contexts -- rather than betting
    on one convention and failing in the other (which is exactly what happened:
    a bare-filename NEPI_SSH_KEY produced "SSH key not found:
    nepi_default_ssh_key" on the first real production-instance launch).

    The config's ssh_key_path is the explicit per-deployment answer, and
    matters because apps_mgr runs app nodes as root, so the ~/.ssh default
    expands to /root/.ssh -- not where the key actually lives."""
    candidates = []

    env_key_path = os.environ.get("NEPI_SSH_KEY_PATH", "")
    if env_key_path:
      candidates.append(env_key_path)

    env_key = os.environ.get("NEPI_SSH_KEY", "")
    if env_key:
      candidates.append(env_key)
      ssh_folder = os.environ.get("NEPI_SSH_FOLDER", "")
      if ssh_folder and os.path.basename(env_key) == env_key:
        candidates.append(os.path.join(ssh_folder, env_key))

    config_key = self.config.get("ssh_key_path", "")
    if config_key:
      candidates.append(config_key)

    candidates.append(DEFAULT_SSH_KEY)
    return [os.path.expanduser(c) for c in candidates]

  def _ssh_key(self):
    candidates = self._ssh_key_candidates()
    for candidate in candidates:
      if os.path.isfile(candidate):
        return candidate
    raise LauncherError("No usable SSH key found. Tried: " + ", ".join(candidates))

  def _ssh_cmd(self, target, command):
    ssh_key = self._ssh_key()
    host = target["host"]
    user = target["ssh_user"]
    port = int(target.get("ssh_port", 22))
    # Confirmed live (2026-09-04): a device running a STALE copy of this
    # app (the /opt/nepi install path is ephemeral, resets to a baseline
    # image on restart -- see NEPI_APP_BUILD_AND_TEST_CHECKLIST.md) hit
    # this with both host and ssh_user empty, because that older
    # os_instance_registry.py's select() had no connection_mode guard yet
    # and blindly copied a shared_storage instance's own (deliberately
    # blank) host/ssh_user onto every target. The resulting destination
    # ("" + "@" + "" = "@") isn't a bad host -- it's not a valid ssh
    # argument at all, so ssh refuses to even parse it and dumps its own
    # usage/help banner (exit 255) instead of a normal connection error,
    # which is a genuinely confusing thing to hit from the RUI ("Install
    # command exited 255: usage: ssh ..."). This guard turns that into an
    # immediate, clear LauncherError regardless of WHY host/user ended up
    # blank (stale deploy, a future select() regression, anything else) --
    # every caller here already surfaces LauncherError as a normal
    # RUI-visible error message.
    if not host or not user:
      raise LauncherError(
          "Refusing to build an SSH command with an empty host ('" + str(host) +
          "') or user ('" + str(user) + "') -- this target's connection_mode is '" +
          str(target.get("connection_mode")) + "', which should never reach _ssh_cmd " +
          "at all if it's 'shared_storage'. Likely a stale app deployment (the " +
          "/opt/nepi install path resets on restart) running old code without the " +
          "connection_mode-aware select()/dispatch logic -- redeploy this app's " +
          "current nepi_api/nepi_app_sim_connector packages.")
    return [
        "ssh",
        "-i", ssh_key,
        "-p", str(port),
        "-o", "StrictHostKeyChecking=no",
        "-o", "ConnectTimeout=" + str(SSH_CONNECT_TIMEOUT_SEC),
        "-o", "ServerAliveInterval=15",
        "-o", "ServerAliveCountMax=3",
        user + "@" + host,
        command,
    ]

  def _run_remote(self, target, command, timeout_sec):
    """One-shot remote command: blocks until it exits, for probes/stops that
    are meant to return quickly. Not for launch_command -- see launch()."""
    ssh_cmd = self._ssh_cmd(target, command)
    try:
      result = subprocess.run(ssh_cmd, capture_output=True, text=True, timeout=timeout_sec)
    except subprocess.TimeoutExpired as e:
      raise LauncherError("SSH command timed out after " + str(timeout_sec) + "s: " + str(e))
    return result

  def _push_file_content(self, target, remote_path, content):
    """Writes `content` to `remote_path` on the VM over the same ssh channel
    launch/stop/ready-check already use -- no scp binary or extra credential
    needed. `remote_path` is expected to be one of this app's own fixed
    model paths (never user-supplied text), so no shell-injection concern
    from interpolating it directly into the remote command string."""
    ssh_cmd = self._ssh_cmd(target, "cat > " + remote_path)
    try:
      result = subprocess.run(ssh_cmd, input=content, capture_output=True, text=True,
                               timeout=SSH_CONNECT_TIMEOUT_SEC)
    except subprocess.TimeoutExpired as e:
      raise LauncherError("SSH push to " + remote_path + " timed out: " + str(e))
    if result.returncode != 0:
      raise LauncherError("SSH push to " + remote_path + " failed: " + (result.stderr or "unknown error"))

  # Wrapper text every launch_command/attach_launch_command in
  # simulator_launch_targets.yaml is authored with -- see
  # _stage_launch_script's own docstring for why it gets stripped back off
  # before staging. Kept as a module-level constant, not inlined into that
  # method, so a future author changing the YAML's own wrapper convention
  # only has one place to update it in step.
  _LAUNCH_SCRIPT_WRAPPER_PREFIX = "bash -lc '"
  _LAUNCH_SCRIPT_WRAPPER_SUFFIX = "'"

  def _stage_launch_script(self, target, target_key, command):
    """Writes a launch/attach command to a remote temp file and returns its
    path, so launch() can exec it via `bash -l <path>` instead of passing
    the whole script as a single inline `bash -lc '...'` SSH remote-command
    argument. Reported live (2026-09-03): the inline-string form is
    intermittently unreliable for gazebo_quadcopter's long launch_command --
    the identical script, byte-for-byte, sometimes had its SSH session exit
    within the launch startup grace period with an empty stderr and no
    remote-side log output at all (not even the first line), while running
    the exact same text as a file via `bash -l <file>` was reliable across
    many repeated manual trials, over the same (double-hop, reverse-
    tunneled) connection. Root cause not fully isolated -- suspected some
    limit or race specific to a single, very long argv element being handed
    to a remote shell as its `-c` command -- so this sidesteps the failure
    mode rather than depending on it being understood.

    Every launch_command/attach_launch_command in
    simulator_launch_targets.yaml is authored as `bash -lc '<script>'` (kept
    that way so a human can still copy either one out of
    manual_fallback_commands and run it as-is) -- the wrapper is stripped
    back off here before writing, or the staged file would itself just
    contain another `bash -lc '<script>'` invocation and reintroduce the
    exact inline-string form this is meant to avoid. Falls back to writing
    `command` unstripped if some future target's launch_command doesn't
    follow that convention -- still correct (bash -l happily runs a file
    that starts with `bash -lc '...'` as its one and only statement), just
    without the reliability fix, and _stage_launch_script has no way to
    know that ahead of time.

    Reuses _push_file_content's own proven cat> mechanism -- no scp binary
    or extra credential needed. remote_path is derived from target_key (a
    fixed identifier from this app's own config, never user-supplied
    text), so no shell-injection concern from interpolating it directly
    into the remote command string."""
    script_body = command
    if (command.startswith(self._LAUNCH_SCRIPT_WRAPPER_PREFIX) and
        command.endswith(self._LAUNCH_SCRIPT_WRAPPER_SUFFIX)):
      script_body = command[len(self._LAUNCH_SCRIPT_WRAPPER_PREFIX):-len(self._LAUNCH_SCRIPT_WRAPPER_SUFFIX)]
    remote_path = "/tmp/nepi_sim_launch_" + target_key + ".sh"
    self._push_file_content(target, remote_path, script_body)
    return remote_path

  #**********************
  # Shared-storage transport -- the connection_mode='shared_storage'
  # counterpart of _ssh_cmd/_run_remote above. See
  # os_instance_registry.py's CONNECTION_MODES and vm_command_watcher.py's
  # own module docstring for the full design: a target selected onto a
  # shared_storage instance (os_instance_registry.select()) carries
  # os_instance_id instead of host/ssh_user/ssh_port, and every method
  # below that currently branches on connection_mode calls this instead of
  # _ssh_cmd/_run_remote for that target.

  def _dispatch_shared_storage(self, target, target_key, action, script_text, timeout_sec):
    """Writes a command file to target's OS instance's mailbox and blocks
    until a matching status file appears (or timeout_sec plus
    SHARED_STORAGE_POLL_GRACE_SEC elapses). Returns the parsed status dict
    (at least 'state'/'exit_code'/'error', see vm_command_watcher.py's own
    _writeStatus) on any response, even a failed one -- callers decide what
    a failed/nonzero result means for their own action, same division of
    responsibility _run_remote/is_installed already have for the SSH path.

    Raises LauncherError only when the watcher never responds AT ALL within
    the deadline -- this is this transport's equivalent of an SSH
    connection-level failure (dead tunnel, unreachable host): "the command
    might not have run at all" is a different, worse condition than "the
    command ran and reported failure", and callers like is_installed()
    already need to tell those apart (see that method's own docstring)."""
    os_instance_id = target.get('os_instance_id', '')
    if not os_instance_id:
      raise LauncherError(
          "'" + target.get('display_name', target_key) + "' is set to the shared_storage "
          "connection mode but has no os_instance_id -- select a shared_storage OS "
          "instance for it first.")
    mailbox = os.path.join(VM_COMMANDS_STORAGE_DIR, os_instance_id)
    try:
      os.makedirs(mailbox, exist_ok=True)
    except OSError as e:
      raise LauncherError("Could not reach the shared-storage mailbox for '" +
                           target.get('display_name', target_key) + "': " + str(e))
    self._shared_storage_seq += 1
    request_id = target_key + '_' + str(int(time.time() * 1000)) + '_' + str(self._shared_storage_seq)
    cmd_path = os.path.join(mailbox, 'cmd_' + request_id + '.json')
    tmp_path = cmd_path + '.tmp'
    with open(tmp_path, 'w') as f:
      json.dump({'action': action, 'target_key': target_key, 'script': script_text,
                 'timeout_sec': timeout_sec}, f)
    os.replace(tmp_path, cmd_path)
    status_path = os.path.join(mailbox, 'status_' + request_id + '.json')
    deadline = time.time() + timeout_sec + SHARED_STORAGE_POLL_GRACE_SEC
    while time.time() < deadline:
      try:
        with open(status_path, 'r') as f:
          return request_id, json.load(f)
      except (OSError, ValueError):
        time.sleep(SHARED_STORAGE_POLL_INTERVAL_SEC)
    raise LauncherError(
        "No response from the shared-storage watcher for '" +
        target.get('display_name', target_key) + "' within " + str(timeout_sec) +
        "s -- is vm_command_watcher.py running on that machine and pointed at "
        "the same nepi_storage mount this device uses?")

  #**********************
  # deploy_state.yaml -- the REAL deploy/kill transport (see
  # DEPLOY_STATE_FILENAME's own comment for why this isn't just the
  # shared_storage fallback's request/response protocol reused: deploy/kill
  # is not request-scoped, it is "what should be running right now",
  # continuously true until told otherwise, which a single persistent
  # 0/empty-or-target_key flag expresses far more simply than a stream of
  # one-shot launch/stop requests would). See vm_command_watcher.py's own
  # module docstring for the full protocol this reads/writes.

  def _deploy_state_path(self, os_instance_id):
    return os.path.join(VM_COMMANDS_STORAGE_DIR, os_instance_id, DEPLOY_STATE_FILENAME)

  def _read_deploy_state(self, os_instance_id):
    try:
      with open(self._deploy_state_path(os_instance_id), 'r') as f:
        return yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
      return {}

  def _write_deploy_desired(self, os_instance_id, target_key, launch_command='',
                             stop_command='', ready_check_command=''):
    """Sets what SHOULD be running -- target_key (with its commands) to
    deploy it, '' to kill whatever's running. The watcher does the actual
    work asynchronously; callers that need to know the outcome poll
    _read_deploy_state's own 'status' section afterward (see launch()'s own
    polling loop)."""
    path = self._deploy_state_path(os_instance_id)
    try:
      os.makedirs(os.path.dirname(path), exist_ok=True)
    except OSError as e:
      raise LauncherError("Could not reach the shared-storage mailbox for OS instance '" +
                           os_instance_id + "': " + str(e))
    doc = self._read_deploy_state(os_instance_id)
    doc['control'] = {'desired_target': target_key, 'last_updated': time.time()}
    doc['target'] = {
        'launch_command': launch_command,
        'stop_command': stop_command,
        'ready_check_command': ready_check_command,
    }
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
      yaml.safe_dump(doc, f, default_flow_style=False, sort_keys=False)
    os.replace(tmp, path)

  def _launch_via_deploy_state(self, target, target_key, command, timeout_sec):
    """Requests target_key as the desired deploy state, then polls
    status.running_target/status.state until the watcher reports success,
    failure, or nothing at all within timeout_sec (+ its own poll grace).
    Returns (True, status_dict) once the watcher reports THIS target_key as
    either 'running' (success) or 'failed' (the caller decides what a
    failure means, same division of responsibility every other shared-
    storage dispatch here already has), or (False, None) if the watcher
    never even acknowledges the request -- distinguishing "ran and failed"
    from "might not have run at all" the same way _dispatch_shared_storage's
    own docstring already does for the request/response protocol."""
    os_instance_id = target.get('os_instance_id', '')
    if not os_instance_id:
      raise LauncherError(
          "'" + target.get('display_name', target_key) + "' is set to the shared_storage "
          "connection mode but has no os_instance_id -- select a shared_storage OS "
          "instance for it first.")
    self._write_deploy_desired(os_instance_id, target_key, launch_command=command,
                               stop_command=target.get('stop_command', ''),
                               ready_check_command=target.get('ready_check_command', ''))
    deadline = time.time() + timeout_sec + SHARED_STORAGE_POLL_GRACE_SEC
    while time.time() < deadline:
      status = self._read_deploy_state(os_instance_id).get('status', {})
      if status.get('running_target') == target_key:
        state = status.get('state')
        if state == 'failed':
          return True, status
        if state == 'running':
          return True, status
      time.sleep(SHARED_STORAGE_POLL_INTERVAL_SEC)
    return False, None

  def push_dimensions(self, target, model_name, dimensions_yaml_text, sdf_override_text):
    """Pushes one model's editable geometry to the VM ahead of a launch --
    see sim_connector_app_node.py's device-side dimensions store for the
    full design (the device is the authoritative copy, surviving a
    container restart; this VM copy is just a synced deployment target,
    re-pushed whenever it changes or the app restarts).

    sdf_override_text (raw-SDF-upload escape hatch) takes precedence and is
    written directly to model.sdf, bypassing generation entirely.
    Otherwise dimensions_yaml_text (curated fields) is written to
    dimensions.yaml and generate_model_sdf.py is invoked remotely to render
    model.sdf from it. Raises LauncherError on any failure -- callers decide
    whether that should block the launch it's ahead of (see
    sim_connector_app_node.py's pushDirtyDimensions, which treats this as
    best-effort and logs rather than aborting).

    Confirmed live (2026-09-04): this method never checked connection_mode
    at all -- unlike launch/stop/install/is_ready, which all branch on
    'shared_storage' before ever building an SSH command -- so it always
    tried SSH regardless of transport. Combined with a stale deployment (an
    older os_instance_registry.py with no connection_mode guard on its own
    select(), see _ssh_cmd's own comment) this is exactly what produced the
    "Install command exited 255: usage: ssh ..." error reported live: this
    same push, not the Install button itself, hit an SSH command built from
    an empty host/user. That specific crash is fixed at the source now (a
    fresh deploy of this file's own select() and _ssh_cmd's new empty-host/
    user guard), but this method still owed its own shared_storage branch
    regardless -- "since reverse ssh isnt an option anymore, this shouldnt
    even come up" applies here exactly as much as it does to Install."""
    remote_dir = "$HOME/nepi_engine_ws/nepi_drones/sim_container/models/" + model_name
    if target.get("connection_mode") == "shared_storage":
      self._push_dimensions_shared_storage(target, model_name, remote_dir,
                                            dimensions_yaml_text, sdf_override_text)
      return
    if sdf_override_text:
      self._push_file_content(target, remote_dir + "/model.sdf", sdf_override_text)
      return
    if not dimensions_yaml_text:
      return
    self._push_file_content(target, remote_dir + "/dimensions.yaml", dimensions_yaml_text)
    generate_cmd = ("python3 $HOME/nepi_engine_ws/nepi_drones/sim_container/scripts/"
                    "generate_model_sdf.py " + model_name)
    result = self._run_remote(target, generate_cmd, timeout_sec=SSH_CONNECT_TIMEOUT_SEC + 10)
    if result.returncode != 0:
      raise LauncherError("generate_model_sdf.py failed for " + model_name + ": " +
                          (result.stderr or result.stdout or "unknown error"))

  def _push_dimensions_shared_storage(self, target, model_name, remote_dir,
                                       dimensions_yaml_text, sdf_override_text):
    """connection_mode='shared_storage' counterpart of push_dimensions's own
    SSH path -- writes the same content and (unless it's a raw SDF
    override) runs the same generate_model_sdf.py, via the same
    _dispatch_shared_storage mailbox every launch/stop/install/is_ready
    already use for this transport, instead of an SSH `cat >` plus a
    separate remote python3 call. One dispatched script does both steps
    (write, then generate) since there's no SSH round-trip cost to avoid
    here the way there is over a real network link. See
    vm_command_watcher.py's own 'push_dimensions' action (reuses its
    existing one-shot handling, the same as 'install'/'check_installed'/
    'ready_check'). The heredoc delimiter is fixed, not randomized --
    dimensions_yaml_text is this app's own generated YAML and
    sdf_override_text is a raw-SDF upload, both already treated as trusted,
    non-arbitrary content by push_dimensions' own SSH path (see
    _push_file_content's docstring on remote_path for the same reasoning
    applied to path interpolation)."""
    heredoc_delim = "NEPI_PUSH_DIMENSIONS_EOF"
    if sdf_override_text:
      script = ("cat > " + remote_dir + "/model.sdf << '" + heredoc_delim + "'\n" +
                sdf_override_text + "\n" + heredoc_delim + "\n")
    elif dimensions_yaml_text:
      script = ("cat > " + remote_dir + "/dimensions.yaml << '" + heredoc_delim + "'\n" +
                dimensions_yaml_text + "\n" + heredoc_delim + "\n" +
                "python3 $HOME/nepi_engine_ws/nepi_drones/sim_container/scripts/"
                "generate_model_sdf.py " + model_name + "\n")
    else:
      return
    target_key = "push_dimensions_" + model_name
    _, status = self._dispatch_shared_storage(
        target, target_key, "push_dimensions", script, SSH_CONNECT_TIMEOUT_SEC + 10)
    if status.get("exit_code") != 0:
      raise LauncherError("push_dimensions failed for " + model_name + ": " +
                          (status.get("error") or "").strip())

  _SAFE_NAME_RE = re.compile(r"^[A-Za-z0-9_-]+$")

  def _require_safe_name(self, name, what):
    # Unlike push_dimensions' model_name (always one of this app's own fixed
    # model names, never user-supplied), scan_name/environment_name below
    # originate from a phone-scan upload -- real user-controlled text -- and
    # get interpolated directly into a remote shell command string. Reject
    # anything outside a safe charset rather than trying to shell-escape it.
    if not name or not self._SAFE_NAME_RE.match(name):
      raise LauncherError(
          "Unsafe " + what + " (must be non-empty, letters/digits/-/_ only): " + repr(name))

  def push_scan_directory(self, target, local_scan_dir, remote_scan_name, timeout_sec=300):
    """Copies a raw phone-scan folder (rgb.mp4, depth/, confidence/, *.csv --
    binary and ~1400 files, not the single small text file push_dimensions
    handles) onto the VM under sim_container/scan_data/raw/<remote_scan_name>/.
    Uses scp rather than _push_file_content's cat pipe -- a text=True
    subprocess pipe would corrupt binary frame data, and per-file ssh
    round-trips over ~1400 depth/confidence PNGs would be far too slow.
    Raises LauncherError on failure."""
    self._require_safe_name(remote_scan_name, "remote_scan_name")
    if not os.path.isdir(local_scan_dir):
      raise LauncherError("Local scan directory not found: " + local_scan_dir)
    remote_parent = "$HOME/nepi_engine_ws/nepi_drones/sim_container/scan_data/raw"
    remote_dir = remote_parent + "/" + remote_scan_name
    # Only the PARENT is pre-created. scp -r flattens a source directory's
    # contents into the destination only when the destination does NOT
    # already exist -- pre-creating remote_dir itself would make scp nest an
    # extra local_scan_dir-basename-named directory inside it instead
    # (confirmed empirically), so remote_dir is left for scp to create fresh.
    mkdir_result = self._run_remote(target, "mkdir -p " + remote_parent,
                                    timeout_sec=SSH_CONNECT_TIMEOUT_SEC)
    if mkdir_result.returncode != 0:
      raise LauncherError("Failed to create remote scan directory: " +
                          (mkdir_result.stderr or "unknown error"))
    ssh_key = self._ssh_key()
    host = target["host"]
    user = target["ssh_user"]
    port = int(target.get("ssh_port", 22))
    scp_cmd = [
        "scp", "-r",
        "-i", ssh_key,
        "-P", str(port),
        "-o", "StrictHostKeyChecking=no",
        "-o", "ConnectTimeout=" + str(SSH_CONNECT_TIMEOUT_SEC),
        local_scan_dir,
        user + "@" + host + ":" + remote_dir,
    ]
    try:
      result = subprocess.run(scp_cmd, capture_output=True, text=True, timeout=timeout_sec)
    except subprocess.TimeoutExpired as e:
      raise LauncherError("scp of scan directory timed out after " +
                          str(timeout_sec) + "s: " + str(e))
    if result.returncode != 0:
      raise LauncherError("scp of scan directory failed: " + (result.stderr or "unknown error"))

  def convert_scan_to_environment(self, target, remote_scan_name, environment_name,
                                  timeout_sec=1800):
    """Remotely invokes scan_to_environment.py on the VM against a scan
    already pushed there by push_scan_directory -- the same remote-invocation
    pattern push_dimensions already uses for generate_model_sdf.py, just a
    much heavier script (TSDF fusion + convex decomposition over ~1400 frames
    takes minutes, hence the generous default timeout -- see
    docs/SCAN_TO_SIM_ENVIRONMENT_PLAN.md). Output lands directly in the VM's
    own sim_container/models/<environment_name>/ -- no push-back needed,
    Gazebo only needs it VM-local. Raises LauncherError on failure."""
    self._require_safe_name(remote_scan_name, "remote_scan_name")
    self._require_safe_name(environment_name, "environment_name")
    remote_scan_dir = ("$HOME/nepi_engine_ws/nepi_drones/sim_container/scan_data/raw/" +
                       remote_scan_name)
    convert_cmd = ("python3 $HOME/nepi_engine_ws/nepi_drones/sim_container/scripts/"
                  "scan_to_environment.py " + remote_scan_dir + " " + environment_name)
    result = self._run_remote(target, convert_cmd, timeout_sec=timeout_sec)
    if result.returncode != 0:
      raise LauncherError("scan_to_environment.py failed for " + environment_name + ": " +
                          (result.stderr or result.stdout or "unknown error"))

  def _is_connection_level_failure(self, returncode, stderr):
    """True when the ssh CLIENT itself never reached the remote command --
    connection refused, timed out, host unreachable, auth rejected -- as
    opposed to the remote command running and exiting non-zero on its own.

    Matters because a dead reverse tunnel and "genuinely not installed" both
    surface as "the command failed", and a caller like is_installed() needs
    to tell them apart: confirmed the hard way when the reverse tunnel died
    across a device reboot and every target -- including gazebo_rover, which
    was never touched -- reported not_installed instead of unknown, sending
    the operator to click Install on something already on the VM. Also the
    gate _try_shared_storage_fallback's own callers use to decide whether a
    failure is even eligible for that fallback -- a real remote script error
    (e.g. gazebo_rover's own "a gzserver is already running" refuse-to-launch
    guard, exit code 3) is NOT a connection problem and must not trigger a
    fallback that would just hit the identical real error a second time
    over a different transport.

    OpenSSH reserves exit code 255 for its own client-side errors and never
    uses it to relay a remote command's exit status (a real remote script
    exiting 255 -- reserved by POSIX shells for "exit status out of range" --
    would be a strange coincidence, so the stderr text is checked too rather
    than trusting the exit code alone).

    Takes returncode/stderr directly rather than a subprocess.run result
    object -- launch()'s own connection-level check is against a Popen it
    already polled and communicate()'d, not a fresh result object, so this
    needs to work from either call shape."""
    if returncode != 255:
      return False
    stderr = (stderr or "").lower()
    connection_phrases = (
        "connection refused", "connection timed out", "operation timed out",
        "no route to host", "could not resolve hostname",
        "network is unreachable", "permission denied (publickey",
        "connection closed by remote host", "connection reset by peer",
    )
    return any(phrase in stderr for phrase in connection_phrases)

  def _shared_storage_watcher_alive(self, os_instance_id):
    """Cheap, local-file-only liveness check for a shared_storage fallback
    watcher -- just a stat+read of one small JSON file, no network stall,
    so this is safe to call on every single connection-level SSH failure
    without adding meaningful latency (unlike re-probing SSH itself, or
    dispatching a real command and waiting on it). Mirrors
    os_instance_registry.py's own _verify_shared_storage (duplicated, not
    imported -- see this file's own module docstring for why these two
    modules stay independent). Returns False (never raises) for an empty/
    missing os_instance_id, a missing heartbeat file, a malformed one, or
    one older than WATCHER_HEARTBEAT_STALE_AFTER_SEC."""
    if not os_instance_id:
      return False
    heartbeat_path = os.path.join(VM_COMMANDS_STORAGE_DIR, os_instance_id,
                                   'watcher_heartbeat.json')
    try:
      with open(heartbeat_path, 'r') as f:
        heartbeat = json.load(f)
    except (OSError, ValueError):
      return False
    alive_at = heartbeat.get('alive_at')
    if not isinstance(alive_at, (int, float)):
      return False
    return (time.time() - alive_at) <= WATCHER_HEARTBEAT_STALE_AFTER_SEC

  def _try_shared_storage_fallback(self, target, target_key, action, script_text, timeout_sec):
    """Automatic, no-operator-action fallback for an ssh-mode target whose
    reverse SSH tunnel just failed at the connection level: requested live
    (2026-09-04) after a reverse-SSH-into-an-operator's-own-machine
    connectivity failure blocked a launch entirely -- "if that happens, fall
    back to the nepi storage (shared file) architecture... so we don't have
    to worry about reverse ssh. make that ready so its fool proof for any
    type of machine."

    If THIS SAME instance (target['os_instance_id'], now set unconditionally
    by os_instance_registry.py's select() regardless of connection_mode --
    see that method's own comment) also has a live vm_command_watcher.py
    running there (checked via _shared_storage_watcher_alive, a cheap local
    read, not another slow network probe), retries the exact same action
    over the shared-storage transport instead of failing the whole call.
    This is the ONLY thing that makes "just start vm_command_watcher.py on
    the same machine, pointed at the same instance_id, as insurance" work
    end to end -- the transport itself (_dispatch_shared_storage) already
    existed for an operator-initiated, separately-registered
    connection_mode='shared_storage' instance; this is what makes it kick in
    automatically for an EXISTING ssh-mode instance's own failures, with no
    separate registration, no mode switch, and no operator action beyond
    having started that one watcher process ahead of time on any machine
    that can mount the same nepi_storage share (which is the whole point of
    that share already being universally reachable -- Windows, Linux, WSL,
    whatever -- unlike a reverse SSH tunnel's own per-OS setup).

    Returns (True, status_dict) on any response from the watcher (even a
    failed one -- caller decides what that means for its own action, same
    division of responsibility _dispatch_shared_storage's own docstring
    already establishes), or (False, None) when there is no fallback to try
    (no os_instance_id recorded on this target, its watcher isn't alive, or
    the watcher never responds at all within the deadline) -- the caller's
    existing SSH-failure error/message path is unchanged in that case, so a
    deployment with no fallback watcher configured behaves exactly as
    before this feature existed.

    On success, flips target['connection_mode'] to 'shared_storage' in
    place so every LATER call for this same target_key (is_ready polling a
    launch that just fell back, a subsequent stop()) also goes straight to
    the shared-storage transport instead of paying another SSH connection
    timeout first -- the same in-place mutation os_instance_registry.py's
    own select() already does for an operator's explicit mode choice, just
    triggered by a failure instead of a click. Left in place for the rest of
    this launcher's lifetime (or until reload_if_changed()/a fresh select()
    resets it) rather than reverted once SSH recovers -- staying on a
    transport that's confirmed working is the point ("so we don't have to
    worry about reverse ssh"), not a reason to keep re-testing the one that
    just failed.

    A "launch" action goes through _launch_via_deploy_state instead of
    _dispatch_shared_storage -- that's the real, primary deploy transport
    now (see DEPLOY_STATE_FILENAME's own comment), not just this fallback's
    own concern, so a launch that falls back here behaves identically to
    one that started in shared_storage mode from the very beginning. Every
    other action (stop/install/check_installed/ready_check) stays on the
    original one-shot request/response protocol -- a "stop" here in
    particular is recovering an SSH-launched process the watcher never
    tracked as its own, so running stop_command directly (rather than
    flipping a desired_target the watcher never set) is the correct
    behavior for this specific scenario, not an oversight."""
    os_instance_id = target.get('os_instance_id', '')
    if not self._shared_storage_watcher_alive(os_instance_id):
      return False, None
    if action == 'launch':
      acked, status = self._launch_via_deploy_state(target, target_key, script_text, timeout_sec)
      if not acked:
        return False, None
      target['connection_mode'] = 'shared_storage'
      return True, status
    try:
      _, status = self._dispatch_shared_storage(target, target_key, action, script_text, timeout_sec)
    except LauncherError:
      return False, None
    target['connection_mode'] = 'shared_storage'
    return True, status

  def _classify_connection_failure(self, target, stderr):
    """Returns a clearer message for a connection-level SSH failure against
    a loopback-host target (see REVERSE_TUNNEL_FALLBACK_COMMANDS above), or
    None to leave the caller's generic message alone. "Permission denied"
    gets its own message: the tunnel itself is evidently up (something
    answered and rejected the key), so pointing at the tunnel commands
    would send the operator to fix the wrong thing -- Step 1 (SSH keys) in
    the setup doc is the real fix there. A non-loopback host's failure is a
    real network/host problem this has no special insight into."""
    host = target.get("host", "")
    if host not in ("127.0.0.1", "localhost", "::1"):
      return None
    stderr_lower = (stderr or "").lower()
    if "permission denied (publickey" in stderr_lower:
      return ("Reached your sim VM, but this device's SSH key isn't authorized there (or the "
              "VM's key isn't authorized on this device) -- see Step 1 (SSH keys) in "
              "nepi_drones/docs/SIM_VM_CONNECTION_SETUP.md.")
    connection_phrases = ("connection refused", "connection timed out", "operation timed out",
                         "no route to host", "connection closed by remote host",
                         "connection reset by peer")
    if any(phrase in stderr_lower for phrase in connection_phrases):
      return ("Could not reach your sim VM -- the reverse SSH tunnel between this device and "
              "the VM does not appear to be running. See the fallback commands to start it.")
    return None

  def launch(self, target_key, attach=False):
    """Starts the target's launch_command over SSH and leaves the
    connection open, held by a tracked Popen, for as long as the simulator
    should keep running -- launch_command ends in `wait <pids>`, so the
    remote shell (and hence the ssh session) only exits once those
    processes do. A one-shot subprocess.run() here would return either too
    early (killing the connection, and with it the remote session, before
    the sim outlives it) or block forever; call wait_until_ready separately
    to know when the simulator is actually usable. Raises LauncherError only
    if the connection fails or exits within the startup grace period (bad
    host, auth rejected, command error) -- once past that window the
    process is assumed to be legitimately holding the sim open.

    attach=True uses attach_launch_command instead -- an explicit operator
    choice to reuse whatever's already running rather than starting fresh
    (offered specifically after launch_command's own "a gzserver is
    already running" refuse-to-launch guard fires; see
    sim_connector_app_node.py's is_gazebo_conflict_error). Raises
    LauncherError if this target has no attach_launch_command configured --
    only gazebo_rover/gazebo_quadcopter have the gzserver-conflict guard
    that makes "attach instead" a meaningful choice at all."""
    target = self.get_target(target_key)
    launch_command_key = "attach_launch_command" if attach else "launch_command"
    launch_command = target.get(launch_command_key, "")
    if not launch_command:
      if attach:
        raise LauncherError(
            "'" + target.get("display_name", target_key) + "' has no attach_launch_command "
            "configured -- it can only be launched fresh, not attached to an existing instance.")
      raise LauncherError(
          "'" + target.get("display_name", target_key) + "' has no launch_command configured yet "
          "-- it can be checked/installed but not deployed.")
    device_host = self.config.get("device_bridge_host", "")
    device_port = self.config.get("device_bridge_port", "")
    command = launch_command.format(
        device_bridge_host=device_host, device_bridge_port=device_port)
    if target.get("connection_mode") == "shared_storage":
      # No local process to hold open here -- the watcher on the OTHER end
      # owns the actual launched process; this side only needs to know it
      # started (or didn't) before returning, exactly like the SSH path's
      # own startup-grace-period check just below, just over a different
      # transport (deploy_state.yaml, see _launch_via_deploy_state's own
      # docstring -- this is the primary, always-on deploy path for a real
      # deployment, not a fallback).
      acked, status = self._launch_via_deploy_state(
          target, target_key, command, LAUNCH_STARTUP_GRACE_SEC)
      if not acked:
        raise LauncherError(
            "No response from the shared-storage watcher for '" +
            target.get("display_name", target_key) + "' -- is vm_command_watcher.py "
            "running on that machine and pointed at the same nepi_storage mount "
            "this device uses?")
      if status.get("state") == "failed":
        raise LauncherError(
            "Launch failed via shared storage: " + status.get("last_error", ""))
      return
    remote_script_path = self._stage_launch_script(target, target_key, command)
    ssh_cmd = self._ssh_cmd(target, "bash -l " + remote_script_path)
    proc = subprocess.Popen(ssh_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    time.sleep(LAUNCH_STARTUP_GRACE_SEC)
    if proc.poll() is not None:
      _, stderr = proc.communicate()
      if self._is_connection_level_failure(proc.returncode, stderr):
        fell_back, status = self._try_shared_storage_fallback(
            target, target_key, "launch", command, LAUNCH_STARTUP_GRACE_SEC)
        if fell_back:
          if status.get("state") == "failed":
            raise LauncherError(
                "Launch failed via shared-storage fallback: " + status.get("last_error", ""))
          return
      tunnel_message = self._classify_connection_failure(target, stderr)
      if tunnel_message:
        raise LauncherError(tunnel_message, manual_fallback_commands=REVERSE_TUNNEL_FALLBACK_COMMANDS)
      raise LauncherError(
          "Launch ssh session exited " + str(proc.returncode) + " within "
          + str(LAUNCH_STARTUP_GRACE_SEC) + "s: " + stderr.strip())
    self._launch_procs[target_key] = proc

  def is_ready(self, target_key):
    """One-shot readiness check via ready_check_command -- exit code 0 means
    ready. Returns False (not raises) on any connection failure (SSH or
    shared-storage), since "can't tell yet" and "not ready yet" should look
    the same to a caller polling this."""
    target = self.get_target(target_key)
    ready_check_command = target.get("ready_check_command")
    if not ready_check_command:
      return True
    if target.get("connection_mode") == "shared_storage":
      # Reads status.ready straight out of deploy_state.yaml rather than
      # dispatching a fresh one-shot ready_check request -- the watcher
      # already owns periodically re-running ready_check_command for
      # whatever IT currently has running (see vm_command_watcher.py's own
      # _maybeCheckDeployReady), since it's the one that knows what's
      # actually running; this is just reading that same answer back.
      os_instance_id = target.get('os_instance_id', '')
      status = self._read_deploy_state(os_instance_id).get('status', {})
      if status.get('running_target') != target_key:
        return False
      return bool(status.get('ready', False))
    try:
      result = self._run_remote(target, ready_check_command, timeout_sec=SSH_CONNECT_TIMEOUT_SEC + 2)
    except LauncherError:
      return False
    if self._is_connection_level_failure(result.returncode, result.stderr):
      fell_back, status = self._try_shared_storage_fallback(
          target, target_key, "ready_check", ready_check_command, SSH_CONNECT_TIMEOUT_SEC + 2)
      if fell_back:
        return status.get("exit_code") == 0
      return False
    return result.returncode == 0

  def wait_until_ready(self, target_key, attempts=READY_CHECK_ATTEMPTS,
                        interval_sec=READY_CHECK_INTERVAL_SEC):
    """Polls is_ready up to `attempts` times. Returns True/False; never
    raises -- a timeout is a normal, expected outcome for a slow launch, not
    an error condition."""
    for _ in range(attempts):
      if self.is_ready(target_key):
        return True
      time.sleep(interval_sec)
    return False

  def stop(self, target_key):
    """Runs stop_command (pkills the sim's remote processes), which makes
    the still-open launch() connection's `wait <pids>` return on its own --
    reaps that connection afterward so it doesn't linger as a zombie.

    For a shared_storage target there is no local connection to reap (the
    watcher on the other end owns the actual launched process, not this
    launcher). Sets desired_target to '' in deploy_state.yaml -- the
    watcher runs THIS target's own remembered stop_command itself (see
    vm_command_watcher.py's own _stopDeployTarget) and is the one place
    that actually knows what's running, so there is nothing further for
    this method to do; best-effort and fire-and-forget, matching this
    method's own SSH-path semantics below (never raises on the remote
    outcome)."""
    target = self.get_target(target_key)
    stop_command = target.get("stop_command")
    if target.get("connection_mode") == "shared_storage":
      os_instance_id = target.get('os_instance_id', '')
      if os_instance_id:
        self._write_deploy_desired(os_instance_id, '')
      return
    if stop_command:
      result = self._run_remote(target, stop_command, timeout_sec=SSH_CONNECT_TIMEOUT_SEC + 5)
      if self._is_connection_level_failure(result.returncode, result.stderr):
        # Best-effort as before if there's no fallback to try -- stop() has
        # never raised on the remote command's own outcome (see this
        # method's own division of labor: worst case, a leftover process
        # gets caught by the next launch's own "already running" guard),
        # so a connection failure with nothing to fall back to still just
        # falls through to reaping the local ssh Popen below, unchanged.
        self._try_shared_storage_fallback(target, target_key, "stop", stop_command,
                                           SSH_CONNECT_TIMEOUT_SEC + 5)
    proc = self._launch_procs.pop(target_key, None)
    if proc is not None:
      try:
        proc.wait(timeout=SSH_CONNECT_TIMEOUT_SEC + 5)
      except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()

  def kill_all_gazebo(self):
    """Explicit, deliberately blunt escape hatch: kills EVERY gzclient and
    gzserver on each configured target's host, regardless of who started
    it or which target's own pgid it belongs to -- offered as its own
    button precisely for the case stop()'s pgid-scoping (by design) can
    never reach: a gzserver this app never launched (left over from manual
    VM use, a crashed prior session, or the separate RBX ArduPilot SITL dev
    flow) blocking every future launch attempt with the "already running"
    refuse-to-launch guard. NOT used by the ordinary stop() path above --
    an earlier version of gazebo_rover's own stop_command did exactly this
    unconditionally and it was deliberately removed for tearing down
    unrelated Gazebo sessions as a routine side effect (see that target's
    own stop_command comment); this exists ONLY as a manual, explicitly
    operator-initiated action, never automatic.

    Runs once per unique (host, ssh_user, ssh_port) across every configured
    target rather than just the one the operator happened to have selected
    -- the whole point is clearing whatever is actually in the way, which
    might not be tracked as any target's own launch. Collects every
    per-host LauncherError rather than raising on the first, so one
    unreachable host doesn't stop the kill from reaching the others.
    Silently skips a target with no host/ssh_user configured (webots_rover
    etc. as `{}` empty placeholders have neither)."""
    seen_hosts = set()
    errors = []
    for target_key, target in self.config["launch_targets"].items():
      if not target or "host" not in target or "ssh_user" not in target:
        continue
      host_key = (target["host"], target["ssh_user"], target.get("ssh_port", 22))
      if host_key in seen_hosts:
        continue
      seen_hosts.add(host_key)
      try:
        # Beyond gzclient/gzserver themselves: every helper/bridge process
        # either target's own stop_command knows how to reach ONLY when it
        # was launched (and pgid-tracked) by THIS app -- exactly the case
        # this escape hatch exists for is when it wasn't (a leftover
        # standalone `sim_vehicle.py`/manual VM session, etc). Found live
        # (2026-08-19): killing gazebo out from under a still-running
        # ArduCopter SITL left MAVProxy/ArduCopter orphaned exactly like the
        # already-fixed "stop doesn't kill SITL" bug, just reached via this
        # button instead. mavproxy.py needs -9 specifically -- see
        # gazebo_quadcopter's own stop_command comment on why plain SIGTERM
        # doesn't work on it (--daemon forks it out of the launching
        # process's session, and it resists SIGTERM even directly).
        self._run_remote(target,
            "pkill -x gzclient 2>/dev/null; pkill -x gzserver 2>/dev/null; "
            "pkill -f \"[s]im_bridge_node.py\" 2>/dev/null; "
            "pkill -f \"[s]im_heartbeat_listener.py\" 2>/dev/null; "
            "pkill -f \"[c]amera_rig_controller.py\" 2>/dev/null; "
            "pkill -f \"[s]im_vehicle.py -v ArduCopter\" 2>/dev/null; "
            "pkill -9 -f \"[m]avproxy.py\" 2>/dev/null; "
            # The actual ArduCopter SITL binary (a grandchild of sim_vehicle.py,
            # run inside its own xterm) -- not reached by any pattern above.
            # stop_command's own pgid kill happens to catch this one too (it's
            # a descendant of the tracked launch group), but kill_all_gazebo
            # has no pgid to fall back on -- it exists precisely for sessions
            # this app never launched. Missing this left the binary itself as
            # a live orphan, confirmed live (2026-08-19): every process this
            # command DOES match died, but arducopter kept running and holding
            # port 5760, blocking the next launch's own "already running"
            # guard exactly like the bug this method was written to fix.
            "pkill -f \"[a]rducopter -S\" 2>/dev/null; "
            "pkill -f \"[c]amera_rig_controller_ardupilot.py\" 2>/dev/null; "
            "pkill -f \"[s]im_connector_bridge_gazebo_quadcopter.py\" 2>/dev/null; "
            "pkill -f \"[g]z_reset_listener.py\" 2>/dev/null; true",
            timeout_sec=SSH_CONNECT_TIMEOUT_SEC + 5)
      except LauncherError as e:
        errors.append(str(e))
    if errors:
      raise LauncherError("; ".join(errors))

  #**********************
  # Per-target dependency check/install. Independent of launch/stop above --
  # a target's dependencies can be checked (and installed) whether or not
  # anything is currently running, and checking is meant to run for every
  # target on a schedule, not just the selected one.

  def is_installed(self, target_key):
    """One-shot dependency check via check_installed_command -- exit code 0
    means present. A target with no check_installed_command configured is
    reported installed unconditionally (nothing to gate on), matching
    is_ready's own "no command configured means don't block" convention.

    Raises LauncherError when ssh itself never reached the host (dead
    tunnel, host unreachable) -- unlike is_ready, where "can't tell yet" and
    "not ready yet" look the same to a polling caller, here the caller
    (checkInstalledAllCb) needs to tell "confirmed missing" apart from
    "couldn't reach the host to check", or a connectivity blip reports a
    simulator as needing install when it's genuinely already on the VM.
    Confirmed the hard way: a dead reverse tunnel made every target,
    including one never touched, report not_installed instead of unknown."""
    target = self.get_target(target_key)
    check_command = target.get("check_installed_command")
    if not check_command:
      return True
    if target.get("connection_mode") == "shared_storage":
      # _dispatch_shared_storage already raises when the watcher never
      # responds at all -- this transport's equivalent of the
      # connection-level failure _is_connection_level_failure detects for
      # the SSH path below, so no extra handling is needed here for that
      # case.
      _, status = self._dispatch_shared_storage(
          target, target_key, "check_installed", check_command,
          SSH_CONNECT_TIMEOUT_SEC + 5)
      return status.get("exit_code") == 0
    result = self._run_remote(target, check_command, timeout_sec=SSH_CONNECT_TIMEOUT_SEC + 5)
    if self._is_connection_level_failure(result.returncode, result.stderr):
      fell_back, status = self._try_shared_storage_fallback(
          target, target_key, "check_installed", check_command, SSH_CONNECT_TIMEOUT_SEC + 5)
      if fell_back:
        return status.get("exit_code") == 0
      tunnel_message = self._classify_connection_failure(target, result.stderr)
      if tunnel_message:
        raise LauncherError(tunnel_message, manual_fallback_commands=REVERSE_TUNNEL_FALLBACK_COMMANDS)
      raise LauncherError(
          "Could not reach '" + target.get("display_name", target_key) + "' to check: "
          + result.stderr.strip())
    return result.returncode == 0

  def install(self, target_key):
    """Runs install_command and blocks until it finishes -- unlike launch(),
    an install has a natural end (the package manager exits) rather than a
    simulator process meant to keep running, so a plain blocking
    subprocess.run is correct here and there is nothing to hold open."""
    target = self.get_target(target_key)
    install_command = target.get("install_command", "")
    if not install_command:
      raise LauncherError(
          "'" + target.get("display_name", target_key) + "' has no install_command configured yet.")
    if target.get("connection_mode") == "shared_storage":
      # _dispatch_shared_storage already raises if the watcher never
      # responds at all within INSTALL_TIMEOUT_SEC -- the sudo-password
      # check below is the one piece of the SSH path's own error handling
      # that's still relevant here (it inspects the REMOTE command's own
      # stderr, not anything SSH-specific).
      _, status = self._dispatch_shared_storage(
          target, target_key, "install", install_command, INSTALL_TIMEOUT_SEC)
      if status.get("exit_code") != 0:
        error_lower = (status.get("error") or "").lower()
        if ("a terminal is required to read the password" in error_lower
            or "sudo: no tty present" in error_lower):
          raise LauncherError(
              "Install failed: sudo needs a password, but Install runs non-interactively "
              "with no way to prompt for one.",
              manual_fallback_commands=SUDO_NOPASSWD_FALLBACK_COMMANDS)
        raise LauncherError(
            "Install command exited " + str(status.get("exit_code")) + ": " +
            (status.get("error") or "").strip())
      return
    result = self._run_remote(target, install_command, timeout_sec=INSTALL_TIMEOUT_SEC)
    if self._is_connection_level_failure(result.returncode, result.stderr):
      fell_back, status = self._try_shared_storage_fallback(
          target, target_key, "install", install_command, INSTALL_TIMEOUT_SEC)
      if fell_back:
        if status.get("exit_code") != 0:
          error_lower = (status.get("error") or "").lower()
          if ("a terminal is required to read the password" in error_lower
              or "sudo: no tty present" in error_lower):
            raise LauncherError(
                "Install failed: sudo needs a password, but Install runs non-interactively "
                "with no way to prompt for one.",
                manual_fallback_commands=SUDO_NOPASSWD_FALLBACK_COMMANDS)
          raise LauncherError(
              "Install command exited " + str(status.get("exit_code")) + ": " +
              (status.get("error") or "").strip())
        return
      tunnel_message = self._classify_connection_failure(target, result.stderr)
      if tunnel_message:
        raise LauncherError(tunnel_message, manual_fallback_commands=REVERSE_TUNNEL_FALLBACK_COMMANDS)
    if result.returncode != 0:
      stderr_lower = (result.stderr or "").lower()
      # Reported live: this exact failure on a genuinely fresh VM, for
      # EVERY target that needs apt-get -- not specific to which simulator
      # was being installed. See SUDO_NOPASSWD_FALLBACK_COMMANDS's own
      # comment for why this can't just be worked around silently.
      if ("a terminal is required to read the password" in stderr_lower
          or "sudo: no tty present" in stderr_lower):
        raise LauncherError(
            "Install failed: sudo needs a password, but Install runs over a "
            "non-interactive SSH connection with no way to prompt for one.",
            manual_fallback_commands=SUDO_NOPASSWD_FALLBACK_COMMANDS)
      raise LauncherError(
          "Install command exited " + str(result.returncode) + ": " + result.stderr.strip())

  def get_manual_fallback_commands(self, target_key):
    """Copy-paste terminal commands for a human to run as a worst-case
    fallback, when auto-install either failed or (like gazebo_quadcopter)
    isn't offered at all. Prefers a target's own explicit
    manual_fallback_commands (a plain multi-line string, one command per
    line -- readable and directly copy-pastable, unlike install_command's
    single semicolon-joined `bash -lc` form meant for programmatic exec) if
    one is configured; otherwise falls back to install_command's own text
    verbatim, since for most targets that alone is already a valid thing to
    paste into a terminal. Returns '' if neither exists -- callers treat that
    as "no fallback to offer", not an error."""
    target = self.get_target(target_key)
    fallback = target.get("manual_fallback_commands", "")
    if fallback:
      return fallback
    return target.get("install_command", "")


if __name__ == "__main__":
  # Standalone smoke test -- no ROS, no app node. Usage:
  #   python3 simulator_launcher.py <config_path> <target_key> [stop]
  import sys
  if len(sys.argv) < 3:
    sys.exit("usage: simulator_launcher.py <config_path> <target_key> [stop]")
  config_path = sys.argv[1]
  target_key = sys.argv[2]
  do_stop = len(sys.argv) > 3 and sys.argv[3] == "stop"

  launcher = SimulatorLauncher(config_path)
  keys, names = launcher.get_available_targets()
  print("Available targets:", list(zip(keys, names)))

  if do_stop:
    print("Stopping " + target_key + "...")
    launcher.stop(target_key)
    print("Stop command sent.")
  else:
    print("Launching " + target_key + "...")
    launcher.launch(target_key)
    print("Launch command sent. Waiting for readiness...")
    ready = launcher.wait_until_ready(target_key)
    print("Ready:", ready)
