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
INSTALL_TIMEOUT_SEC = 600


class LauncherError(Exception):
  pass


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
    best-effort and logs rather than aborting)."""
    remote_dir = "$HOME/nepi_engine_ws/nepi_drones/sim_container/models/" + model_name
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

  def _is_connection_level_failure(self, result):
    """True when the ssh CLIENT itself never reached the remote command --
    connection refused, timed out, host unreachable, auth rejected -- as
    opposed to the remote command running and exiting non-zero on its own.

    Matters because a dead reverse tunnel and "genuinely not installed" both
    surface as "the command failed", and a caller like is_installed() needs
    to tell them apart: confirmed the hard way when the reverse tunnel died
    across a device reboot and every target -- including gazebo_rover, which
    was never touched -- reported not_installed instead of unknown, sending
    the operator to click Install on something already on the VM.

    OpenSSH reserves exit code 255 for its own client-side errors and never
    uses it to relay a remote command's exit status (a real remote script
    exiting 255 -- reserved by POSIX shells for "exit status out of range" --
    would be a strange coincidence, so the stderr text is checked too rather
    than trusting the exit code alone)."""
    if result.returncode != 255:
      return False
    stderr = (result.stderr or "").lower()
    connection_phrases = (
        "connection refused", "connection timed out", "operation timed out",
        "no route to host", "could not resolve hostname",
        "network is unreachable", "permission denied (publickey",
        "connection closed by remote host", "connection reset by peer",
    )
    return any(phrase in stderr for phrase in connection_phrases)

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
    ssh_cmd = self._ssh_cmd(target, command)
    proc = subprocess.Popen(ssh_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    time.sleep(LAUNCH_STARTUP_GRACE_SEC)
    if proc.poll() is not None:
      _, stderr = proc.communicate()
      raise LauncherError(
          "Launch ssh session exited " + str(proc.returncode) + " within "
          + str(LAUNCH_STARTUP_GRACE_SEC) + "s: " + stderr.strip())
    self._launch_procs[target_key] = proc

  def is_ready(self, target_key):
    """One-shot readiness check via ready_check_command -- exit code 0 means
    ready. Returns False (not raises) on any SSH failure, since "can't tell
    yet" and "not ready yet" should look the same to a caller polling this."""
    target = self.get_target(target_key)
    ready_check_command = target.get("ready_check_command")
    if not ready_check_command:
      return True
    try:
      result = self._run_remote(target, ready_check_command, timeout_sec=SSH_CONNECT_TIMEOUT_SEC + 2)
    except LauncherError:
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
    reaps that connection afterward so it doesn't linger as a zombie."""
    target = self.get_target(target_key)
    stop_command = target.get("stop_command")
    if stop_command:
      self._run_remote(target, stop_command, timeout_sec=SSH_CONNECT_TIMEOUT_SEC + 5)
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
    result = self._run_remote(target, check_command, timeout_sec=SSH_CONNECT_TIMEOUT_SEC + 5)
    if self._is_connection_level_failure(result):
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
    result = self._run_remote(target, install_command, timeout_sec=INSTALL_TIMEOUT_SEC)
    if result.returncode != 0:
      raise LauncherError(
          "Install command exited " + str(result.returncode) + ": " + result.stderr.strip())


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
