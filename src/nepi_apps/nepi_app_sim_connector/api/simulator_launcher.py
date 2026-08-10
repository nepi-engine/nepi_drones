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
    """Returns (keys, display_names) for every non-empty entry in
    launch_targets -- placeholder entries ({}) are not offered, matching
    how an unconfigured target has nothing this class could actually run."""
    keys = []
    names = []
    for key, entry in self.config["launch_targets"].items():
      if entry:
        keys.append(key)
        names.append(entry.get("display_name", key))
    return keys, names

  def get_target(self, target_key):
    targets = self.config["launch_targets"]
    if target_key not in targets or not targets[target_key]:
      raise LauncherError("Unknown or unconfigured launch target: " + str(target_key))
    return targets[target_key]

  def get_default_robot_config(self, target_key):
    return self.get_target(target_key).get("default_robot_config", "")

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

  def launch(self, target_key):
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
    process is assumed to be legitimately holding the sim open."""
    target = self.get_target(target_key)
    launch_command = target.get("launch_command", "")
    if not launch_command:
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
    Raises LauncherError on an SSH-level failure -- unlike is_ready, where
    "can't tell yet" and "not ready yet" look the same to a polling caller,
    here the caller (checkInstalledAllCb) needs to tell "confirmed missing"
    apart from "couldn't reach the host to check" so it doesn't report a
    simulator as needing install when the real problem is connectivity."""
    target = self.get_target(target_key)
    check_command = target.get("check_installed_command")
    if not check_command:
      return True
    result = self._run_remote(target, check_command, timeout_sec=SSH_CONNECT_TIMEOUT_SEC + 5)
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
