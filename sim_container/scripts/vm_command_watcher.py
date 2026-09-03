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

# Runs on a laptop/VM registered as a "shared_storage" Sim Connector OS
# instance (see os_instance_registry.py's connection_mode -- additive
# alongside the existing SSH/reverse-tunnel instance type, not a replacement
# for it). Requested live (2026-09-04): reverse SSH from the device into an
# operator's own machine is a real security concern (a trusted key that can
# run ARBITRARY remote commands); nepi_storage is already a shared SMB drive
# both sides can read/write, so use that as the transport instead -- no
# listening port anywhere, no SSH key trusted for arbitrary command
# execution. The trust boundary becomes "whatever this script is willing to
# do", which is exactly: run the same launch_command/stop_command/
# install_command/check_installed_command text simulator_launch_targets.yaml
# already authors for the SSH path, just locally instead of over SSH --
# nothing this script does is a capability the operator's own machine didn't
# already have.
#
# Protocol (files under <storage_root>/databases/nepi_app_sim_connector/
# vm_commands/<instance_id>/, i.e. the SAME nepi_storage tree
# os_instance_registry.py's own OS_INSTANCES_STORAGE_DIR already lives
# under -- see that module's docstring for why instance data belongs in the
# read/write database tree, not the checked-in config tree):
#   - Device writes  cmd_<request_id>.json    {action, target_key, script,
#                                               timeout_sec}
#   - This watcher picks up any cmd_*.json it finds, deletes it (so it's
#     processed exactly once), and writes/updates
#                     status_<request_id>.json {target_key, action, state,
#                                                pid, exit_code, error,
#                                                updated_at}
#   - This watcher also writes watcher_heartbeat.json every
#     HEARTBEAT_INTERVAL_SEC, purely so the device side can tell "a watcher
#     is actually alive and polling this mailbox" apart from "nothing has
#     ever run here" -- the shared_storage equivalent of an SSH probe.
#
# "script" is the EXACT text simulator_launch_targets.yaml already authors
# for launch_command/stop_command/install_command/check_installed_command
# (a `bash -lc '<script>'` string, after device_bridge_host/port
# substitution) -- this watcher strips that same wrapper and stages the
# body to a local temp file before running it, exactly like
# simulator_launcher.py's own _stage_launch_script does for the SSH path
# (see that method's docstring for why: a very long inline `bash -c` string
# handed to a fresh process is intermittently unreliable, a file is not).
# Nothing about the authored commands themselves needs to differ between
# the SSH and shared_storage transports.
#
# Deliberately zero nepi_sdk/rospy dependency -- this runs on an arbitrary
# operator machine that may not have ROS installed at all, only Python 3.
#
# Single-mailbox-at-a-time by design, not per-request-concurrent: this
# app's own existing convention is that only one simulator ever runs at a
# time (see SIM_OS_INSTANCES_PLAN.md's "explicitly not doing" section), so
# a launch/stop for one target_key and an install-check for another
# happening at the literal same instant is not a real scenario this needs
# to optimize for -- request-scoped files still keep concurrent CHECK
# requests (e.g. startInstalledCheckAll() checking several targets) from
# clobbering each other's results, which is the actual concurrency this
# protocol needs to survive.

import argparse
import glob
import json
import os
import subprocess
import time

POLL_INTERVAL_SEC = 1.0
HEARTBEAT_INTERVAL_SEC = 5.0
LAUNCH_STARTUP_GRACE_SEC = 5

# Mirrors simulator_launcher.py's own _LAUNCH_SCRIPT_WRAPPER_PREFIX/SUFFIX --
# duplicated, not imported, so this script has zero dependency on the app's
# own Python package (it may run on a machine with no NEPI code installed
# at all beyond this one file).
_SCRIPT_WRAPPER_PREFIX = "bash -lc '"
_SCRIPT_WRAPPER_SUFFIX = "'"


def _strip_wrapper(script_text):
  if (script_text.startswith(_SCRIPT_WRAPPER_PREFIX) and
      script_text.endswith(_SCRIPT_WRAPPER_SUFFIX)):
    return script_text[len(_SCRIPT_WRAPPER_PREFIX):-len(_SCRIPT_WRAPPER_SUFFIX)]
  return script_text


def _write_json_atomic(path, data):
  # Atomic replace so the device side never reads a half-written file --
  # same reasoning as config_mgr's own YAML writes elsewhere in this
  # platform, just for JSON here.
  tmp = path + '.tmp'
  with open(tmp, 'w') as f:
    json.dump(data, f)
  os.replace(tmp, path)


class Watcher(object):
  """One instance per running vm_command_watcher.py process, one process
  per registered shared_storage OS instance."""

  def __init__(self, storage_root, instance_id):
    self.mailbox = os.path.join(storage_root, 'databases',
                                 'nepi_app_sim_connector', 'vm_commands',
                                 instance_id)
    os.makedirs(self.mailbox, exist_ok=True)
    self.work_dir = os.path.join(self.mailbox, 'work')
    os.makedirs(self.work_dir, exist_ok=True)
    # target_key -> {'proc': Popen, 'request_id': str} -- tracks the ONE
    # currently-launched process per target, so a repeat "is it still
    # running" poll (a fresh check_installed/launch-status request) can be
    # answered without re-running anything, mirroring simulator_launcher.py's
    # own self._launch_procs.
    self.launch_procs = {}
    self.processed_request_ids = set()

  def _status_path(self, request_id):
    return os.path.join(self.mailbox, 'status_' + request_id + '.json')

  def _heartbeat_path(self):
    return os.path.join(self.mailbox, 'watcher_heartbeat.json')

  def run(self):
    print('vm_command_watcher: watching ' + self.mailbox)
    last_heartbeat = 0.0
    while True:
      now = time.time()
      if now - last_heartbeat > HEARTBEAT_INTERVAL_SEC:
        _write_json_atomic(self._heartbeat_path(),
                            {'alive_at': now, 'pid': os.getpid()})
        last_heartbeat = now
      self._refreshLaunchStates()
      self._checkForCommands()
      time.sleep(POLL_INTERVAL_SEC)

  def _refreshLaunchStates(self):
    # Long-running launches keep their status file updated even with no
    # new incoming command -- the device polls is_ready()/launcher_status
    # continuously, not just once right after launching.
    for target_key, entry in list(self.launch_procs.items()):
      proc = entry['proc']
      rc = proc.poll()
      if rc is not None:
        _, stderr = proc.communicate()
        state = 'exited' if rc == 0 else 'failed'
        self._writeStatus(entry['request_id'], target_key, 'launch', state,
                           exit_code=rc, error=(stderr or '').strip())
        del self.launch_procs[target_key]

  def _checkForCommands(self):
    for path in sorted(glob.glob(os.path.join(self.mailbox, 'cmd_*.json'))):
      request_id = os.path.basename(path)[len('cmd_'):-len('.json')]
      try:
        with open(path, 'r') as f:
          cmd = json.load(f)
      except (OSError, ValueError):
        # Still mid-write by the device side -- try again next tick rather
        # than deleting a command this watcher never actually saw.
        continue
      # Delete first, THEN process -- a command is consumed exactly once
      # even if this watcher crashes mid-handling (the alternative, delete
      # after processing, would silently re-run a launch/stop on restart).
      try:
        os.remove(path)
      except OSError:
        pass
      if request_id in self.processed_request_ids:
        continue
      self.processed_request_ids.add(request_id)
      self._handleCommand(request_id, cmd)

  def _handleCommand(self, request_id, cmd):
    action = cmd.get('action', '')
    target_key = cmd.get('target_key', '')
    script_text = cmd.get('script', '')
    timeout_sec = cmd.get('timeout_sec', 30)
    script_path = os.path.join(self.work_dir, 'nepi_watcher_' + request_id + '.sh')
    try:
      with open(script_path, 'w') as f:
        f.write(_strip_wrapper(script_text))
    except OSError as e:
      self._writeStatus(request_id, target_key, action, 'failed',
                         error='Could not stage command script: ' + str(e))
      return

    if action == 'launch':
      self._handleLaunch(request_id, target_key, script_path)
    elif action == 'stop':
      self._handleOneShot(request_id, target_key, action, script_path, timeout_sec)
      # Mark the ORIGINAL launch's own status file (not just this stop
      # request's) as no longer running too -- otherwise a device-side
      # poller still reading status_<the launch's own request_id>.json
      # (the file it's been polling all along for "is it still up") would
      # see a stale "running" forever, since popping launch_procs here
      # means _refreshLaunchStates() never gets a chance to notice this
      # process actually died.
      launched = self.launch_procs.pop(target_key, None)
      if launched is not None:
        self._writeStatus(launched['request_id'], target_key, 'launch', 'exited')
    elif action in ('install', 'check_installed', 'ready_check'):
      self._handleOneShot(request_id, target_key, action, script_path, timeout_sec)
    else:
      self._writeStatus(request_id, target_key, action, 'failed',
                         error='Unknown action: ' + str(action))

  def _handleLaunch(self, request_id, target_key, script_path):
    existing = self.launch_procs.get(target_key)
    if existing is not None and existing['proc'].poll() is None:
      self._writeStatus(request_id, target_key, 'launch', 'failed',
                         error=target_key + ' is already running')
      return
    proc = subprocess.Popen(['bash', '-l', script_path],
                             stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    self.launch_procs[target_key] = {'proc': proc, 'request_id': request_id}
    time.sleep(LAUNCH_STARTUP_GRACE_SEC)
    rc = proc.poll()
    if rc is not None:
      _, stderr = proc.communicate()
      self._writeStatus(request_id, target_key, 'launch', 'failed',
                         exit_code=rc, error=(stderr or '').strip())
      del self.launch_procs[target_key]
    else:
      self._writeStatus(request_id, target_key, 'launch', 'running', pid=proc.pid)

  def _handleOneShot(self, request_id, target_key, action, script_path, timeout_sec):
    try:
      result = subprocess.run(['bash', '-l', script_path], capture_output=True,
                               text=True, timeout=timeout_sec)
      rc = result.returncode
      output = ((result.stdout or '') + (result.stderr or '')).strip()
    except subprocess.TimeoutExpired:
      rc = None
      output = 'Timed out after ' + str(timeout_sec) + 's'
    state = 'exited' if rc == 0 else 'failed'
    self._writeStatus(request_id, target_key, action, state,
                       exit_code=rc, error=('' if rc == 0 else output))

  def _writeStatus(self, request_id, target_key, action, state, pid=None,
                    exit_code=None, error=''):
    _write_json_atomic(self._status_path(request_id), {
        'request_id': request_id,
        'target_key': target_key,
        'action': action,
        'state': state,
        'pid': pid,
        'exit_code': exit_code,
        'error': error,
        'updated_at': time.time(),
    })


def main():
  parser = argparse.ArgumentParser(description=(
      "Watches this OS instance's command mailbox under a locally-mounted "
      "nepi_storage share and runs launch/stop/install/check commands "
      "locally -- no SSH, no listening port. See this file's own module "
      "docstring for the full protocol."))
  parser.add_argument('--storage-root', required=True,
                       help='Local path where nepi_storage is mounted (e.g. /mnt/nepi_storage)')
  parser.add_argument('--instance-id', required=True,
                       help="This OS instance's id, matching os_instance_registry.py")
  args = parser.parse_args()
  Watcher(args.storage_root, args.instance_id).run()


if __name__ == '__main__':
  main()
