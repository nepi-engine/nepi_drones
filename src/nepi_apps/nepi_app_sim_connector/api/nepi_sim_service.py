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

# Device side of the NEPI simulator management services in nepi_simulations:
# nepi_gazebo.sh (GAZEBO) and nepi_mujoco.sh (MUJOCO). Each service runs on the
# sim machine, polls its config file in NEPI storage for request flags, and
# reports its state back into the same file. It is the same poll-config /
# act-on-change / report-status loop nepi_docker.sh runs against
# nepi_docker_config.yaml, so this module mirrors that pairing's device side:
#
#   - load_config/save_config mirror nepi_sdk/nepi_system.py's
#     load/save_nepi_docker_config.
#   - start/stop mirror nepi_sdk/nepi_docker.py's request functions (e.g.
#     switchActiveAndInactiveContainers): set a flag and return. The service
#     clears the flag once it has acted.
#   - wait_for_request mirrors nepi_docker.py's saveImage, which polls the same
#     file until the host-side service has finished the job.
#
# The two services speak the same protocol with their own key prefix
# (GAZEBO_*, MUJOCO_*). Keys this side WRITES: <PREFIX>START, <PREFIX>STOP, and
# the staged <PREFIX>UPDATE_ENVIRONMENT_CONFIG / <PREFIX>UPDATE_ROBOT_CONFIG a
# start applies. Keys it only READS: <PREFIX>STATE, <PREFIX>LAST_ERROR,
# <PREFIX>PID, <PREFIX>LAST_POLL, <PREFIX>CURRENT_*. See each config file's own
# comments for the full protocol.
#
# Replaces nepi_gazebo.py (2026-10-05), which was this same code for Gazebo
# alone.

import os
import time

from nepi_sdk import nepi_utils

# A robot config folder's optional capability file -- see ROBOT_CONFIGS/README.md
# in the share for its fields.
ROBOT_CAPABILITIES_FILENAME = 'nepi_robot_config.yaml'

STATES = ['idle', 'starting', 'running', 'stopping', 'failed']
# States a request is still passing through, not an outcome.
TRANSIENT_STATES = ['starting', 'stopping']
# Outcomes that complete each request (see wait_for_request). DONE states mean
# the request's goal is met however it got there. A stop of a simulation that
# had already crashed leaves the state at 'failed' (the service's stop is a
# no-op with nothing running) -- still "nothing running", so still done.
START_DONE_STATES = ['running']
START_FAILED_STATES = ['failed']
STOP_DONE_STATES = ['idle', 'failed']
STOP_FAILED_STATES = []

# The service updates <PREFIX>LAST_POLL every poll tick (about once a second).
# Treated as unreachable once that value has not CHANGED for this long --
# measured on this device's monotonic clock, never by comparing the epoch value
# itself against this device's time: the value is the sim machine's clock, and
# this device's clock has no working time source (it resets to 1970 whenever
# its container is recreated).
SERVICE_STALE_AFTER_SEC = 20

POLL_INTERVAL_SEC = 1.0
# Both sides rewrite the whole file, and the service rewrites it every tick
# (LAST_POLL), so a request can occasionally be overwritten before the service
# reads it. A flag found cleared while the state shows nothing happened is
# re-set after this long.
REASSERT_AFTER_SEC = 3.0
# How many times a failure result identical to the one already in the file
# before the request is re-asserted before being believed (see
# wait_for_request).
REASSERT_LIMIT = 2


def get_config_int(config_dict, config_key):
  try:
    return int(config_dict.get(config_key, 0))
  except (TypeError, ValueError):
    return 0


def get_config_str(config_dict, config_key):
  value = config_dict.get(config_key, '')
  return '' if value is None else str(value)


class SimService(object):
  """One simulator management service: its folder in NEPI storage, its config
  file and key prefix, what counts as a world file and a robot folder in its
  share, and the request/heartbeat protocol shared by every service."""

  def __init__(self, name, key_prefix, folder_name, config_filename,
               world_extension, robot_model_files):
    # name: shown to the operator (e.g. "Gazebo").
    self.name = name
    self.key_prefix = key_prefix
    # The service's folder in NEPI storage (the service resolves it as
    # <mounted nepi_storage>/databases/sims/<folder_name>). Its config/,
    # ROBOT_CONFIGS/ and ENVIRONMENT_CONFIGS/ are created and kept in sync
    # with the sim machine by the service's own sync script, never here.
    self.sim_folder = os.path.join('/mnt/nepi_storage/databases/sims', folder_name)
    # The request/status file the service watches. A missing file means the
    # service has not been set up yet, not something to paper over with a
    # blank copy.
    self.config_file = os.path.join(self.sim_folder, 'config', config_filename)
    self.world_extension = world_extension
    # Files a ROBOT_CONFIGS folder must have for the service to accept it.
    self.robot_model_files = list(robot_model_files)
    # Heartbeat tracking -- see note_service_poll.
    self._last_poll_value = None
    self._last_poll_changed_at = None

  def key(self, suffix):
    return self.key_prefix + suffix

  #######################
  ## Config file

  def config_exists(self):
    return os.path.isfile(self.config_file)

  def load_config(self):
    """Returns the config dict, or None if the file is missing, empty, or
    unreadable (e.g. caught mid-write by the other side)."""
    if not self.config_exists():
      return None
    config_dict = nepi_utils.read_dict_from_file(self.config_file)
    if not isinstance(config_dict, dict) or len(config_dict.keys()) == 0:
      return None
    return config_dict

  def save_config(self, config_dict):
    """Writes config_dict to a temp file and swaps it into place, so the
    service never reads a half-written file -- its loader restores the file
    from its own .bak copy when a read fails to parse, which would silently
    discard this write. Keeps the existing file's owner and mode: this node
    runs as root, and a root-owned replacement could lock the sim machine's
    side (writing over the storage share as the NEPI user) out of the file."""
    if config_dict is None or len(config_dict.keys()) == 0:
      return False
    tmp_file = self.config_file + '.tmp'
    if not nepi_utils.write_dict_to_file(config_dict, tmp_file):
      return False
    try:
      stat_info = os.stat(self.config_file)
      os.chown(tmp_file, stat_info.st_uid, stat_info.st_gid)
      os.chmod(tmp_file, stat_info.st_mode & 0o7777)
    except OSError:
      pass
    try:
      os.replace(tmp_file, self.config_file)
    except OSError:
      return False
    return True

  #######################
  ## Selectable configs in the share

  def list_environment_configs(self):
    """The world files in the share's ENVIRONMENT_CONFIGS, sorted -- exactly
    what the service accepts as <PREFIX>UPDATE_ENVIRONMENT_CONFIG."""
    folder = os.path.join(self.sim_folder, 'ENVIRONMENT_CONFIGS')
    try:
      names = os.listdir(folder)
    except OSError:
      return []
    return sorted(n for n in names
                  if n.endswith(self.world_extension) and os.path.isfile(os.path.join(folder, n)))

  def list_robot_configs(self):
    """{folder name: capabilities dict} for every folder in the share's
    ROBOT_CONFIGS that the service accepts as <PREFIX>UPDATE_ROBOT_CONFIG
    (it has every file in robot_model_files). Capabilities come from the
    folder's optional ROBOT_CAPABILITIES_FILENAME, {} when it has none."""
    folder = os.path.join(self.sim_folder, 'ROBOT_CONFIGS')
    try:
      names = os.listdir(folder)
    except OSError:
      return {}
    robot_configs = dict()
    for name in sorted(names):
      path = os.path.join(folder, name)
      if not all(os.path.isfile(os.path.join(path, f)) for f in self.robot_model_files):
        continue
      capabilities = dict()
      capabilities_file = os.path.join(path, ROBOT_CAPABILITIES_FILENAME)
      if os.path.isfile(capabilities_file):
        file_dict = nepi_utils.read_dict_from_file(capabilities_file)
        if isinstance(file_dict, dict):
          capabilities = file_dict
      robot_configs[name] = capabilities
    return robot_configs

  #######################
  ## Service heartbeat

  def note_service_poll(self, config_dict):
    """Records the LAST_POLL value in config_dict, restarting the staleness
    clock whenever it has changed since the last call. The first value seen
    counts as a change, so a service is presumed alive until it has had
    SERVICE_STALE_AFTER_SEC to prove otherwise."""
    value = config_dict.get(self.key('LAST_POLL'))
    if value != self._last_poll_value or self._last_poll_changed_at is None:
      self._last_poll_value = value
      self._last_poll_changed_at = time.monotonic()

  def is_service_alive(self):
    if self._last_poll_changed_at is None:
      return False
    return (time.monotonic() - self._last_poll_changed_at) <= SERVICE_STALE_AFTER_SEC

  #######################
  ## Requests

  def send_request(self, flag_suffix, staged_values = None):
    """Sets <PREFIX><flag_suffix> to 1, together with any staged_values
    (e.g. {'UPDATE_ENVIRONMENT_CONFIG': ...}, suffixes without the prefix) in
    the same single write, so the service can never see the request without
    the selections it is meant to apply."""
    config_dict = self.load_config()
    if config_dict is None:
      return False
    if staged_values:
      config_dict.update({self.key(k): v for k, v in staged_values.items()})
    config_dict[self.key(flag_suffix)] = 1
    return self.save_config(config_dict)

  def start(self, staged_values = None):
    """Requests a start. The service treats a start as the apply step: it
    promotes any staged UPDATE_* selections, then starts the simulation if
    nothing is running, or changes the running one to match them."""
    return self.send_request('START', staged_values)

  def stop(self):
    return self.send_request('STOP')

  def clear_request(self, flag_suffix):
    """Withdraws a request the service never picked up, so it doesn't fire
    unexpectedly whenever the service next comes back. Only writes if the flag
    is still set -- every write is a whole-file rewrite the service could race."""
    config_dict = self.load_config()
    if config_dict is None or get_config_int(config_dict, self.key(flag_suffix)) != 1:
      return True
    config_dict[self.key(flag_suffix)] = 0
    return self.save_config(config_dict)

  def _outcome(self, config_dict):
    return (get_config_str(config_dict, self.key('STATE')),
            get_config_str(config_dict, self.key('LAST_ERROR')),
            get_config_int(config_dict, self.key('PID')))

  def wait_for_request(self, flag_suffix, done_states, failed_states, timeout_sec,
                       before_dict = None, staged_values = None):
    """Blocks until the service has acted on the flag -- cleared it back to 0
    -- and STATE shows the outcome. before_dict is the config as read just
    before the request was written; staged_values are whatever the request was
    sent with, re-sent along with it if it has to be re-asserted.

    Returns (True, config_dict) once it has, or (False, error_message) if the
    service stops responding or timeout_sec runs out first. The caller reads
    the outcome from config_dict's STATE/LAST_ERROR.

    A cleared flag alone doesn't prove the service saw THIS request: it may
    have been overwritten first, leaving the file exactly as it was. So:
      - a done_states outcome is accepted outright -- the goal is met either
        way;
      - a failed_states outcome is accepted only once it differs from
        before_dict (state, error, or PID), or after REASSERT_LIMIT re-asserts
        still give the same answer -- otherwise a new start after a crash
        would just report the old crash;
      - anything else with the flag cleared means nothing happened, and the
        flag is set again."""
    deadline = time.monotonic() + timeout_sec
    last_asserted_at = time.monotonic()
    reasserts = 0
    while time.monotonic() < deadline:
      time.sleep(POLL_INTERVAL_SEC)
      config_dict = self.load_config()
      if config_dict is None:
        continue
      self.note_service_poll(config_dict)
      if not self.is_service_alive():
        return False, ("The " + self.name + " service on the sim machine stopped responding (" +
                       self.config_file + " has not been updated in over " +
                       str(SERVICE_STALE_AFTER_SEC) + "s)")
      if get_config_int(config_dict, self.key(flag_suffix)) == 1:
        continue
      state = get_config_str(config_dict, self.key('STATE'))
      if state in done_states:
        return True, config_dict
      if state in TRANSIENT_STATES:
        continue
      if state in failed_states:
        unchanged = before_dict is not None and self._outcome(config_dict) == self._outcome(before_dict)
        if not unchanged or reasserts >= REASSERT_LIMIT:
          return True, config_dict
      if time.monotonic() - last_asserted_at >= REASSERT_AFTER_SEC:
        self.send_request(flag_suffix, staged_values)
        last_asserted_at = time.monotonic()
        reasserts += 1
    return False, ("Timed out after " + str(timeout_sec) + "s waiting for the " + self.name +
                   " service to finish the request")


# A Gazebo robot folder is spawned from model.sdf, and Gazebo also needs
# model.config to treat the folder as a model.
GAZEBO = SimService(name = 'Gazebo', key_prefix = 'GAZEBO_', folder_name = 'gazebo',
                    config_filename = 'nepi_gazebo_config.yaml', world_extension = '.world',
                    robot_model_files = ['model.config', 'model.sdf'])
MUJOCO = SimService(name = 'MuJoCo', key_prefix = 'MUJOCO_', folder_name = 'mujoco',
                    config_filename = 'nepi_mujoco_config.yaml', world_extension = '.xml',
                    robot_model_files = ['model.xml'])
