#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_drivers) repo
# (see https://https://github.com/nepi-engine/nepi_drivers)
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

# Discovery for the MuJoCo simulated robot RBX driver. This is
# rbx_webots_discovery.py with only naming changes -- the purge-then-probe-
# then-launch structure, the heartbeat-with-ALIVE-reply requirement, and the
# relaunch backoff are all identical, because the underlying problem is
# identical: a simulator process running on a separate host with its own
# process lifecycle, reachable only through raw TCP ports (mujoco_rbx_bridge.py
# has no ROS graph of its own to be visible through, same as Gazebo/Webots).
#
# UPDATED (2026-09-21): flipped from dialing a configured host:heartbeat_port
# to an always-on device-side listener that mujoco_rbx_bridge.py now dials
# INTO -- same fix, same reasoning, as rbx_webots_discovery.py's own
# 2026-09-21 update (see that file's module comment for the full writeup:
# this dev VM cannot be reached from the NEPI device on any port at all,
# confirmed live).
#
# Unlike the ardupilot driver there is no companion protocol node to launch: the
# bridge connection is a plain socket the launched node holds open itself, so
# only one process per robot is tracked.

import os
import socket
import threading
import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

PKG_NAME = 'RBX_MUJOCO' # Use in display menus
FILE_TYPE = 'DISCOVERY'


#########################################
# MuJoCo Discover Method
#########################################

### Function to try and connect to a MuJoCo bridge instance and also monitor and clean up previously connected devices
class MujocoDiscovery:

  NODE_LOAD_TIME_SEC = 10
  launch_time_dict = dict()
  retry = True
  dont_retry_list = []

  active_devices_dict = dict()
  node_launch_name = "mujoco"

  # Device id for the single robot this driver discovers per MuJoCo bridge
  # instance. Multi-robot worlds are out of scope for this pass.
  DEVICE_ID = 'robot'

  # The heartbeat ping mujoco_rbx_bridge.py sends.
  ALIVE_REPLY = b'ALIVE'

  # Same values and same reasoning as rbx_webots_discovery.py's own.
  HEARTBEAT_LISTEN_TIMEOUT_SEC = 4
  HEARTBEAT_MISS_THRESHOLD = 2

  # Heartbeat listener state -- deliberately class-level, see
  # rbx_webots_discovery.py's own comment for why.
  heartbeat_last_seen = dict()
  heartbeat_lock = threading.Lock()
  heartbeat_listeners_started = set()

  ################################################
  def __init__(self):
    ############
    # Create Message Logger
    self.log_name = PKG_NAME.lower() + "_discovery"
    self.logger = nepi_sdk.logger(log_name = self.log_name)
    self.heartbeat_miss_counts = dict()
    self.active_devices_dict = dict()
    self.launch_time_dict = dict()
    self.dont_retry_list = []
    time.sleep(1)
    self.logger.log_info("Starting Initialization")
    self.logger.log_info("Initialization Complete")

  def _startHeartbeatListener(self, port):
    if port in self.heartbeat_listeners_started:
      return
    self.heartbeat_listeners_started.add(port)

    def _handle(conn, peer_ip):
      try:
        conn.settimeout(3)
        data = conn.recv(16)
        if data.startswith(self.ALIVE_REPLY):
          with self.heartbeat_lock:
            self.heartbeat_last_seen[(peer_ip, str(port))] = time.time()
      except Exception:
        pass
      finally:
        try:
          conn.close()
        except Exception:
          pass

    def _acceptLoop():
      srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
      srv.bind(('0.0.0.0', port))
      srv.listen(16)
      while True:
        try:
          conn, addr = srv.accept()
        except Exception:
          continue
        threading.Thread(target = _handle, args = (conn, addr[0]), daemon = True).start()

    threading.Thread(target = _acceptLoop, daemon = True).start()

  ##########  Drv Standard Discovery Function
  ### Function to try and connect to a MuJoCo bridge instance and also monitor and clean up previously connected devices
  def discoveryFunction(self, available_paths_list, active_paths_list, base_namespace, drv_dict, retry_enabled = True):
    self.drv_dict = drv_dict
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace

    ########################
    # Get discovery options
    try:
      options = drv_dict['DISCOVERY_DICT']['OPTIONS']
      self.heartbeat_port = int(options['heartbeat_port']['value'])
      self.bridge_port = int(options['bridge_port']['value'])
    except Exception as e:
      self.logger.log_warn("Failed to load options " + str(e))
      return None

    # Retry behavior
    self.retry = retry_enabled
    if self.retry == True:
      self.dont_retry_list = []
    ########################

    self._startHeartbeatListener(self.heartbeat_port)

    ### Purge Unresponsive Connections
    path_purge_list = []
    for path_str in self.active_devices_dict.keys():
      success = self.checkOnDevice(path_str)
      if success == False:
        path_purge_list.append(path_str)
    # Clean up the active_devices_dict
    for path_str in path_purge_list:
      del self.active_devices_dict[path_str]
      if path_str in self.active_paths_list:
        self.active_paths_list.remove(path_str)

    ### Checking the heartbeat listener for any peer that has pinged recently
    for ip_addr_str in self._recentPeersForPort(self.heartbeat_port):
      path_str = "MUJOCO_" + ip_addr_str + "_" + str(self.heartbeat_port)
      if path_str not in self.active_paths_list and path_str not in self.dont_retry_list:
        self.logger.log_info("MuJoCo heartbeat detected at " + ip_addr_str + ":" +
                             str(self.heartbeat_port) + ". Launching mujoco rbx node")
        success = self.launchMujocoDeviceNode(path_str)
        if success:
          self.active_paths_list.append(path_str)

    # Wrap Up
    return self.active_paths_list

  def _recentPeersForPort(self, port):
    now = time.time()
    with self.heartbeat_lock:
      return [addr for (addr, p), seen in self.heartbeat_last_seen.items()
              if p == str(port) and (now - seen) < self.HEARTBEAT_LISTEN_TIMEOUT_SEC]


  ################################################
  ##########  Device Monitor Processes

  def checkOnDevice(self, path_str):
    # Returns True if the device's rbx node process is still alive and the
    # MuJoCo heartbeat still answers
    active = True
    if path_str not in self.active_devices_dict.keys():
      return False

    device_entry = self.active_devices_dict[path_str]
    rbx_subproc = device_entry["rbx_subproc"]

    purge_node = False
    if rbx_subproc is None or rbx_subproc.poll() is not None:
      self.logger.log_warn("MuJoCo rbx node process for " + path_str +
                           " is no longer running... purging from managed list")
      purge_node = True
    else:
      [con_type, ip_addr_str, ip_port_str] = path_str.split("_")
      if self.checkForMujocoDevice(ip_addr_str, ip_port_str) == False:
        miss_count = self.heartbeat_miss_counts.get(path_str, 0) + 1
        self.heartbeat_miss_counts[path_str] = miss_count
        if miss_count >= self.HEARTBEAT_MISS_THRESHOLD:
          self.logger.log_warn("MuJoCo heartbeat missed " + str(miss_count) +
                               " times in a row for " + path_str + "... purging from managed list")
          purge_node = True
        else:
          self.logger.log_warn("MuJoCo heartbeat miss " + str(miss_count) + "/" +
                               str(self.HEARTBEAT_MISS_THRESHOLD) + " for " + path_str +
                               " -- not purging yet")
      else:
        self.heartbeat_miss_counts[path_str] = 0

    if purge_node:
      self.heartbeat_miss_counts.pop(path_str, None)
      self.killDeviceProcesses(device_entry)
      if path_str in self.active_paths_list:
        self.active_paths_list.remove(path_str)
      if path_str in self.dont_retry_list:
        self.dont_retry_list.remove(path_str)
      active = False
    return active


  def killDeviceProcesses(self, device_entry):
    # Kill the mujoco rbx node subprocess for a device entry
    rbx_node_name = device_entry.get("rbx_node_name")
    rbx_subproc = device_entry.get("rbx_subproc")
    if rbx_subproc is not None:
      self.logger.log_info("Killing mujoco rbx node: " + str(rbx_node_name))
      nepi_drvs.killDriverNode(rbx_node_name, rbx_subproc)


  ##########  MUJOCO PROCESSES

  def checkForMujocoDevice(self, ip_addr_str, ip_port_str):
    with self.heartbeat_lock:
      last_seen = self.heartbeat_last_seen.get((ip_addr_str, str(ip_port_str)), 0)
    return (time.time() - last_seen) < self.HEARTBEAT_LISTEN_TIMEOUT_SEC


  def launchMujocoDeviceNode(self, path_str):
    # path_str format: "MUJOCO_<host>_<heartbeat_port>"
    success = False
    launch_id = path_str
    [con_type, ip_addr_str, ip_port_str] = path_str.split("_")

    # Check if should try to launch (backoff to prevent rapid relaunch loops)
    launch_check = True
    if launch_id in self.launch_time_dict.keys():
      launch_time = self.launch_time_dict[launch_id]
      cur_time = nepi_sdk.get_time()
      launch_check = (cur_time - launch_time) > self.NODE_LOAD_TIME_SEC
    if launch_check == False:
      return False

    ### Start the mujoco RBX node for this instance
    mujoco_device_name = self.node_launch_name + "_" + self.DEVICE_ID
    rbx_node_name = nepi_system.get_device_alias(mujoco_device_name)

    # Setup required param server drv_dict for the mujoco node. This param is
    # the entire contract between discovery and the node. No host/port to
    # dial out to any more -- the node just listens on bridge_port and
    # mujoco_rbx_bridge.py dials in (see rbx_mujoco_node.py's own bridgeLoop).
    file_name = self.drv_dict['NODE_DICT']['file_name']
    self.drv_dict['DEVICE_DICT'] = {
      'device_name': mujoco_device_name,
      'device_path': path_str,
      'bridge_port': self.bridge_port
    }
    dict_param_name = nepi_sdk.create_namespace(self.base_namespace, rbx_node_name + "/drv_dict")
    nepi_sdk.set_param(dict_param_name, self.drv_dict)

    self.logger.log_info("Starting mujoco rbx node: " + rbx_node_name)
    # Guarded so a launch-helper exception (e.g. the node file not yet deployed)
    # reads as a failed launch instead of taking the whole driver offline --
    # drivers_mgr disables a driver whose discoveryFunction raises.
    # LD_PRELOAD needed here, not a general drivers_mgr/launchDriverNode fix --
    # same root cause and same fix as rbx_ardupilot_discovery.py's own
    # launchDeviceNode (see that method's own comment for the full writeup):
    # this node's `from nepi_api.device_if_rbx import RBXRobotIF` (-> nepi_pc
    # -> `import open3d`) crashes with "libgomp.so.1: cannot allocate memory
    # in static TLS block" on this aarch64 build whenever cv2 (also imported
    # by this node, earlier) has already claimed libgomp's one static TLS
    # slot. Preloading libgomp before the interpreter starts guarantees it
    # gets the only claim regardless of import order. Restored right after
    # the spawn call returns so this doesn't leak into unrelated drivers_mgr
    # spawns that never needed it.
    ld_preload_key = 'LD_PRELOAD'
    prev_ld_preload = os.environ.get(ld_preload_key)
    os.environ[ld_preload_key] = '/lib/aarch64-linux-gnu/libgomp.so.1'
    try:
      [success, msg, rbx_subproc] = nepi_drvs.launchDriverNode(file_name, rbx_node_name)
    except Exception as e:
      [success, msg, rbx_subproc] = [False, str(e), None]
    finally:
      if prev_ld_preload is None:
        os.environ.pop(ld_preload_key, None)
      else:
        os.environ[ld_preload_key] = prev_ld_preload

    # Process launch results
    self.launch_time_dict[launch_id] = nepi_sdk.get_time()
    if success:
      self.logger.log_info("Launched node: " + rbx_node_name)
      device_entry = dict()
      device_entry["rbx_node_name"] = rbx_node_name
      device_entry["rbx_subproc"] = rbx_subproc
      self.active_devices_dict[path_str] = device_entry
    else:
      self.logger.log_warn("Failed to launch node: " + rbx_node_name + " with msg: " + str(msg))
      if self.retry == False:
        self.logger.log_warn("Will not retry launch for node: " + rbx_node_name)
        self.dont_retry_list.append(launch_id)
    return success


  def killAllDevices(self, active_paths_list):
    path_purge_list = []
    for key in self.active_devices_dict.keys():
      path_purge_list.append(key)
    for path_str in path_purge_list:
      device_entry = self.active_devices_dict[path_str]
      if self.retry == False:
        self.dont_retry_list.append(path_str)
      self.killDeviceProcesses(device_entry)
      if path_str in active_paths_list:
        active_paths_list.remove(path_str)
    for path_str in path_purge_list:
      del self.active_devices_dict[path_str]
    nepi_sdk.sleep(1)
    return active_paths_list


#########################################
# Main
#########################################
if __name__ == '__main__':
  MujocoDiscovery()
