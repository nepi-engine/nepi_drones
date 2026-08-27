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
# Unlike the ardupilot driver there is no companion protocol node to launch: the
# bridge connection is a plain socket the launched node holds open itself, so
# only one process per robot is tracked.

import socket
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

  # The heartbeat listener replies with this on every connection.
  ALIVE_REPLY = b'ALIVE'
  PROBE_TIMEOUT_SEC = 2
  PROBE_REPLY_BYTES = 16

  ################################################
  def __init__(self):
    ############
    # Create Message Logger
    self.log_name = PKG_NAME.lower() + "_discovery"
    self.logger = nepi_sdk.logger(log_name = self.log_name)
    time.sleep(1)
    self.logger.log_info("Starting Initialization")
    self.logger.log_info("Initialization Complete")


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
      self.host = str(options['host']['value'])
      self.heartbeat_port = str(options['heartbeat_port']['value'])
      self.bridge_port = int(options['bridge_port']['value'])
    except Exception as e:
      self.logger.log_warn("Failed to load options " + str(e))
      return None

    # Retry behavior
    self.retry = retry_enabled
    if self.retry == True:
      self.dont_retry_list = []
    ########################

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

    ### Check the configured MuJoCo bridge instance's heartbeat listener
    path_str = "MUJOCO_" + self.host + "_" + self.heartbeat_port
    if path_str not in self.active_paths_list and path_str not in self.dont_retry_list:
      if self.checkForMujocoDevice(self.host, self.heartbeat_port):
        self.logger.log_info("MuJoCo heartbeat detected at " + self.host + ":" +
                             self.heartbeat_port + ". Launching mujoco rbx node")
        success = self.launchMujocoDeviceNode(path_str)
        if success:
          self.active_paths_list.append(path_str)

    # Wrap Up
    return self.active_paths_list


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
    # Check that the rbx node process is still running
    if rbx_subproc is None or rbx_subproc.poll() is not None:
      self.logger.log_warn("MuJoCo rbx node process for " + path_str +
                           " is no longer running... purging from managed list")
      purge_node = True
    else:
      # Check that the MuJoCo heartbeat listener still answers
      host_str = device_entry["host"]
      port_str = device_entry["heartbeat_port"]
      if self.checkForMujocoDevice(host_str, port_str) == False:
        self.logger.log_warn("MuJoCo heartbeat no longer answering for " + path_str +
                             "... purging from managed list")
        purge_node = True

    if purge_node:
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

  def checkForMujocoDevice(self, host_str, port_str):
    # Probe the MuJoCo-side heartbeat listener and require its ALIVE reply. A
    # bare successful connect is not sufficient evidence: through a forwarded
    # port, connect succeeds against the local forwarder even when the far-end
    # listener is down.
    found_device = False
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(self.PROBE_TIMEOUT_SEC)
    try:
      result = sock.connect_ex((host_str, int(port_str)))
      if result == 0:
        reply = sock.recv(self.PROBE_REPLY_BYTES)
        if reply.startswith(self.ALIVE_REPLY):
          found_device = True
    except Exception:
      found_device = False
    finally:
      try:
        sock.close()
      except Exception:
        pass
    return found_device


  def launchMujocoDeviceNode(self, path_str):
    # path_str format: "MUJOCO_<host>_<heartbeat_port>"
    success = False
    launch_id = path_str

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
    # the entire contract between discovery and the node.
    file_name = self.drv_dict['NODE_DICT']['file_name']
    self.drv_dict['DEVICE_DICT'] = {
      'device_name': mujoco_device_name,
      'device_path': path_str,
      'host': self.host,
      'heartbeat_port': int(self.heartbeat_port),
      'bridge_port': self.bridge_port
    }
    dict_param_name = nepi_sdk.create_namespace(self.base_namespace, rbx_node_name + "/drv_dict")
    nepi_sdk.set_param(dict_param_name, self.drv_dict)

    self.logger.log_info("Starting mujoco rbx node: " + rbx_node_name)
    # Guarded so a launch-helper exception (e.g. the node file not yet deployed)
    # reads as a failed launch instead of taking the whole driver offline --
    # drivers_mgr disables a driver whose discoveryFunction raises.
    try:
      [success, msg, rbx_subproc] = nepi_drvs.launchDriverNode(file_name, rbx_node_name)
    except Exception as e:
      [success, msg, rbx_subproc] = [False, str(e), None]

    # Process launch results
    self.launch_time_dict[launch_id] = nepi_sdk.get_time()
    if success:
      self.logger.log_info("Launched node: " + rbx_node_name)
      device_entry = dict()
      device_entry["rbx_node_name"] = rbx_node_name
      device_entry["rbx_subproc"] = rbx_subproc
      device_entry["host"] = self.host
      device_entry["heartbeat_port"] = self.heartbeat_port
      device_entry["bridge_port"] = self.bridge_port
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
