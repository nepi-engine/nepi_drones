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

import time
import socket

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system

PKG_NAME = 'RBX_SIM' # Use in display menus
FILE_TYPE = 'DISCOVERY'


#########################################
# Sim Discover Method
#########################################

### Function to try and connect to the simulator and also monitor and clean up previously connected devices
class SimDiscovery:

  NODE_LOAD_TIME_SEC = 10
  launch_time_dict = dict()
  retry = True
  dont_retry_list = []

  active_devices_dict = dict()
  node_launch_name = "sim"

  # The generic-rover Gazebo simulation runs on the dev VM, whose sim stack
  # (sim_rover_gazebo / sim_rover_gazebo_multi in sim_rover_dev_env.sh)
  # starts a tiny plain-TCP heartbeat listener (sim_heartbeat_listener.py)
  # per robot, forwarded to this device's own loopback by the existing
  # reverse SSH tunnel (nepi_tunnel). The two machines have separate ROS
  # masters, so the sim's /sim/heartbeat ROS topic is invisible here -- the
  # raw heartbeat TCP ports are the liveness signal.
  sim_addr_list = ['127.0.0.1']
  # Robot slots (Phase 4): one (heartbeat_port, bridge_port) pair plus
  # VM-side identity per simulated robot. Each slot whose heartbeat answers
  # gets its own rbx_sim node wired to its own bridge port -- slots probe
  # independently, so the single-robot workflow (only rover1's 9022/9023
  # served by sim_rover_gazebo) still launches exactly one node, and the
  # multi-robot workflow (sim_rover_gazebo_multi serving both pairs) gets
  # one node per robot. Ports follow the 902x sim-utility block numbering
  # (9021 gz reset, 9022/9023 rover1 heartbeat/bridge -- the original
  # single-robot pair -- 9024/9025 rover2 heartbeat/bridge), clear of the
  # 576x MAVLink ports; all forwarded by the same reverse tunnel. The
  # heartbeat is a transient probe; the bridge is the persistent JSON-lines
  # connection the launched rbx_sim node holds open (sim_bridge_node.py
  # single-robot, sim_bridge_multi_node.py multi-robot).
  # The *_topic entries are the robot's simulator-local ROS topic names on
  # the VM's own ROS master, passed through DEVICE_DICT for reference only
  # (the node talks to the bridge port; the VM side owns its topic wiring --
  # names below match the multi-robot world; the single-robot world uses
  # unnumbered /rover/* names).
  SIM_ROBOT_SLOTS = [
    {'device_id': 'rover1',
     'heartbeat_port': '9022',
     'bridge_port': 9023,
     'cmd_vel_topic': '/rover1/cmd_vel',
     'odom_topic': '/rover1/odom',
     'image_topic': '/rover1/camera/image_raw'},
    {'device_id': 'rover2',
     'heartbeat_port': '9024',
     'bridge_port': 9025,
     'cmd_vel_topic': '/rover2/cmd_vel',
     'odom_topic': '/rover2/odom',
     'image_topic': '/rover2/camera/image_raw'},
  ]
  # The listener replies ALIVE on every connection. Checking for that reply
  # (not just a successful connect) matters: with an ssh -R forward, connect()
  # succeeds against the local sshd even when nothing is listening on the far
  # end -- sshd only closes the connection after failing to reach the VM side.
  SIM_ALIVE_REPLY = b'ALIVE'

  ################################################
  # A single missed heartbeat probe killed and relaunched a healthy rbx
  # node outright (confirmed live, 2026-08-31: a busy dev VM occasionally
  # drops or delays one TCP probe through the reverse tunnel to
  # sim_heartbeat_listener.py -- not a real "the sim died" signal, just
  # transient contention) -- this put the node in an endless restart loop,
  # since each relaunch takes several seconds to reach steady state and the
  # very next discovery cycle's probe could just as easily hit another
  # transient blip before it gets there. Raised from an initial 3 to 6
  # (2026-09-01) after a fresh device+VM reboot showed the reverse tunnel
  # can still throw brief BURSTS of 2-3 consecutive misses while it settles,
  # not just isolated singles -- 6 comfortably absorbs that without
  # meaningfully slowing down detection of a genuinely dead simulator
  # (discovery runs every 1-3s per nepi_drivers' own polling interval, so this is still a
  # few-second worst case). Doesn't apply to the OTHER purge condition in
  # checkOnDevice (the rbx node subprocess itself having actually exited) --
  # that's an unambiguous, instantaneous signal with nothing to debounce.
  HEARTBEAT_MISS_THRESHOLD = 6

  def __init__(self):
    ############
    # Create Message Logger
    self.log_name = PKG_NAME.lower() + "_discovery"
    self.logger = nepi_sdk.logger(log_name = self.log_name)
    self.heartbeat_miss_counts = dict()
    time.sleep(1)
    self.logger.log_info("Starting Initialization")
    self.logger.log_info("Initialization Complete")


  ##########  Drv Standard Discovery Function
  ### Function to try and connect to the simulator and also monitor and clean up previously connected devices
  def discoveryFunction(self, available_paths_list, active_paths_list, base_namespace, drv_dict, retry_enabled = True):
    self.drv_dict = drv_dict
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace

    ########################
    # Get discovery options
    try:
      connection_type = drv_dict['DISCOVERY_DICT']['OPTIONS']['connection']['value']
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

    ### Checking each robot slot's heartbeat listener
    if connection_type == 'SIMULATOR':
      for ip_addr_str in self.sim_addr_list:
        for robot_slot in self.SIM_ROBOT_SLOTS:
          ip_port_str = robot_slot['heartbeat_port']
          path_str = "SIM_" + ip_addr_str + "_" + ip_port_str
          if path_str not in self.active_paths_list and path_str not in self.dont_retry_list:
            found_device = self.checkForSimDevice(ip_addr_str, ip_port_str)
            if found_device:
              self.logger.log_info("Sim heartbeat detected at " + ip_addr_str + ":" + ip_port_str + ". Launching sim rbx node for " + robot_slot['device_id'])
              success = self.launchSimDeviceNode(path_str, robot_slot)
              if success:
                self.active_paths_list.append(path_str)
    # Wrap Up
    return self.active_paths_list


  ################################################
  ##########  Device Monitor Processes

  def checkOnDevice(self, path_str):
    # Returns True if the device's rbx node process is still alive and the sim heartbeat still answers
    active = True
    if path_str not in self.active_devices_dict.keys():
      return False

    device_entry = self.active_devices_dict[path_str]
    sim_subproc = device_entry["sim_subproc"]

    purge_node = False
    # Check that the rbx node process is still running -- unambiguous and
    # instantaneous, nothing to debounce here.
    if sim_subproc is None or sim_subproc.poll() is not None:
      self.logger.log_warn("Sim rbx node process for " + path_str + " is no longer running... purging from managed list")
      purge_node = True
    else:
      # Check that the simulator's heartbeat listener still answers -- see
      # HEARTBEAT_MISS_THRESHOLD's own comment for why a single miss isn't
      # purged on the spot.
      [con_type, ip_addr_str, ip_port_str] = path_str.split("_")
      if self.checkForSimDevice(ip_addr_str, ip_port_str) == False:
        miss_count = self.heartbeat_miss_counts.get(path_str, 0) + 1
        self.heartbeat_miss_counts[path_str] = miss_count
        if miss_count >= self.HEARTBEAT_MISS_THRESHOLD:
          self.logger.log_warn("Sim heartbeat missed " + str(miss_count) +
                               " times in a row for " + path_str + "... purging from managed list")
          purge_node = True
        else:
          self.logger.log_warn("Sim heartbeat miss " + str(miss_count) + "/" +
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
    # Kill the sim rbx node subprocess for a device entry
    sim_node_name = device_entry.get("sim_node_name")
    sim_subproc = device_entry.get("sim_subproc")
    if sim_subproc is not None:
      self.logger.log_info("Killing sim rbx node: " + str(sim_node_name))
      nepi_drvs.killDriverNode(sim_node_name, sim_subproc)


  ########## SIMULATOR PROCESSES ############

  def checkForSimDevice(self, ip_addr_str, ip_port_str):
    # Probe the sim heartbeat listener's TCP port (through the reverse tunnel)
    # and require its ALIVE reply -- a bare connect succeeds against sshd even
    # when the far-end listener is down (see class comment above).
    #
    # Raised from 2s to 6s (2026-09-01): even with HEARTBEAT_MISS_THRESHOLD
    # raised to 6, a live rover was still purged mid-session -- the reverse
    # tunnel carrying real traffic (Gazebo's own topics, image streams) adds
    # real round-trip latency on top of the connect+ALIVE-reply exchange, and
    # a 2s socket timeout was tight enough to misread that latency as "not
    # answering" under normal running load, not just at startup.
    found_device = False
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(6)
    try:
      result = sock.connect_ex((ip_addr_str, int(ip_port_str)))
      if result == 0:
        reply = sock.recv(16)
        if reply.startswith(self.SIM_ALIVE_REPLY):
          found_device = True
    except Exception:
      found_device = False
    finally:
      sock.close()
    return found_device


  def launchSimDeviceNode(self, path_str, robot_slot):
    # path_str format: "SIM_<host>_<heartbeat_port>"; robot_slot is the
    # SIM_ROBOT_SLOTS entry whose heartbeat answered
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

    ### Start the sim RBX node for this robot slot
    device_id_str = robot_slot['device_id']   # -> rbx node "sim_rover1" / "sim_rover2"
    sim_device_name = self.node_launch_name + "_" + device_id_str
    sim_node_name = nepi_system.get_device_alias(sim_device_name)

    # Setup required param server drv_dict for the sim node
    file_name = self.drv_dict['NODE_DICT']['file_name']
    self.drv_dict['DEVICE_DICT'] = {
      'device_name': sim_device_name,
      'device_path': path_str,
      'sim_host': ip_addr_str,
      'sim_port': int(ip_port_str),
      'bridge_port': robot_slot['bridge_port'],
      'cmd_vel_topic': robot_slot['cmd_vel_topic'],
      'odom_topic': robot_slot['odom_topic'],
      'image_topic': robot_slot['image_topic']
    }
    dict_param_name = nepi_sdk.create_namespace(self.base_namespace, sim_node_name + "/drv_dict")
    nepi_sdk.set_param(dict_param_name, self.drv_dict)

    self.logger.log_info("Starting sim rbx node: " + sim_node_name)
    # Guarded so a launch-helper exception (e.g. the node file not yet
    # deployed) reads as a failed launch instead of taking the whole driver
    # offline (drivers_mgr disables a driver whose discoveryFunction raises)
    try:
      [success, msg, sim_subproc] = nepi_drvs.launchDriverNode(file_name, sim_node_name)
    except Exception as e:
      [success, msg, sim_subproc] = [False, str(e), None]

    # Process launch results
    self.launch_time_dict[launch_id] = nepi_sdk.get_time()
    if success:
      self.logger.log_info("Launched node: " + sim_node_name)
      device_entry = dict()
      device_entry["sim_node_name"] = sim_node_name
      device_entry["sim_subproc"] = sim_subproc
      self.active_devices_dict[path_str] = device_entry
    else:
      self.logger.log_warn("Failed to launch node: " + sim_node_name + " with msg: " + msg)
      if self.retry == False:
        self.logger.log_warn("Will not retry launch for node: " + sim_node_name)
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
  SimDiscovery()
