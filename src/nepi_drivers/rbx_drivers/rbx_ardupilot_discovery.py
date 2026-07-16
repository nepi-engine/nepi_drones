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

import os
import subprocess
import time
import serial
import socket

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system
from nepi_sdk import nepi_serial

PKG_NAME = 'RBX_ARDUPILOT' # Use in display menus
FILE_TYPE = 'DISCOVERY'


#########################################
# Ardupilot Discover Method
#########################################

### Function to try and connect to device and also monitor and clean up previously connected devices
class ArdupilotDiscovery:

  NODE_LOAD_TIME_SEC = 10
  launch_time_dict = dict()
  retry = True
  dont_retry_list = []

  active_devices_dict = dict()
  node_launch_name = "ardupilot"

  baudrate_list = ['57600']
  ip_addr_list = ['192.168.179.5']
  ip_udp_port_list = ['14550']
  ip_tcp_port_list = ['14550']

  # ArduPilot SITL running locally alongside mavros/the RBX driver (local VM/WSL dev setup,
  # per docs/SIMULATOR_DEV_GUIDE.md) -- not a specific networked host, so this works for
  # any developer's own local setup.
  sitl_addr_list = ['127.0.0.1']
  sitl_tcp_port_list = ['5760']

  includeDevices = []
  excludedDevices = ['ttyACM']

  enable_fake_gps = False

  # Mavros (mavlink) launch configuration
  APM_PLUGINLISTS_PATH = '/opt/nepi/nepi_engine/share/mavros/launch/apm_pluginlists.yaml'
  APM_CONFIG_PATH = '/opt/nepi/nepi_engine/share/mavros/launch/apm_config.yaml'

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
  ### Function to try and connect to device and also monitor and clean up previously connected devices
  def discoveryFunction(self, available_paths_list, active_paths_list, base_namespace, drv_dict, retry_enabled = True):
    self.drv_dict = drv_dict
    self.available_paths_list = available_paths_list
    self.active_paths_list = active_paths_list
    self.base_namespace = base_namespace

    ########################
    # Get discovery options
    try:
      connection_type = drv_dict['DISCOVERY_DICT']['OPTIONS']['connection']['value']
      fake_gps_val = drv_dict['DISCOVERY_DICT']['OPTIONS']['fake_gps']['value']
      self.enable_fake_gps = (str(fake_gps_val).lower() == 'true')
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

    ### Checking for devices on available paths
    # RUN SERIAL PROCESSES
    if connection_type == 'SERIAL':
      # Create path search options
      self.path_list = nepi_serial.get_serial_ports_list()
      for path_str in self.path_list:
        valid_path = True
        if path_str in self.active_paths_list or path_str in self.dont_retry_list:
          valid_path = False
        if valid_path:
          for exclude_device in self.excludedDevices:
            if path_str.find(exclude_device) != -1:
              valid_path = False
        if valid_path:
          [found_device, path_str, comp_id, sys_id, baud_str] = self.checkForSerialDevice(path_str)
          if found_device:
            success = self.launchSerialDeviceNode(path_str, comp_id, sys_id, baud_str)
            self.logger.log_info("Serial mavlink launch process returned: " + str(success))
            if success:
              self.active_paths_list.append(path_str)
    # RUN IP PROCESSES
    elif connection_type == 'TCP' or connection_type == "UDP":
      ip_addr_list = self.ip_addr_list
      # RUN TCP PROCESSES
      if connection_type == 'TCP':
        for ip_addr_str in ip_addr_list:
          for ip_port_str in self.ip_tcp_port_list:
            path_str = connection_type + "_" + ip_addr_str + "_" + ip_port_str
            if path_str not in self.active_paths_list and path_str not in self.dont_retry_list:
              [found_device, path_str] = self.checkForTcpDevice(path_str)
              if found_device:
                success = self.launchTcpDeviceNode(path_str)
                if success:
                  self.active_paths_list.append(path_str)
      # RUN UDP PROCESSES
      elif connection_type == 'UDP':
        for ip_addr_str in ip_addr_list:
          for ip_udp_port_str in self.ip_udp_port_list:
            path_str = connection_type + "_" + ip_addr_str + "_" + ip_udp_port_str
            if path_str not in self.active_paths_list and path_str not in self.dont_retry_list:
              [found_device, path_str] = self.checkForUdpDevice(path_str)
              if found_device:
                success = self.launchUdpDeviceNode(path_str)
                if success:
                  self.active_paths_list.append(path_str)
    # RUN SITL PROCESS (ArduPilot Software-In-The-Loop over TCP)
    elif connection_type == 'SITL':
      for ip_addr_str in self.sitl_addr_list:
        for ip_port_str in self.sitl_tcp_port_list:
          path_str = "SITL_" + ip_addr_str + "_" + ip_port_str
          if path_str not in self.active_paths_list and path_str not in self.dont_retry_list:
            # Reuse the TCP reachability probe: only launch mavros once SITL's
            # MAVLink TCP server is actually accepting connections.
            [found_device, path_str] = self.checkForTcpDevice(path_str)
            if found_device:
              success = self.launchSitlDeviceNode(path_str)
              if success:
                self.active_paths_list.append(path_str)
    # Wrap Up
    return self.active_paths_list


  ################################################
  ##########  Shared Processes

  def checkOnDevice(self, path_str):
    # Returns True if the device's mavlink process is still alive and its path is still present
    active = True
    if path_str not in self.active_devices_dict.keys():
      return False

    device_entry = self.active_devices_dict[path_str]
    mavlink_subproc = device_entry["mavlink_subproc"]

    purge_node = False
    # Check that the mavlink process is still running
    if mavlink_subproc is None or mavlink_subproc.poll() is not None:
      self.logger.log_warn("Mavlink process for " + path_str + " is no longer running... purging from managed list")
      purge_node = True
    # For serial connections, check that the port still exists
    elif path_str.startswith('/dev/') and path_str not in self.available_paths_list:
      self.logger.log_warn("Port associated with node no longer detected " + path_str)
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
    # Kill the mavlink, ardupilot, and (optional) fake_gps subprocesses for a device entry
    mav_node_name = device_entry.get("mav_node_name")
    ardu_node_name = device_entry.get("ardu_node_name")
    fgps_node_name = device_entry.get("fgps_node_name")
    mavlink_subproc = device_entry.get("mavlink_subproc")
    ardu_subproc = device_entry.get("ardu_subproc")
    fgps_subproc = device_entry.get("fgps_subproc")

    if ardu_subproc is not None:
      self.logger.log_info("Killing ardupilot node: " + str(ardu_node_name))
      nepi_drvs.killDriverNode(ardu_node_name, ardu_subproc)
    if fgps_subproc is not None:
      self.logger.log_info("Killing fake_gps node: " + str(fgps_node_name))
      nepi_drvs.killDriverNode(fgps_node_name, fgps_subproc)
    if mavlink_subproc is not None:
      self.logger.log_info("Killing mavlink node: " + str(mav_node_name))
      nepi_drvs.killDriverNode(mav_node_name, mavlink_subproc)


  def launchDeviceNode(self, path_str, device_id_str, mav_comp_id, mav_sys_id, fcu_url, gcs_url):
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

    ### Start Mavlink (mavros) Node Launch Process
    mav_node_name = "mavlink_" + device_id_str
    mav_node_namespace = nepi_sdk.create_namespace(self.base_namespace, mav_node_name)
    self.logger.log_info("Starting mavlink node setup: " + mav_node_name)
    # Load the proper configs for APM
    subprocess.run(['rosparam', 'load', self.APM_PLUGINLISTS_PATH, mav_node_namespace])
    subprocess.run(['rosparam', 'load', self.APM_CONFIG_PATH, mav_node_namespace])
    # Adjust the timesync_rate to cut down on log noise
    nepi_sdk.set_param(mav_node_namespace + '/conn/timesync_rate', 1.0)
    # Allow the HIL plugin. Disabled in apm configs for some reason
    plugin_blacklist = nepi_sdk.get_param(mav_node_namespace + '/plugin_blacklist')
    if plugin_blacklist is not None and 'hil' in plugin_blacklist:
      plugin_blacklist.remove('hil')
      nepi_sdk.set_param(mav_node_namespace + '/plugin_blacklist', plugin_blacklist)

    # Launch Mavlink Node
    self.logger.log_info("Launching mavlink node: " + mav_node_name)
    node_run_cmd = ['rosrun', 'mavros', 'mavros_node', '__name:=' + mav_node_name,
                    '_fcu_url:=' + fcu_url, '_gcs_url:=' + gcs_url]
    try:
      mav_subproc = subprocess.Popen(node_run_cmd)
    except Exception as e:
      self.logger.log_warn("Failed to launch mavlink node: " + mav_node_name + " (" + str(e) + ")")
      return False

    ### Start the ardupilot RBX node for this mavlink connection
    ardu_device_name = self.node_launch_name + "_" + device_id_str
    ardu_node_name = nepi_system.get_device_alias(ardu_device_name)

    # Setup required param server drv_dict for the ardupilot node
    file_name = self.drv_dict['NODE_DICT']['file_name']
    self.drv_dict['DEVICE_DICT'] = {
      'device_name': ardu_device_name,
      'device_path': path_str,
      'mavlink_node_name': mav_node_name,
      'fcu_url': fcu_url,
      'gcs_url': gcs_url,
      'mav_sys_id': mav_sys_id,
      'mav_comp_id': mav_comp_id,
      'fake_gps': self.enable_fake_gps
    }
    dict_param_name = nepi_sdk.create_namespace(self.base_namespace, ardu_node_name + "/drv_dict")
    nepi_sdk.set_param(dict_param_name, self.drv_dict)

    self.logger.log_info("Starting ardupilot rbx node: " + ardu_node_name)
    [success, msg, ardu_subproc] = nepi_drvs.launchDriverNode(file_name, ardu_node_name)

    # Process launch results
    self.launch_time_dict[launch_id] = nepi_sdk.get_time()
    if success:
      self.logger.log_info("Launched node: " + ardu_node_name)
      device_entry = dict()
      device_entry["sysid"] = mav_sys_id
      device_entry["compid"] = mav_comp_id
      device_entry["mav_node_name"] = mav_node_name
      device_entry["ardu_node_name"] = ardu_node_name
      device_entry["fgps_node_name"] = None
      device_entry["mavlink_subproc"] = mav_subproc
      device_entry["ardu_subproc"] = ardu_subproc
      device_entry["fgps_subproc"] = None
      self.active_devices_dict[path_str] = device_entry
    else:
      self.logger.log_warn("Failed to launch node: " + ardu_node_name + " with msg: " + msg)
      # The ardupilot node failed; tear down the mavlink process we started for it
      nepi_drvs.killDriverNode(mav_node_name, mav_subproc)
      if self.retry == False:
        self.logger.log_warn("Will not retry launch for node: " + ardu_node_name)
        self.dont_retry_list.append(launch_id)
    return success


  ########## SERIAL PROCESSES ############

  def checkForSerialDevice(self, path_str):
    found_device = False
    mav_comp_id = None
    mav_sys_id = 0
    baud_str = self.baudrate_list[0]
    for baud_str in self.baudrate_list:
      baud_int = int(baud_str)
      self.logger.log_warn("Connecting to serial port " + path_str + " with baudrate: " + baud_str)
      try:
        # Try and open serial port
        serial_port = serial.Serial(path_str, baud_int, timeout = 1)
      except Exception as e:
        self.logger.log_warn("Unable to open serial port " + path_str + " with baudrate: " + baud_str + "(" + str(e) + ")")
        continue

      for i in range(0, 500): # Read up to 500 packets waiting for heartbeat
        try:
          bytes_read = serial_port.read_until(b'\xFD', 280) # MAVLINK_2 packet start magic number, up to MAVLINK_2 max bytes in packet
          bytes_read_count = len(bytes_read)
        except Exception as e:
          continue

        if bytes_read_count == 0 or bytes_read_count == 255: # Timed out or read the max mavlink bytes in a packet
          break

        try:
          pkt_hdr = serial_port.read(9) # MAVLINK_2 packet header length
        except Exception as e:
          self.logger.log_warn("read failed (" + str(e) + ")")
          continue

        # Initialize as a non-heartbeat packet
        pkt_len = 255
        comp_id = 255
        msg_id_l = 255
        sys_id = 0
        if len(pkt_hdr) == 9:
          # This decoding assumes mavlink_2 format packet
          pkt_len = pkt_hdr[0]
          sys_id = pkt_hdr[4]
          comp_id = pkt_hdr[5]
          msg_id_l, msg_id_m, msg_id_h = pkt_hdr[6], pkt_hdr[7], pkt_hdr[8]

        # Identify a heartbeat packet by tell-tale signs
        if pkt_len == 9 and msg_id_l == 0x0 and msg_id_m == 0x0 and msg_id_h == 0x0: # Heartbeat message id = 0x00 00 00
          if sys_id > 0 and sys_id < 240:
            found_device = True
            mav_comp_id = comp_id
            mav_sys_id = sys_id
            self.logger.log_info("Found mavlink autonomous device at: " + path_str + " with baudrate " + baud_str + " with sys_id " + str(mav_sys_id))
            break
      # Clean up the serial port
      self.logger.log_warn("Closing serial port " + path_str)
      serial_port.close()
      time.sleep(1)
      if found_device:
        break
    return found_device, path_str, mav_comp_id, mav_sys_id, baud_str


  def launchSerialDeviceNode(self, path_str, mav_comp_id, mav_sys_id, baud_str):
    success = False
    if mav_comp_id is not None and mav_sys_id is not None:
      device_id_str = path_str.split('/')[-1]
      fcu_url = path_str + ':' + baud_str
      gcs_url = ""
      success = self.launchDeviceNode(path_str, device_id_str, mav_comp_id, mav_sys_id, fcu_url, gcs_url)
    return success


  ########## TCP PROCESSES ############

  def checkForTcpDevice(self, path_str):
    found_device = False
    [con_type, ip_addr_str, ip_port_str] = path_str.split("_")
    self.logger.log_warn("Mavlink_AD: Checking TCP connection: " + ip_addr_str + " " + ip_port_str)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    result = sock.connect_ex((ip_addr_str, int(ip_port_str)))
    if result == 0:
      found_device = True
      self.logger.log_warn("Mavlink_AD: Found TCP device on ip address: " + ip_addr_str + " port: " + ip_port_str + " is open")
      sock.close()
    else:
      self.logger.log_warn("Mavlink_AD: Did not find TCP device on ip address: " + ip_addr_str + " port: " + ip_port_str)
    return found_device, path_str


  def launchTcpDeviceNode(self, path_str):
    [con_type, ip_addr_str, ip_port_str] = path_str.split("_")
    ip_addr_str_list = ip_addr_str.split('.')
    ip_str_short = ''.join(ip_addr_str_list)
    device_id_str = ip_str_short + "_" + ip_port_str
    mav_comp_id = 1
    mav_sys_id = 1
    fcu_url = "tcp://" + ip_addr_str + ":" + ip_port_str
    gcs_url = ""
    return self.launchDeviceNode(path_str, device_id_str, mav_comp_id, mav_sys_id, fcu_url, gcs_url)


  ########## SITL PROCESS ############

  def launchSitlDeviceNode(self, path_str):
    # path_str format: "SITL_<host>_<port>"
    [con_type, ip_addr_str, ip_port_str] = path_str.split("_")
    device_id_str = "sitl"          # -> mavros node "mavlink_sitl", rbx node "ardupilot_sitl"
    mav_comp_id = 1
    mav_sys_id = 1                   # SITL default SYSID_THISMAV = 1
    fcu_url = "tcp://" + ip_addr_str + ":" + ip_port_str
    gcs_url = ""
    # SITL simulates its own GPS + compass. Force fake GPS OFF so the injected
    # GPS_INPUT can't fight the simulated sensors, regardless of the option value.
    self.enable_fake_gps = False
    return self.launchDeviceNode(path_str, device_id_str, mav_comp_id, mav_sys_id, fcu_url, gcs_url)


  ########## UDP PROCESSES ############

  def checkForUdpDevice(self, path_str):
    # UDP is connectionless; assume the configured endpoint is reachable
    found_device = True
    return found_device, path_str


  def launchUdpDeviceNode(self, path_str):
    [con_type, ip_addr_str, ip_port_str] = path_str.split("_")
    ip_addr_str_list = ip_addr_str.split('.')
    ip_str_short = ''.join(ip_addr_str_list)
    device_id_str = ip_str_short + "_" + ip_port_str
    mav_comp_id = 1
    mav_sys_id = 1
    fcu_url = "udp://192.168.179.103:14555@192.168.179.5:14550"
    gcs_url = ""
    return self.launchDeviceNode(path_str, device_id_str, mav_comp_id, mav_sys_id, fcu_url, gcs_url)


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
  ArdupilotDiscovery()
