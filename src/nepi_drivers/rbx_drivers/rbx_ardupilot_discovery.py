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
import select
import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_drvs
from nepi_sdk import nepi_system
from nepi_sdk import nepi_serial

PKG_NAME = 'RBX_ARDUPILOT' # Use in display menus
FILE_TYPE = 'DISCOVERY'


class SitlMavlinkRelay:
  """Device-side half of the VM<->device MAVLink TCP relay used only when
  connection_type == 'SITL' -- see mavlink_relay_vm.py (in nepi_drones'
  sim_container/scripts/) for the VM-side half and the full root-cause
  writeup of why this exists: the reverse SSH tunnel that used to make the
  VM's real SITL MAVLink port transparently reachable at 127.0.0.1:5771 on
  THIS device is gone now that every sim_connector bridge dials OUT from
  the VM to the device instead. Confirmed live (2026-09-08): with nothing
  forwarding it anymore, this class's own module (rbx_ardupilot_discovery)
  logged "Did not find TCP device on ip address: 127.0.0.1 port: 5771" in
  an endless loop the entire time a real quadcopter sim was up and running
  fine on the VM -- no RBX device ever registered for it, so nothing
  Settings-related, no images, nothing worked for that target at all.

  Listens on DEVICE_LISTEN_PORT for mavlink_relay_vm.py to dial in (the
  same "VM dials device" direction every other bridge here already uses),
  and re-exposes whatever it forwards as a plain local TCP server on
  127.0.0.1:SITL_LOCAL_PORT -- exactly the address sitl_addr_list/
  sitl_tcp_port_list and launchSitlDeviceNode's own fcu_url already
  expect, so neither this file's own probe nor mavros's real persistent
  connection needed to change at all.

  One VM connection and one local (discovery-probe-or-mavros) connection
  relayed at a time, matching the one dedicated --out port SITL itself
  already uses for this consumer (see sitl_tcp_port_list's own comment for
  why only one canonical port is used here). A local connection accepted
  before the VM has dialed in just blocks harmlessly reading zero bytes
  until its own peer times out and closes (checkForTcpDevice's 2s socket
  timeout, or mavros's own retry) -- never delivers data until a real VM
  link exists, so a merely-accepted-but-silent socket still correctly
  reads as "absent" exactly like checkForTcpDevice's own comment already
  requires (the same false-positive a reverse-tunnel's sshd end always
  risked).
  """
  DEVICE_LISTEN_PORT = 9031
  SITL_LOCAL_PORT = 5771

  _started = False
  _start_lock = threading.Lock()
  _vm_conn = None
  _vm_conn_lock = threading.Lock()
  # Guards against exactly the crash confirmed live (2026-09-08): a fresh
  # local connection (e.g. checkForTcpDevice's own probe, which can still
  # overlap mavros's own connection attempt for a moment right at the
  # "not yet active" -> "active" transition) used to get its own relay
  # thread spawned unconditionally, so TWO local connections could pump
  # against the SAME shared vm_conn socket from two threads at once with
  # no coordination -- interleaved reads/writes on one socket from two
  # threads, and one thread's own `finally: local_conn.close()` closing
  # vm_conn out from under the other. Surfaced as mavros's own tcp0 link
  # dying with "Connection reset by peer" / "terminate called ... Resource
  # deadlock avoided" shortly after every launch attempt. Only one local
  # connection is ever relayed at a time now -- a second one arriving while
  # the first is still active is closed immediately instead of started.
  _local_active_lock = threading.Lock()

  @classmethod
  def ensure_started(cls, logger):
    with cls._start_lock:
      if cls._started:
        return
      cls._started = True
      threading.Thread(target=cls._vmAcceptLoop, args=(logger,), daemon=True).start()
      threading.Thread(target=cls._localAcceptLoop, args=(logger,), daemon=True).start()

  @classmethod
  def _vmAcceptLoop(cls, logger):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Explicit, not inherited -- nepi_sdk/rospy sets a process-global
    # socket.setdefaulttimeout(60) (same gotcha sim_bridge_node.py's own
    # bridgeServerLoop already documents), which every plain socket.socket()
    # call in this SAME process silently inherits. Confirmed live
    # (2026-09-08): without this, srv.accept() raised socket.timeout after
    # exactly 60s with no VM connection yet, and since that exception
    # wasn't caught, it killed this entire thread permanently -- the VM's
    # mavlink_relay_vm.py then had nothing left to dial into for the rest
    # of drivers_mgr's life, with no error visible anywhere except this
    # thread's own now-silent death.
    srv.settimeout(None)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
      srv.bind(('0.0.0.0', cls.DEVICE_LISTEN_PORT))
      srv.listen(1)
    except Exception as e:
      logger.log_warn("SitlMavlinkRelay: could not bind VM-facing port %d: %s" %
                       (cls.DEVICE_LISTEN_PORT, str(e)))
      return
    logger.log_info("SitlMavlinkRelay: listening for the VM's mavlink_relay_vm.py on port %d" %
                     cls.DEVICE_LISTEN_PORT)
    while True:
      try:
        conn, addr = srv.accept()
      except Exception as e:
        # Self-healing, not fatal -- see this method's own settimeout(None)
        # comment for why a bare accept() here used to die permanently
        # instead of just logging and continuing.
        logger.log_warn("SitlMavlinkRelay: VM-facing accept() error (continuing): " + str(e))
        time.sleep(1.0)
        continue
      conn.settimeout(None)
      logger.log_info("SitlMavlinkRelay: VM connected from " + str(addr))
      with cls._vm_conn_lock:
        old = cls._vm_conn
        cls._vm_conn = conn
      if old is not None:
        try:
          old.close()
        except Exception:
          pass

  @classmethod
  def _localAcceptLoop(cls, logger):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.settimeout(None)  # see _vmAcceptLoop's own comment for why
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
      srv.bind(('127.0.0.1', cls.SITL_LOCAL_PORT))
      srv.listen(1)
    except Exception as e:
      logger.log_warn("SitlMavlinkRelay: could not bind local port %d: %s" %
                       (cls.SITL_LOCAL_PORT, str(e)))
      return
    logger.log_info("SitlMavlinkRelay: listening for local clients on 127.0.0.1:%d" %
                     cls.SITL_LOCAL_PORT)
    while True:
      try:
        conn, addr = srv.accept()
      except Exception as e:
        logger.log_warn("SitlMavlinkRelay: local accept() error (continuing): " + str(e))
        time.sleep(1.0)
        continue
      conn.settimeout(None)
      threading.Thread(target=cls._relayLocalConn, args=(conn, logger), daemon=True).start()

  @classmethod
  def _relayLocalConn(cls, local_conn, logger):
    # Exclusive, non-blocking -- see _local_active_lock's own comment for
    # the crash this prevents. A second local connection arriving while
    # one is already being relayed is closed immediately rather than ever
    # touching vm_conn: it's either a redundant probe (harmless to drop --
    # checkForTcpDevice tries again in ~2s regardless) or, in the very rare
    # case it's actually mavros reconnecting while a stale probe hadn't
    # finished yet, mavros itself retries its own connection on failure.
    if not cls._local_active_lock.acquire(blocking=False):
      try:
        local_conn.close()
      except Exception:
        pass
      return
    try:
      with cls._vm_conn_lock:
        vm_conn = cls._vm_conn
      if vm_conn is None:
        # No VM side yet -- block harmlessly until the local peer's own
        # timeout gives up and closes (see this class's own docstring for
        # why never delivering data here is the correct behavior, not a
        # bug).
        try:
          local_conn.recv(1)
        except Exception:
          pass
        return
      vm_conn_failed = cls._pump(local_conn, vm_conn)
      # Only tear down vm_conn when IT actually failed -- confirmed live
      # (2026-09-08) as the real cause of mavros's own "Connection reset by
      # peer" / "Resource deadlock avoided" crash, which the exclusivity
      # lock above did NOT fix on its own: checkForTcpDevice's own probe
      # connects, reads one byte, then closes itself every ~2s BY DESIGN
      # (see that method's own comment) -- that is the LOCAL side ending
      # normally, not vm_conn failing, but the previous version of this
      # method treated ANY pump() return as "vm_conn is dead" regardless of
      # which side actually closed, so every single routine probe cycle
      # destroyed the one shared vm_conn -- including out from under
      # mavros's own, completely unrelated, still-healthy connection,
      # forcing mavlink_relay_vm.py's own reconnect and handing mavros a
      # mid-stream reset. _pump now reports which side actually died so
      # only a genuine vm_conn failure clears it here.
      if vm_conn_failed:
        with cls._vm_conn_lock:
          if cls._vm_conn is vm_conn:
            cls._vm_conn = None
        try:
          vm_conn.close()
        except Exception:
          pass
    except Exception as e:
      logger.log_warn("SitlMavlinkRelay: local relay error: " + str(e))
    finally:
      try:
        local_conn.close()
      except Exception:
        pass
      cls._local_active_lock.release()

  @staticmethod
  def _pump(a, b):
    # Returns True only if `b` (vm_conn, by this class's own calling
    # convention -- a is always the local connection) is the side that
    # actually failed; False if only `a` (the local side) closed/errored.
    # See _relayLocalConn's own comment for why this distinction is the
    # actual fix -- a plain "did pump() return" signal can't tell a normal,
    # expected local-side close (e.g. a probe) apart from a real vm_conn
    # failure, and conflating them was killing vm_conn (and anything else
    # relaying through it) on every routine probe cycle.
    a.setblocking(False)
    b.setblocking(False)
    while True:
      r, _, x = select.select([a, b], [], [a, b], 5.0)
      if b in x:
        return True
      if a in x:
        return False
      for s in r:
        try:
          data = s.recv(4096)
        except BlockingIOError:
          continue
        except Exception:
          return s is b
        if not data:
          return s is b
        dst = b if s is a else a
        try:
          dst.sendall(data)
        except Exception:
          return dst is b


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
  # 5771 is MAVProxy's dedicated --out port -- sitl_gazebo/gazebo_sitl on the dev VM always
  # expose this via --out=tcpin:0.0.0.0:5771, alongside MAVProxy's own primary connection
  # on SITL's raw port 5760 (used for --console/--map). Deliberately NOT also trying 5760
  # here: with mavros and MAVProxy both live at once (5771 and 5760), discovery alternates
  # between them every ~1Hz cycle, each time concluding "the other one died" and killing
  # the node it just launched -- an infinite thrash loop. One canonical port avoids that.
  sitl_addr_list = ['127.0.0.1']
  sitl_tcp_port_list = ['5771']

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
      # Idempotent -- see SitlMavlinkRelay's own docstring for why this is
      # needed at all now (the reverse SSH tunnel that used to make the
      # VM's real SITL MAVLink port transparently reachable at
      # 127.0.0.1:5771 on this device is gone). Safe to call every
      # discoveryFunction tick: only actually starts its two listener
      # threads once.
      SitlMavlinkRelay.ensure_started(self.logger)
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
    # Returns True if the device's mavlink AND ardupilot processes are both
    # still alive and its path is still present.
    #
    # Originally checked mavlink_subproc alone -- confirmed live (2026-08-11)
    # that this misses a real, repeatable failure mode: mavros can stay up
    # and perfectly healthy while the ardupilot RBX node itself exits (the
    # exact mechanism wasn't pinned down -- rospy's own signal_shutdown
    # [atexit] with no exception logged beforehand, i.e. a clean shutdown
    # from *something*, not a crash) minutes after a successful launch. With
    # only the mavlink check, that leaves active_devices_dict believing
    # everything is fine forever after -- nothing purges the stale entry, so
    # nothing ever retries the launch, and the RBX device panel that
    # appeared briefly in the RUI's Devices list never comes back on its
    # own. Checking ardu_subproc (and fake_gps, if this launch enabled it)
    # the same way closes that gap: any one of the three dying now purges
    # and retries the whole device, exactly like a mavlink death always did.
    active = True
    if path_str not in self.active_devices_dict.keys():
      return False

    device_entry = self.active_devices_dict[path_str]
    mavlink_subproc = device_entry["mavlink_subproc"]
    ardu_subproc = device_entry.get("ardu_subproc")
    fgps_subproc = device_entry.get("fgps_subproc")

    purge_node = False
    # Check that the mavlink process is still running
    if mavlink_subproc is None or mavlink_subproc.poll() is not None:
      self.logger.log_warn("Mavlink process for " + path_str + " is no longer running... purging from managed list")
      purge_node = True
    # Check that the ardupilot RBX node itself is still running -- see this
    # method's own docstring for why this can't be inferred from mavlink
    # alone.
    elif ardu_subproc is None or ardu_subproc.poll() is not None:
      self.logger.log_warn("Ardupilot RBX node for " + path_str + " is no longer running... purging from managed list")
      purge_node = True
    # Only present when fake_gps was enabled for this launch -- absence is
    # normal, not a failure.
    elif fgps_subproc is not None and fgps_subproc.poll() is not None:
      self.logger.log_warn("Fake GPS node for " + path_str + " is no longer running... purging from managed list")
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
    # LD_PRELOAD needed here, not a general drivers_mgr/launchDriverNode fix --
    # confirmed live (2026-09-08): rbx_ardupilot_node.py crashed on its own
    # `from nepi_api.device_if_rbx import RBXRobotIF` (-> nepi_pc -> `import
    # open3d`) with "libgomp.so.1: cannot allocate memory in static TLS
    # block" every single time it was launched this way, while the EXACT
    # SAME import in a plain `python3 -c` shell succeeded fine -- narrowed
    # to import ORDER: this script's own `import cv2` (line ~28, well before
    # device_if_rbx) already claims libgomp's static TLS slot via OpenCV's
    # own OpenMP/BLAS backend on this aarch64 build, leaving none for
    # open3d's own later, separate use of the same library. Preloading
    # libgomp before the interpreter even starts guarantees it gets the
    # first (and only, since it's the same library either way) claim,
    # regardless of which importer asks for it first. subprocess.Popen
    # inside launchDriverNode has no env= override, so it inherits
    # os.environ as-is -- set LD_PRELOAD here, restore it right after the
    # spawn call returns (the child already captured its own env copy at
    # fork/exec time) so this doesn't leak into unrelated drivers_mgr spawns
    # (mavros's own launch just above, or any other driver type) that never
    # needed it and shouldn't carry it silently forever.
    ld_preload_key = 'LD_PRELOAD'
    prev_ld_preload = os.environ.get(ld_preload_key)
    os.environ[ld_preload_key] = '/lib/aarch64-linux-gnu/libgomp.so.1'
    try:
      [success, msg, ardu_subproc] = nepi_drvs.launchDriverNode(file_name, ardu_node_name)
    finally:
      if prev_ld_preload is None:
        os.environ.pop(ld_preload_key, None)
      else:
        os.environ[ld_preload_key] = prev_ld_preload

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
    try:
      result = sock.connect_ex((ip_addr_str, int(ip_port_str)))
      if result == 0:
        # A successful connect is NOT proof the SITL is there. These addresses
        # are reached through a reverse SSH tunnel, where the listening socket
        # belongs to the local sshd: sshd accepts first and only then tries to
        # reach the far end, so connect() succeeds against a completely dead
        # simulator. rbx_sim_discovery.py already learned this and requires a
        # real reply from its heartbeat listener; this probe never got the same
        # treatment, and the consequences were severe.
        #
        # With connect-only, a dead SITL looked present forever, so discovery
        # relaunched mavros into a dead endpoint in an endless ~2 s loop:
        # mavros hit "tcp0: receive: End of file" and aborted with
        # "std::system_error: Resource deadlock avoided", checkOnDevice saw the
        # dead mavlink process and purged, taking the ardupilot RBX node with
        # it, then relaunched both. The RBX node never survived long enough to
        # finish RBXRobotIF init, so it never registered its own subscribers --
        # /rbx/setup_action had "Subscribers: None" -- and every LAUNCH/TAKEOFF
        # published from the RUI went nowhere at all, silently. Quick one-shot
        # motor commands could still land in a lucky window, which is exactly
        # the reported symptom: "the manual motor commands seem to work, but
        # the command to actually make it fly in the air doesn't". Autonomous
        # controls then stay locked too, since autonomousControlsReady()
        # requires takeoff_complete.
        #
        # Requiring actual MAVLink traffic distinguishes the two. A live
        # SITL/MAVProxy endpoint streams heartbeats continuously, so bytes
        # arrive within the socket timeout without us sending anything, and
        # every frame starts with a MAVLink magic byte (0xFE v1 / 0xFD v2).
        # A tunnel to a dead far end gives EOF (b"") or a timeout instead.
        try:
          data = sock.recv(1)
        except Exception:
          data = b""
        if data[:1] in (b'\xfe', b'\xfd'):
          found_device = True
          self.logger.log_warn("Mavlink_AD: Found live MAVLink endpoint on " + ip_addr_str + ":" + ip_port_str)
        elif len(data) > 0:
          # Mid-frame bytes: still a live stream, just not aligned to a frame
          # start. Accept it -- the alternative is rejecting a working SITL
          # over probe timing.
          found_device = True
          self.logger.log_warn("Mavlink_AD: Found live TCP stream (non-magic first byte) on " + ip_addr_str + ":" + ip_port_str)
        else:
          self.logger.log_warn("Mavlink_AD: Port " + ip_addr_str + ":" + ip_port_str +
                               " accepted the connection but sent no MAVLink data -- treating as absent "
                               "(a reverse-tunnel port accepts even when the far-end simulator is gone)")
      else:
        self.logger.log_warn("Mavlink_AD: Did not find TCP device on ip address: " + ip_addr_str + " port: " + ip_port_str)
    except Exception as e:
      self.logger.log_warn("Mavlink_AD: TCP probe failed for " + ip_addr_str + ":" + ip_port_str + ": " + str(e))
    finally:
      try:
        sock.close()
      except Exception:
        pass
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
