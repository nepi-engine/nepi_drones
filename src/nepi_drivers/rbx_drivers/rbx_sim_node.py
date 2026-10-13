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

# RBX driver node for a Gazebo-simulated rover (RBX_SIM), following
# rbx_ardupilot_node.py's RBXRobotIF integration pattern.
#
# LIFECYCLE. nepi_app_sim_connector starts and stops this node, not
# drivers_mgr discovery: when a simulator service (nepi_simulations'
# nepi_gazebo.sh or nepi_mujoco.sh) reports a robot config running whose
# nepi_robot_config.yaml names nepi_driver: rbx_sim, the app writes this
# node's drv_dict and launches it, and when the service reports anything
# other than running (Kill, a
# crash, a stop on the VM) the app kills it. So the robot is listed in
# Devices -> Robots exactly as long as the simulated robot exists. The old
# rbx_sim_discovery.py heartbeat path (port 9022) is no longer used.
#
# BRIDGE. The simulator runs on a separate machine, so this node talks to the
# VM's bridge over the network, on DEVICE_DICT's bridge_port, UDP and TCP
# both: Gazebo's nepi_gazebo_bridge.py, or the bridge built into MuJoCo's
# nepi_mujoco_sim.py, which speaks the same protocol. The VM always speaks first
# (it sends telemetry to this node's UDP port, and connects to its TCP port),
# so nothing on the VM has to accept an inbound connection -- a firewalled
# VM (Windows + WSL2) still works. Every message is one JSON object; on TCP
# they are newline-delimited.
#
#   UDP  VM -> device  {"type":"telemetry","seq","x","y","yaw","linear_x","angular_z","stamp"}
#                      10 Hz, from the simulated robot's pose. Receiving it is what
#                      "bridge connected" means (see SimBridgeLink.is_connected).
#   UDP  device -> VM  {"type":"velocity","seq","linear_x","linear_y","angular_z"}
#                      20 Hz, ALWAYS, zeros when idle, sent to wherever the
#                      latest telemetry came from. The VM zeroes the rover
#                      when this stream stops for 250 ms, so a lost link stops
#                      the robot rather than leaving it driving.
#   TCP  VM -> device  {"type":"image","camera","format":"jpeg","data":"<base64>","stamp"}
#   TCP  device -> VM  {"type":"reset"}
#
# UDP carries the two streams where only the newest value matters and a
# late packet is worse than a lost one; each carries a seq so a stale or
# duplicated packet is dropped. Images and one-shot commands stay on TCP,
# where a camera frame cannot be half-delivered and a reset cannot be lost.
# Keeping them apart also means a camera frame can never delay a position
# update, which it could when everything shared one TCP stream.
#
# CAPABILITIES come from the robot config the app passes in drv_dict
# (ROBOT_CONFIG, the folder's nepi_robot_config.yaml): a flag that is false
# there hands RBXRobotIF a None function, so the control is not offered.
#
# CONTROL. The rover only understands instantaneous velocity, so this node
# runs its own closed-loop 2D controller (gotoControlCb) toward the
# RBXRobotIF-supplied position/yaw target until within the RBX error bounds.
# No ARM/DISARM, flight modes, battery, or WGS84 location: those are left
# empty/None and RBXRobotIF reports the matching has_* capabilities False.
# Home is local ENU x/y/z meters carried over RBXRobotIF's GeoPoint-shaped
# home plumbing (see getHome/setHome).
#
# Not carried over from the sim_container version yet, because each one
# respawned or rebuilt the robot model and the Gazebo service now owns the
# model (it spawns the robot folder's model.sdf): the camera offset/tilt/FOV
# Settings, the environment Setting and REFRESH_ENVIRONMENT (environments are
# whole .world files chosen at Deploy now), and wheel independence (needs the
# crab-steer plugin, which the ROBOT_CONFIGS rover does not load).

import base64
import copy
import json
import math
import socket
import threading
import time

import numpy as np
import cv2

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_img

from std_msgs.msg import UInt32, String
from sensor_msgs.msg import Image

from nepi_interfaces.msg import AxisControls, ErrorBounds
from geographic_msgs.msg import GeoPoint

from nepi_api.device_if_rbx import RBXRobotIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'RBX_SIM' # Use in display menus
FILE_TYPE = 'NODE'

FACTORY_BRIDGE_PORT = 9023


#########################################
# Bridge Link
#########################################

class SimBridgeLink(object):
  """UDP + TCP link to the VM's simulator bridge. See the module comment
  for the wire format. Plain sockets and threads, no ROS, so it can be
  exercised without a NEPI install."""

  # No telemetry for this long reads as disconnected. Telemetry arrives at
  # 10 Hz, so this rides out a few lost packets without flapping.
  LINK_TIMEOUT_SEC = 1.0
  SEQ_RESTART_GAP = 50
  BIND_RETRY_SEC = 3.0
  RECV_TIMEOUT_SEC = 0.5

  def __init__(self, port, telemetryFunction, imageFunction, logInfoFunction, logWarnFunction):
    self.port = int(port)
    self.telemetryFunction = telemetryFunction
    self.imageFunction = imageFunction
    self.logInfo = logInfoFunction
    self.logWarn = logWarnFunction

    self.running = True
    self.lock = threading.Lock()
    self.udp_sock = None
    self.udp_peer = None
    self.rx_seq = None
    self.tx_seq = 0
    self.last_rx_time = 0.0
    self.tcp_server = None
    self.tcp_conn = None
    self.tcp_send_lock = threading.Lock()

    for target in (self._udpLoop, self._tcpLoop):
      thread = threading.Thread(target = target)
      thread.daemon = True
      thread.start()

  def is_connected(self):
    with self.lock:
      last_rx_time = self.last_rx_time
    return (time.monotonic() - last_rx_time) < self.LINK_TIMEOUT_SEC

  def has_data_connection(self):
    with self.lock:
      return self.tcp_conn is not None

  def get_peer(self):
    with self.lock:
      return self.udp_peer

  def send_velocity(self, linear_x, angular_z, linear_y = 0.0):
    with self.lock:
      sock = self.udp_sock
      peer = self.udp_peer
      self.tx_seq += 1
      seq = self.tx_seq
    if not self.running or sock is None or peer is None:
      return False
    packet = {'type': 'velocity', 'seq': seq,
              'linear_x': float(linear_x), 'linear_y': float(linear_y), 'angular_z': float(angular_z)}
    try:
      sock.sendto(json.dumps(packet).encode(), peer)
      return True
    except Exception as e:
      if self.running:
        self.logWarn("Failed to send velocity to sim bridge: " + str(e))
      return False

  def send_line(self, line_dict, description):
    with self.lock:
      conn = self.tcp_conn
    if conn is None:
      self.logWarn(description + " dropped -- sim bridge data connection is not up")
      return False
    try:
      with self.tcp_send_lock:
        conn.sendall((json.dumps(line_dict) + '\n').encode())
      return True
    except Exception as e:
      self.logWarn("Failed to send " + description.lower() + " to sim bridge: " + str(e))
      return False

  def close(self):
    # The node's timers keep firing after its on_shutdown hook runs, so the
    # sockets are cleared here and send_velocity stops once running is False.
    self.running = False
    with self.lock:
      socks = [self.udp_sock, self.tcp_server, self.tcp_conn]
      self.udp_sock = None
      self.udp_peer = None
      self.tcp_server = None
      self.tcp_conn = None
    for sock in socks:
      if sock is not None:
        self._closeSocket(sock)

  def _closeSocket(self, sock):
    # shutdown before close: another thread may be blocked in recv on this
    # socket, and a bare close neither wakes it nor tells the far end.
    try:
      sock.shutdown(socket.SHUT_RDWR)
    except Exception:
      pass
    try:
      sock.close()
    except Exception:
      pass

  def _bind(self, sock_type):
    while self.running:
      sock = socket.socket(socket.AF_INET, sock_type)
      try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', self.port))
        return sock
      except Exception as e:
        sock.close()
        self.logWarn("Sim bridge " + ('UDP' if sock_type == socket.SOCK_DGRAM else 'TCP') +
                     " bind on port " + str(self.port) + " failed: " + str(e))
        time.sleep(self.BIND_RETRY_SEC)
    return None

  def _udpLoop(self):
    sock = self._bind(socket.SOCK_DGRAM)
    if sock is None:
      return
    sock.settimeout(self.RECV_TIMEOUT_SEC)
    with self.lock:
      self.udp_sock = sock
    self.logInfo("Listening for sim bridge telemetry on UDP port " + str(self.port))
    while self.running:
      try:
        data, addr = sock.recvfrom(65535)
      except socket.timeout:
        continue
      except Exception:
        if not self.running:
          return
        continue
      try:
        msg = json.loads(data.decode())
      except Exception as e:
        self.logWarn("Bad packet from sim bridge: " + str(e))
        continue
      if msg.get('type') != 'telemetry':
        continue
      seq = msg.get('seq')
      with self.lock:
        if addr != self.udp_peer:
          # A new sender (first packet, or the VM bridge restarted on a new
          # port) starts its own sequence.
          if self.udp_peer is not None:
            self.logInfo("Sim bridge telemetry now coming from " + str(addr))
          self.udp_peer = addr
          self.rx_seq = None
        elif (seq is not None and self.rx_seq is not None and
              self.rx_seq - self.SEQ_RESTART_GAP < seq <= self.rx_seq):
          # Stale or duplicated. One far behind is a restarted bridge
          # counting from 1 again, not a late packet, and is taken.
          continue
        if seq is not None:
          self.rx_seq = seq
        self.last_rx_time = time.monotonic()
      self.telemetryFunction(msg)

  def _tcpLoop(self):
    server = self._bind(socket.SOCK_STREAM)
    if server is None:
      return
    server.listen(1)
    with self.lock:
      self.tcp_server = server
    self.logInfo("Listening for sim bridge data connection on TCP port " + str(self.port))
    while self.running:
      try:
        conn, addr = server.accept()
      except Exception:
        if not self.running:
          return
        continue
      conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
      with self.lock:
        old_conn = self.tcp_conn
        self.tcp_conn = conn
      if old_conn is not None:
        # A reconnecting VM replaces a connection that may be half-open.
        self._closeSocket(old_conn)
      self.logInfo("Sim bridge data connection from " + str(addr[0]))
      thread = threading.Thread(target = self._tcpReadLoop, args = (conn,))
      thread.daemon = True
      thread.start()

  def _tcpReadLoop(self, conn):
    buf = b''
    while self.running:
      try:
        data = conn.recv(65536)
      except Exception:
        data = b''
      if not data:
        break
      buf += data
      while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        if not line.strip():
          continue
        try:
          msg = json.loads(line)
        except Exception as e:
          self.logWarn("Bad line from sim bridge: " + str(e))
          continue
        if msg.get('type') == 'image':
          self.imageFunction(msg)
    with self.lock:
      if self.tcp_conn is conn:
        self.tcp_conn = None
    self._closeSocket(conn)
    if self.running:
      self.logWarn("Sim bridge data connection lost -- waiting for the VM to reconnect")


#########################################
# Node Class
#########################################

class SimNode:

  # The four always-live camera views the VM bridge relays (robot = the
  # rover's own first-person camera, scene = its chase camera; color and a
  # colorized depth view of each), published under
  # <node>/color_2d_image/<view>, which is where nepi_app_sim_connector's
  # view mirrors and NepiDeviceRBX.js's image list look for them.
  CAMERA_VIEWS = ("robot_color", "scene_color", "robot_depth", "scene_depth")

  # Settings, in the nepi_controls init-dict form. On/off settings are
  # TRUE/FALSE Selections because NepiDeviceRBX-Controls.js treats any value
  # other than "FALSE" as on.
  INIT_SETTINGS = dict(
    max_linear_speed_mps = {'type': 'Float', 'bounds': [0.05, 5.0], 'default': 0.5},
    max_angular_rate_dps = {'type': 'Float', 'bounds': [5.0, 180.0], 'default': 45.0},
    # Gates every goto here in the driver, not only in the RUI, so a client
    # that bypasses the RUI cannot do what was turned off either.
    autonomous_movement_enabled = {'type': 'Selection', 'options': ['TRUE', 'FALSE'], 'default': 'TRUE'},
    # Whether manual motor commands also drive the rover's body, or only
    # move the motor readouts.
    move_with_manual_enabled = {'type': 'Selection', 'options': ['TRUE', 'FALSE'], 'default': 'TRUE'},
  )

  # What each setup-action name a robot config may list does here. RESET is
  # the robot-config spelling, RESET_SIM the one this driver has always
  # reported; both teleport the rover back to its start pose.
  SETUP_ACTION_NAMES = ("RESET_SIM", "RESET", "RETURN_HOME")
  DEFAULT_SETUP_ACTIONS = ["RESET_SIM", "RETURN_HOME"]
  DEFAULT_MOTOR_COUNT = 4

  RBX_STATES = []
  RBX_MODES = []
  RBX_GO_ACTIONS = []

  GO_HOME_TIMEOUT_SEC = 60.0
  GO_HOME_POLL_INTERVAL_SEC = 0.2

  CONTROLLER_RATE_HZ = 20
  NAVPOSE_UPDATE_RATE = 10

  # Manual motor ratios -> the same linear/angular command gotoControlCb
  # sends, by differential-drive kinematics. Even motor indices are the left
  # side, odd the right (a 4-motor rover is front_left, front_right,
  # rear_left, rear_right). MOTOR_WHEEL_BASE_M approximates generic_rover's
  # track width.
  MOTOR_WHEEL_BASE_M = 0.4

  # Closed-loop goto controller: proportional gains, plus a turn-in-place
  # gate so the rover rotates toward the target bearing before driving.
  GOTO_KP_LIN = 0.5       # (m/s per m of distance error)
  GOTO_KP_ANG = 1.5       # (rad/s per rad of heading error)
  GOTO_TURN_GATE_RAD = math.radians(30.0)
  # Stop inside half the RBX error bounds so the interface's own convergence
  # check passes cleanly instead of hovering at the edge.
  GOTO_TOL_FRACTION = 0.5
  FACTORY_GOTO_TOL_M = 1.0
  FACTORY_GOTO_TOL_RAD = math.radians(1.0)

  # RBXRobotIF's 25 s factory cmd_timeout is too short for a
  # non-holonomic rover that may need two ~180 degree turns plus the drive
  # itself at the factory 0.5 m/s.
  GOTO_CMD_TIMEOUT_SEC = 60

  # RBXRobotIF's factory error bounds (2.0 m) make any goto under 2 m count as
  # already reached: its own convergence check passes on the starting error,
  # and GOTO_TOL_FRACTION of 2.0 m gives this driver a 1.0 m arrival distance,
  # so a 1 m goto after a reset never moved. 0.5 m keeps short moves real
  # (0.25 m driver arrival); rotation and stabilize time stay at factory.
  GOTO_MAX_ERROR_M = 0.5
  GOTO_MAX_ERROR_DEG = 2.0
  GOTO_STABILIZED_SEC = 1.0

  DRV_DICT_READ_RETRIES = 5
  DRV_DICT_READ_RETRY_SEC = 0.5

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  drv_dict = dict()

  def __init__(self):
    ####  NODE Initialization ####
    nepi_sdk.init_node(name = self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()

    ##############################
    # Create Msg Class
    self.msg_if = MsgIF()
    self.msg_if.pub_info("Starting Node Initialization Processes")

    ##############################
    # drv_dict, written by nepi_app_sim_connector just before launching this
    # node. Read from the same absolute path the app writes (base namespace
    # + node name), not '~drv_dict': the node is launched with only __name
    # remapped, so a relative read resolves somewhere nothing wrote to.
    drv_dict_param_name = nepi_sdk.create_namespace(self.base_namespace, self.node_name + '/drv_dict')
    last_error = None
    for attempt in range(self.DRV_DICT_READ_RETRIES):
      self.drv_dict = nepi_sdk.get_param(drv_dict_param_name, dict())
      try:
        self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
        self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
        self.bridge_port = int(self.drv_dict['DEVICE_DICT'].get('bridge_port', FACTORY_BRIDGE_PORT))
        last_error = None
        break
      except Exception as e:
        last_error = e
        nepi_sdk.sleep(self.DRV_DICT_READ_RETRY_SEC)
    if last_error is not None:
      self.msg_if.pub_warn("Failed to load Device Dict " + str(last_error))
      nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
      return
    self.robot_config = self.drv_dict.get('ROBOT_CONFIG', None)
    if not isinstance(self.robot_config, dict):
      self.robot_config = None

    ##############################
    # State
    self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    self.have_telemetry = False
    self.goto_target = None       # dict(x_m, y_m, yaw_deg or None) in sim ENU world frame
    self.goto_target_lock = threading.Lock()
    self.stop_triggered = False
    self.home_x_m = 0.0
    self.home_y_m = 0.0
    self.home_z_m = 0.0
    self.rbx_if = None

    motor_count = self.getConfigInt('motor_count', self.DEFAULT_MOTOR_COUNT)
    self.motor_ratios = [0.0] * max(0, motor_count)

    setup_actions = self.DEFAULT_SETUP_ACTIONS
    if self.robot_config is not None:
      setup_actions = [str(a) for a in self.robot_config.get('setup_actions', [])
                       if str(a) in self.SETUP_ACTION_NAMES]
    self.setup_actions = list(setup_actions)

    ##############################
    # Camera views, published as plain Image topics under this node.
    self.image_topic_name = nepi_sdk.create_namespace(self.node_namespace, 'color_2d_image')
    self.image_pubs = dict()
    for view in self.CAMERA_VIEWS:
      self.image_pubs[view] = nepi_sdk.create_publisher(self.image_topic_name + "/" + view, Image, queue_size = 1)

    ##############################
    # Settings
    self.settings_dict = self.initSettingsDict()

    self.axis_controls = AxisControls()
    self.axis_controls.x = True
    self.axis_controls.y = True
    self.axis_controls.z = False
    self.axis_controls.roll = False
    self.axis_controls.pitch = False
    self.axis_controls.yaw = True

    ##############################
    # Bridge to the VM
    self.link = SimBridgeLink(self.bridge_port,
                              telemetryFunction = self.processTelemetry,
                              imageFunction = self.processImage,
                              logInfoFunction = self.msg_if.pub_info,
                              logWarnFunction = lambda msg: self.msg_if.pub_warn(msg, throttle_s = 5.0))

    ##############################
    # Launch the NEPI RBX interface
    self.msg_if.pub_info("Launching NEPI RBX interface...")
    self.device_info_dict = dict(device_name = self.device_name,
                                 path = self.device_path,
                                 serial_number = "",
                                 hw_version = "",
                                 sw_version = "")

    has_motors = len(self.motor_ratios) > 0
    self.rbx_if = RBXRobotIF(device_info = self.device_info_dict,
                             # 'simulator', not the 'control_system' default:
                             # nepi_app_sim_connector finds its robot by this.
                             data_source_description = 'simulator',
                             getSettingsFunction = self.getSettingsFunction,
                             setSettingFunction = self.setSettingFunction,
                             axisControls = self.axis_controls,
                             getBatteryPercentFunction = None,
                             states = self.RBX_STATES,
                             getStateIndFunction = self.getStateInd,
                             setStateIndFunction = self.setStateInd,
                             modes = self.RBX_MODES,
                             getModeIndFunction = self.getModeInd,
                             setModeIndFunction = self.setModeInd,
                             checkStopFunction = self.checkStopFunction,
                             setup_actions = self.setup_actions,
                             setSetupActionIndFunction = self.setSetupActionInd if self.setup_actions else None,
                             go_actions = self.RBX_GO_ACTIONS,
                             setGoActionIndFunction = self.setGoActionInd,
                             manualControlsReadyFunction = self.manualControlsReady,
                             getMotorControlRatios = self.getMotorControlRatios if has_motors else None,
                             setMotorControlRatio = self.setMotorControlRatio if has_motors else None,
                             autonomousControlsReadyFunction = self.autonomousControlsReady,
                             getHomeFunction = self.getHome if self.hasCapability('has_set_home') else None,
                             setHomeFunction = self.setHome if self.hasCapability('has_set_home') else None,
                             goHomeFunction = self.returnHomeAction if self.hasCapability('has_go_home') else None,
                             goStopFunction = self.goStop if self.hasCapability('has_go_stop') else None,
                             gotoPoseFunction = self.gotoPose if self.hasCapability('has_goto_pose') else None,
                             gotoPositionFunction = self.gotoPosition if self.hasCapability('has_goto_position') else None,
                             gotoLocationFunction = None,
                             getNavPoseCb = self.getNavPoseCb,
                             navpose_update_rate = self.NAVPOSE_UPDATE_RATE,
                             msg_if = self.msg_if
                            )

    self.msg_if.pub_info("... RBX interface running")
    time.sleep(1)

    ## See GOTO_CMD_TIMEOUT_SEC.
    self.rbx_if.setCmdTimeoutCb(UInt32(data = self.GOTO_CMD_TIMEOUT_SEC))
    ## See GOTO_MAX_ERROR_M.
    self.rbx_if.setErrorBoundsCb(ErrorBounds(max_distance_error_m = self.GOTO_MAX_ERROR_M,
                                             max_rotation_error_deg = self.GOTO_MAX_ERROR_DEG,
                                             min_stabilize_time_s = self.GOTO_STABILIZED_SEC))
    ## Point the interface's image source at this rover's own robot camera,
    ## regardless of what a previous run left in config.
    self.rbx_if.setImageTopicCb(String(data = self.image_topic_name + "/robot_color"))

    ## Start the closed-loop goto controller
    controller_interval = float(1) / self.CONTROLLER_RATE_HZ
    nepi_sdk.start_timer_process(controller_interval, self.gotoControlCb)

    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.on_shutdown(self.cleanup_actions)
    nepi_sdk.spin()


  #**********************
  # Robot config

  def hasCapability(self, flag):
    # No robot config (e.g. launched by hand) keeps the full rover control
    # set this driver has always offered.
    if self.robot_config is None:
      return True
    return self.robot_config.get(flag, False) is True

  def getConfigInt(self, key, default):
    if self.robot_config is None:
      return default
    try:
      return int(self.robot_config.get(key, 0))
    except (TypeError, ValueError):
      return 0

  #**********************
  # Setting functions

  def initSettingsDict(self):
    settings_dict = nepi_controls.create_controls_dict(copy.deepcopy(self.INIT_SETTINGS))
    self.msg_if.pub_info("Initialized Settings: " + str(nepi_controls.get_values_dict(settings_dict)))
    return settings_dict

  def refreshSettingsDict(self):
    return copy.deepcopy(self.settings_dict)

  def getSettingsFunction(self):
    # A copy: SettingsIF keeps whatever this returns as its own dict and
    # mutates it in place.
    return self.refreshSettingsDict()

  def setSettingFunction(self, setting_name, setting_value):
    setting_str = setting_name + ":" + str(setting_value)
    if setting_name not in self.settings_dict.keys():
      return [False, self.node_name + " Setting name " + setting_str + " is not supported",
              copy.deepcopy(self.settings_dict)]
    if nepi_controls.get_clean_value(self.settings_dict, setting_name, setting_value) is None:
      return [False, self.node_name + " Setting data " + setting_str + " is not valid",
              copy.deepcopy(self.settings_dict)]
    self.settings_dict = nepi_controls.set_value(self.settings_dict, setting_name, setting_value)
    return [True, self.node_name + " UPDATED SETTINGS " + setting_str, copy.deepcopy(self.settings_dict)]

  def getSettingFloat(self, setting_name):
    return float(nepi_controls.get_value(self.settings_dict, setting_name))

  def getSettingEnabled(self, setting_name):
    return str(nepi_controls.get_value(self.settings_dict, setting_name)) != 'FALSE'

  ##########################
  # RBX Interface Functions

  def getStateInd(self):
    # No robot states -- RBXRobotIF still calls this unconditionally.
    return 0

  def setStateInd(self, state_ind):
    return False

  def getModeInd(self):
    return 0

  def setModeInd(self, mode_ind):
    return False

  def checkStopFunction(self):
    triggered = self.stop_triggered
    self.stop_triggered = False # Reset Stop Trigger
    return triggered

  def manualControlsReady(self):
    # A live bridge is all a direct motor command needs.
    return self.link.is_connected()

  def setMotorControlRatio(self, motor_ind, speed_ratio):
    # Only updates local state -- gotoControlCb is the single sender of
    # velocity, every tick, so a one-shot send from here would be
    # overwritten by its next idle (0, 0) anyway. -1..1: a rover's wheels
    # reverse.
    if motor_ind < 0 or motor_ind >= len(self.motor_ratios):
      self.msg_if.pub_warn("Motor control ignored: motor index " + str(motor_ind) + " out of range")
      return
    self.motor_ratios[motor_ind] = max(-1.0, min(1.0, speed_ratio))

  def getMotorControlRatios(self):
    return self.motor_ratios

  def autonomousControlsReady(self):
    # Every goto needs a live bridge (which implies fresh telemetry, so the
    # target is computed from a real current position) and autonomous
    # movement switched on.
    if not self.getSettingEnabled('autonomous_movement_enabled'):
      return False
    return self.link.is_connected()

  def goStop(self):
    self.stop_triggered = True
    self.clearGotoTarget()
    self.link.send_velocity(0.0, 0.0)
    return True

  def gotoPose(self, attitude_enu_degs):
    # Only yaw is achievable on a ground rover: hold position, turn in place.
    self.msg_if.pub_info("Recieved Pose setpoint command: " + str(attitude_enu_degs))
    with self.goto_target_lock:
      self.goto_target = {'x_m': self.navpose_dict['x_m'],
                          'y_m': self.navpose_dict['y_m'],
                          'yaw_deg': attitude_enu_degs[2]}

  def gotoPosition(self, point_enu_m, orientation_enu_deg):
    # RBXRobotIF passes an ENU offset from the current position plus an
    # absolute target orientation. z is ignored: the rover stays on the
    # ground plane.
    self.msg_if.pub_info("Recieved Position setpoint command: " + str(point_enu_m))
    with self.goto_target_lock:
      self.goto_target = {'x_m': self.navpose_dict['x_m'] + point_enu_m.x,
                          'y_m': self.navpose_dict['y_m'] + point_enu_m.y,
                          'yaw_deg': orientation_enu_deg[2]}

  def getNavPoseCb(self):
    # None until the first telemetry: NPXDeviceIF derives which NavPose
    # fields exist from the first real reading, and waits for one when this
    # returns None rather than locking in a blank dict's all-False flags.
    if not self.have_telemetry:
      return None
    return self.navpose_dict

  #######################
  ### Setup/Go-Action Functions

  def setSetupActionInd(self, action_ind):
    # action_ind is already bounds-checked by RBXRobotIF.
    action = self.setup_actions[action_ind]
    if action in ("RESET_SIM", "RESET"):
      return self.resetSimAction()
    elif action == "RETURN_HOME":
      return self.returnHomeAction()
    return False

  def setGoActionInd(self, action_ind):
    # Unreachable with empty RBX_GO_ACTIONS
    return False

  #######################
  ### Home Functions

  def getHome(self):
    # No WGS84 reference on this rover, so RBXRobotIF's GeoPoint home carries
    # local ENU x/y/z meters in its latitude/longitude/altitude fields. See
    # processTelemetry's matching latitude/longitude, which is what makes
    # "use current position as home" capture the real position.
    home = GeoPoint()
    home.latitude = self.home_x_m
    home.longitude = self.home_y_m
    home.altitude = self.home_z_m
    return home

  def setHome(self, geo_point):
    self.home_x_m = geo_point.latitude
    self.home_y_m = geo_point.longitude
    self.home_z_m = geo_point.altitude
    return True

  def returnHomeAction(self):
    # Drives home under the closed-loop controller and blocks until arrival
    # or timeout, since both callers treat the return value as the outcome.
    if not self.autonomousControlsReady():
      return False
    with self.goto_target_lock:
      self.goto_target = {'x_m': self.home_x_m, 'y_m': self.home_y_m, 'yaw_deg': None}
    start_time = nepi_utils.get_time()
    while (nepi_utils.get_time() - start_time) < self.GO_HOME_TIMEOUT_SEC:
      with self.goto_target_lock:
        reached = self.goto_target is None
      if reached:
        return True
      time.sleep(self.GO_HOME_POLL_INTERVAL_SEC)
    return False

  def resetSimAction(self):
    # An instant teleport back to the start pose, done on the VM. Motor
    # ratios are cleared too, or the next control tick would resend a stale
    # manual command and undo the reset.
    self.clearGotoTarget()
    self.motor_ratios = [0.0] * len(self.motor_ratios)
    self.link.send_velocity(0.0, 0.0)
    return self.link.send_line({'type': 'reset'}, "Reset sim")

  #######################
  ### Goto Controller Processes

  def clearGotoTarget(self):
    with self.goto_target_lock:
      self.goto_target = None

  def gotoControlCb(self, timer):
    # Closed-loop differential-drive controller: turn toward the target
    # bearing, drive when roughly aligned, then rotate to the final yaw goal.
    # A velocity command goes out on EVERY tick, zeros when idle -- the VM
    # stops the rover when this stream stops, and a lost packet is replaced
    # 50 ms later.
    with self.goto_target_lock:
      target = self.goto_target

    lin = 0.0
    ang = 0.0
    if target is not None:
      cur_x = self.navpose_dict['x_m']
      cur_y = self.navpose_dict['y_m']
      cur_yaw_rad = math.radians(self.navpose_dict['yaw_deg'])

      max_lin = self.getSettingFloat('max_linear_speed_mps')
      max_ang = math.radians(self.getSettingFloat('max_angular_rate_dps'))
      tol_m = self.FACTORY_GOTO_TOL_M
      tol_rad = self.FACTORY_GOTO_TOL_RAD
      if self.rbx_if is not None:
        tol_m = self.rbx_if.rbx_info.error_bounds.max_distance_error_m * self.GOTO_TOL_FRACTION
        tol_rad = math.radians(self.rbx_if.rbx_info.error_bounds.max_rotation_error_deg) * self.GOTO_TOL_FRACTION

      dx = target['x_m'] - cur_x
      dy = target['y_m'] - cur_y
      dist = math.hypot(dx, dy)

      if dist > tol_m:
        # Drive phase: point at the target, drive when roughly aligned
        bearing_err = self.normalizeAngle(math.atan2(dy, dx) - cur_yaw_rad)
        ang = max(-max_ang, min(max_ang, self.GOTO_KP_ANG * bearing_err))
        if abs(bearing_err) < self.GOTO_TURN_GATE_RAD:
          lin = max(0.0, min(max_lin, self.GOTO_KP_LIN * dist))
      else:
        # Final yaw phase (skipped if no yaw goal)
        yaw_err = 0.0
        if target['yaw_deg'] is not None:
          yaw_err = self.normalizeAngle(math.radians(target['yaw_deg']) - cur_yaw_rad)
        if abs(yaw_err) > tol_rad:
          ang = max(-max_ang, min(max_ang, self.GOTO_KP_ANG * yaw_err))
        else:
          # Target reached: clear (lin/ang stay 0.0 -- rover stops)
          self.clearGotoTarget()
          self.msg_if.pub_info("Goto target reached")
    elif any(self.motor_ratios) and self.getSettingEnabled('move_with_manual_enabled'):
      # No active goto -- an active manual motor command drives this same
      # tick, so there is exactly one sender.
      lin, ang = self.motorControlToVelocity()
    self.link.send_velocity(lin, ang)

  def motorControlToVelocity(self):
    left_ratios = self.motor_ratios[0::2]
    right_ratios = self.motor_ratios[1::2]
    left = sum(left_ratios) / len(left_ratios) if left_ratios else 0.0
    right = sum(right_ratios) / len(right_ratios) if right_ratios else left
    max_lin = self.getSettingFloat('max_linear_speed_mps')
    lin = (left + right) / 2.0 * max_lin
    ang = (right - left) / self.MOTOR_WHEEL_BASE_M * max_lin
    # Same turn-rate bound the goto controller respects.
    max_ang = math.radians(self.getSettingFloat('max_angular_rate_dps'))
    ang = max(-max_ang, min(max_ang, ang))
    return lin, ang

  def normalizeAngle(self, angle_rad):
    while angle_rad > math.pi:
      angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
      angle_rad += 2.0 * math.pi
    return angle_rad

  #######################
  ### Bridge Callbacks

  def processImage(self, msg):
    # One JPEG camera frame -> a raw sensor_msgs/Image on the matching view.
    try:
      image_pub = self.image_pubs.get(msg.get('camera', 'robot_color'))
      if image_pub is None:
        return
      encoded_bytes = base64.b64decode(msg['data'])
      cv2_img = cv2.imdecode(np.frombuffer(encoded_bytes, dtype = np.uint8), cv2.IMREAD_COLOR)
      if cv2_img is None:
        raise ValueError("cv2.imdecode returned None")
      image_pub.publish(nepi_img.cv2img_to_rosimg(cv2_img, encoding = "bgr8"))
    except Exception as e:
      self.msg_if.pub_warn("Failed to process camera image frame: " + str(e), throttle_s = 5.0)

  def processTelemetry(self, telem):
    # Bridge telemetry -> the navpose dict getNavPoseCb serves (published by
    # RBXRobotIF's NPXDeviceIF, and what its goto convergence checks read).
    now = nepi_utils.get_time()
    x_m = float(telem.get('x', 0.0))
    y_m = float(telem.get('y', 0.0))
    yaw_rad = float(telem.get('yaw', 0.0))
    lin_mps = float(telem.get('linear_x', 0.0))
    ang_radps = float(telem.get('angular_z', 0.0))

    # Position: sim world frame is ENU, ground vehicle so z stays 0
    self.navpose_dict['has_position'] = True
    self.navpose_dict['time_position'] = now
    self.navpose_dict['x_m'] = x_m
    self.navpose_dict['y_m'] = y_m
    self.navpose_dict['z_m'] = 0.0
    # No WGS84 fix, but RBXRobotIF's set_home_current reads these
    # regardless of has_location -- see getHome.
    self.navpose_dict['latitude'] = x_m
    self.navpose_dict['longitude'] = y_m
    self.navpose_dict['altitude_m'] = 0.0
    # Body-forward speed decomposed into the nav frame
    self.navpose_dict['x_m_per_sec'] = lin_mps * math.cos(yaw_rad)
    self.navpose_dict['y_m_per_sec'] = lin_mps * math.sin(yaw_rad)
    self.navpose_dict['z_m_per_sec'] = 0.0
    # A real velocity from the sim, so say so: NavPose.has_velocity defaults
    # False and gates x/y/z_m_per_sec on its own.
    self.navpose_dict['has_velocity'] = True
    self.navpose_dict['time_velocity'] = now

    # Orientation: flat-ground rover, only yaw is meaningful
    self.navpose_dict['has_orientation'] = True
    self.navpose_dict['time_orientation'] = now
    self.navpose_dict['roll_deg'] = 0.0
    self.navpose_dict['pitch_deg'] = 0.0
    self.navpose_dict['yaw_deg'] = math.degrees(yaw_rad)
    self.navpose_dict['yaw_deg_per_sec'] = math.degrees(ang_radps)
    self.have_telemetry = True

  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    """Stops the rover on node shutdown."""
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    self.link.send_velocity(0.0, 0.0)
    self.link.close()


#########################################
# Main
#########################################
if __name__ == '__main__':
  SimNode()
