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

# The one generic app node that hosts a SimDeviceIF instance plus the single,
# well-known TCP/JSON listener any simulator's own bridge script dials into.
# Connecting a new simulator means writing a small bridge script on the
# simulator side that speaks this protocol -- not writing new NEPI code. This
# node is simulator-agnostic: it will be pointed at other simulators with no
# code change here.
#
# This node is the ONLY place that understands the wire protocol.
# api/device_if_sim.py stays protocol-agnostic, exactly as device_if_rbx.py never
# knows what transport its drivers speak or that a bridge socket exists.
#
# Wire protocol -- newline-delimited JSON both ways on one persistent
# connection, dispatched by "type" key presence:
#
#   in  -- bare line (no "type" key): NavPose telemetry, covering the full
#          NavPose contract rather than any one vehicle's shape. Every field
#          optional, gated exactly the way NavPose.msg itself is: x_m/y_m/z_m
#          => has_position, roll_deg/pitch_deg/yaw_deg => has_orientation,
#          latitude/longitude => has_location, altitude_m => has_altitude. One
#          shape fits a wheeled ground robot (position only) and a flying robot
#          (position + orientation + location + altitude) with no per-vehicle
#          special-casing.
#   in  -- {"type":"sensor_topics","topics":[{"topic_name":...,"msg_type":...},...]}
#          The bridge announces its current live topic list; fed straight into
#          getAvailableSensorTopicsFunction's return value.
#   in  -- {"type":"environment_options","options":[...]}
#          Same idea for available_environment_options.
#   in  -- {"type":"image","topic_name":...,"data":"<base64 jpeg>","stamp":...}
#          topic_name distinguishes multiple announced cameras; omitted means
#          the currently active image topic. Only frames matching the active
#          topic are decoded and republished -- nothing subscribes to the others.
#
#   out -- {"type":"motor_control","motor_ind":N,"speed_ratio":R}
#   out -- {"type":"goto_position","x_meters":...,"y_meters":...,"z_meters":...,"yaw_deg":...}
#   out -- {"type":"goto_pose","roll_deg":...,"pitch_deg":...,"yaw_deg":...}
#   out -- {"type":"goto_location","lat":...,"long":...,"altitude_meters":...,"yaw_deg":...}
#          Field names match GotoPosition.msg / GotoPose.msg / GotoLocation.msg
#          1:1 -- no reason to invent different names on the wire.
#   out -- {"type":"go_home"} / {"type":"go_stop"}
#   out -- {"type":"setup_action","action":"<string>"} / {"type":"go_action","action":"<string>"}
#   out -- {"type":"camera_settings","view_mode":...}
#   out -- {"type":"set_active_image_topic","topic_name":...}
#   out -- {"type":"environment_option","option":...,"enabled":bool}
#   out -- {"type":"robot_config","config":"<name>"}
#          Tells the simulator which kind of robot is wanted. Its own type key,
#          following the existing type-keyed convention rather than inventing a
#          second framing.
#
# Capability timing. The contract decides capabilities once at construction and
# caches them, which is what lets a client render controls purely from the
# flags. A generic connector cannot know a specific simulator's wheel or motor
# counts, or which goto functions make sense, until after a bridge connects --
# by which point this process and its SimDeviceIF already exist. Resolution:
# the two fields the contract calls genuinely dynamic
# (available_sensor_topics, available_environment_options) are live-refreshed
# from the bridge; everything else is a per-deployment config decision, read
# from a named SIM_VEHICLE_DICT robot-config entry. Switching robot config is
# the one case that legitimately changes a robot's kind, and it re-derives the
# cached report in place through SimDeviceIF.apply_capability_profile -- see
# that method's docstring for why that is wire-safe.

import base64
import copy
import json
import socket
import threading

# nepi_api.device_if_sim (imported below) pulls in nepi_api.device_if_npx ->
# nepi_api.system_if -> open3d transitively. On this aarch64 target, open3d's
# native libs (via libgomp) grab static TLS at import time, and if cv2 (also
# native, also TLS-hungry) has already been imported first, open3d's later
# import fails outright: "ImportError: .../libgomp.so.1: cannot allocate
# memory in static TLS block" -- confirmed by reproducing the exact failure
# standalone and confirming the fix is import order, not a code bug. Importing
# device_if_sim (and therefore open3d) before cv2 reserves its TLS slots
# first and avoids the exhaustion. Do not reorder cv2 above this without
# re-verifying against that failure.
from nepi_api.device_if_sim import SimDeviceIF

import numpy as np
import cv2

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_img

from std_msgs.msg import Bool, Empty, String
from sensor_msgs.msg import Image
from geographic_msgs.msg import GeoPoint

from nepi_interfaces.msg import AxisControls, DeviceRBXStatus

from nepi_api.messages_if import MsgIF

PKG_NAME = 'SIM_CONNECTOR'

# Listen port a simulator's bridge script dials into. Configurable per
# deployment via the params yaml.
FACTORY_LISTEN_PORT = 9030

# Factory robot-config profile: capability-empty, which renders no controls at
# all. That is the intended safe default, not a broken state -- an operator
# picks a real robot config from the selector, or edits SIM_VEHICLE_DICT.
FACTORY_ROBOT_CONFIG_NAME = 'default'
FACTORY_ROBOT_CONFIG = dict(
    description = 'No capabilities. Safe default until a robot config is selected.',
    wheel_count = 0,
    motor_count = 0,
    has_goto_position = False,
    has_goto_pose = False,
    has_goto_location = False,
    has_go_home = False,
    has_set_home = False,
    has_go_stop = False,
    setup_actions = [],
    go_actions = [],
    has_camera_view_control = False,
    available_camera_view_modes = [],
    has_environment_controls = False,
)

# Simulator discovery. Generic and simulator-agnostic on both axes:
#   - device type: a device is a candidate only if it publishes a status topic
#     of this message type, which is what "is an addressable NEPI robot device"
#     means on the wire.
#   - capability: a candidate is a simulator only if it declares itself one
#     through its own data_source_description, a first-class constructor
#     argument every device interface already publishes. A driver opts in by
#     setting one string; nothing here matches on any simulator's product name,
#     world name, or model name.
# Any future simulator driver appears in the selector by declaring the same
# thing, with no change to this app.
SIM_DEVICE_STATUS_MSG_TYPES = ['DeviceRBXStatus']
SIM_SOURCE_DESCRIPTION = 'simulator'
SIM_DISCOVERY_RATE_HZ = 1.0

# A discovered device drops off the list if its status goes quiet this long,
# so a driver node that dies stops being offered.
SIM_DEVICE_STALE_SEC = 10.0

BRIDGE_ACCEPT_BACKLOG = 1
TELEMETRY_AGE_IF_NEVER_CONNECTED = -1.0
BRIDGE_RECV_BYTES = 4096

STATUS_PUBLISH_RATE_HZ = 1.0


#########################################
# Node Class
#########################################

class NepiSimConnectorApp:

  DEFAULT_NODE_NAME = "app_sim_connector"

  sim_if = None

  def __init__(self):
    ####  APP NODE INIT SETUP ####
    nepi_sdk.init_node(name = self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()

    ##############################
    # Create Msg Class
    self.msg_if = MsgIF(log_name = self.class_name)
    self.msg_if.pub_info("Starting Node Initialization Processes")

    ##############################
    # Per-deployment robot configs. apps_mgr loads every top-level key of the
    # params yaml onto this app's own param namespace before launching it, so
    # SIM_VEHICLE_DICT is read from there the same way a driver node reads its
    # discovery-supplied DEVICE_DICT.
    vehicle_dict_ns = nepi_sdk.create_namespace(self.node_namespace, 'SIM_VEHICLE_DICT')
    self.vehicle_dict = nepi_sdk.get_param(vehicle_dict_ns, dict())
    if not isinstance(self.vehicle_dict, dict):
      self.vehicle_dict = dict()

    self.listen_port = int(self.vehicle_dict.get('listen_port', FACTORY_LISTEN_PORT))

    robot_configs = self.vehicle_dict.get('robot_configs', dict())
    if not isinstance(robot_configs, dict):
      robot_configs = dict()
    self.robot_configs = copy.deepcopy(robot_configs)
    # The capability-empty factory profile is always present as the fallback,
    # whatever the yaml says, so there is never a state with no valid selection.
    if FACTORY_ROBOT_CONFIG_NAME not in self.robot_configs:
      self.robot_configs[FACTORY_ROBOT_CONFIG_NAME] = copy.deepcopy(FACTORY_ROBOT_CONFIG)

    default_config = str(self.vehicle_dict.get('default_robot_config', FACTORY_ROBOT_CONFIG_NAME))
    if default_config not in self.robot_configs:
      default_config = FACTORY_ROBOT_CONFIG_NAME
    self.selected_robot_config = default_config
    self.profile = self.buildProfileFromConfig(default_config)

    ##############################
    # Bridge connection and live announced state -- the genuinely dynamic
    # fields, refreshed from whatever the connected bridge last sent.
    self.client_conn = None
    self.client_lock = threading.Lock()
    self.last_telemetry_time = 0.0
    self.connected_since = None
    self.available_sensor_topics = []          # [(topic_name, msg_type), ...]
    self.available_environment_options = []
    self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    self.active_image_topic = ""
    self.motor_ratios = [0.0] * int(self.profile['motor_count'])

    ##############################
    # Simulator discovery state. Populated by a timer scan of the live ROS
    # graph; never by a blocking wait.
    self.sim_scan_lock = threading.Lock()
    self.sim_device_subs = dict()      # status topic -> subscriber handle
    self.sim_device_info = dict()      # status topic -> dict(name, source, time)
    self.selected_simulator = ""

    ##############################
    # Home position state -- reused GeoPoint plumbing with its three floats
    # reinterpreted as local ENU x/y/z meters, the same reinterpretation an RBX
    # driver for a vehicle with no independent WGS84 reference already uses. A
    # deployment whose simulator does have a real WGS84 reference can keep the
    # telemetry-supplied latitude/longitude as home instead; this app does not
    # need to know which case it is.
    self.home_x_m = 0.0
    self.home_y_m = 0.0
    self.home_z_m = 0.0

    ##############################
    # Goto target state for the thin goto delegators (see device_if_sim.py's own
    # documented scope note). Stored only so a future convergence controller has
    # somewhere to read from; this app runs no convergence controller itself --
    # forwarding the setpoint and letting the simulator-side bridge or vehicle
    # model reach it is the contract's intent.
    self.goto_target_lock = threading.Lock()
    self.goto_target = None

    ##############################
    # Image republish -- decode and republish only the currently active image
    # topic, the one thing a client actually displays, on a name qualified by
    # this node's own name so a second instance on the same device cannot
    # collide with it.
    self.image_topic_name = self.node_name + "/color_2d_image"
    self.image_pub = nepi_sdk.create_publisher(self.image_topic_name, Image, queue_size = 1)

    ##############################
    # Launch the generic SimDeviceIF. Every callback below is a thin
    # wire-protocol sender or getter; device_if_sim.py never knows there is a
    # TCP bridge underneath.
    device_info = dict(device_name = self.node_name, path = "",
                       serial_number = "", hw_version = "", sw_version = "")

    self.sim_if = SimDeviceIF(
        device_info = device_info,
        getNavPoseCb = self.getNavPoseCb,
        getAvailableSensorTopicsFunction = self.getAvailableSensorTopics,
        getAvailableEnvironmentOptionsFunction = self.getAvailableEnvironmentOptions,
        setActiveImageTopicFunction = self.setActiveImageTopic,
        getBridgeConnectedFunction = self.isBridgeConnected,
        getTelemetryAgeFunction = self.getTelemetryAge,
        getAvailableSimulatorsFunction = self.getAvailableSimulators,
        getSelectedSimulatorFunction = self.getSelectedSimulator,
        setSelectedSimulatorFunction = self.setSelectedSimulator,
        getAvailableRobotConfigsFunction = self.getAvailableRobotConfigs,
        getSelectedRobotConfigFunction = self.getSelectedRobotConfig,
        setSelectedRobotConfigFunction = self.setSelectedRobotConfig,
        msg_if = self.msg_if,
        **self.buildCapabilityKwargs(self.profile)
    )

    ##############################
    # Bridge server thread. This app OWNS the listen socket -- it is the one
    # stable, well-known connection surface a simulator's bridge dials into,
    # not a per-simulator client reaching out.
    self.server_thread = threading.Thread(target = self.bridgeServerLoop)
    self.server_thread.daemon = True
    self.server_thread.start()

    ##############################
    # Simulator discovery scan, and the app's own status cadence
    nepi_sdk.start_timer_process(float(1) / SIM_DISCOVERY_RATE_HZ, self.simDiscoveryCb)
    nepi_sdk.start_timer_process(float(1) / STATUS_PUBLISH_RATE_HZ, self.statusPublishCb)

    self.msg_if.pub_info("Sim connector listening on 0.0.0.0:" + str(self.listen_port))
    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.on_shutdown(self.cleanup_actions)
    nepi_sdk.spin()

  #**********************
  # Robot-config profiles

  def buildProfileFromConfig(self, config_name):
    # A robot config is a capability profile read whole from the params yaml. A
    # missing key means the capability is off, so a partially-written config
    # entry degrades to fewer controls rather than to a crash.
    entry = self.robot_configs.get(config_name, dict())
    if not isinstance(entry, dict):
      entry = dict()
    return dict(
        wheel_count = int(entry.get('wheel_count', 0)),
        motor_count = int(entry.get('motor_count', 0)),
        has_goto_position = bool(entry.get('has_goto_position', False)),
        has_goto_pose = bool(entry.get('has_goto_pose', False)),
        has_goto_location = bool(entry.get('has_goto_location', False)),
        has_go_home = bool(entry.get('has_go_home', False)),
        has_set_home = bool(entry.get('has_set_home', False)),
        has_go_stop = bool(entry.get('has_go_stop', False)),
        setup_actions = list(entry.get('setup_actions', [])),
        go_actions = list(entry.get('go_actions', [])),
        has_camera_view_control = bool(entry.get('has_camera_view_control', False)),
        available_camera_view_modes = list(entry.get('available_camera_view_modes', [])),
        has_environment_controls = bool(entry.get('has_environment_controls', False)),
    )

  def buildCapabilityKwargs(self, profile):
    # Turns a capability profile into the exact keyword set SimDeviceIF derives
    # its has_* flags from: a real bound method where the profile says the robot
    # has that capability, None where it does not. This is the only place the
    # config-to-callback mapping lives, so construction and a later
    # apply_capability_profile can never disagree.
    has_motors = profile['motor_count'] > 0
    has_any_goto = (profile['has_goto_position'] or profile['has_goto_pose']
                    or profile['has_goto_location'])

    axis_controls = AxisControls()
    axis_controls.x = profile['has_goto_position']
    axis_controls.y = profile['has_goto_position']
    # A wheeled robot moves in the ground plane, so z is only meaningful once
    # the profile declares no wheels (a flight or subsea vehicle).
    axis_controls.z = profile['has_goto_position'] and profile['wheel_count'] == 0
    axis_controls.roll = profile['has_goto_pose']
    axis_controls.pitch = profile['has_goto_pose']
    axis_controls.yaw = profile['has_goto_pose'] or profile['has_goto_position']

    return dict(
        axisControls = axis_controls,
        wheel_count = profile['wheel_count'],
        motor_count = profile['motor_count'],
        setMotorControlRatio = self.setMotorControlRatio if has_motors else None,
        getMotorControlRatios = self.getMotorControlRatios if has_motors else None,
        manualControlsReadyFunction = self.isBridgeConnected if has_motors else None,
        autonomousControlsReadyFunction = self.autonomousControlsReady if has_any_goto else None,
        setup_actions = profile['setup_actions'],
        setSetupActionIndFunction = self.setSetupActionInd if profile['setup_actions'] else None,
        go_actions = profile['go_actions'],
        setGoActionIndFunction = self.setGoActionInd if profile['go_actions'] else None,
        getHomeFunction = self.getHome if profile['has_set_home'] else None,
        setHomeFunction = self.setHome if profile['has_set_home'] else None,
        goHomeFunction = self.goHome if profile['has_go_home'] else None,
        goStopFunction = self.goStop if profile['has_go_stop'] else None,
        gotoPoseFunction = self.gotoPose if profile['has_goto_pose'] else None,
        gotoPositionFunction = self.gotoPosition if profile['has_goto_position'] else None,
        gotoLocationFunction = self.gotoLocation if profile['has_goto_location'] else None,
        setCameraViewModeFunction = (self.setCameraViewMode
                                     if profile['has_camera_view_control'] else None),
        available_camera_view_modes = profile['available_camera_view_modes'],
        setEnvironmentOptionFunction = (self.setEnvironmentOption
                                        if profile['has_environment_controls'] else None),
        available_environment_options = list(self.available_environment_options),
    )

  #**********************
  # Robot config selector

  def getAvailableRobotConfigs(self):
    return sorted(self.robot_configs.keys())

  def getSelectedRobotConfig(self):
    return self.selected_robot_config

  def setSelectedRobotConfig(self, config_name):
    config_name = str(config_name)
    if config_name not in self.robot_configs:
      self.msg_if.pub_warn("Robot config '" + config_name + "' is not one of " +
                           str(self.getAvailableRobotConfigs()) + ", ignoring")
      return
    self.selected_robot_config = config_name
    self.profile = self.buildProfileFromConfig(config_name)
    self.motor_ratios = [0.0] * int(self.profile['motor_count'])
    # Selecting a config is selecting a kind of robot, and a robot's kind IS its
    # capability set -- so the cached report is re-derived in place from the new
    # profile. SimDeviceIF.apply_capability_profile republishes info and status
    # itself, so the new flags reach a client within one status interval.
    #
    # NOTE: self.sim_if is None until the SimDeviceIF constructor call below
    # (in __init__) fully returns -- and that constructor's own sub-IF setup
    # (NavPoseIF/SaveDataIF/SettingsIF/Transform3DIF) can take several seconds.
    # Its select_robot_config SUBSCRIBER is live before that finishes (it's
    # registered early in NodeClassIF's own setup), so a config-select message
    # arriving during that startup window silently no-ops here rather than
    # queuing -- confirmed via live testing, not a defect worth chasing further
    # given it only matters in the first few seconds after node start.
    if self.sim_if is not None:
      self.sim_if.apply_capability_profile(**self.buildCapabilityKwargs(self.profile))
    # Tell the simulator which kind of robot is wanted.
    self.sendLineToBridge({'type': 'robot_config', 'config': config_name}, "Robot config")
    self.msg_if.pub_info("Selected robot config: " + config_name)

  #**********************
  # Simulator selector. Discovery is a timer scan of the live ROS graph, matched
  # on device type plus the device's own declared source description. Never a
  # blocking wait, and never a match on any simulator product name.

  def simDiscoveryCb(self, timer):
    try:
      topics, msg_types = nepi_sdk.find_topics_by_msgs(SIM_DEVICE_STATUS_MSG_TYPES)
    except Exception as e:
      self.msg_if.pub_warn("Simulator scan failed: " + str(e), throttle_s = 10.0)
      return

    # find_topics_by_msgs returns two PARALLEL lists, not a list of tuples --
    # zip them rather than iterating one of them as pairs.
    found = dict(zip(topics, msg_types))

    with self.sim_scan_lock:
      # Subscribe to any newly seen candidate device status topic. The declared
      # source description only rides on the status message, so a subscription
      # is how a candidate is qualified.
      for topic in found.keys():
        if topic in self.sim_device_subs:
          continue
        try:
          sub = nepi_sdk.create_subscriber(topic, DeviceRBXStatus,
                                          self.simDeviceStatusCb,
                                          queue_size = 1, callback_args = (topic,))
        except Exception as e:
          self.msg_if.pub_warn("Failed to subscribe candidate " + topic + ": " + str(e),
                               throttle_s = 10.0)
          continue
        if sub is None:
          # create_subscriber logs and returns None on failure rather than
          # raising, so this is the real failure path -- retried next scan.
          continue
        self.sim_device_subs[topic] = sub

      # Drop candidates whose status topic has gone away entirely.
      for topic in list(self.sim_device_subs.keys()):
        if topic in found:
          continue
        sub = self.sim_device_subs.pop(topic)
        self.sim_device_info.pop(topic, None)
        try:
          sub.unregister()
        except Exception:
          pass

    # A selection that is no longer available falls back to no selection rather
    # than silently pointing at a device that is gone.
    if self.selected_simulator != "":
      available, _names = self.getAvailableSimulators()
      if self.selected_simulator not in available:
        self.msg_if.pub_warn("Selected simulator " + self.selected_simulator +
                             " is no longer available, clearing selection")
        self.selected_simulator = ""

  def simDeviceStatusCb(self, msg, args):
    topic = args[0] if isinstance(args, tuple) else args
    with self.sim_scan_lock:
      self.sim_device_info[topic] = dict(
          device_name = msg.device_name,
          data_source_description = msg.data_source_description,
          time = nepi_utils.get_time())

  def getAvailableSimulators(self):
    # Returns two parallel lists (namespaces, display names) -- the same
    # reported-list shape the RUI's other selectors consume. Empty is valid.
    namespaces = []
    names = []
    now = nepi_utils.get_time()
    with self.sim_scan_lock:
      for topic in sorted(self.sim_device_info.keys()):
        entry = self.sim_device_info[topic]
        if entry['data_source_description'] != SIM_SOURCE_DESCRIPTION:
          continue
        if (now - entry['time']) > SIM_DEVICE_STALE_SEC:
          continue
        # The device namespace is the status topic minus its trailing '/status'.
        namespace = topic
        if namespace.endswith('/status'):
          namespace = namespace[:-len('/status')]
        namespaces.append(namespace)
        names.append(entry['device_name'] if entry['device_name'] else namespace)
    return namespaces, names

  def getSelectedSimulator(self):
    return self.selected_simulator

  def setSelectedSimulator(self, namespace):
    namespace = str(namespace)
    if namespace in ("", "None"):
      self.selected_simulator = ""
      self.msg_if.pub_info("Cleared simulator selection")
      return
    available, _names = self.getAvailableSimulators()
    if namespace not in available:
      self.msg_if.pub_warn("Simulator '" + namespace + "' is not currently available, ignoring")
      return
    self.selected_simulator = namespace
    self.msg_if.pub_info("Selected simulator: " + namespace)

  #**********************
  # Connection health

  def isBridgeConnected(self):
    with self.client_lock:
      return self.client_conn is not None

  def getTelemetryAge(self):
    if self.connected_since is None:
      return TELEMETRY_AGE_IF_NEVER_CONNECTED
    return nepi_utils.get_time() - self.last_telemetry_time

  def autonomousControlsReady(self):
    # Goto commands need a live bridge AND fresh telemetry, since a setpoint is
    # computed against the current pose. A direct motor command does not, which
    # is why manual control gates on connection alone.
    if not self.isBridgeConnected():
      return False
    age = self.getTelemetryAge()
    return age >= 0.0 and age < SIM_DEVICE_STALE_SEC

  #**********************
  # SimDeviceIF callbacks -- thin wire-protocol senders and getters, gated at
  # construction (and on a config change) by the selected robot config

  def setMotorControlRatio(self, motor_ind, speed_ratio):
    if motor_ind < 0 or motor_ind >= len(self.motor_ratios):
      self.msg_if.pub_warn("Motor control ignored: motor index " + str(motor_ind) + " out of range")
      return
    self.motor_ratios[motor_ind] = max(0.0, min(1.0, speed_ratio))
    self.sendLineToBridge({'type': 'motor_control', 'motor_ind': motor_ind,
                          'speed_ratio': self.motor_ratios[motor_ind]}, "Motor control")

  def getMotorControlRatios(self):
    return self.motor_ratios

  def gotoPosition(self, msg):
    with self.goto_target_lock:
      self.goto_target = {'x_meters': msg.x_meters, 'y_meters': msg.y_meters,
                          'z_meters': msg.z_meters, 'yaw_deg': msg.yaw_deg}
    self.sendLineToBridge({'type': 'goto_position', 'x_meters': msg.x_meters,
                           'y_meters': msg.y_meters, 'z_meters': msg.z_meters,
                           'yaw_deg': msg.yaw_deg}, "Goto position")

  def gotoPose(self, attitude_enu_degs):
    self.sendLineToBridge({'type': 'goto_pose', 'roll_deg': attitude_enu_degs[0],
                           'pitch_deg': attitude_enu_degs[1],
                           'yaw_deg': attitude_enu_degs[2]}, "Goto pose")

  def gotoLocation(self, msg):
    self.sendLineToBridge({'type': 'goto_location', 'lat': msg.lat, 'long': msg.long,
                           'altitude_meters': msg.altitude_meters,
                           'yaw_deg': msg.yaw_deg}, "Goto location")

  def getHome(self):
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

  def goHome(self):
    self.sendLineToBridge({'type': 'go_home'}, "Go home")
    return self.isBridgeConnected()

  def goStop(self):
    with self.goto_target_lock:
      self.goto_target = None
    self.sendLineToBridge({'type': 'go_stop'}, "Go stop")
    return self.isBridgeConnected()

  def setSetupActionInd(self, action_ind):
    actions = self.profile['setup_actions']
    if action_ind < 0 or action_ind >= len(actions):
      return False
    self.sendLineToBridge({'type': 'setup_action', 'action': actions[action_ind]},
                          "Setup action")
    return self.isBridgeConnected()

  def setGoActionInd(self, action_ind):
    actions = self.profile['go_actions']
    if action_ind < 0 or action_ind >= len(actions):
      return False
    self.sendLineToBridge({'type': 'go_action', 'action': actions[action_ind]}, "Go action")
    return self.isBridgeConnected()

  def setCameraViewMode(self, view_mode):
    self.sendLineToBridge({'type': 'camera_settings', 'view_mode': view_mode},
                          "Camera settings")

  def setEnvironmentOption(self, option):
    # The contract's environment control is a single option string today, not an
    # (option, enabled) pair. Forwarded as an enable until that control's own
    # on/off semantics are designed; the only real precedent (an obstacle-course
    # toggle) is a one-way "turn this on".
    self.sendLineToBridge({'type': 'environment_option', 'option': option,
                           'enabled': True}, "Environment option")

  def setActiveImageTopic(self, topic_name):
    self.active_image_topic = topic_name
    self.sendLineToBridge({'type': 'set_active_image_topic', 'topic_name': topic_name},
                          "Set active image topic")

  def getAvailableSensorTopics(self):
    return list(self.available_sensor_topics)

  def getAvailableEnvironmentOptions(self):
    return list(self.available_environment_options)

  def getNavPoseCb(self):
    return self.navpose_dict

  #**********************
  # Bridge server. Single active client at a time.

  def bridgeServerLoop(self):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # rospy installs a process-global socket.setdefaulttimeout(60) during
    # init_node, which accept() would otherwise apply to every accepted
    # connection. A command stream is legitimately idle for long stretches, so a
    # recv timeout must not be read as client death: clear the timeout and block
    # instead. A real disconnect still unblocks recv with EOF. This is the
    # socket-handling care documented in src/nepi_drivers/CLAUDE.md, and it must
    # happen after init_node, which it does -- this thread starts at the end of
    # __init__.
    srv.settimeout(None)
    try:
      srv.bind(('0.0.0.0', self.listen_port))
      srv.listen(BRIDGE_ACCEPT_BACKLOG)
    except Exception as e:
      self.msg_if.pub_warn("Could not listen on port " + str(self.listen_port) + ": " + str(e) +
                           ". No simulator bridge can connect until this is resolved.")
      return

    while not nepi_sdk.is_shutdown():
      try:
        conn, addr = srv.accept()
        conn.settimeout(None)
      except Exception:
        continue
      self.msg_if.pub_info("Bridge client connected from " + str(addr))
      with self.client_lock:
        self.client_conn = conn
        self.connected_since = nepi_utils.get_time()
        self.last_telemetry_time = self.connected_since
      self.serveClient(conn)
      with self.client_lock:
        if self.client_conn is conn:
          self.client_conn = None
      try:
        conn.close()
      except Exception:
        pass
      self.msg_if.pub_info("Bridge client disconnected")

  def serveClient(self, conn):
    buf = b''
    while not nepi_sdk.is_shutdown():
      try:
        data = conn.recv(BRIDGE_RECV_BYTES)
      except Exception as e:
        self.msg_if.pub_warn("Bridge client recv error: " + repr(e))
        return
      if not data:
        self.msg_if.pub_info("Bridge client closed connection (EOF)")
        return
      buf += data
      while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        if line.strip():
          self.processBridgeLine(line)

  def processBridgeLine(self, line):
    try:
      msg = json.loads(line)
    except Exception as e:
      self.msg_if.pub_warn("Bad line from bridge: " + str(e), throttle_s = 5.0)
      return
    if not isinstance(msg, dict):
      self.msg_if.pub_warn("Ignoring non-object line from bridge", throttle_s = 5.0)
      return
    msg_type = msg.get('type')
    if msg_type == 'sensor_topics':
      self.processSensorTopicsLine(msg)
    elif msg_type == 'environment_options':
      self.processEnvironmentOptionsLine(msg)
    elif msg_type == 'image':
      self.processImageLine(msg)
    else:
      # A line with no "type" key is telemetry -- the dispatch-by-key-presence
      # convention the existing bridge scripts already use. An unrecognized type
      # lands here too and is harmless: nothing in a NavPose parse is required.
      self.processTelemetryLine(msg)

  def processSensorTopicsLine(self, msg):
    topics = msg.get('topics', [])
    parsed = []
    for entry in topics:
      if not isinstance(entry, dict):
        continue
      topic_name = entry.get('topic_name', '')
      msg_type = entry.get('msg_type', '')
      if topic_name and msg_type:
        parsed.append((topic_name, msg_type))
    self.available_sensor_topics = parsed

  def processEnvironmentOptionsLine(self, msg):
    options = msg.get('options', [])
    if not isinstance(options, list):
      return
    self.available_environment_options = [str(o) for o in options]

  def processImageLine(self, msg):
    topic_name = msg.get('topic_name', self.active_image_topic)
    if self.active_image_topic and topic_name != self.active_image_topic:
      return  # Not the selected camera -- nothing subscribes to it
    try:
      jpeg_bytes = base64.b64decode(msg['data'])
      arr = np.frombuffer(jpeg_bytes, dtype = np.uint8)
      cv2_img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
      if cv2_img is None:
        raise ValueError("cv2.imdecode returned None")
      ros_img = nepi_img.cv2img_to_rosimg(cv2_img, encoding = "bgr8")
      self.image_pub.publish(ros_img)
    except Exception as e:
      self.msg_if.pub_warn("Failed to process camera image frame: " + str(e), throttle_s = 5.0)

  def processTelemetryLine(self, telem):
    # Generalized past any one vehicle's shape to the full NavPose contract:
    # every field optional, each block gated by presence exactly the way
    # NavPose.msg's own has_* flags are.
    now = nepi_utils.get_time()
    nd = self.navpose_dict

    if any(k in telem for k in ('x_m', 'y_m', 'z_m')):
      nd['has_position'] = True
      nd['time_position'] = now
      nd['x_m'] = float(telem.get('x_m', nd['x_m']))
      nd['y_m'] = float(telem.get('y_m', nd['y_m']))
      nd['z_m'] = float(telem.get('z_m', nd['z_m']))
      nd['x_m_per_sec'] = float(telem.get('x_m_per_sec', nd['x_m_per_sec']))
      nd['y_m_per_sec'] = float(telem.get('y_m_per_sec', nd['y_m_per_sec']))
      nd['z_m_per_sec'] = float(telem.get('z_m_per_sec', nd['z_m_per_sec']))

    if any(k in telem for k in ('roll_deg', 'pitch_deg', 'yaw_deg')):
      nd['has_orientation'] = True
      nd['time_orientation'] = now
      nd['roll_deg'] = float(telem.get('roll_deg', nd['roll_deg']))
      nd['pitch_deg'] = float(telem.get('pitch_deg', nd['pitch_deg']))
      nd['yaw_deg'] = float(telem.get('yaw_deg', nd['yaw_deg']))
      nd['roll_deg_per_sec'] = float(telem.get('roll_deg_per_sec', nd['roll_deg_per_sec']))
      nd['pitch_deg_per_sec'] = float(telem.get('pitch_deg_per_sec', nd['pitch_deg_per_sec']))
      nd['yaw_deg_per_sec'] = float(telem.get('yaw_deg_per_sec', nd['yaw_deg_per_sec']))

    if any(k in telem for k in ('latitude', 'longitude')):
      nd['has_location'] = True
      nd['time_location'] = now
      nd['latitude'] = float(telem.get('latitude', nd['latitude']))
      nd['longitude'] = float(telem.get('longitude', nd['longitude']))

    if 'heading_deg' in telem:
      nd['has_heading'] = True
      nd['time_heading'] = now
      nd['heading_deg'] = float(telem.get('heading_deg', nd['heading_deg']))

    if 'altitude_m' in telem:
      nd['has_altitude'] = True
      nd['time_altitude'] = now
      nd['altitude_m'] = float(telem.get('altitude_m', nd['altitude_m']))

    if 'depth_m' in telem:
      nd['has_depth'] = True
      nd['time_depth'] = now
      nd['depth_m'] = float(telem.get('depth_m', nd['depth_m']))

    self.last_telemetry_time = now

  def sendLineToBridge(self, line_dict, description):
    with self.client_lock:
      conn = self.client_conn
      if conn is None:
        self.msg_if.pub_warn(description + " dropped, no bridge client connected",
                             throttle_s = 5.0)
        return
      # The send happens under the same lock as the client_conn read: this is a
      # single TCP stream, and two unsynchronized sendall calls can interleave
      # their bytes and corrupt the newline-delimited JSON the far side parses.
      try:
        conn.sendall((json.dumps(line_dict) + '\n').encode())
      except Exception as e:
        self.msg_if.pub_warn("Failed to send " + description.lower() + " to bridge: " + str(e))
        if self.client_conn is conn:
          self.client_conn = None
        try:
          conn.close()
        except Exception:
          pass

  #**********************
  # Status

  def statusPublishCb(self, timer):
    # SimDeviceIF owns the 2 Hz device status. This slower tick exists so the
    # two selector reports refresh promptly after a discovery scan even if the
    # device status cadence is ever retuned.
    if self.sim_if is not None:
      self.sim_if.publish_status()

  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    """Closes the bridge connection on node shutdown.

    Closing the accepted connection here unblocks the server thread's recv, so a
    connected simulator bridge sees a clean disconnect rather than a half-open
    socket it has to time out on.
    """
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    with self.client_lock:
      conn = self.client_conn
      self.client_conn = None
    if conn is not None:
      try:
        conn.close()
      except Exception:
        pass


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiSimConnectorApp()
