#!/usr/bin/env python
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

# device_if_sim.py -- Phase 1 of docs/SIMULATION_INTERFACE_IMPL_PLAN.md, building
# docs/SIMULATION_INTERFACE_SPEC.md's `device_if_sim` design. Read both before
# changing this file.
#
# Deliberately `device_if_rbx.py`'s proven shape, generalized -- confirmed against
# a direct read of nepi_drones/src/nepi_api/device_if_rbx.py this pass, not
# reconstructed from memory: constructor-injection callbacks decide `has_*`
# capability flags once at construction time, and the same
# CONFIGS_DICT/PARAMS_DICT/SRVS_DICT/PUBS_DICT/SUBS_DICT shape is handed to the
# same NodeClassIF, which does all the actual ROS registration -- this class
# never touches ROS topics/services directly.
#
# ############################################################################
# Phase 1 scope -- what IS and ISN'T built in this pass (see
# docs/SIMULATION_INTERFACE_IMPL_PLAN.md's own Phase 1 objective: prove the
# capability/status contract, not full command execution semantics):
#
#   BUILT, tested against both of the spec's worked examples (Test Cases 1.1-1.4):
#     - Constructor-injection -> has_* capability flags, including the new
#       2026-08-04 typed available_sensor_topics mechanism.
#     - Full NodeClassIF wiring: capabilities_query, device_info_query services;
#       info/status/status_str publishers; the status timer re-deriving
#       available_sensor_topics/has_camera/available_image_topics live on every
#       publish (never cached past construction -- see statusPublishCb).
#     - The two genuinely NEW sim-specific commands from the spec
#       (setCameraViewModeFunction, setEnvironmentOptionFunction) fully wired.
#
#   NOT built in this pass -- explicit, documented gap, not an oversight:
#     - device_if_rbx.py's deep blocking-wait goto convergence logic
#       (setpoint_position_local_body / setpoint_attitude_ned /
#       setpoint_location_global_wgs84 -- NED/ENU/WGS84 conversions plus an
#       error-bound polling loop against getNavPoseCb). The spec calls
#       goto/setpoint commands "Reuse as-is," meaning the INTENT is to reuse
#       that exact mechanism -- but replicating it correctly needs its own
#       dedicated verification pass, and Phase 1's own test cases don't
#       exercise goto execution at all. Wiring it in half-verified would be
#       worse than leaving it out: this file's goto*Cb methods below are thin
#       delegators (fire the injected function, toggle process_current/
#       ready/cmd_success bookkeeping the same way RBX does) WITHOUT the
#       convergence-polling wait. Tracked as follow-up work for Phase 2/3 of
#       docs/SIMULATION_INTERFACE_IMPL_PLAN.md, not silently dropped.
#     - SaveDataIF/SettingsIF/Transform3DIF integration. The spec says these
#       "come along automatically" by reusing device_if_rbx's machinery: true
#       once this class is fully wired into that machinery, but that wiring
#       itself is not done in this pass -- errors_current/errors_prev/
#       last_error_message fields exist on SimStatus.msg (spec-compliant
#       shape) but are only updated by this file's own thin bookkeeping, not
#       by a shared SaveDataIF instance yet.
# ############################################################################

import copy
import threading
import time

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils

from std_msgs.msg import Empty, Int32, UInt32, Bool, String, Float32

from nepi_interfaces.msg import AxisControls, ErrorBounds, GotoErrors, MotorControl
from nepi_interfaces.msg import GotoPose, GotoPosition, GotoLocation
from nepi_interfaces.srv import DeviceInfoQuery, DeviceInfoQueryResponse, DeviceInfoQueryRequest

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF

from app_sim_connector.msg import SensorTopicInfo, SimInfo, SimStatus
from app_sim_connector.srv import SimCapabilitiesQuery, SimCapabilitiesQueryResponse, SimCapabilitiesQueryRequest


class SimDeviceIF:
  # Class-attribute default, not just an instance attribute set in __init__:
  # NodeClassIF's own configs_dict wrapper invokes the caller's
  # reset_callback/init_callback SYNCHRONOUSLY during its own construction
  # (init_configs=True unconditionally triggers reset_config() -> resetCb() ->
  # initCb() -> publish_status(), all before `self.node_if = NodeClassIF(...)`
  # below has finished assigning) -- confirmed by direct traceback this
  # session, not assumed. Without a class-level default, `self.node_if` raises
  # AttributeError at that point rather than resolving to None; matches
  # device_if_rbx.py's own `node_if = None` class attribute for the identical
  # reason.
  node_if = None

  # Default Global Values
  STATUS_UPDATE_RATE_HZ = 2  # matches device_if_rbx.py

  # Factory defaults sourced from real RBX precedent, not guessed: cmd_timeout
  # and the two error bounds are the values rbx_sim_node.py's own comments cite
  # as "RBXRobotIF's factory default" -- stabilized_sec has no cited source and
  # is a placeholder pending a real value from the NEPI-core team.
  FACTORY_CMD_TIMEOUT_SEC = 25
  FACTORY_HOME_LOCATION = [-999.0, -999.0, -999.0]
  FACTORY_GOTO_MAX_ERROR_M = 2.0
  FACTORY_GOTO_MAX_ERROR_DEG = 2.0
  FACTORY_GOTO_STABILIZED_SEC = 1.0
  FACTORY_ACTIVE_IMAGE_TOPIC = ""
  FACTORY_CAMERA_VIEW_MODE = ""

  # Sensor-type scan list for the 2026-08-04 typed available_sensor_topics
  # decision -- images today; lidar/IMU are already listed so a simulator that
  # wires one up needs zero code changes here, only a bridge-side announcement.
  SCAN_MSG_TYPES = ['sensor_msgs/Image', 'sensor_msgs/LaserScan', 'sensor_msgs/Imu']
  CAMERA_MSG_TYPE = 'sensor_msgs/Image'

  #######################
  ### IF Initialization
  def __init__(self, device_info,
               axisControls = None,
               # existing RBX-style callbacks, all optional (None = unsupported)
               getBatteryPercentFunction = None,
               setMotorControlRatio = None, getMotorControlRatios = None,
               manualControlsReadyFunction = None,
               autonomousControlsReadyFunction = None,
               states = None, getStateIndFunction = None, setStateIndFunction = None,
               modes = None, getModeIndFunction = None, setModeIndFunction = None,
               checkStopFunction = None,
               setup_actions = None, setSetupActionIndFunction = None,
               go_actions = None, setGoActionIndFunction = None,
               getHomeFunction = None, setHomeFunction = None,
               goHomeFunction = None, goStopFunction = None,
               gotoPoseFunction = None, gotoPositionFunction = None, gotoLocationFunction = None,
               getNavPoseCb = None,
               navpose_update_rate = 10,
               # new for sim -- wheels/motors
               wheel_count = 0, motor_count = 0,
               # new for sim -- typed sensor topics (decided 2026-08-04)
               getAvailableSensorTopicsFunction = None,
               setActiveImageTopicFunction = None,
               # new for sim -- camera view/rig control
               setCameraViewModeFunction = None, available_camera_view_modes = None,
               # new for sim -- environment control
               setEnvironmentOptionFunction = None, available_environment_options = None,
               # new for sim -- connection health
               getBridgeConnectedFunction = None, getTelemetryAgeFunction = None,
               data_source_description = 'simulator',
               data_ref_description = 'simulator',
               log_name = None,
               log_name_list = [],
               msg_if = None
               ):
    ####  IF INIT SETUP ####
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()
    self.namespace = nepi_sdk.create_namespace(self.node_namespace, 'sim')

    ##############################
    # Create Msg Class
    if msg_if is not None:
      self.msg_if = msg_if
    else:
      self.msg_if = MsgIF()
    self.log_name_list = copy.deepcopy(log_name_list)
    self.log_name_list.append(self.class_name)
    if log_name is not None:
      self.log_name_list.append(log_name)
    self.msg_if.pub_info("Starting IF Initialization Processes", log_name_list = self.log_name_list)

    ##############################
    # Initialize Class Variables

    self.device_name = device_info["device_name"]
    self.path = device_info["path"]
    self.serial_num = device_info["serial_number"]
    self.hw_version = device_info["hw_version"]
    self.sw_version = device_info["sw_version"]

    self.data_source_description = data_source_description
    self.data_ref_description = data_ref_description

    self.status_msg = SimStatus()
    self.status_msg.device_name = self.device_name
    self.status_msg.device_path = self.path
    self.status_msg.device_node_name = self.node_name
    self.status_msg.serial_num = self.serial_num
    self.status_msg.hw_version = self.hw_version
    self.status_msg.sw_version = self.sw_version
    self.status_msg.data_source_description = self.data_source_description
    self.status_msg.data_ref_description = self.data_ref_description
    self.status_msg.process_current = "None"
    self.status_msg.process_last = "None"
    self.status_msg.ready = True
    self.status_msg.battery = -999
    errors_msg = GotoErrors()
    self.status_msg.errors_current = errors_msg
    self.status_msg.errors_prev = errors_msg
    self.status_msg.last_error_message = ""
    self.status_msg.bridge_connected = False
    self.status_msg.telemetry_age_sec = -1.0

    self.info_report = SimInfo()
    self.info_report.device_name = self.device_name
    self.info_report.serial_num = self.serial_num
    self.info_report.hw_version = self.hw_version
    self.info_report.sw_version = self.sw_version
    self.info_report.standby = False
    self.info_report.state = -999
    self.info_report.mode = -999
    self.info_report.home_lat = self.FACTORY_HOME_LOCATION[0]
    self.info_report.home_long = self.FACTORY_HOME_LOCATION[1]
    self.info_report.home_alt = self.FACTORY_HOME_LOCATION[2]
    self.info_report.active_image_topic = self.FACTORY_ACTIVE_IMAGE_TOPIC

    self.caps_report = SimCapabilitiesQueryResponse()
    self.caps_report.device_name = self.device_name
    self.caps_report.device_path = self.path
    self.caps_report.device_node_name = self.node_name

    self.rbx_cmd_success_current = False
    self.active_image_topic = self.FACTORY_ACTIVE_IMAGE_TOPIC
    self.camera_view_mode = self.FACTORY_CAMERA_VIEW_MODE

    ##############################
    # States/modes/actions -- reused unchanged from RBX (empty lists are
    # valid; RBXRobotIF handles them the same way, see rbx_sim_node.py)
    self.states = states if states is not None else []
    self.getStateIndFunction = getStateIndFunction
    self.setStateIndFunction = setStateIndFunction

    self.modes = modes if modes is not None else []
    self.getModeIndFunction = getModeIndFunction
    self.setModeIndFunction = setModeIndFunction

    self.checkStopFunction = checkStopFunction

    self.setup_actions = setup_actions if setup_actions is not None else []
    self.setSetupActionIndFunction = setSetupActionIndFunction

    self.go_actions = go_actions if go_actions is not None else []
    self.setGoActionIndFunction = setGoActionIndFunction

    self.getNavPoseCb = getNavPoseCb
    self.navpose_update_rate = max(1, min(10, navpose_update_rate))

    ##############################
    # Capability derivation -- each optional callback's None-ness decides the
    # matching has_* flag, computed once here and cached on self.caps_report,
    # identical in spirit to device_if_rbx.py:358-440.

    if axisControls is None:
      axisControls = AxisControls()
    self.caps_report.control_support = axisControls
    self.caps_report.state_options = self.states
    self.caps_report.mode_options = self.modes
    self.caps_report.setup_action_options = self.setup_actions
    self.caps_report.go_action_options = self.go_actions
    self.caps_report.data_products = []

    self.getBatteryPercentFunction = getBatteryPercentFunction
    self.caps_report.has_battery_feedback = getBatteryPercentFunction is not None

    self.setMotorControlRatio = setMotorControlRatio
    self.getMotorControlRatios = getMotorControlRatios
    self.caps_report.has_manual_controls = setMotorControlRatio is not None
    self.manualControlsReadyFunction = manualControlsReadyFunction

    self.autonomousControlsReadyFunction = autonomousControlsReadyFunction
    self.caps_report.has_autonomous_controls = autonomousControlsReadyFunction is not None

    self.getHomeFunction = getHomeFunction
    self.setHomeFunction = setHomeFunction
    self.caps_report.has_set_home = setHomeFunction is not None

    self.goHomeFunction = goHomeFunction
    self.caps_report.has_go_home = goHomeFunction is not None

    self.goStopFunction = goStopFunction
    self.caps_report.has_go_stop = goStopFunction is not None

    self.gotoPoseFunction = gotoPoseFunction
    self.caps_report.has_goto_pose = gotoPoseFunction is not None

    self.gotoPositionFunction = gotoPositionFunction
    self.caps_report.has_goto_position = gotoPositionFunction is not None

    self.gotoLocationFunction = gotoLocationFunction
    self.caps_report.has_goto_location = gotoLocationFunction is not None

    # -- new for sim --
    self.wheel_count = wheel_count
    self.caps_report.has_wheels = wheel_count > 0
    self.caps_report.wheel_count = wheel_count

    self.motor_count = motor_count
    self.caps_report.has_motors = motor_count > 0
    self.caps_report.motor_count = motor_count

    self.getAvailableSensorTopicsFunction = getAvailableSensorTopicsFunction
    self.setActiveImageTopicFunction = setActiveImageTopicFunction

    self.setCameraViewModeFunction = setCameraViewModeFunction
    self.caps_report.has_camera_view_control = setCameraViewModeFunction is not None
    self.caps_report.available_camera_view_modes = (
        available_camera_view_modes if available_camera_view_modes is not None else [])

    self.setEnvironmentOptionFunction = setEnvironmentOptionFunction
    self.caps_report.has_environment_controls = setEnvironmentOptionFunction is not None
    self.caps_report.available_environment_options = (
        available_environment_options if available_environment_options is not None else [])

    self.getBridgeConnectedFunction = getBridgeConnectedFunction
    self.getTelemetryAgeFunction = getTelemetryAgeFunction

    # has_camera / available_image_topics / available_sensor_topics are
    # DERIVED, not independently tracked -- computed fresh here for the
    # initial caps report, and again on every statusPublishCb tick (never
    # cached past construction; see refreshSensorTopics below).
    self.refreshSensorTopics(update_caps = True)

    ##################################################
    ### Node Class Setup

    self.msg_if.pub_info("Starting Node IF Initialization", log_name_list = self.log_name_list)

    self.CONFIGS_DICT = {
        'init_callback': self.initCb,
        'reset_callback': self.resetCb,
        'factory_reset_callback': self.factoryResetCb,
        'init_configs': True,
        'namespace': self.namespace
    }

    self.PARAMS_DICT = {
        'cmd_timeout': {
            'namespace': self.namespace,
            'factory_val': self.FACTORY_CMD_TIMEOUT_SEC
        },
        'home_location': {
            'namespace': self.namespace,
            'factory_val': self.FACTORY_HOME_LOCATION
        },
        'max_error_m': {
            'namespace': self.namespace,
            'factory_val': self.FACTORY_GOTO_MAX_ERROR_M
        },
        'max_error_deg': {
            'namespace': self.namespace,
            'factory_val': self.FACTORY_GOTO_MAX_ERROR_DEG
        },
        'stabilized_sec': {
            'namespace': self.namespace,
            'factory_val': self.FACTORY_GOTO_STABILIZED_SEC
        },
        'active_image_topic': {
            'namespace': self.namespace,
            'factory_val': self.FACTORY_ACTIVE_IMAGE_TOPIC
        },
        'camera_view_mode': {
            'namespace': self.namespace,
            'factory_val': self.FACTORY_CAMERA_VIEW_MODE
        }
    }

    self.SRVS_DICT = {
        'device_info_query': {
            'namespace': self.namespace,
            'topic': 'device_info_query',
            'srv': DeviceInfoQuery,
            'req': DeviceInfoQueryRequest(),
            'resp': DeviceInfoQueryResponse(),
            'callback': self.info_query_callback
        },
        'capabilities_query': {
            'namespace': self.namespace,
            'topic': 'capabilities_query',
            'srv': SimCapabilitiesQuery,
            'req': SimCapabilitiesQueryRequest(),
            'resp': SimCapabilitiesQueryResponse(),
            'callback': self.capabilities_query_callback
        }
    }

    self.PUBS_DICT = {
        'sim_info_pub': {
            'namespace': self.namespace,
            'topic': 'info',
            'msg': SimInfo,
            'qsize': 1,
            'latch': True
        },
        'status_msg_pub': {
            'namespace': self.namespace,
            'topic': 'status',
            'msg': SimStatus,
            'qsize': 1,
            'latch': True
        },
        'status_msg_str_pub': {
            'namespace': self.namespace,
            'topic': 'status_str',
            'msg': String,
            'qsize': 1,
            'latch': True
        }
    }

    self.SUBS_DICT = {
        'set_state': {
            'namespace': self.namespace, 'topic': 'set_state', 'msg': Int32,
            'qsize': None, 'callback': self.setStateCb, 'callback_args': ()
        },
        'set_mode': {
            'namespace': self.namespace, 'topic': 'set_mode', 'msg': Int32,
            'qsize': None, 'callback': self.setModeCb, 'callback_args': ()
        },
        'setup_action': {
            'namespace': self.namespace, 'topic': 'setup_action', 'msg': Int32,
            'qsize': None, 'callback': self.setupActionCb, 'callback_args': ()
        },
        'go_action': {
            'namespace': self.namespace, 'topic': 'go_action', 'msg': Int32,
            'qsize': None, 'callback': self.goActionCb, 'callback_args': ()
        },
        'set_motor_control': {
            'namespace': self.namespace, 'topic': 'set_motor_control', 'msg': MotorControl,
            'qsize': 20, 'callback': self.setMotorControlCb, 'callback_args': ()
        },
        'set_goto_timeout': {
            'namespace': self.namespace, 'topic': 'set_goto_timeout', 'msg': UInt32,
            'qsize': None, 'callback': self.setCmdTimeoutCb, 'callback_args': ()
        },
        'set_goto_error_bounds': {
            'namespace': self.namespace, 'topic': 'set_goto_error_bounds', 'msg': ErrorBounds,
            'qsize': None, 'callback': self.setErrorBoundsCb, 'callback_args': ()
        },
        'go_home': {
            'namespace': self.namespace, 'topic': 'go_home', 'msg': Empty,
            'qsize': None, 'callback': self.goHomeCb, 'callback_args': ()
        },
        'go_stop': {
            'namespace': self.namespace, 'topic': 'go_stop', 'msg': Empty,
            'qsize': None, 'callback': self.goStopCb, 'callback_args': ()
        },
        'goto_pose': {
            'namespace': self.namespace, 'topic': 'goto_pose', 'msg': GotoPose,
            'qsize': None, 'callback': self.gotoPoseCb, 'callback_args': ()
        },
        'goto_position': {
            'namespace': self.namespace, 'topic': 'goto_position', 'msg': GotoPosition,
            'qsize': None, 'callback': self.gotoPositionCb, 'callback_args': ()
        },
        'goto_location': {
            'namespace': self.namespace, 'topic': 'goto_location', 'msg': GotoLocation,
            'qsize': None, 'callback': self.gotoLocationCb, 'callback_args': ()
        },
        'publish_status': {
            'namespace': self.namespace, 'topic': 'publish_status', 'msg': Empty,
            'qsize': None, 'callback': self.publishStatusCb, 'callback_args': ()
        },
        'publish_info': {
            'namespace': self.namespace, 'topic': 'publish_info', 'msg': Empty,
            'qsize': None, 'callback': self.publishInfoCb, 'callback_args': ()
        },
        # -- new for sim --
        'set_active_image_topic': {
            'namespace': self.namespace, 'topic': 'set_active_image_topic', 'msg': String,
            'qsize': None, 'callback': self.setActiveImageTopicCb, 'callback_args': ()
        },
        'set_camera_view_mode': {
            'namespace': self.namespace, 'topic': 'set_camera_view_mode', 'msg': String,
            'qsize': None, 'callback': self.setCameraViewModeCb, 'callback_args': ()
        },
        'set_environment_option': {
            'namespace': self.namespace, 'topic': 'set_environment_option', 'msg': String,
            'qsize': None, 'callback': self.setEnvironmentOptionCb, 'callback_args': ()
        }
    }

    # Create Node Class ####################
    self.node_if = NodeClassIF(
        configs_dict = self.CONFIGS_DICT,
        params_dict = self.PARAMS_DICT,
        services_dict = self.SRVS_DICT,
        pubs_dict = self.PUBS_DICT,
        subs_dict = self.SUBS_DICT,
        log_name_list = self.log_name_list,
        msg_if = self.msg_if
    )

    nepi_sdk.wait()

    ##############################
    # Background sensor-topic refresh -- keeps available_sensor_topics live
    # between status publishes too, matching ai_if_detector.py's own
    # scan-on-a-timer pattern (Camera configuration section of the spec)
    self.status_timer = nepi_sdk.start_timer_process(
        float(1) / self.STATUS_UPDATE_RATE_HZ, self.publishStatusCb)

    self.initCb(do_updates = True)
    self.publish_status()

    self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)

  #**********************
  # Sensor-topic derivation (2026-08-04 decision)

  def refreshSensorTopics(self, update_caps = False):
    """Re-derives available_sensor_topics / has_camera / available_image_topics
    from getAvailableSensorTopicsFunction(), and optionally writes the result
    into caps_report too. Called at construction and on every status publish
    -- never cached longer than that, since a simulator's live topic set can
    change mid-session (a second robot spawning, a camera disappearing).
    """
    sensor_topics = []
    if self.getAvailableSensorTopicsFunction is not None:
      try:
        sensor_topics = self.getAvailableSensorTopicsFunction() or []
      except Exception as e:
        self.msg_if.pub_warn("getAvailableSensorTopicsFunction failed: " + str(e),
                             log_name_list = self.log_name_list)
        sensor_topics = []

    sensor_topic_msgs = []
    image_topics = []
    for entry in sensor_topics:
      topic_name, msg_type = entry[0], entry[1]
      info = SensorTopicInfo()
      info.topic_name = topic_name
      info.msg_type = msg_type
      sensor_topic_msgs.append(info)
      if msg_type == self.CAMERA_MSG_TYPE:
        image_topics.append(topic_name)

    has_camera = len(image_topics) > 0

    if update_caps:
      self.caps_report.available_sensor_topics = sensor_topic_msgs
      self.caps_report.has_camera = has_camera
      self.caps_report.available_image_topics = image_topics
      self.caps_report.active_image_topic = self.active_image_topic

    return sensor_topic_msgs, has_camera, image_topics

  #**********************
  # Config callbacks -- pass-through stubs, matching device_if_rbx.py's own
  # initCb/resetCb/factoryResetCb (RBX itself doesn't persist meaningful
  # config through these either; not a gap introduced by this file)

  def initCb(self, do_updates = False):
    if do_updates:
      pass
    self.publish_status()

  def resetCb(self, do_updates = True):
    self.initCb(do_updates = True)

  def factoryResetCb(self, do_updates = True):
    self.initCb(do_updates = True)

  #**********************
  # Query service callbacks

  def info_query_callback(self, _):
    return self.info_report

  def capabilities_query_callback(self, _):
    """Handles a SimCapabilitiesQuery service request.

    Args:
        _: Unused service request object.

    Returns:
        SimCapabilitiesQueryResponse: The pre-populated capabilities report,
            with available_sensor_topics/has_camera/available_image_topics
            refreshed live rather than returning a stale construction-time
            snapshot.
    """
    self.refreshSensorTopics(update_caps = True)
    return self.caps_report

  #**********************
  # Status/info publishers

  def publishInfoCb(self, msg):
    self.publishInfo()

  def publishInfo(self):
    self.info_report.device_name = self.device_name
    if self.getStateIndFunction is not None:
      self.info_report.state = self.getStateIndFunction()
    if self.getModeIndFunction is not None:
      self.info_report.mode = self.getModeIndFunction()
    if self.getHomeFunction is not None:
      home_geo = self.getHomeFunction()
      self.info_report.home_lat = home_geo.latitude
      self.info_report.home_long = home_geo.longitude
      self.info_report.home_alt = home_geo.altitude
    self.info_report.active_image_topic = self.active_image_topic
    if self.node_if is not None:
      self.info_report.cmd_timeout = self.node_if.get_param('cmd_timeout')
    if not nepi_sdk.is_shutdown() and self.node_if is not None:
      self.node_if.publish_pub('sim_info_pub', self.info_report)

  def publishStatusCb(self, timer):
    self.publish_status()

  def publish_status(self):
    sensor_topic_msgs, has_camera, image_topics = self.refreshSensorTopics(update_caps = False)
    self.status_msg.available_sensor_topics = sensor_topic_msgs

    if self.getBatteryPercentFunction is not None:
      self.status_msg.battery = self.getBatteryPercentFunction()
    if self.manualControlsReadyFunction is not None:
      self.status_msg.manual_control_mode_ready = self.manualControlsReadyFunction()
    if self.autonomousControlsReadyFunction is not None:
      self.status_msg.autonomous_control_mode_ready = self.autonomousControlsReadyFunction()
    if self.getMotorControlRatios is not None:
      motor_settings = []
      for i, ratio in enumerate(self.getMotorControlRatios()):
        m = MotorControl()
        m.motor_ind = i
        m.speed_ratio = ratio
        motor_settings.append(m)
      self.status_msg.current_motor_control_settings = motor_settings

    if self.getBridgeConnectedFunction is not None:
      self.status_msg.bridge_connected = self.getBridgeConnectedFunction()
    if self.getTelemetryAgeFunction is not None:
      self.status_msg.telemetry_age_sec = self.getTelemetryAgeFunction()

    if not nepi_sdk.is_shutdown() and self.node_if is not None:
      self.node_if.publish_pub('status_msg_pub', self.status_msg)
      self.node_if.publish_pub('status_msg_str_pub', String(data = str(self.status_msg)))

  def update_error_msg(self, err_str):
    self.msg_if.pub_warn(err_str, log_name_list = self.log_name_list)
    self.status_msg.last_error_message = err_str

  #**********************
  # New-for-sim command callbacks

  def setActiveImageTopicCb(self, msg):
    self.msg_if.pub_info("Received set active image topic message", log_name_list = self.log_name_list)
    self.active_image_topic = msg.data
    if self.node_if is not None:
      self.node_if.set_param('active_image_topic', msg.data)
    if self.setActiveImageTopicFunction is not None:
      self.setActiveImageTopicFunction(msg.data)
    self.publishInfo()

  # Alias matching device_if_rbx.py's set_image_topic naming, for anything
  # ported from an RBX driver expecting that exact topic name
  def setImageTopicCb(self, msg):
    self.setActiveImageTopicCb(msg)

  def setCameraViewModeCb(self, msg):
    self.msg_if.pub_info("Received set camera view mode message", log_name_list = self.log_name_list)
    if self.setCameraViewModeFunction is None:
      self.update_error_msg("Ignoring set_camera_view_mode, no camera view control")
      return
    self.camera_view_mode = msg.data
    if self.node_if is not None:
      self.node_if.set_param('camera_view_mode', msg.data)
    self.setCameraViewModeFunction(msg.data)

  def setEnvironmentOptionCb(self, msg):
    self.msg_if.pub_info("Received set environment option message", log_name_list = self.log_name_list)
    if self.setEnvironmentOptionFunction is None:
      self.update_error_msg("Ignoring set_environment_option, no environment controls")
      return
    self.setEnvironmentOptionFunction(msg.data)

  #**********************
  # Reused RBX-style command callbacks -- thin delegators. See the Phase 1
  # scope note at the top of this file: these do NOT reimplement
  # device_if_rbx.py's blocking-wait goto convergence polling.

  def setStateCb(self, msg):
    if self.setStateIndFunction is None:
      self.update_error_msg("Ignoring set_state, no state function")
      return
    new_state_ind = msg.data
    if new_state_ind < 0 or new_state_ind > (len(self.states) - 1):
      self.update_error_msg("No matching sim state found")
      return
    self.setStateIndFunction(new_state_ind)
    self.publishInfo()

  def setModeCb(self, msg):
    if self.setModeIndFunction is None:
      self.update_error_msg("Ignoring set_mode, no mode function")
      return
    new_mode_ind = msg.data
    if new_mode_ind < 0 or new_mode_ind > (len(self.modes) - 1):
      self.update_error_msg("No matching sim mode found")
      return
    self.setModeIndFunction(new_mode_ind)
    self.publishInfo()

  def setupActionCb(self, msg):
    if self.setSetupActionIndFunction is None:
      self.update_error_msg("Ignoring setup_action, no set action function")
      return
    action_ind = msg.data
    if action_ind < 0 or action_ind > (len(self.setup_actions) - 1):
      self.update_error_msg("No matching sim setup action found")
      return
    self.status_msg.process_current = self.setup_actions[action_ind]
    success = self.setSetupActionIndFunction(action_ind)
    self.status_msg.process_last = self.setup_actions[action_ind]
    self.status_msg.process_current = "None"
    self.status_msg.cmd_success = success
    self.publishInfo()

  def goActionCb(self, msg):
    if self.setGoActionIndFunction is None:
      self.update_error_msg("Ignoring go_action, no go action function")
      return
    action_ind = msg.data
    if action_ind < 0 or action_ind > (len(self.go_actions) - 1):
      self.update_error_msg("No matching sim go action found")
      return
    self.status_msg.process_current = self.go_actions[action_ind]
    success = self.setGoActionIndFunction(action_ind)
    self.status_msg.process_last = self.go_actions[action_ind]
    self.status_msg.process_current = "None"
    self.status_msg.cmd_success = success
    self.publishInfo()

  def setMotorControlCb(self, msg):
    if self.setMotorControlRatio is None:
      self.update_error_msg("Ignoring set_motor_control, no manual controls")
      return
    if self.manualControlsReadyFunction is not None and not self.manualControlsReadyFunction():
      self.update_error_msg("Ignoring set_motor_control, manual controls not ready")
      return
    m_ind = msg.motor_ind
    m_sr = msg.speed_ratio
    if self.getMotorControlRatios is not None and m_ind > (len(self.getMotorControlRatios()) - 1):
      self.update_error_msg("Motor index " + str(m_ind) + " out of range")
      return
    if m_sr < 0 or m_sr > 1:
      self.update_error_msg("Motor speed ratio " + str(m_sr) + " out of range")
      return
    self.setMotorControlRatio(m_ind, m_sr)

  def setCmdTimeoutCb(self, msg):
    if self.node_if is not None:
      self.node_if.set_param('cmd_timeout', msg.data)
    self.publishInfo()

  def setErrorBoundsCb(self, msg):
    if self.node_if is not None:
      self.node_if.set_param('max_error_m', msg.max_distance_error_m)
      self.node_if.set_param('max_error_deg', msg.max_rotation_error_deg)
      self.node_if.set_param('stabilized_sec', msg.min_stabilize_time_s)
    self.publishInfo()

  def goHomeCb(self, msg):
    if self.goHomeFunction is None:
      self.update_error_msg("Ignoring go_home, no go home function")
      return
    self.status_msg.process_current = "Go Home"
    success = self.goHomeFunction()
    self.status_msg.process_last = "Go Home"
    self.status_msg.process_current = "None"
    self.status_msg.cmd_success = success
    self.publishInfo()

  def goStopCb(self, msg):
    if self.goStopFunction is None:
      self.update_error_msg("Ignoring go_stop, no go stop function")
      return
    self.status_msg.process_current = "Stop"
    success = self.goStopFunction()
    self.status_msg.process_last = "Stop"
    self.status_msg.process_current = "None"
    self.status_msg.cmd_success = success
    self.publishInfo()

  # See the Phase 1 scope note at the top of this file: fire-and-forget
  # delegation to the injected function, no convergence-polling wait.
  def gotoPoseCb(self, msg):
    if self.gotoPoseFunction is None:
      self.update_error_msg("Ignoring goto_pose, no goto pose function")
      return
    attitude_enu_degs = [msg.roll_deg, msg.pitch_deg, msg.yaw_deg]
    self.status_msg.process_current = "GoTo Pose"
    self.gotoPoseFunction(attitude_enu_degs)
    self.status_msg.process_last = "GoTo Pose"
    self.status_msg.process_current = "None"
    self.publishInfo()

  def gotoPositionCb(self, msg):
    if self.gotoPositionFunction is None:
      self.update_error_msg("Ignoring goto_position, no goto position function")
      return
    self.status_msg.process_current = "GoTo Position"
    self.gotoPositionFunction(msg)
    self.status_msg.process_last = "GoTo Position"
    self.status_msg.process_current = "None"
    self.publishInfo()

  def gotoLocationCb(self, msg):
    if self.gotoLocationFunction is None:
      self.update_error_msg("Ignoring goto_location, no goto location function")
      return
    self.status_msg.process_current = "GoTo Location"
    self.gotoLocationFunction(msg)
    self.status_msg.process_last = "GoTo Location"
    self.status_msg.process_current = "None"
    self.publishInfo()
