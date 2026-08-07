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

# device_if_sim.py -- SimDeviceIF, the generic NEPI <-> simulator contract.
#
# Deliberately device_if_rbx.py's proven shape, generalized: a driver or app
# hands in plain Python callback functions, and whichever ones are None vs. real
# functions decides the matching has_* capability flag. Capabilities are decided
# once, at construction, and cached -- which is what makes the RUI's
# capability-flag-driven rendering work at all. The same
# CONFIGS_DICT/PARAMS_DICT/SRVS_DICT/PUBS_DICT/SUBS_DICT shape is handed to the
# same NodeClassIF, which does all the actual ROS registration; this class never
# touches ROS topics or services directly.
#
# This class is protocol-agnostic. It knows nothing about TCP, about JSON, or
# about any particular simulator, exactly as device_if_rbx.py knows nothing about
# the transport its drivers speak. The hosting node owns the wire protocol.
#
# ############################################################################
# Scope -- what IS and ISN'T built here.
#
#   BUILT:
#     - Constructor-injection -> has_* capability flags, cached on caps_report.
#     - The typed available_sensor_topics mechanism, with has_camera and
#       available_image_topics DERIVED from that one list every derivation pass
#       (never cached past it, never a second independent scan).
#     - Full NodeClassIF wiring: capabilities_query and device_info_query
#       services; latched info/status/status_str publishers; a 2 Hz status timer
#       that re-derives the sensor topic list on every tick.
#     - NavPose publishing, delegated to NPXDeviceIF exactly the way
#       device_if_rbx.py does it whenever a caller supplies getNavPoseCb, plus
#       the same 10 Hz bridge from that navpose into the current_* control
#       attributes.
#     - The two sim-specific commands (set_camera_view_mode,
#       set_environment_option) and the two selectors (select_simulator,
#       select_robot_config).
#     - apply_capability_profile(), the one sanctioned re-derivation path -- see
#       its docstring for why a robot-config change is allowed to move the
#       cached flags and why that is wire-safe.
#
#   NOT built -- explicit, documented gaps, not oversights:
#     - device_if_rbx.py's blocking-wait goto convergence logic
#       (setpoint_position_local_body / setpoint_attitude_ned /
#       setpoint_location_global_wgs84: NED/ENU/WGS84 conversions plus an
#       error-bound polling loop against getNavPoseCb). Replicating it correctly
#       needs its own verification pass against a live simulator, and wiring it
#       in half-verified is worse than leaving it out: it would report a
#       cmd_success that nothing checked. The goto*Cb methods below are
#       therefore thin delegators -- fire the injected function, toggle the same
#       process_current / process_last / cmd_success bookkeeping RBX does --
#       WITHOUT the convergence wait. The connected simulator's own bridge or
#       vehicle model owns reaching the setpoint.
#     - SaveDataIF / SettingsIF / Transform3DIF integration. The fields these
#       would populate exist on SimStatus.msg (contract-compliant shape) but the
#       shared machinery is not constructed here. Wiring SettingsIF needs a
#       per-simulator cap/factory settings source this generic class has no
#       defensible default for.
# ############################################################################

import copy
import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav

from std_msgs.msg import Empty, Int32, UInt32, Bool, String, Float32

from geographic_msgs.msg import GeoPoint

from nepi_interfaces.msg import AxisControls, ErrorBounds, GotoErrors, MotorControl
from nepi_interfaces.msg import GotoPose, GotoPosition, GotoLocation
from nepi_interfaces.srv import DeviceInfoQuery, DeviceInfoQueryResponse, DeviceInfoQueryRequest

from nepi_api.messages_if import MsgIF
from nepi_api.node_if import NodeClassIF
from nepi_api.device_if_npx import NPXDeviceIF

from nepi_app_sim_connector.msg import SensorTopicInfo, SimInfo, SimStatus
from nepi_app_sim_connector.srv import SimCapabilitiesQuery, SimCapabilitiesQueryResponse, SimCapabilitiesQueryRequest


class SimDeviceIF:
  # Class-attribute defaults, not just instance attributes set in __init__.
  # NodeClassIF's own configs_dict wrapper invokes the caller's reset_callback /
  # init_callback SYNCHRONOUSLY during its own construction (init_configs=True
  # unconditionally triggers reset_config() -> resetCb() -> initCb() ->
  # publish_status(), all before `self.node_if = NodeClassIF(...)` below has
  # finished assigning). Without a class-level default, self.node_if raises
  # AttributeError at that point rather than resolving to None. This mirrors
  # device_if_rbx.py's own `node_if = None` class attribute, which exists for
  # exactly this reason.
  node_if = None
  npx_if = None
  ready = False

  # Default Global Values
  STATUS_UPDATE_RATE_HZ = 2  # matches device_if_rbx.py
  UPDATE_NAVPOSE_RATE_HZ = 10  # matches device_if_rbx.py's current_* bridge

  # Factory defaults sourced from device_if_rbx.py's own factory control values,
  # not guessed. FACTORY_HOME_LOCATION is the "no home known" sentinel triple
  # rather than RBX's Seattle coordinates: a generic simulator has no meaningful
  # default location, and -999 is the sentinel the RBX home plumbing already
  # treats as "use current".
  FACTORY_CMD_TIMEOUT_SEC = 25
  FACTORY_HOME_LOCATION = [-999.0, -999.0, -999.0]
  FACTORY_GOTO_MAX_ERROR_M = 2.0
  FACTORY_GOTO_MAX_ERROR_DEG = 2.0
  FACTORY_GOTO_STABILIZED_SEC = 1.0
  FACTORY_ACTIVE_IMAGE_TOPIC = ""
  FACTORY_CAMERA_VIEW_MODE = ""

  # Message types scanned for the typed available_sensor_topics list. Images
  # today; lidar and IMU are already listed so a simulator that wires one up
  # needs zero code changes here, only a bridge-side announcement.
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
               # new for sim -- typed sensor topics
               getAvailableSensorTopicsFunction = None,
               setActiveImageTopicFunction = None,
               # new for sim -- camera view/rig control
               setCameraViewModeFunction = None, available_camera_view_modes = None,
               # new for sim -- environment control
               setEnvironmentOptionFunction = None,
               available_environment_options = None,
               getAvailableEnvironmentOptionsFunction = None,
               # new for sim -- connection health
               getBridgeConnectedFunction = None, getTelemetryAgeFunction = None,
               # new for sim -- the two selectors. Reported lists plus active
               # selections; the hosting node owns what a selection means.
               getAvailableSimulatorsFunction = None,
               getSelectedSimulatorFunction = None,
               setSelectedSimulatorFunction = None,
               getAvailableRobotConfigsFunction = None,
               getSelectedRobotConfigFunction = None,
               setSelectedRobotConfigFunction = None,
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

    self.device_info = device_info
    self.device_name = device_info["device_name"]
    self.path = device_info["path"]
    self.serial_num = device_info["serial_number"]
    self.hw_version = device_info["hw_version"]
    self.sw_version = device_info["sw_version"]

    self.data_source_description = data_source_description
    self.data_ref_description = data_ref_description

    # Guards the cached caps_report against a capabilities_query landing in the
    # middle of apply_capability_profile's re-derivation.
    self.caps_lock = threading.Lock()

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
    self.status_msg.ready = False
    self.status_msg.battery = -999
    self.status_msg.errors_current = GotoErrors()
    self.status_msg.errors_prev = GotoErrors()
    self.status_msg.last_error_message = ""
    self.status_msg.last_cmd_string = ""
    self.status_msg.bridge_connected = False
    self.status_msg.telemetry_age_sec = -1.0

    self.info_report = SimInfo()
    self.info_report.connected = False
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
    self.info_report.cmd_timeout = self.FACTORY_CMD_TIMEOUT_SEC

    self.caps_report = SimCapabilitiesQueryResponse()
    self.caps_report.device_name = self.device_name
    self.caps_report.device_path = self.path
    self.caps_report.device_node_name = self.node_name
    self.caps_report.data_products = []

    self.cmd_success_current = False
    self.active_image_topic = self.FACTORY_ACTIVE_IMAGE_TOPIC
    self.camera_view_mode = self.FACTORY_CAMERA_VIEW_MODE
    self.last_cmd_string = ""

    # current_* control attributes, bridged from the device's own navpose on a
    # timer. Same shape and same purpose as device_if_rbx.py's: whatever future
    # pass adds real goto convergence reads these, not the raw dict.
    self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    self.current_heading_deg = 0.0
    self.current_geoid_height_m = 0.0
    self.current_orientation_enu_degs = [0.0, 0.0, 0.0]
    self.current_orientation_ned_degs = [0.0, 0.0, 0.0]
    self.current_position_enu_m = [0.0, 0.0, 0.0]
    self.current_position_ned_m = [0.0, 0.0, 0.0]
    self.current_location_wgs84_geo = [0.0, 0.0, 0.0]
    self.current_location_amsl_geo = [0.0, 0.0, 0.0]

    ##############################
    # Selector callbacks. Reported-list-plus-active-selection, same shape the
    # contract already uses for available_image_topics / active_image_topic. The
    # hosting node owns discovery and what a selection means; this class only
    # owns the ROS surface for it.
    self.getAvailableSimulatorsFunction = getAvailableSimulatorsFunction
    self.getSelectedSimulatorFunction = getSelectedSimulatorFunction
    self.setSelectedSimulatorFunction = setSelectedSimulatorFunction
    self.getAvailableRobotConfigsFunction = getAvailableRobotConfigsFunction
    self.getSelectedRobotConfigFunction = getSelectedRobotConfigFunction
    self.setSelectedRobotConfigFunction = setSelectedRobotConfigFunction

    ##############################
    # Connection-health callbacks
    self.getBridgeConnectedFunction = getBridgeConnectedFunction
    self.getTelemetryAgeFunction = getTelemetryAgeFunction

    ##############################
    # Sensor-topic and environment-option callbacks
    self.getAvailableSensorTopicsFunction = getAvailableSensorTopicsFunction
    self.getAvailableEnvironmentOptionsFunction = getAvailableEnvironmentOptionsFunction

    ##############################
    # NavPose
    self.getNavPoseCb = getNavPoseCb
    self.navpose_update_rate = max(1, min(10, navpose_update_rate))

    ##############################
    # Capability derivation. Each optional callback's None-ness decides the
    # matching has_* flag, computed once here and cached on self.caps_report --
    # identical in spirit to device_if_rbx.py's own derivation block.
    self.applyCapabilityProfile(
        axisControls = axisControls,
        getBatteryPercentFunction = getBatteryPercentFunction,
        setMotorControlRatio = setMotorControlRatio,
        getMotorControlRatios = getMotorControlRatios,
        manualControlsReadyFunction = manualControlsReadyFunction,
        autonomousControlsReadyFunction = autonomousControlsReadyFunction,
        states = states, getStateIndFunction = getStateIndFunction,
        setStateIndFunction = setStateIndFunction,
        modes = modes, getModeIndFunction = getModeIndFunction,
        setModeIndFunction = setModeIndFunction,
        checkStopFunction = checkStopFunction,
        setup_actions = setup_actions,
        setSetupActionIndFunction = setSetupActionIndFunction,
        go_actions = go_actions,
        setGoActionIndFunction = setGoActionIndFunction,
        getHomeFunction = getHomeFunction, setHomeFunction = setHomeFunction,
        goHomeFunction = goHomeFunction, goStopFunction = goStopFunction,
        gotoPoseFunction = gotoPoseFunction,
        gotoPositionFunction = gotoPositionFunction,
        gotoLocationFunction = gotoLocationFunction,
        wheel_count = wheel_count, motor_count = motor_count,
        setActiveImageTopicFunction = setActiveImageTopicFunction,
        setCameraViewModeFunction = setCameraViewModeFunction,
        available_camera_view_modes = available_camera_view_modes,
        setEnvironmentOptionFunction = setEnvironmentOptionFunction,
        available_environment_options = available_environment_options)

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
        'sim_device_info_query': {
            'namespace': self.namespace,
            'topic': 'device_info_query',
            'srv': DeviceInfoQuery,
            'req': DeviceInfoQueryRequest(),
            'resp': DeviceInfoQueryResponse(),
            'callback': self.info_query_callback
        },
        'sim_capabilities_query': {
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
        'sim_status_pub': {
            'namespace': self.namespace,
            'topic': 'status',
            'msg': SimStatus,
            'qsize': 1,
            'latch': True
        },
        'sim_status_str_pub': {
            'namespace': self.namespace,
            'topic': 'status_str',
            'msg': String,
            'qsize': 1,
            'latch': True
        }
    }

    self.SUBS_DICT = {
        'sim_set_state': {
            'namespace': self.namespace, 'topic': 'set_state', 'msg': Int32,
            'qsize': None, 'callback': self.setStateCb, 'callback_args': ()
        },
        'sim_set_mode': {
            'namespace': self.namespace, 'topic': 'set_mode', 'msg': Int32,
            'qsize': None, 'callback': self.setModeCb, 'callback_args': ()
        },
        'sim_setup_action': {
            'namespace': self.namespace, 'topic': 'setup_action', 'msg': Int32,
            'qsize': None, 'callback': self.setupActionCb, 'callback_args': ()
        },
        'sim_go_action': {
            'namespace': self.namespace, 'topic': 'go_action', 'msg': Int32,
            'qsize': None, 'callback': self.goActionCb, 'callback_args': ()
        },
        'sim_set_motor_control': {
            # A "set all motors" action fires one message per motor in a
            # near-simultaneous burst, so this is sized comfortably above any
            # plausible motor count rather than left at the qsize=1 default --
            # same reasoning as device_if_rbx.py's own set_motor_control entry.
            'namespace': self.namespace, 'topic': 'set_motor_control', 'msg': MotorControl,
            'qsize': 20, 'callback': self.setMotorControlCb, 'callback_args': ()
        },
        'sim_set_goto_timeout': {
            'namespace': self.namespace, 'topic': 'set_goto_timeout', 'msg': UInt32,
            'qsize': None, 'callback': self.setCmdTimeoutCb, 'callback_args': ()
        },
        'sim_set_goto_error_bounds': {
            'namespace': self.namespace, 'topic': 'set_goto_error_bounds', 'msg': ErrorBounds,
            'qsize': None, 'callback': self.setErrorBoundsCb, 'callback_args': ()
        },
        'sim_set_home': {
            'namespace': self.namespace, 'topic': 'set_home', 'msg': GeoPoint,
            'qsize': None, 'callback': self.setHomeCb, 'callback_args': ()
        },
        'sim_set_home_current': {
            'namespace': self.namespace, 'topic': 'set_home_current', 'msg': Empty,
            'qsize': None, 'callback': self.setHomeCurrentCb, 'callback_args': ()
        },
        'sim_go_home': {
            'namespace': self.namespace, 'topic': 'go_home', 'msg': Empty,
            'qsize': None, 'callback': self.goHomeCb, 'callback_args': ()
        },
        'sim_go_stop': {
            'namespace': self.namespace, 'topic': 'go_stop', 'msg': Empty,
            'qsize': None, 'callback': self.goStopCb, 'callback_args': ()
        },
        'sim_goto_pose': {
            'namespace': self.namespace, 'topic': 'goto_pose', 'msg': GotoPose,
            'qsize': None, 'callback': self.gotoPoseCb, 'callback_args': ()
        },
        'sim_goto_position': {
            'namespace': self.namespace, 'topic': 'goto_position', 'msg': GotoPosition,
            'qsize': None, 'callback': self.gotoPositionCb, 'callback_args': ()
        },
        'sim_goto_location': {
            'namespace': self.namespace, 'topic': 'goto_location', 'msg': GotoLocation,
            'qsize': None, 'callback': self.gotoLocationCb, 'callback_args': ()
        },
        'sim_publish_status': {
            'namespace': self.namespace, 'topic': 'publish_status', 'msg': Empty,
            'qsize': None, 'callback': self.publishStatusCb, 'callback_args': ()
        },
        'sim_publish_info': {
            'namespace': self.namespace, 'topic': 'publish_info', 'msg': Empty,
            'qsize': None, 'callback': self.publishInfoCb, 'callback_args': ()
        },
        'sim_set_active_image_topic': {
            'namespace': self.namespace, 'topic': 'set_active_image_topic', 'msg': String,
            'qsize': None, 'callback': self.setActiveImageTopicCb, 'callback_args': ()
        },
        'sim_set_camera_view_mode': {
            'namespace': self.namespace, 'topic': 'set_camera_view_mode', 'msg': String,
            'qsize': None, 'callback': self.setCameraViewModeCb, 'callback_args': ()
        },
        'sim_set_environment_option': {
            'namespace': self.namespace, 'topic': 'set_environment_option', 'msg': String,
            'qsize': None, 'callback': self.setEnvironmentOptionCb, 'callback_args': ()
        },
        'sim_select_simulator': {
            'namespace': self.namespace, 'topic': 'select_simulator', 'msg': String,
            'qsize': None, 'callback': self.selectSimulatorCb, 'callback_args': ()
        },
        'sim_select_robot_config': {
            'namespace': self.namespace, 'topic': 'select_robot_config', 'msg': String,
            'qsize': None, 'callback': self.selectRobotConfigCb, 'callback_args': ()
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
    # Update vals from param server
    self.initCb(do_updates = True)

    ##############################
    # NavPose, delegated to NPXDeviceIF exactly the way device_if_rbx.py does
    # it. Two caveats inherited from that class, not introduced here:
    # NPXDeviceIF probes getNavPoseCb() once at construction and freezes its own
    # has_location / has_position / ... flags from that first return, and it
    # disables itself if that first call returns None. A simulator bridge that
    # connects after this point therefore publishes a NavPose whose has_* flags
    # are all False. Same behavior as every RBX driver today.
    if self.getNavPoseCb is not None:
      self.msg_if.pub_info("Starting NPX Device IF Initialization", log_name_list = self.log_name_list)
      self.npx_if = NPXDeviceIF(self.device_info,
          node_namespace = self.node_namespace,
          data_source_description = self.data_source_description,
          data_ref_description = self.data_ref_description,
          getNavPoseCb = self.getNavPoseCb,
          max_navpose_update_rate = self.navpose_update_rate,
          log_name_list = self.log_name_list,
          msg_if = self.msg_if)

    ##############################
    # Start Node Processes
    self.status_msg.ready = True
    self.info_report.connected = True
    self.ready = True

    self.status_timer = nepi_sdk.start_timer_process(
        float(1) / self.STATUS_UPDATE_RATE_HZ, self.statusPublishCb)

    if self.getNavPoseCb is not None:
      self.navpose_timer = nepi_sdk.start_timer_process(
          float(1) / self.UPDATE_NAVPOSE_RATE_HZ, self.updateNavposeCb)

    self.publishInfo()
    self.publish_status()

    self.msg_if.pub_info("IF Initialization Complete", log_name_list = self.log_name_list)

  #**********************
  # Public capability re-derivation

  def apply_capability_profile(self, **profile):
    """Re-derives and republishes the cached capability report from a new profile.

    The contract decides capabilities once at construction and caches them,
    because that is what lets a client render controls purely from the flags.
    Selecting a different robot config is the one case that genuinely changes a
    robot's kind -- wheel_count, motor_count, and which goto functions exist are
    exactly what "kind of robot" means -- so it has to be able to move those
    flags. This method is that single sanctioned path: it re-runs the same
    derivation the constructor runs, in place, then force-publishes info and
    status so a client sees the new profile within one status interval.

    It is wire-safe because the ROS surface never changes. Every publisher,
    subscriber, and service is registered once at construction regardless of any
    flag, every command callback independently guards on its injected function
    being None and no-ops safely, and ROS names derive from the namespace rather
    than from any capability. Only the reported flags move.

    Args:
        **profile: Any subset of the constructor's capability keyword arguments
            (see applyCapabilityProfile). Keys omitted are treated as None or
            empty, i.e. the capability is turned off -- a profile is applied
            whole, not merged, so a config switch cannot leave a stale flag set.

    Returns:
        bool: True once the new profile has been derived and published.
    """
    self.applyCapabilityProfile(**profile)
    self.publishInfo()
    self.publish_status()
    return True

  def applyCapabilityProfile(self,
                            axisControls = None,
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
                            gotoPoseFunction = None, gotoPositionFunction = None,
                            gotoLocationFunction = None,
                            wheel_count = 0, motor_count = 0,
                            setActiveImageTopicFunction = None,
                            setCameraViewModeFunction = None,
                            available_camera_view_modes = None,
                            setEnvironmentOptionFunction = None,
                            available_environment_options = None):
    with self.caps_lock:
      if axisControls is None:
        axisControls = AxisControls()
      self.caps_report.control_support = axisControls

      self.states = states if states is not None else []
      self.getStateIndFunction = getStateIndFunction
      self.setStateIndFunction = setStateIndFunction
      self.caps_report.state_options = self.states

      self.modes = modes if modes is not None else []
      self.getModeIndFunction = getModeIndFunction
      self.setModeIndFunction = setModeIndFunction
      self.caps_report.mode_options = self.modes

      self.checkStopFunction = checkStopFunction

      self.setup_actions = setup_actions if setup_actions is not None else []
      self.setSetupActionIndFunction = setSetupActionIndFunction
      self.caps_report.setup_action_options = self.setup_actions

      self.go_actions = go_actions if go_actions is not None else []
      self.setGoActionIndFunction = setGoActionIndFunction
      self.caps_report.go_action_options = self.go_actions

      self.getBatteryPercentFunction = getBatteryPercentFunction
      self.caps_report.has_battery_feedback = getBatteryPercentFunction is not None

      self.setMotorControlRatio = setMotorControlRatio
      self.getMotorControlRatios = getMotorControlRatios
      self.manualControlsReadyFunction = manualControlsReadyFunction
      self.caps_report.has_manual_controls = setMotorControlRatio is not None

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

      self.setActiveImageTopicFunction = setActiveImageTopicFunction

      self.setCameraViewModeFunction = setCameraViewModeFunction
      self.caps_report.has_camera_view_control = setCameraViewModeFunction is not None
      self.caps_report.available_camera_view_modes = (
          available_camera_view_modes if available_camera_view_modes is not None else [])

      self.setEnvironmentOptionFunction = setEnvironmentOptionFunction
      self.caps_report.has_environment_controls = setEnvironmentOptionFunction is not None
      self.caps_report.available_environment_options = (
          available_environment_options if available_environment_options is not None else [])

    # has_camera / available_image_topics / available_sensor_topics are DERIVED,
    # not independently tracked -- refreshed here and again on every status tick.
    self.refreshSensorTopics(update_caps = True)

  #**********************
  # Sensor-topic and environment-option derivation

  def refreshSensorTopics(self, update_caps = False):
    # Re-derives available_sensor_topics / has_camera / available_image_topics
    # from getAvailableSensorTopicsFunction(), optionally writing the result into
    # caps_report too. Never cached longer than one derivation pass: a
    # simulator's live topic set can change mid-session (a second robot
    # spawning, a camera disappearing), and a has_camera=True left standing over
    # an empty list would be a lie the RUI renders controls from.
    sensor_topics = []
    if self.getAvailableSensorTopicsFunction is not None:
      try:
        sensor_topics = self.getAvailableSensorTopicsFunction() or []
      except Exception as e:
        self.msg_if.pub_warn("getAvailableSensorTopicsFunction failed: " + str(e),
                             throttle_s = 5.0, log_name_list = self.log_name_list)
        sensor_topics = []

    sensor_topic_msgs = []
    image_topics = []
    for entry in sensor_topics:
      try:
        topic_name, msg_type = entry[0], entry[1]
      except Exception:
        continue
      info = SensorTopicInfo()
      info.topic_name = topic_name
      info.msg_type = msg_type
      sensor_topic_msgs.append(info)
      if msg_type == self.CAMERA_MSG_TYPE:
        image_topics.append(topic_name)

    has_camera = len(image_topics) > 0

    if update_caps:
      with self.caps_lock:
        self.caps_report.available_sensor_topics = sensor_topic_msgs
        self.caps_report.has_camera = has_camera
        self.caps_report.available_image_topics = image_topics
        self.caps_report.active_image_topic = self.active_image_topic

    return sensor_topic_msgs, has_camera, image_topics

  def refreshEnvironmentOptions(self):
    # available_environment_options is bridge-announced, so it is live-refreshed
    # from its own getter rather than frozen at construction -- the contract
    # table lists it alongside available_sensor_topics as genuinely dynamic. A
    # missing getter leaves whatever the profile declared statically.
    if self.getAvailableEnvironmentOptionsFunction is None:
      return
    try:
      options = self.getAvailableEnvironmentOptionsFunction() or []
    except Exception as e:
      self.msg_if.pub_warn("getAvailableEnvironmentOptionsFunction failed: " + str(e),
                           throttle_s = 5.0, log_name_list = self.log_name_list)
      return
    with self.caps_lock:
      self.caps_report.available_environment_options = list(options)

  #**********************
  # Config callbacks -- pass-through stubs, matching device_if_rbx.py's own
  # initCb / resetCb / factoryResetCb (RBX does not persist meaningful config
  # through these either; not a gap introduced here)

  def initCb(self, do_updates = False):
    if self.node_if is not None:
      self.active_image_topic = self.node_if.get_param('active_image_topic')
      self.camera_view_mode = self.node_if.get_param('camera_view_mode')
    if do_updates:
      pass
    self.publish_status()

  def resetCb(self, do_updates = True):
    self.initCb(do_updates = True)

  def factoryResetCb(self, do_updates = True):
    self.initCb(do_updates = True)

  #**********************
  # NavPose bridge

  def updateNavposeCb(self, timer):
    self.updateCurrentNavpose()

  def updateCurrentNavpose(self):
    # Bridges the device's own navpose into the current_* control attributes,
    # the same way device_if_rbx.py's _updateCurrentNavpose does. Nothing in
    # this file consumes them yet -- the goto delegators do not run a
    # convergence check (see the scope note at the top) -- but they are the
    # documented input a future convergence pass reads, and keeping them live
    # now means that pass does not also have to add the plumbing.
    if self.getNavPoseCb is None:
      return
    try:
      navpose_dict = self.getNavPoseCb()
    except Exception as e:
      self.msg_if.pub_warn("getNavPoseCb failed: " + str(e), throttle_s = 5.0,
                           log_name_list = self.log_name_list)
      return
    if navpose_dict is None:
      return
    try:
      self.navpose_dict = navpose_dict
      self.current_heading_deg = navpose_dict['heading_deg']
      self.current_geoid_height_m = navpose_dict['geoid_height_meters']
      self.current_orientation_enu_degs = [navpose_dict['roll_deg'],
                                          navpose_dict['pitch_deg'],
                                          navpose_dict['yaw_deg']]
      ned_dict = nepi_nav.convert_navpose_enu2ned(copy.deepcopy(navpose_dict))
      self.current_orientation_ned_degs = [ned_dict['roll_deg'],
                                          ned_dict['pitch_deg'],
                                          ned_dict['yaw_deg']]
      self.current_position_enu_m = [navpose_dict['x_m'], navpose_dict['y_m'],
                                     navpose_dict['z_m']]
      self.current_position_ned_m = [ned_dict['x_m'], ned_dict['y_m'], ned_dict['z_m']]
      self.current_location_wgs84_geo = [navpose_dict['latitude'],
                                         navpose_dict['longitude'],
                                         navpose_dict['altitude_m']]
      amsl_dict = nepi_nav.convert_navpose_wgs842amsl(copy.deepcopy(navpose_dict))
      self.current_location_amsl_geo = [amsl_dict['latitude'], amsl_dict['longitude'],
                                        amsl_dict['altitude_m']]
    except Exception as e:
      self.msg_if.pub_warn("Failed to update current navpose: " + str(e), throttle_s = 5.0,
                           log_name_list = self.log_name_list)

  #**********************
  # Query service callbacks

  def info_query_callback(self, _):
    """Handles a DeviceInfoQuery service request.

    Args:
        _: Unused service request object.

    Returns:
        DeviceInfoQueryResponse: The device info report, populated with this
            device's name, path, node name and namespace, and version strings.
    """
    resp = DeviceInfoQueryResponse()
    resp.device_name = self.device_name
    resp.device_path = self.path
    resp.node_name = self.node_name
    resp.node_namespace = self.node_namespace
    resp.serial_num = self.serial_num
    resp.hw_version = self.hw_version
    resp.sw_version = self.sw_version
    resp.type = 'SIM'
    return resp

  def capabilities_query_callback(self, _):
    """Handles a SimCapabilitiesQuery service request.

    Args:
        _: Unused service request object.

    Returns:
        SimCapabilitiesQueryResponse: The cached capabilities report, with the
            derived sensor-topic block and the bridge-announced environment
            options refreshed first rather than returning a stale
            construction-time snapshot.
    """
    self.refreshSensorTopics(update_caps = True)
    self.refreshEnvironmentOptions()
    with self.caps_lock:
      return copy.deepcopy(self.caps_report)

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
    self.info_report.camera_view_mode = self.camera_view_mode
    if self.node_if is not None:
      error_bounds = ErrorBounds()
      error_bounds.max_distance_error_m = self.node_if.get_param('max_error_m')
      error_bounds.max_rotation_error_deg = self.node_if.get_param('max_error_deg')
      error_bounds.min_stabilize_time_s = self.node_if.get_param('stabilized_sec')
      self.info_report.error_bounds = error_bounds
      self.info_report.cmd_timeout = self.node_if.get_param('cmd_timeout')
    if not nepi_sdk.is_shutdown() and self.node_if is not None:
      self.node_if.publish_pub('sim_info_pub', self.info_report)

  def statusPublishCb(self, timer):
    self.publish_status()

  def publishStatusCb(self, msg):
    self.publish_status()

  def publish_status(self):
    """Assembles and publishes the timed simulator device status.

    Re-derives the typed sensor-topic list and the bridge-announced environment
    options, refreshes battery, control readiness, motor state, connection
    health, and both selector reports, then publishes to the status and
    status_str topics.
    """
    # publish_status is invoked during NodeClassIF construction (via the config
    # reset callback) before the publishers exist; self.ready is only True at
    # the end of __init__, so those early calls no-op. Same guard, same reason,
    # as device_if_rbx.py's own.
    if self.ready is False:
      return

    sensor_topic_msgs, has_camera, image_topics = self.refreshSensorTopics(update_caps = True)
    self.refreshEnvironmentOptions()
    self.status_msg.available_sensor_topics = sensor_topic_msgs

    self.status_msg.device_name = self.device_name
    self.status_msg.last_cmd_string = self.last_cmd_string

    if self.getBatteryPercentFunction is not None:
      self.status_msg.battery = self.getBatteryPercentFunction()
    else:
      self.status_msg.battery = -999

    if self.manualControlsReadyFunction is not None:
      self.status_msg.manual_control_mode_ready = self.manualControlsReadyFunction()
    else:
      self.status_msg.manual_control_mode_ready = False

    if self.autonomousControlsReadyFunction is not None:
      self.status_msg.autonomous_control_mode_ready = self.autonomousControlsReadyFunction()
    else:
      self.status_msg.autonomous_control_mode_ready = False

    if self.getMotorControlRatios is not None:
      self.status_msg.current_motor_control_settings = self.get_motor_controls_status_msg(
          self.getMotorControlRatios())
    else:
      self.status_msg.current_motor_control_settings = []

    if self.getBridgeConnectedFunction is not None:
      self.status_msg.bridge_connected = self.getBridgeConnectedFunction()
    if self.getTelemetryAgeFunction is not None:
      self.status_msg.telemetry_age_sec = self.getTelemetryAgeFunction()

    # Both selectors report an empty list and an empty selection safely when
    # nothing is available -- no getter is required, and none of these calls
    # blocks or waits on a simulator appearing.
    simulators = []
    simulator_names = []
    if self.getAvailableSimulatorsFunction is not None:
      try:
        simulators, simulator_names = self.getAvailableSimulatorsFunction()
      except Exception as e:
        self.msg_if.pub_warn("getAvailableSimulatorsFunction failed: " + str(e),
                             throttle_s = 5.0, log_name_list = self.log_name_list)
        simulators, simulator_names = [], []
    self.status_msg.available_simulators = list(simulators)
    self.status_msg.available_simulator_names = list(simulator_names)
    self.status_msg.selected_simulator = self.getSelectionStr(self.getSelectedSimulatorFunction)

    robot_configs = []
    if self.getAvailableRobotConfigsFunction is not None:
      try:
        robot_configs = self.getAvailableRobotConfigsFunction() or []
      except Exception as e:
        self.msg_if.pub_warn("getAvailableRobotConfigsFunction failed: " + str(e),
                             throttle_s = 5.0, log_name_list = self.log_name_list)
        robot_configs = []
    self.status_msg.available_robot_configs = list(robot_configs)
    self.status_msg.selected_robot_config = self.getSelectionStr(self.getSelectedRobotConfigFunction)

    if not nepi_sdk.is_shutdown() and self.node_if is not None:
      self.node_if.publish_pub('sim_status_pub', self.status_msg)
      self.node_if.publish_pub('sim_status_str_pub', String(data = str(self.status_msg)))

  def getSelectionStr(self, getter):
    if getter is None:
      return ""
    try:
      value = getter()
    except Exception as e:
      self.msg_if.pub_warn("Selection getter failed: " + str(e), throttle_s = 5.0,
                           log_name_list = self.log_name_list)
      return ""
    return "" if value is None else str(value)

  def get_motor_controls_status_msg(self, motor_controls):
    """Builds a list of MotorControl messages from a speed-ratio list.

    Args:
        motor_controls (list): Ordered list of speed ratios, one per motor.

    Returns:
        list: A list of MotorControl messages with motor_ind and speed_ratio
            populated for each motor.
    """
    mcs = []
    for i, ratio in enumerate(motor_controls):
      mc = MotorControl()
      mc.motor_ind = i
      mc.speed_ratio = ratio
      mcs.append(mc)
    return mcs

  def update_error_msg(self, err_str):
    """Records and logs a command-rejection or failure message.

    Args:
        err_str (str): Calm, specific description of what happened.
    """
    self.msg_if.pub_warn(err_str, log_name_list = self.log_name_list)
    self.status_msg.last_error_message = err_str

  #**********************
  # Selector command callbacks

  def selectSimulatorCb(self, msg):
    self.msg_if.pub_info("Received select simulator message", log_name_list = self.log_name_list)
    if self.setSelectedSimulatorFunction is None:
      self.update_error_msg("Ignoring select_simulator, no simulator selection function")
      return
    self.setSelectedSimulatorFunction(msg.data)
    self.publish_status()

  def selectRobotConfigCb(self, msg):
    self.msg_if.pub_info("Received select robot config message", log_name_list = self.log_name_list)
    if self.setSelectedRobotConfigFunction is None:
      self.update_error_msg("Ignoring select_robot_config, no robot config selection function")
      return
    # The hosting node applies the new profile through
    # apply_capability_profile, which republishes info and status itself.
    self.setSelectedRobotConfigFunction(msg.data)

  #**********************
  # New-for-sim command callbacks

  def setActiveImageTopicCb(self, msg):
    self.msg_if.pub_info("Received set active image topic message", log_name_list = self.log_name_list)
    self.active_image_topic = msg.data
    if self.node_if is not None:
      self.node_if.set_param('active_image_topic', msg.data)
      self.node_if.save_config()
    if self.setActiveImageTopicFunction is not None:
      self.setActiveImageTopicFunction(msg.data)
    self.publishInfo()

  def setCameraViewModeCb(self, msg):
    self.msg_if.pub_info("Received set camera view mode message", log_name_list = self.log_name_list)
    if self.setCameraViewModeFunction is None:
      self.update_error_msg("Ignoring set_camera_view_mode, no camera view control")
      return
    self.camera_view_mode = msg.data
    if self.node_if is not None:
      self.node_if.set_param('camera_view_mode', msg.data)
      self.node_if.save_config()
    self.setCameraViewModeFunction(msg.data)
    self.publishInfo()

  def setEnvironmentOptionCb(self, msg):
    self.msg_if.pub_info("Received set environment option message", log_name_list = self.log_name_list)
    if self.setEnvironmentOptionFunction is None:
      self.update_error_msg("Ignoring set_environment_option, no environment controls")
      return
    self.setEnvironmentOptionFunction(msg.data)

  #**********************
  # Reused RBX-style command callbacks -- thin delegators. See the scope note at
  # the top of this file: these do NOT reimplement device_if_rbx.py's
  # blocking-wait goto convergence polling.

  def setStateCb(self, msg):
    if self.setStateIndFunction is None:
      self.update_error_msg("Ignoring set_state, no state function")
      return
    new_state_ind = msg.data
    if new_state_ind < 0 or new_state_ind > (len(self.states) - 1):
      self.update_error_msg("No matching sim state found")
      return
    self.setStateIndFunction(new_state_ind)
    self.last_cmd_string = "set_sim_state('" + str(self.states[new_state_ind]) + "')"
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
    self.last_cmd_string = "set_sim_mode('" + str(self.modes[new_mode_ind]) + "')"
    self.publishInfo()

  def setupActionCb(self, msg):
    if self.setSetupActionIndFunction is None:
      self.update_error_msg("Ignoring setup_action, no set action function")
      return
    action_ind = msg.data
    if action_ind < 0 or action_ind > (len(self.setup_actions) - 1):
      self.update_error_msg("No matching sim setup action found")
      return
    if self.status_msg.ready is False:
      self.update_error_msg("Ignoring setup_action, another command process is active")
      return
    action = self.setup_actions[action_ind]
    self.status_msg.process_current = action
    self.status_msg.ready = False
    success = self.setSetupActionIndFunction(action_ind)
    self.status_msg.process_last = action
    self.status_msg.process_current = "None"
    self.status_msg.cmd_success = bool(success)
    self.status_msg.ready = True
    self.last_cmd_string = "setup_sim_action('" + str(action) + "')"
    self.publishInfo()

  def goActionCb(self, msg):
    if self.setGoActionIndFunction is None:
      self.update_error_msg("Ignoring go_action, no go action function")
      return
    action_ind = msg.data
    if action_ind < 0 or action_ind > (len(self.go_actions) - 1):
      self.update_error_msg("No matching sim go action found")
      return
    if self.status_msg.ready is False:
      self.update_error_msg("Ignoring go_action, another command process is active")
      return
    action = self.go_actions[action_ind]
    self.status_msg.process_current = action
    self.status_msg.ready = False
    success = self.setGoActionIndFunction(action_ind)
    self.status_msg.process_last = action
    self.status_msg.process_current = "None"
    self.status_msg.cmd_success = bool(success)
    self.status_msg.ready = True
    self.last_cmd_string = "go_sim_action('" + str(action) + "')"
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
      self.node_if.save_config()
    self.publishInfo()

  def setErrorBoundsCb(self, msg):
    if self.node_if is not None:
      self.node_if.set_param('max_error_m', msg.max_distance_error_m)
      self.node_if.set_param('max_error_deg', msg.max_rotation_error_deg)
      self.node_if.set_param('stabilized_sec', msg.min_stabilize_time_s)
      self.node_if.save_config()
    self.publishInfo()

  def setHomeCb(self, msg):
    if self.setHomeFunction is None:
      self.update_error_msg("Ignoring set_home, no set home function")
      return
    # -999 on any axis means "keep the current value for that axis", the same
    # sentinel convention the goto messages use.
    new_home = [msg.latitude, msg.longitude, msg.altitude]
    for i, val in enumerate(new_home):
      if val == -999:
        new_home[i] = self.current_location_wgs84_geo[i]
    home_geo = GeoPoint()
    home_geo.latitude = new_home[0]
    home_geo.longitude = new_home[1]
    home_geo.altitude = new_home[2]
    self.setHomeFunction(home_geo)
    if self.node_if is not None:
      self.node_if.set_param('home_location', new_home)
      self.node_if.save_config()
    self.publishInfo()

  def setHomeCurrentCb(self, msg):
    if self.setHomeFunction is None:
      self.update_error_msg("Ignoring set_home_current, no set home function")
      return
    # Built as a GeoPoint rather than handed the raw three-element list, because
    # setHomeFunction's contract is a GeoPoint.
    home_geo = GeoPoint()
    home_geo.latitude = self.current_location_wgs84_geo[0]
    home_geo.longitude = self.current_location_wgs84_geo[1]
    home_geo.altitude = self.current_location_wgs84_geo[2]
    self.setHomeFunction(home_geo)
    if self.node_if is not None:
      self.node_if.set_param('home_location', list(self.current_location_wgs84_geo))
      self.node_if.save_config()
    self.publishInfo()

  def goHomeCb(self, msg):
    if self.goHomeFunction is None:
      self.update_error_msg("Ignoring go_home, no go home function")
      return
    self.status_msg.process_current = "Go Home"
    self.status_msg.ready = False
    self.update_prev_errors()
    success = self.goHomeFunction()
    self.status_msg.process_last = "Go Home"
    self.status_msg.process_current = "None"
    self.status_msg.cmd_success = bool(success)
    self.status_msg.ready = True
    self.last_cmd_string = "go_sim_home()"
    self.publishInfo()

  def goStopCb(self, msg):
    if self.goStopFunction is None:
      self.update_error_msg("Ignoring go_stop, no go stop function")
      return
    self.status_msg.process_current = "Stop"
    self.status_msg.ready = False
    self.update_prev_errors()
    success = self.goStopFunction()
    self.status_msg.process_last = "Stop"
    self.status_msg.process_current = "None"
    self.status_msg.cmd_success = bool(success)
    self.status_msg.ready = True
    self.last_cmd_string = "go_sim_stop()"
    self.publishInfo()

  # The three goto delegators. See the scope note at the top of this file:
  # fire-and-forget delegation to the injected function, no convergence-polling
  # wait, so cmd_success is left untouched rather than claiming a success that
  # nothing verified.
  def gotoPoseCb(self, msg):
    if self.gotoPoseFunction is None:
      self.update_error_msg("Ignoring goto_pose, no goto pose function")
      return
    if self.autonomousControlsReadyFunction is not None and not self.autonomousControlsReadyFunction():
      self.update_error_msg("Ignoring goto_pose, autonomous controls not ready")
      return
    attitude_enu_degs = [msg.roll_deg, msg.pitch_deg, msg.yaw_deg]
    self.status_msg.process_current = "GoTo Pose"
    self.update_prev_errors()
    self.gotoPoseFunction(attitude_enu_degs)
    self.status_msg.process_last = "GoTo Pose"
    self.status_msg.process_current = "None"
    self.last_cmd_string = "goto_sim_pose('" + str(attitude_enu_degs) + "')"
    self.publishInfo()

  def gotoPositionCb(self, msg):
    if self.gotoPositionFunction is None:
      self.update_error_msg("Ignoring goto_position, no goto position function")
      return
    if self.autonomousControlsReadyFunction is not None and not self.autonomousControlsReadyFunction():
      self.update_error_msg("Ignoring goto_position, autonomous controls not ready")
      return
    self.status_msg.process_current = "GoTo Position"
    self.update_prev_errors()
    self.gotoPositionFunction(msg)
    self.status_msg.process_last = "GoTo Position"
    self.status_msg.process_current = "None"
    self.last_cmd_string = ("goto_sim_position('" + str([msg.x_meters, msg.y_meters,
                            msg.z_meters, msg.yaw_deg]) + "')")
    self.publishInfo()

  def gotoLocationCb(self, msg):
    if self.gotoLocationFunction is None:
      self.update_error_msg("Ignoring goto_location, no goto location function")
      return
    if self.autonomousControlsReadyFunction is not None and not self.autonomousControlsReadyFunction():
      self.update_error_msg("Ignoring goto_location, autonomous controls not ready")
      return
    self.status_msg.process_current = "GoTo Location"
    self.update_prev_errors()
    self.gotoLocationFunction(msg)
    self.status_msg.process_last = "GoTo Location"
    self.status_msg.process_current = "None"
    self.last_cmd_string = ("goto_sim_location('" + str([msg.lat, msg.long,
                            msg.altitude_meters, msg.yaw_deg]) + "')")
    self.publishInfo()

  def update_prev_errors(self):
    """Rolls the current setpoint errors into the previous-errors report."""
    self.status_msg.errors_prev = copy.deepcopy(self.status_msg.errors_current)
    self.status_msg.errors_current = GotoErrors()

  def get_namespace(self):
    """Returns this interface's ROS namespace.

    Returns:
        str: The sim sub-namespace under the hosting node, e.g.
            "/nepi/device1/app_sim_connector/sim".
    """
    return self.namespace
