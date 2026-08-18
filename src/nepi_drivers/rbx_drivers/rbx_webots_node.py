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

# RBX driver node for a Webots simulated robot -- rbx_gazebo_node.py's exact
# pattern, ported. The whole point of matching rbx_webots_bridge.py's wire
# protocol to sim_bridge_node.py's (see that bridge's own docstring) is that
# this file needed only renaming, not new logic: same bridge-loop/reconnect
# shape, same closed-loop 2D goto controller (a Webots controller robot has no
# onboard autopilot to delegate to either), same capability gaps for the same
# reasons (no arm/disarm, no battery, no WGS84 location).
#
# ############################################################################
# Single-camera, single-topic: this world (rbx_rover.wbt, copied from
# sim_connector_rover.wbt) has only ONE Camera device with no repositionable
# rig, unlike Gazebo's generic_rover which has two independently-posed camera
# links. An earlier version of this driver reported the one camera under two
# names (SCENE_CAMERA/ROBOT_CAMERA) with a camera_view_mode Setting to
# "switch" between them, copied from the Gazebo rover's pattern -- but that
# offered a view that does not exist here (same issue found and fixed in
# rbx_webots_quadcopter_node.py). Now reported under a single CAMERA_NAME
# topic, no view-mode Setting. Reference frames/offsets are still
# declarative-only, matching rbx_gazebo_node.py's own gap.
# ############################################################################

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
from nepi_sdk import nepi_settings
from nepi_sdk import nepi_img

from std_msgs.msg import UInt32, String
from sensor_msgs.msg import Image

from nepi_interfaces.msg import AxisControls
from geographic_msgs.msg import GeoPoint

from nepi_api.device_if_rbx import RBXRobotIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'RBX_WEBOTS' # Use in display menus
FILE_TYPE = 'NODE'


#########################################
# Node Class
#########################################

class WebotsNode:

  # This world has exactly one fixed Camera device with no repositionable
  # rig -- reported under a single honest topic name. An earlier version of
  # this driver reported it under two names (SCENE_CAMERA/ROBOT_CAMERA) with
  # a camera_view_mode Setting to "switch" between them, copied from
  # rbx_sim_node.py's rover pattern -- but the Gazebo rover's two names
  # correspond to two REAL, independently-posed camera links, while this
  # world has exactly one, so that Setting was a pure UI fiction offering a
  # view that does not exist (same issue found and fixed in
  # rbx_webots_quadcopter_node.py -- see that file's CAMERA_NAME comment).
  CAMERA_NAME = "robot_camera"

  ROBOT_MAIN_REFERENCE_FRAME = "base_link"

  # This world has no obstacle-course model -- environment options are an
  # honest no-op on the bridge side (matching sim_connector_bridge_webots.py's
  # own documented gap), but still declared here so the capability/UI surface
  # is consistent with the Gazebo driver rather than silently absent.
  ENVIRONMENT_OPTIONS = ["FLAT_GROUND", "OBSTACLE_COURSE"]
  OBSTACLE_COURSE_OPTION = "OBSTACLE_COURSE"

  # Ported from rbx_sim_node.py (the Gazebo rover's real, current driver --
  # see docs/WEBOTS_RBX_DRIVER_PLAN.md's note on this file having been built
  # from the stale rbx_gazebo_node.py template originally, before RBX_SIM's
  # camera-offset/capability-toggle Settings existed). Unlike Gazebo's rover,
  # this world has ONE fixed Camera device, not a repositionable rig or two
  # independently-posed camera links -- camera_offset_x/y/z are declared here
  # (kept as an honest "not wired up yet" placeholder, unlike camera_view_mode
  # and scene_offset_x/y/z, which are gone entirely -- there genuinely is one
  # real camera that could someday get a repositionable mount, unlike a
  # second camera that simply does not exist) so the RUI renders the same
  # offset control surface Gazebo's rover has, but sendCameraSettings below
  # sends them to webots_rbx_bridge.py as a documented no-op, matching this
  # driver's existing honest treatment of RESET_SIM/environment.
  CAMERA_SETTING_NAMES = ("camera_offset_x", "camera_offset_y", "camera_offset_z")
  ENVIRONMENT_SETTING_NAMES = ("environment",)

  # Sim Connector's own per-robot-config "customize the capabilities that are
  # open" toggles -- same mechanism and same names as rbx_sim_node.py's own
  # CAPABILITY_SETTING_NAMES (see that file's comment for the full reasoning).
  # Enforced in autonomousControlsReady/teleopControlsReady below, not just
  # hidden in the RUI, so a client bypassing the RUI can't do what was turned
  # off either.
  CAPABILITY_SETTING_NAMES = ("autonomous_movement_enabled", "teleop_movement_enabled",
                              "camera_controls_enabled", "enabled_image_sources")

  CAP_SETTINGS = dict(
    max_linear_speed_mps = {"type":"Float","name":"max_linear_speed_mps","options":["0.05","2.0"]},
    max_angular_rate_dps = {"type":"Float","name":"max_angular_rate_dps","options":["5.0","180.0"]},
    environment = {"type":"Discrete","name":"environment","options":ENVIRONMENT_OPTIONS},
    camera_offset_x = {"type":"Float","name":"camera_offset_x","options":["-10.0","10.0"]},
    camera_offset_y = {"type":"Float","name":"camera_offset_y","options":["-10.0","10.0"]},
    camera_offset_z = {"type":"Float","name":"camera_offset_z","options":["-10.0","10.0"]},
    autonomous_movement_enabled = {"type":"Discrete","name":"autonomous_movement_enabled","options":["TRUE","FALSE"]},
    teleop_movement_enabled = {"type":"Discrete","name":"teleop_movement_enabled","options":["TRUE","FALSE"]},
    camera_controls_enabled = {"type":"Discrete","name":"camera_controls_enabled","options":["TRUE","FALSE"]},
    # No fixed options -- the candidate topic set is per-deployment.
    enabled_image_sources = {"type":"String","name":"enabled_image_sources"}
  )

  # max_linear_speed_mps factory/range lower than rbx_sim_node.py's: this
  # world's MOTOR_MAX_LINEAR_MPS (0.3, see sim_connector_bridge_webots.py) is
  # itself lower than Gazebo's (0.5) -- matching the physical model, not an
  # arbitrary choice.
  FACTORY_SETTINGS = dict(
    max_linear_speed_mps = {"type":"Float","name":"max_linear_speed_mps","value":"0.3"},
    max_angular_rate_dps = {"type":"Float","name":"max_angular_rate_dps","value":"45.0"},
    environment = {"type":"Discrete","name":"environment","value":ENVIRONMENT_OPTIONS[0]},
    # Placeholder factory values, not measured against this world's single
    # fixed camera pose (there is no physical offset to reproduce here the
    # way rbx_sim_node.py's values reproduce generic_rover/model.sdf's real
    # camera_link poses) -- these exist so the RUI's offset inputs have
    # something sane to show, not because moving them does anything yet.
    camera_offset_x = {"type":"Float","name":"camera_offset_x","value":"0.0"},
    camera_offset_y = {"type":"Float","name":"camera_offset_y","value":"0.0"},
    camera_offset_z = {"type":"Float","name":"camera_offset_z","value":"0.0"},
    # Both default to enabled: a robot config that never touches these
    # settings behaves exactly as this driver did before this feature existed.
    autonomous_movement_enabled = {"type":"Discrete","name":"autonomous_movement_enabled","value":"TRUE"},
    teleop_movement_enabled = {"type":"Discrete","name":"teleop_movement_enabled","value":"TRUE"},
    camera_controls_enabled = {"type":"Discrete","name":"camera_controls_enabled","value":"TRUE"},
    # Empty = unrestricted -- see the CAPABILITY_SETTING_NAMES comment above.
    enabled_image_sources = {"type":"String","name":"enabled_image_sources","value":""}
  )

  FACTORY_SETTINGS_OVERRIDES = dict()

  RBX_STATES = []
  RBX_MODES = []
  # RESET_SIM is a real setup action here, same as Gazebo's, but the bridge
  # side is a logged no-op: this world's Robot node is not a Supervisor, so it
  # cannot teleport itself (matching sim_connector_bridge_webots.py's own
  # documented RESET gap). Kept in the action list rather than omitted so the
  # capability surface matches Gazebo's -- the command is honestly accepted
  # and forwarded, just not honored physically, which is what "return False"
  # would otherwise misreport (the bridge send itself succeeds).
  RBX_SETUP_ACTIONS = ["RESET_SIM", "RETURN_HOME"]
  RBX_GO_ACTIONS = []

  GO_HOME_TIMEOUT_SEC = 60.0
  GO_HOME_POLL_INTERVAL_SEC = 0.2

  RECONNECT_INTERVAL_SEC = 3.0
  SOCKET_TIMEOUT_SEC = 5.0

  CONTROLLER_RATE_HZ = 20
  NAVPOSE_UPDATE_RATE = 10
  TELEMETRY_FRESH_SEC = 2.0
  # Ported from rbx_sim_node.py -- a keyboard teleop client is expected to
  # re-send on an interval while a key is held (see the RUI's teleop panel);
  # this is the self-healing timeout for a dropped stop/keyup packet, not the
  # client's own send interval.
  TELEOP_CMD_TIMEOUT_SEC = 0.75

  # Motor 0 = left side (wheel1+wheel3), motor 1 = right side (wheel2+wheel4)
  # on rbx_rover.wbt -- a 4-wheel model driven as a 2-sided tank drive, exactly
  # like Gazebo's 4-wheel generic_rover. MOTOR_MAX_LINEAR_MPS/WHEEL_BASE_M
  # match sim_connector_bridge_webots.py's own MOTOR_MAX_LINEAR_MPS/WHEEL_TRACK_M
  # constants -- this conversion has to agree with the bridge's physical model.
  MOTOR_MAX_LINEAR_MPS = 0.3
  MOTOR_WHEEL_BASE_M = 0.12

  GOTO_KP_LIN = 0.5
  GOTO_KP_ANG = 1.5
  GOTO_TURN_GATE_RAD = math.radians(30.0)
  GOTO_TOL_FRACTION = 0.5
  FACTORY_GOTO_TOL_M = 1.0
  FACTORY_GOTO_TOL_RAD = math.radians(1.0)

  # See rbx_gazebo_node.py's own GOTO_CMD_TIMEOUT_SEC comment for the full
  # reasoning -- identical non-holonomic-ground-vehicle argument applies here.
  GOTO_CMD_TIMEOUT_SEC = 60

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  drv_dict = dict()

  rbx_if = None

  def __init__(self):
    ####  NODE Initialization ####
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
    # Gather Driver Settings from param server drv_dict
    self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
    try:
      self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
      self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
      self.sim_host = self.drv_dict['DEVICE_DICT']['host']
      self.bridge_port = self.drv_dict['DEVICE_DICT']['bridge_port']
    except Exception as e:
      self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
      nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
      return

    ##############################
    # Bridge connection and telemetry state
    self.sock = None
    self.sock_lock = threading.Lock()
    self.last_telemetry_time = 0.0
    self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

    ##############################
    # Image relay. One real camera, one topic -- see the class-level
    # CAMERA_NAME comment for why this isn't reported under two names.
    self.image_topic_name = self.device_name + "/color_2d_image"
    self.image_pub = nepi_sdk.create_publisher(self.image_topic_name, Image, queue_size = 1)

    self.sensor_topics = [
      (self.image_topic_name + "/" + self.CAMERA_NAME, 'sensor_msgs/Image'),
    ]

    ##############################
    # Goto controller state
    self.goto_target = None
    self.goto_target_lock = threading.Lock()
    self.stop_triggered = False

    ##############################
    # Teleop state -- ported from rbx_sim_node.py. Set only from
    # setTeleopVelocity. teleop_last_cmd_time backs a self-healing timeout
    # (TELEOP_CMD_TIMEOUT_SEC) for a dropped stop/keyup packet from the RUI's
    # teleop panel, same reasoning as rbx_sim_node.py's own comment.
    self.teleop_linear_x = 0.0
    self.teleop_angular_z = 0.0
    self.teleop_last_cmd_time = 0.0

    ##############################
    # Manual motor-ratio state: motor 0 = left, motor 1 = right
    self.motor_ratios = [0.0, 0.0]

    ##############################
    # Home position state: local ENU x/y/z meters (see rbx_gazebo_node.py's
    # getHome/setHome for why this reuses the GeoPoint plumbing)
    self.home_x_m = 0.0
    self.home_y_m = 0.0
    self.home_z_m = 0.0

    ##############################
    # Initialize RBX Settings
    self.settings_dict = copy.deepcopy(self.FACTORY_SETTINGS)
    self.cap_settings = self.getCapSettings()
    self.factory_settings = self.getFactorySettings()

    self.axis_controls = AxisControls()
    self.axis_controls.x = True
    self.axis_controls.y = True
    self.axis_controls.z = False
    self.axis_controls.roll = False
    self.axis_controls.pitch = False
    self.axis_controls.yaw = True

    ##############################
    # Bridge client thread: connects, reads telemetry, reconnects on failure
    self.bridge_thread = threading.Thread(target = self.bridgeLoop)
    self.bridge_thread.daemon = True
    self.bridge_thread.start()

    ##############################
    # Launch the NEPI RBX interface.
    self.msg_if.pub_info("Launching NEPI RBX interface...")
    self.device_info_dict = dict(device_name = self.device_name,
                                 path = self.device_path,
                                 serial_number = "",
                                 hw_version = "",
                                 sw_version = "")
    self.msg_if.pub_info(str(self.device_info_dict))

    self.rbx_if = RBXRobotIF(device_info = self.device_info_dict,
                             capSettings = self.cap_settings,
                             factorySettings = self.factory_settings,
                             settingUpdateFunction = self.settingUpdateFunction,
                             getSettingsFunction = self.getSettings,
                             axisControls = self.axis_controls,
                             getBatteryPercentFunction = None,
                             states = self.RBX_STATES,
                             getStateIndFunction = self.getStateInd,
                             setStateIndFunction = self.setStateInd,
                             modes = self.RBX_MODES,
                             getModeIndFunction = self.getModeInd,
                             setModeIndFunction = self.setModeInd,
                             checkStopFunction = self.checkStopFunction,
                             setup_actions = self.RBX_SETUP_ACTIONS,
                             setSetupActionIndFunction = self.setSetupActionInd,
                             go_actions = self.RBX_GO_ACTIONS,
                             setGoActionIndFunction = self.setGoActionInd,
                             manualControlsReadyFunction = self.manualControlsReady,
                             getMotorControlRatios = self.getMotorControlRatios,
                             setMotorControlRatio = self.setMotorControlRatio,
                             teleopControlsReadyFunction = self.teleopControlsReady,
                             setTeleopVelocityFunction = self.setTeleopVelocity,
                             autonomousControlsReadyFunction = self.autonomousControlsReady,
                             getHomeFunction = self.getHome,
                             setHomeFunction = self.setHome,
                             goHomeFunction = self.returnHomeAction,
                             goStopFunction = self.goStop,
                             gotoPoseFunction = self.gotoPose,
                             gotoPositionFunction = self.gotoPosition,
                             gotoLocationFunction = None,
                             getNavPoseCb = self.getNavPoseCb,
                             navpose_update_rate = self.NAVPOSE_UPDATE_RATE,
                             data_source_description = 'simulator',
                             data_ref_description = 'simulator',
                             msg_if = self.msg_if
                            )

    self.msg_if.pub_info("... RBX interface running")
    time.sleep(1)

    self.rbx_if.setCmdTimeoutCb(UInt32(data = self.GOTO_CMD_TIMEOUT_SEC))
    self.rbx_if.setImageTopicCb(String(data = self.image_topic_name))

    controller_interval = float(1) / self.CONTROLLER_RATE_HZ
    nepi_sdk.start_timer_process(controller_interval, self.gotoControlCb)

    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.on_shutdown(self.cleanup_actions)
    nepi_sdk.spin()


  #**********************
  # Setting functions

  def getCapSettings(self):
    return self.CAP_SETTINGS

  def getFactorySettings(self):
    settings = self.getSettings()
    for setting_name in settings.keys():
      if setting_name in self.FACTORY_SETTINGS_OVERRIDES:
        settings[setting_name]['value'] = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
    return settings

  def getSettings(self):
    return self.settings_dict

  def settingUpdateFunction(self, setting):
    success = False
    setting_str = str(setting)
    setting_name = setting['name']
    msg = ""
    if nepi_settings.check_valid_setting(setting, self.cap_settings):
      if setting_name in self.settings_dict.keys():
        self.settings_dict[setting_name]['value'] = setting['value']
        success = True
      else:
        msg = (self.node_name + " Setting name" + setting_str + " is not supported")
      if success == True:
        msg = (self.node_name + " UPDATED SETTINGS " + setting_str)
        if setting_name in self.CAMERA_SETTING_NAMES:
          self.sendCameraSettings()
        if setting_name in self.ENVIRONMENT_SETTING_NAMES:
          self.setEnvironmentAction(setting['value'])
    else:
      msg = (self.node_name + " Setting data" + setting_str + " is not valid")
    return success, msg


  ##########################
  # RBX Interface Functions

  def getStateInd(self):
    return 0

  def setStateInd(self, state_ind):
    return False

  def getModeInd(self):
    return 0

  def setModeInd(self, mode_ind):
    return False

  def checkStopFunction(self):
    triggered = self.stop_triggered
    self.stop_triggered = False
    return triggered

  def manualControlsReady(self):
    # Gates manual motor-ratio commands the same way autonomousControlsReady
    # gates goto commands: require a live bridge connection. Fresh telemetry
    # is not required here (unlike goto) since a direct motor command doesn't
    # depend on knowing the current position/heading. NOT gated on
    # teleop_movement_enabled -- that toggle is teleopControlsReady's own
    # concern below (matches rbx_sim_node.py exactly: motor-ratio control and
    # teleop velocity are two distinct RBXRobotIF-facing capabilities).
    with self.sock_lock:
      return self.sock is not None

  def teleopControlsReady(self):
    if self.settings_dict['teleop_movement_enabled']['value'] != 'TRUE':
      return False
    with self.sock_lock:
      return self.sock is not None

  def setTeleopVelocity(self, linear_x, linear_y, linear_z, angular_z):
    # Ported from rbx_sim_node.py -- linear_y/linear_z ignored (a ground
    # rover has no strafe or altitude axis). Ratios in [-1,1], scaled by the
    # SAME max_linear_speed_mps/max_angular_rate_dps Settings that already
    # cap goto speed: one "how fast" knob governs autonomous and teleop
    # movement both, not a second one to keep in sync with it.
    max_lin = float(self.settings_dict['max_linear_speed_mps']['value'])
    max_ang = math.radians(float(self.settings_dict['max_angular_rate_dps']['value']))
    self.teleop_linear_x = max(-1.0, min(1.0, linear_x)) * max_lin
    self.teleop_angular_z = max(-1.0, min(1.0, angular_z)) * max_ang
    self.teleop_last_cmd_time = nepi_utils.get_time()

  def setMotorControlRatio(self, motor_ind, speed_ratio):
    if motor_ind < 0 or motor_ind >= len(self.motor_ratios):
      self.msg_if.pub_warn("Motor control ignored: motor index " + str(motor_ind) + " out of range")
      return
    self.motor_ratios[motor_ind] = max(0.0, min(1.0, speed_ratio))

  def getMotorControlRatios(self):
    return self.motor_ratios

  def autonomousControlsReady(self):
    # Gates all goto commands: require the Sim Connector's own
    # autonomous_movement_enabled toggle (checked here, not just hidden in
    # the RUI, so a client bypassing the RUI can't do what was turned off
    # either -- see rbx_sim_node.py's identical check for the full
    # reasoning), plus a live bridge connection with fresh telemetry so goto
    # targets are computed from a real current position.
    if self.settings_dict['autonomous_movement_enabled']['value'] != 'TRUE':
      return False
    with self.sock_lock:
      connected = self.sock is not None
    fresh = (nepi_utils.get_time() - self.last_telemetry_time) < self.TELEMETRY_FRESH_SEC
    return connected and fresh

  def goStop(self):
    self.stop_triggered = True
    self.clearGotoTarget()
    self.sendVelocityCmd(0.0, 0.0)
    return True

  def gotoPose(self, attitude_enu_degs):
    self.msg_if.pub_info("Received Pose setpoint command: " + str(attitude_enu_degs))
    with self.goto_target_lock:
      self.goto_target = {'x_m': self.navpose_dict['x_m'],
                          'y_m': self.navpose_dict['y_m'],
                          'yaw_deg': attitude_enu_degs[2]}

  def gotoPosition(self, point_enu_m, orientation_enu_deg):
    self.msg_if.pub_info("Received Position setpoint command: " + str(point_enu_m))
    with self.goto_target_lock:
      self.goto_target = {'x_m': self.navpose_dict['x_m'] + point_enu_m.x,
                          'y_m': self.navpose_dict['y_m'] + point_enu_m.y,
                          'yaw_deg': orientation_enu_deg[2]}

  def getNavPoseCb(self):
    return self.navpose_dict

  #######################
  ### Setup-Action Functions

  def setSetupActionInd(self, action_ind):
    action = self.RBX_SETUP_ACTIONS[action_ind]
    if action == "RESET_SIM":
      return self.resetSimAction()
    elif action == "RETURN_HOME":
      return self.returnHomeAction()
    return False

  #######################
  ### Go-Action Functions

  def setGoActionInd(self, action_ind):
    return False

  #######################
  ### Home Functions

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

  def returnHomeAction(self):
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
    # Fire-and-forget, same as rbx_gazebo_node.py's -- but see the class-level
    # RBX_SETUP_ACTIONS note: this world's Robot node is not a Supervisor, so
    # the bridge logs this and does not actually teleport. Returns True/False
    # based on whether the command was actually sent, not on whether a physical
    # reset happened (which this driver has no way to confirm either way).
    self.clearGotoTarget()
    self.sendVelocityCmd(0.0, 0.0)
    with self.sock_lock:
      connected = self.sock is not None
    if not connected:
      return False
    self.sendLineToBridge({'type': 'reset'}, "Reset sim")
    return True

  def setEnvironmentAction(self, environment_value):
    # Fire-and-forget, same as rbx_gazebo_node.py's -- this world has no
    # obstacle-course model, so the bridge logs this and does not spawn/delete
    # anything (see the ENVIRONMENT_OPTIONS class comment).
    with self.sock_lock:
      connected = self.sock is not None
    if not connected:
      return False
    enabled = (environment_value == self.OBSTACLE_COURSE_OPTION)
    self.sendLineToBridge({'type': 'environment_option',
                           'option': self.OBSTACLE_COURSE_OPTION,
                           'enabled': enabled},
                          "Environment " + str(environment_value))
    return True

  #######################
  ### Goto Controller Processes

  def clearGotoTarget(self):
    with self.goto_target_lock:
      self.goto_target = None

  def gotoControlCb(self, timer):
    with self.goto_target_lock:
      target = self.goto_target

    lin = 0.0
    ang = 0.0
    if target is not None:
      cur_x = self.navpose_dict['x_m']
      cur_y = self.navpose_dict['y_m']
      cur_yaw_rad = math.radians(self.navpose_dict['yaw_deg'])

      max_lin = float(self.settings_dict['max_linear_speed_mps']['value'])
      max_ang = math.radians(float(self.settings_dict['max_angular_rate_dps']['value']))
      tol_m = self.FACTORY_GOTO_TOL_M
      tol_rad = self.FACTORY_GOTO_TOL_RAD
      if self.rbx_if is not None:
        tol_m = self.rbx_if.rbx_info.error_bounds.max_distance_error_m * self.GOTO_TOL_FRACTION
        tol_rad = (math.radians(self.rbx_if.rbx_info.error_bounds.max_rotation_error_deg)
                   * self.GOTO_TOL_FRACTION)

      dx = target['x_m'] - cur_x
      dy = target['y_m'] - cur_y
      dist = math.hypot(dx, dy)

      if dist > tol_m:
        bearing_err = self.normalizeAngle(math.atan2(dy, dx) - cur_yaw_rad)
        ang = max(-max_ang, min(max_ang, self.GOTO_KP_ANG * bearing_err))
        if abs(bearing_err) < self.GOTO_TURN_GATE_RAD:
          lin = max(0.0, min(max_lin, self.GOTO_KP_LIN * dist))
      else:
        yaw_err = 0.0
        if target['yaw_deg'] is not None:
          yaw_err = self.normalizeAngle(math.radians(target['yaw_deg']) - cur_yaw_rad)
        if abs(yaw_err) > tol_rad:
          ang = max(-max_ang, min(max_ang, self.GOTO_KP_ANG * yaw_err))
        else:
          self.clearGotoTarget()
          self.msg_if.pub_info("Goto target reached")
    elif (nepi_utils.get_time() - self.teleop_last_cmd_time) < self.TELEOP_CMD_TIMEOUT_SEC and \
         (self.teleop_linear_x != 0.0 or self.teleop_angular_z != 0.0):
      # No active goto -- a recent, non-zero teleop command takes over this
      # same tick, same "exactly one authoritative sender" reasoning as manual
      # motor control below. Checked BEFORE motor_ratios: teleop and manual
      # motor control are two different control TYPES a user selects one of at
      # a time in the RUI, and a stale nonzero motor_ratios left over from a
      # previous Manual-mode session must not fight a live teleop command.
      # Ported from rbx_sim_node.py -- see that file's identical block.
      lin, ang = self.teleop_linear_x, self.teleop_angular_z
    elif any(self.motor_ratios):
      lin, ang = self.motorControlToVelocity()
    self.sendVelocityCmd(lin, ang)

  def motorControlToVelocity(self):
    left = self.motor_ratios[0]
    right = self.motor_ratios[1]
    lin = (left + right) / 2.0 * self.MOTOR_MAX_LINEAR_MPS
    ang = (right - left) / self.MOTOR_WHEEL_BASE_M * self.MOTOR_MAX_LINEAR_MPS
    return lin, ang

  def normalizeAngle(self, angle_rad):
    while angle_rad > math.pi:
      angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
      angle_rad += 2.0 * math.pi
    return angle_rad

  #######################
  ### Bridge Processes

  def bridgeLoop(self):
    while not nepi_sdk.is_shutdown():
      sock = None
      try:
        sock = socket.create_connection((self.sim_host, int(self.bridge_port)),
                                        timeout = self.SOCKET_TIMEOUT_SEC)
        sock.settimeout(self.SOCKET_TIMEOUT_SEC)
      except Exception as e:
        self.msg_if.pub_warn("Bridge connect to " + str(self.sim_host) + ":" +
                             str(self.bridge_port) + " failed: " + str(e), throttle_s = 10.0)
        time.sleep(self.RECONNECT_INTERVAL_SEC)
        continue
      with self.sock_lock:
        self.sock = sock
      self.msg_if.pub_info("Connected to sim bridge at " + str(self.sim_host) +
                           ":" + str(self.bridge_port))
      self.sendCameraSettings()
      self.setEnvironmentAction(self.settings_dict['environment']['value'])
      buf = b''
      while not nepi_sdk.is_shutdown():
        try:
          data = sock.recv(4096)
        except socket.timeout:
          data = b''
        except Exception:
          data = b''
        if not data:
          break
        buf += data
        while b'\n' in buf:
          line, buf = buf.split(b'\n', 1)
          if line.strip():
            self.processBridgeLine(line)
      with self.sock_lock:
        self.sock = None
      try:
        sock.close()
      except Exception:
        pass
      self.msg_if.pub_warn("Sim bridge connection lost -- retrying in " +
                           str(self.RECONNECT_INTERVAL_SEC) + "s")
      time.sleep(self.RECONNECT_INTERVAL_SEC)

  def processBridgeLine(self, line):
    try:
      msg = json.loads(line)
    except Exception as e:
      self.msg_if.pub_warn("Bad line from bridge: " + str(e), throttle_s = 5.0)
      return
    if not isinstance(msg, dict):
      return
    if msg.get('type') == 'image':
      self.processImageLine(msg)
    else:
      self.processTelemetryLine(msg)

  def processImageLine(self, msg):
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
    now = nepi_utils.get_time()
    x_m = float(telem.get('x', 0.0))
    y_m = float(telem.get('y', 0.0))
    yaw_rad = float(telem.get('yaw', 0.0))
    lin_mps = float(telem.get('linear_x', 0.0))
    ang_radps = float(telem.get('angular_z', 0.0))

    self.navpose_dict['has_position'] = True
    self.navpose_dict['time_position'] = now
    self.navpose_dict['x_m'] = x_m
    self.navpose_dict['y_m'] = y_m
    self.navpose_dict['z_m'] = 0.0
    self.navpose_dict['latitude'] = x_m
    self.navpose_dict['longitude'] = y_m
    self.navpose_dict['altitude_m'] = 0.0
    self.navpose_dict['x_m_per_sec'] = lin_mps * math.cos(yaw_rad)
    self.navpose_dict['y_m_per_sec'] = lin_mps * math.sin(yaw_rad)
    self.navpose_dict['z_m_per_sec'] = 0.0

    self.navpose_dict['has_orientation'] = True
    self.navpose_dict['time_orientation'] = now
    self.navpose_dict['roll_deg'] = 0.0
    self.navpose_dict['pitch_deg'] = 0.0
    self.navpose_dict['yaw_deg'] = math.degrees(yaw_rad)
    self.navpose_dict['yaw_deg_per_sec'] = math.degrees(ang_radps)

    self.last_telemetry_time = now

  def sendVelocityCmd(self, linear_x, angular_z):
    cmd = {'linear_x': linear_x, 'angular_z': angular_z}
    self.sendLineToBridge(cmd, "Velocity command")

  def sendCameraSettings(self):
    # No view_mode -- see CAMERA_NAME's own comment, there is nothing to
    # switch between. offset_x/y/z included for parity with rbx_sim_node.py's
    # wire shape -- webots_rbx_bridge.py currently logs-and-ignores them
    # (this world's single fixed Camera device has no repositionable rig),
    # matching this driver's existing honest treatment of RESET_SIM/environment.
    cmd = {
      'type': 'camera_settings',
      'offset_x': float(self.settings_dict['camera_offset_x']['value']),
      'offset_y': float(self.settings_dict['camera_offset_y']['value']),
      'offset_z': float(self.settings_dict['camera_offset_z']['value']),
    }
    self.sendLineToBridge(cmd, "Camera settings")

  def sendLineToBridge(self, line_dict, description):
    with self.sock_lock:
      sock = self.sock
      if sock is None:
        self.msg_if.pub_warn(description + " dropped -- sim bridge not connected",
                             throttle_s = 5.0)
        return
      try:
        sock.sendall((json.dumps(line_dict) + '\n').encode())
      except Exception as e:
        self.msg_if.pub_warn("Failed to send " + description.lower() + " to bridge: " + str(e))

  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    """Stops the robot on node shutdown by sending a zero velocity command."""
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    self.sendVelocityCmd(0.0, 0.0)


#########################################
# Main
#########################################
if __name__ == '__main__':
  WebotsNode()
