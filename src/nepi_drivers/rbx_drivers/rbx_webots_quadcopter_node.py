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

# RBX driver node for the Webots simulated quadcopter -- see
# docs/WEBOTS_QUADCOPTER_DRIVER_PLAN.md for the full design. Ported from
# rbx_webots_node.py (the rover driver), extended to a real Z/altitude axis
# (goto/gotoPose now track z_m, not always 0) and TAKEOFF/LAND setup actions,
# with manual motor-ratio control removed entirely -- a Supervisor-velocity
# body (see webots_rbx_bridge_quadcopter.py) has no per-rotor ratio that means
# anything, matching how even the real ArduPilot RBX driver doesn't expose
# raw motor mixing via manual control either. Teleop (3D velocity + yaw rate)
# IS exposed -- natural fit for "manually fly it".
#
# No ARM/DISARM state machine (RBX_STATES/RBX_MODES stay empty, same
# simplicity level as the rover): a Supervisor-injected-velocity body has no
# real safety envelope to gate, so inventing one would just be complexity
# with no real behavior behind it.
#
# This world has exactly one fixed Camera device with no repositionable rig,
# reported under a single honest topic name (CAMERA_NAME below) -- earlier
# versions of this driver copied the rover's SCENE_CAMERA/ROBOT_CAMERA dual
# naming with a "THIRD_PERSON" view mode, but that view never existed here.

import copy
import base64
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

PKG_NAME = 'RBX_WEBOTS_QUADCOPTER' # Use in display menus
FILE_TYPE = 'NODE'


#########################################
# Node Class
#########################################

class WebotsQuadcopterNode:

  # Exactly ONE real Camera device in this world, with no repositionable
  # rig at all (see webots_rbx_bridge_quadcopter.py's own camera_settings
  # handling -- a documented no-op). Earlier versions of this driver
  # reported it under two names (SCENE_CAMERA/ROBOT_CAMERA) with a
  # camera_view_mode Setting to "switch" between them, copied from
  # rbx_sim_node.py's rover pattern -- but the rover's two names correspond
  # to two REAL, independently-posed camera links, while this world has
  # exactly one, so that Setting was a pure UI fiction offering a
  # "THIRD_PERSON" choice that does not exist (confirmed live 2026-08-18:
  # nothing physically changes when it's set -- see the module docstring
  # this replaces). One real camera, one honestly-named topic, no
  # view-mode Setting at all.
  CAMERA_NAME = "robot_camera"

  ROBOT_MAIN_REFERENCE_FRAME = "base_link"

  # No obstacle-course model in this world either -- same honest no-op
  # treatment as the rover driver.
  ENVIRONMENT_OPTIONS = ["FLAT_GROUND", "OBSTACLE_COURSE"]
  OBSTACLE_COURSE_OPTION = "OBSTACLE_COURSE"

  # camera_offset_x/y/z kept (unlike camera_view_mode/scene_offset_*, which
  # are gone entirely) as an honest "not wired up yet" placeholder matching
  # this codebase's established precedent for a real-but-currently-inert
  # capability (RESET_SIM on a non-Supervisor world, environment on a world
  # with no obstacle model) -- there genuinely is one real camera that could
  # someday get a repositionable mount, unlike a second camera that simply
  # does not exist.
  CAMERA_SETTING_NAMES = ("camera_offset_x", "camera_offset_y", "camera_offset_z")
  ENVIRONMENT_SETTING_NAMES = ("environment",)

  # Sim Connector's own per-robot-config capability toggles -- same mechanism
  # as rbx_webots_node.py/rbx_sim_node.py. teleop_movement_enabled gates
  # teleopControlsReady below; autonomous_movement_enabled gates
  # autonomousControlsReady (goto AND takeoff/land, which are just goto
  # targets under the hood).
  CAPABILITY_SETTING_NAMES = ("autonomous_movement_enabled", "teleop_movement_enabled",
                              "camera_controls_enabled", "enabled_image_sources")

  CAP_SETTINGS = dict(
    max_linear_speed_mps = {"type":"Float","name":"max_linear_speed_mps","options":["0.05","2.0"]},
    max_vertical_speed_mps = {"type":"Float","name":"max_vertical_speed_mps","options":["0.05","1.5"]},
    max_angular_rate_dps = {"type":"Float","name":"max_angular_rate_dps","options":["5.0","180.0"]},
    takeoff_height_m = {"type":"Float","name":"takeoff_height_m","options":["0.1","10.0"]},
    environment = {"type":"Discrete","name":"environment","options":ENVIRONMENT_OPTIONS},
    camera_offset_x = {"type":"Float","name":"camera_offset_x","options":["-10.0","10.0"]},
    camera_offset_y = {"type":"Float","name":"camera_offset_y","options":["-10.0","10.0"]},
    camera_offset_z = {"type":"Float","name":"camera_offset_z","options":["-10.0","10.0"]},
    autonomous_movement_enabled = {"type":"Discrete","name":"autonomous_movement_enabled","options":["TRUE","FALSE"]},
    teleop_movement_enabled = {"type":"Discrete","name":"teleop_movement_enabled","options":["TRUE","FALSE"]},
    camera_controls_enabled = {"type":"Discrete","name":"camera_controls_enabled","options":["TRUE","FALSE"]},
    enabled_image_sources = {"type":"String","name":"enabled_image_sources"}
  )

  # max_linear/angular defaults match webots_rbx_bridge_quadcopter.py's own
  # MAX_LINEAR_MPS/MAX_ANGULAR_RADPS clamps -- this conversion has to agree
  # with the bridge's own clamping, same reasoning the rover driver's
  # MOTOR_MAX_LINEAR_MPS match already established.
  FACTORY_SETTINGS = dict(
    max_linear_speed_mps = {"type":"Float","name":"max_linear_speed_mps","value":"1.0"},
    max_vertical_speed_mps = {"type":"Float","name":"max_vertical_speed_mps","value":"0.75"},
    max_angular_rate_dps = {"type":"Float","name":"max_angular_rate_dps","value":"45.0"},
    # Well above the default goto convergence tolerance (RBXRobotIF's factory
    # max_distance_error_m=2.0 * GOTO_TOL_FRACTION=0.5 = 1.0m) -- confirmed
    # live that 1.5m landed WITHIN that 1.0m tolerance of the spawn height,
    # so TAKEOFF correctly reported "reached" after climbing only to ~0.5m,
    # a real but unsatisfying demo of a "rises to altitude" action even
    # though the underlying goto/tolerance logic was working exactly as
    # designed. 3.0m leaves clear margin.
    takeoff_height_m = {"type":"Float","name":"takeoff_height_m","value":"3.0"},
    environment = {"type":"Discrete","name":"environment","value":ENVIRONMENT_OPTIONS[0]},
    camera_offset_x = {"type":"Float","name":"camera_offset_x","value":"0.0"},
    camera_offset_y = {"type":"Float","name":"camera_offset_y","value":"0.0"},
    camera_offset_z = {"type":"Float","name":"camera_offset_z","value":"0.0"},
    autonomous_movement_enabled = {"type":"Discrete","name":"autonomous_movement_enabled","value":"TRUE"},
    teleop_movement_enabled = {"type":"Discrete","name":"teleop_movement_enabled","value":"TRUE"},
    camera_controls_enabled = {"type":"Discrete","name":"camera_controls_enabled","value":"TRUE"},
    enabled_image_sources = {"type":"String","name":"enabled_image_sources","value":""}
  )

  FACTORY_SETTINGS_OVERRIDES = dict()

  # No ARM/DISARM/flight-mode machinery -- see module docstring. RBXRobotIF
  # handles empty lists correctly (bounds checks reject any set_state/
  # set_mode index, status shows "Not Set").
  RBX_STATES = []
  RBX_MODES = []
  # TAKEOFF/LAND are real, blocking setup actions here (unlike RESET_SIM,
  # which stays a documented no-op below) -- this world's Robot node IS a
  # Supervisor and its bridge actually applies commanded velocity, so a
  # goto-to-altitude target genuinely moves the body.
  RBX_SETUP_ACTIONS = ["TAKEOFF", "LAND", "RESET_SIM", "RETURN_HOME"]
  RBX_GO_ACTIONS = []

  GO_HOME_TIMEOUT_SEC = 60.0
  GO_HOME_POLL_INTERVAL_SEC = 0.2
  TAKEOFF_LAND_TIMEOUT_SEC = 30.0
  TAKEOFF_LAND_POLL_INTERVAL_SEC = 0.2
  # Ground level in this world's local frame -- LAND targets this altitude,
  # not 0 velocity/thrust cutoff (there is no real ground-contact sensing on
  # a Supervisor-velocity body, so "landed" means "reached this height").
  GROUND_LEVEL_M = 0.05

  RECONNECT_INTERVAL_SEC = 3.0
  SOCKET_TIMEOUT_SEC = 5.0

  CONTROLLER_RATE_HZ = 20
  NAVPOSE_UPDATE_RATE = 10
  TELEMETRY_FRESH_SEC = 2.0
  TELEOP_CMD_TIMEOUT_SEC = 0.75

  GOTO_KP_LIN = 0.5
  GOTO_KP_ANG = 1.5
  # Proportional gain for the independent vertical (Z) axis -- runs every
  # tick regardless of the horizontal turn/drive phase below, since climbing
  # doesn't require facing the target the way horizontal travel does.
  GOTO_KP_VERT = 0.5
  GOTO_TURN_GATE_RAD = math.radians(30.0)
  GOTO_TOL_FRACTION = 0.5
  FACTORY_GOTO_TOL_M = 1.0
  FACTORY_GOTO_TOL_RAD = math.radians(1.0)

  # See rbx_webots_node.py's own GOTO_CMD_TIMEOUT_SEC comment -- same
  # non-holonomic-body headroom argument applies to a body whose horizontal
  # motion still turns-then-drives rather than freely strafing toward a goal.
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
    # Goto controller state -- z_m added vs. the rover driver's goto_target.
    self.goto_target = None
    self.goto_target_lock = threading.Lock()
    self.stop_triggered = False

    ##############################
    # Teleop state -- full 3D this time (linear_y/linear_z are real strafe/
    # climb axes for a quadcopter, unlike the rover's ground-only teleop).
    self.teleop_linear_x = 0.0
    self.teleop_linear_y = 0.0
    self.teleop_linear_z = 0.0
    self.teleop_angular_z = 0.0
    self.teleop_last_cmd_time = 0.0

    ##############################
    # No motor-ratio state -- see module docstring, manual motor control is
    # not exposed for this robot type at all.

    ##############################
    # Home position state: local ENU x/y/z meters (see rbx_webots_node.py's
    # getHome/setHome for why this reuses the GeoPoint plumbing)
    self.home_x_m = 0.0
    self.home_y_m = 0.0
    self.home_z_m = 0.0

    ##############################
    # Initialize RBX Settings
    self.settings_dict = copy.deepcopy(self.FACTORY_SETTINGS)
    self.cap_settings = self.getCapSettings()
    self.factory_settings = self.getFactorySettings()

    # z True (real altitude axis) and roll/pitch stay False (no real attitude
    # control -- gotoPose is still yaw-only, same simplification the rover
    # driver makes for its single rotation axis).
    self.axis_controls = AxisControls()
    self.axis_controls.x = True
    self.axis_controls.y = True
    self.axis_controls.z = True
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
                             # No manual motor-ratio control for this robot
                             # type -- None leaves has_manual_controls False
                             # (see device_if_rbx.py's own None-check), the
                             # same convention any driver uses to report a
                             # capability it genuinely doesn't have.
                             manualControlsReadyFunction = None,
                             getMotorControlRatios = None,
                             setMotorControlRatio = None,
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

  def teleopControlsReady(self):
    if self.settings_dict['teleop_movement_enabled']['value'] != 'TRUE':
      return False
    with self.sock_lock:
      return self.sock is not None

  def setTeleopVelocity(self, linear_x, linear_y, linear_z, angular_z):
    # Full 3D this time -- linear_y (strafe) and linear_z (climb/descend) are
    # both real axes for a quadcopter, unlike the rover's ground-only teleop.
    # Ratios in [-1,1], scaled by the same max_linear_speed_mps/
    # max_vertical_speed_mps/max_angular_rate_dps Settings that already cap
    # goto speed.
    max_lin = float(self.settings_dict['max_linear_speed_mps']['value'])
    max_vert = float(self.settings_dict['max_vertical_speed_mps']['value'])
    max_ang = math.radians(float(self.settings_dict['max_angular_rate_dps']['value']))
    self.teleop_linear_x = max(-1.0, min(1.0, linear_x)) * max_lin
    self.teleop_linear_y = max(-1.0, min(1.0, linear_y)) * max_lin
    self.teleop_linear_z = max(-1.0, min(1.0, linear_z)) * max_vert
    self.teleop_angular_z = max(-1.0, min(1.0, angular_z)) * max_ang
    self.teleop_last_cmd_time = nepi_utils.get_time()

  def autonomousControlsReady(self):
    # Gates goto AND takeoff/land (both just goto targets under the hood) --
    # same reasoning as rbx_webots_node.py's identical check.
    if self.settings_dict['autonomous_movement_enabled']['value'] != 'TRUE':
      return False
    with self.sock_lock:
      connected = self.sock is not None
    fresh = (nepi_utils.get_time() - self.last_telemetry_time) < self.TELEMETRY_FRESH_SEC
    return connected and fresh

  def goStop(self):
    self.stop_triggered = True
    self.clearGotoTarget()
    self.sendVelocityCmd(0.0, 0.0, 0.0, 0.0)
    return True

  def gotoPose(self, attitude_enu_degs):
    self.msg_if.pub_info("Received Pose setpoint command: " + str(attitude_enu_degs))
    with self.goto_target_lock:
      self.goto_target = {'x_m': self.navpose_dict['x_m'],
                          'y_m': self.navpose_dict['y_m'],
                          'z_m': self.navpose_dict['z_m'],
                          'yaw_deg': attitude_enu_degs[2]}

  def gotoPosition(self, point_enu_m, orientation_enu_deg):
    self.msg_if.pub_info("Received Position setpoint command: " + str(point_enu_m))
    with self.goto_target_lock:
      self.goto_target = {'x_m': self.navpose_dict['x_m'] + point_enu_m.x,
                          'y_m': self.navpose_dict['y_m'] + point_enu_m.y,
                          'z_m': self.navpose_dict['z_m'] + point_enu_m.z,
                          'yaw_deg': orientation_enu_deg[2]}

  def getNavPoseCb(self):
    return self.navpose_dict

  #######################
  ### Setup-Action Functions

  def setSetupActionInd(self, action_ind):
    action = self.RBX_SETUP_ACTIONS[action_ind]
    if action == "TAKEOFF":
      return self.takeoffAction()
    elif action == "LAND":
      return self.landAction()
    elif action == "RESET_SIM":
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
      self.goto_target = {'x_m': self.home_x_m, 'y_m': self.home_y_m,
                          'z_m': self.home_z_m, 'yaw_deg': None}
    return self.pollGotoTarget(self.GO_HOME_TIMEOUT_SEC, self.GO_HOME_POLL_INTERVAL_SEC)

  def takeoffAction(self):
    # Rises straight up to takeoff_height_m at the current x/y, holding
    # current yaw. Blocking, same poll pattern as returnHomeAction.
    if not self.autonomousControlsReady():
      return False
    target_z = float(self.settings_dict['takeoff_height_m']['value'])
    with self.goto_target_lock:
      self.goto_target = {'x_m': self.navpose_dict['x_m'], 'y_m': self.navpose_dict['y_m'],
                          'z_m': target_z, 'yaw_deg': None}
    return self.pollGotoTarget(self.TAKEOFF_LAND_TIMEOUT_SEC, self.TAKEOFF_LAND_POLL_INTERVAL_SEC)

  def landAction(self):
    # Descends straight down to GROUND_LEVEL_M at the current x/y -- no real
    # ground-contact sensing on a Supervisor-velocity body (see class
    # docstring), "landed" means "reached this height".
    if not self.autonomousControlsReady():
      return False
    with self.goto_target_lock:
      self.goto_target = {'x_m': self.navpose_dict['x_m'], 'y_m': self.navpose_dict['y_m'],
                          'z_m': self.GROUND_LEVEL_M, 'yaw_deg': None}
    return self.pollGotoTarget(self.TAKEOFF_LAND_TIMEOUT_SEC, self.TAKEOFF_LAND_POLL_INTERVAL_SEC)

  def pollGotoTarget(self, timeout_sec, poll_interval_sec):
    start_time = nepi_utils.get_time()
    while (nepi_utils.get_time() - start_time) < timeout_sec:
      with self.goto_target_lock:
        reached = self.goto_target is None
      if reached:
        return True
      time.sleep(poll_interval_sec)
    return False

  def resetSimAction(self):
    # Real teleport this time -- this world's Robot node IS a Supervisor
    # (unlike the rover's), so webots_rbx_bridge_quadcopter.py's reset
    # handler actually applies it, not a documented no-op. Still fire-and-
    # forget from this driver's own perspective: it has no way to confirm
    # the teleport happened beyond the command having been sent.
    self.clearGotoTarget()
    self.sendVelocityCmd(0.0, 0.0, 0.0, 0.0)
    with self.sock_lock:
      connected = self.sock is not None
    if not connected:
      return False
    self.sendLineToBridge({'type': 'reset'}, "Reset sim")
    return True

  def setEnvironmentAction(self, environment_value):
    # Fire-and-forget, same as rbx_webots_node.py's -- this world has no
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
    vert = 0.0
    if target is not None:
      cur_x = self.navpose_dict['x_m']
      cur_y = self.navpose_dict['y_m']
      cur_z = self.navpose_dict['z_m']
      cur_yaw_rad = math.radians(self.navpose_dict['yaw_deg'])

      max_lin = float(self.settings_dict['max_linear_speed_mps']['value'])
      max_vert = float(self.settings_dict['max_vertical_speed_mps']['value'])
      max_ang = math.radians(float(self.settings_dict['max_angular_rate_dps']['value']))
      tol_m = self.FACTORY_GOTO_TOL_M
      tol_rad = self.FACTORY_GOTO_TOL_RAD
      if self.rbx_if is not None:
        tol_m = self.rbx_if.rbx_info.error_bounds.max_distance_error_m * self.GOTO_TOL_FRACTION
        tol_rad = (math.radians(self.rbx_if.rbx_info.error_bounds.max_rotation_error_deg)
                   * self.GOTO_TOL_FRACTION)

      # Vertical (Z) axis: independent of the horizontal turn/drive phase
      # below -- climbing/descending doesn't require facing the target the
      # way horizontal travel does, so this always runs every tick.
      dz = target['z_m'] - cur_z
      vert_done = abs(dz) <= tol_m
      if not vert_done:
        vert = max(-max_vert, min(max_vert, self.GOTO_KP_VERT * dz))

      # Horizontal (X/Y) axis: same turn-then-drive controller as
      # rbx_webots_node.py's rover (still no real strafe-toward-target, same
      # simplification).
      dx = target['x_m'] - cur_x
      dy = target['y_m'] - cur_y
      dist = math.hypot(dx, dy)

      if dist > tol_m:
        horiz_done = False
        bearing_err = self.normalizeAngle(math.atan2(dy, dx) - cur_yaw_rad)
        ang = max(-max_ang, min(max_ang, self.GOTO_KP_ANG * bearing_err))
        if abs(bearing_err) < self.GOTO_TURN_GATE_RAD:
          lin = max(0.0, min(max_lin, self.GOTO_KP_LIN * dist))
      else:
        yaw_err = 0.0
        if target['yaw_deg'] is not None:
          yaw_err = self.normalizeAngle(math.radians(target['yaw_deg']) - cur_yaw_rad)
        if abs(yaw_err) > tol_rad:
          horiz_done = False
          ang = max(-max_ang, min(max_ang, self.GOTO_KP_ANG * yaw_err))
        else:
          horiz_done = True

      if horiz_done and vert_done:
        self.clearGotoTarget()
        self.msg_if.pub_info("Goto target reached")
    elif (nepi_utils.get_time() - self.teleop_last_cmd_time) < self.TELEOP_CMD_TIMEOUT_SEC and \
         (self.teleop_linear_x != 0.0 or self.teleop_linear_y != 0.0 or
          self.teleop_linear_z != 0.0 or self.teleop_angular_z != 0.0):
      # No active goto -- a recent, non-zero teleop command takes over this
      # same tick, same reasoning as rbx_webots_node.py's identical block.
      self.sendVelocityCmd(self.teleop_linear_x, self.teleop_linear_y,
                           self.teleop_linear_z, self.teleop_angular_z)
      return
    self.sendVelocityCmd(lin, 0.0, vert, ang)

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
    # z/linear_y/linear_z are new vs. rbx_webots_node.py's rover telemetry
    # (always 0 there) -- real axes here.
    now = nepi_utils.get_time()
    x_m = float(telem.get('x', 0.0))
    y_m = float(telem.get('y', 0.0))
    z_m = float(telem.get('z', 0.0))
    yaw_rad = float(telem.get('yaw', 0.0))
    lin_x_mps = float(telem.get('linear_x', 0.0))
    lin_y_mps = float(telem.get('linear_y', 0.0))
    lin_z_mps = float(telem.get('linear_z', 0.0))
    ang_radps = float(telem.get('angular_z', 0.0))

    self.navpose_dict['has_position'] = True
    self.navpose_dict['time_position'] = now
    self.navpose_dict['x_m'] = x_m
    self.navpose_dict['y_m'] = y_m
    self.navpose_dict['z_m'] = z_m
    self.navpose_dict['latitude'] = x_m
    self.navpose_dict['longitude'] = y_m
    self.navpose_dict['altitude_m'] = z_m
    # World-frame velocity components (webots_rbx_bridge_quadcopter.py
    # differentiates GPS position directly in world frame, unlike the
    # rover's bridge which reports a body-frame forward speed derived from
    # its own world-frame delta -- reported here directly, no yaw rotation
    # needed).
    self.navpose_dict['x_m_per_sec'] = lin_x_mps
    self.navpose_dict['y_m_per_sec'] = lin_y_mps
    self.navpose_dict['z_m_per_sec'] = lin_z_mps

    self.navpose_dict['has_orientation'] = True
    self.navpose_dict['time_orientation'] = now
    self.navpose_dict['roll_deg'] = 0.0
    self.navpose_dict['pitch_deg'] = 0.0
    self.navpose_dict['yaw_deg'] = math.degrees(yaw_rad)
    self.navpose_dict['yaw_deg_per_sec'] = math.degrees(ang_radps)

    self.last_telemetry_time = now

  def sendVelocityCmd(self, linear_x, linear_y, linear_z, angular_z):
    cmd = {'linear_x': linear_x, 'linear_y': linear_y,
           'linear_z': linear_z, 'angular_z': angular_z}
    self.sendLineToBridge(cmd, "Velocity command")

  def sendCameraSettings(self):
    # No view_mode -- see CAMERA_NAME's own comment, there is nothing to
    # switch between.
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
    self.sendVelocityCmd(0.0, 0.0, 0.0, 0.0)


#########################################
# Main
#########################################
if __name__ == '__main__':
  WebotsQuadcopterNode()
