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

# RBX driver node for a MuJoCo simulated robot -- rbx_webots_node.py's exact
# pattern, ported (which was itself ported from rbx_gazebo_node.py). The
# whole point of matching mujoco_rbx_bridge.py's wire protocol to
# sim_bridge_node.py's/webots_rbx_bridge.py's (see that bridge's own
# docstring) is the same reason this file needed only renaming, not new
# logic: same bridge-loop/reconnect shape, same closed-loop 2D goto
# controller (a MuJoCo bridge process has no onboard autopilot to delegate to
# either), same capability gaps for the same reasons (no arm/disarm, no
# battery, no WGS84 location) -- except RESET_SIM, which is genuine here (see
# resetSimAction below).
#
# ############################################################################
# 4 independently-actuated wheels, not 2: rbx_rover.xml (this model) has no
# shared-side-motor constraint the way Gazebo's diff-drive plugin or Webots'
# 2-motor-device model does -- MuJoCo actuates each of the 4 wheel joints
# independently. So this driver exposes 4 motor slots, matching
# rbx_sim_node.py's (Gazebo's) current pattern, not rbx_webots_node.py's
# 2-slot one: motorControlToVelocity averages each side's pair before sending
# a single lin/ang command over the wire (mujoco_rbx_bridge.py only ever
# receives one lin/ang pair, same as every other bridge here).
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
from nepi_sdk import nepi_controls
from nepi_sdk import nepi_img

from std_msgs.msg import UInt32, String
from sensor_msgs.msg import Image

from nepi_interfaces.msg import AxisControls
from geographic_msgs.msg import GeoPoint

from nepi_api.device_if_rbx import RBXRobotIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'RBX_MUJOCO' # Use in display menus
FILE_TYPE = 'NODE'


#########################################
# Node Class
#########################################

class MujocoNode:

  # UPDATED (2026-09-21): rbx_rover.xml now has a real second (scene/chase)
  # camera plus depth rendering on both cameras, matching
  # rbx_webots_node.py's identical same-day fix (requested live: "get mujoco
  # to that same point too, with all the same features i asked for with
  # webots"). Four always-live topics now, matching rbx_sim_node.py's/
  # rbx_webots_node.py's exact convention.
  ROBOT_COLOR_TOPIC_SUFFIX = "robot_color"
  SCENE_COLOR_TOPIC_SUFFIX = "scene_color"
  ROBOT_DEPTH_TOPIC_SUFFIX = "robot_depth"
  SCENE_DEPTH_TOPIC_SUFFIX = "scene_depth"

  # Which publisher each bridge "camera" tag routes to -- see
  # rbx_sim_node.py's/rbx_webots_node.py's identical CAMERA_PUB_ATTR.
  CAMERA_PUB_ATTR = {
    "robot_color": "image_pub_robot_color",
    "scene_color": "image_pub_scene_color",
    "robot_depth": "image_pub_robot_depth",
    "scene_depth": "image_pub_scene_depth",
  }

  ROBOT_MAIN_REFERENCE_FRAME = "base_link"

  # OBSTACLE_COURSE really toggles now (2026-09-21) --
  # mujoco_rbx_bridge.py's setObstacleCourseEnabled flips the always-
  # compiled-in obstacle-course geoms' rgba alpha + contype/conaffinity
  # live (generate_environment_xml.py), matching Webots' equivalent
  # spawn/despawn feature.
  ENVIRONMENT_OPTIONS = ["FLAT_GROUND", "OBSTACLE_COURSE"]
  OBSTACLE_COURSE_OPTION = "OBSTACLE_COURSE"

  # Real, live-applied offsets now (2026-09-21) -- mujoco_rbx_bridge.py's
  # applyCameraSettings writes model.cam_pos/model.cam_fovy directly, no
  # respawn needed (same idea as webots_rbx_bridge.py's Supervisor field
  # writes, simpler here since MuJoCo already exposes these as mutable
  # per-step model arrays). yaw/tilt ADDED (2026-09-22) -- requested live:
  # "the lock scene camera to robot feature... get it working for gazebo
  # and then the rest." mujoco_rbx_bridge.py's applyCameraSettings now also
  # writes model.cam_quat, composed the same way rbx_sim_node.py's
  # scene_offset_yaw/tilt already work for Gazebo.
  CAMERA_SETTING_NAMES = ("camera_offset_x", "camera_offset_y", "camera_offset_z",
                          "camera_offset_yaw", "camera_offset_tilt",
                          "scene_offset_x", "scene_offset_y", "scene_offset_z",
                          "scene_offset_yaw", "scene_offset_tilt",
                          "camera_fov_deg")
  ENVIRONMENT_SETTING_NAMES = ("environment",)

  # Factory scene (chase) camera mount: (-2.5, 0, 1.65) relative to the
  # chassis -- same offset rbx_rover.xml's own scene_camera uses and the
  # same constants rbx_sim_node.py's own FACTORY_SCENE_TILT_DEG documents.
  FACTORY_SCENE_TILT_DEG = math.degrees(math.atan2(1.65, 2.5))

  # Sim Connector's own per-robot-config "customize the capabilities that are
  # open" toggles -- same mechanism and same names as rbx_sim_node.py's/
  # rbx_webots_node.py's own CAPABILITY_SETTING_NAMES.
  CAPABILITY_SETTING_NAMES = ("autonomous_movement_enabled",
                              "camera_controls_enabled", "enabled_image_sources")

  CAP_SETTINGS = dict(
    max_linear_speed_mps = {"type":"Float","name":"max_linear_speed_mps","options":["0.05","2.0"]},
    max_angular_rate_dps = {"type":"Float","name":"max_angular_rate_dps","options":["5.0","180.0"]},
    environment = {"type":"Discrete","name":"environment","options":ENVIRONMENT_OPTIONS},
    camera_offset_x = {"type":"Float","name":"camera_offset_x","options":["-10.0","10.0"]},
    camera_offset_y = {"type":"Float","name":"camera_offset_y","options":["-10.0","10.0"]},
    camera_offset_z = {"type":"Float","name":"camera_offset_z","options":["-10.0","10.0"]},
    camera_offset_yaw = {"type":"Float","name":"camera_offset_yaw","options":["-180.0","180.0"]},
    camera_offset_tilt = {"type":"Float","name":"camera_offset_tilt","options":["-90.0","90.0"]},
    scene_offset_x = {"type":"Float","name":"scene_offset_x","options":["-10.0","10.0"]},
    scene_offset_y = {"type":"Float","name":"scene_offset_y","options":["-10.0","10.0"]},
    scene_offset_z = {"type":"Float","name":"scene_offset_z","options":["-10.0","10.0"]},
    scene_offset_yaw = {"type":"Float","name":"scene_offset_yaw","options":["-180.0","180.0"]},
    scene_offset_tilt = {"type":"Float","name":"scene_offset_tilt","options":["-90.0","90.0"]},
    camera_fov_deg = {"type":"Float","name":"camera_fov_deg","options":["10.0","150.0"]},
    autonomous_movement_enabled = {"type":"Discrete","name":"autonomous_movement_enabled","options":["TRUE","FALSE"]},
    camera_controls_enabled = {"type":"Discrete","name":"camera_controls_enabled","options":["TRUE","FALSE"]},
    # No fixed options -- the candidate topic set is per-deployment.
    enabled_image_sources = {"type":"String","name":"enabled_image_sources"}
  )

  # max_linear_speed_mps factory/range matches rbx_sim_node.py's (Gazebo's):
  # rbx_rover.xml uses the SAME physical dimensions as generic_rover/
  # model.sdf (see that model's dimensions.yaml), so the same speed range is
  # physically appropriate here too -- unlike rbx_webots_node.py's lower
  # range, which matches rbx_rover.wbt's smaller/different physical scale.
  FACTORY_SETTINGS = dict(
    max_linear_speed_mps = {"type":"Float","name":"max_linear_speed_mps","value":"0.5"},
    max_angular_rate_dps = {"type":"Float","name":"max_angular_rate_dps","value":"45.0"},
    environment = {"type":"Discrete","name":"environment","value":ENVIRONMENT_OPTIONS[0]},
    # Real, wired offsets now -- zero means "at rbx_rover.xml's own factory
    # mount point", same convention as rbx_webots_node.py's identical fix.
    camera_offset_x = {"type":"Float","name":"camera_offset_x","value":"0.0"},
    camera_offset_y = {"type":"Float","name":"camera_offset_y","value":"0.0"},
    camera_offset_z = {"type":"Float","name":"camera_offset_z","value":"0.0"},
    camera_offset_yaw = {"type":"Float","name":"camera_offset_yaw","value":"0.0"},
    camera_offset_tilt = {"type":"Float","name":"camera_offset_tilt","value":"0.0"},
    scene_offset_x = {"type":"Float","name":"scene_offset_x","value":"0.0"},
    scene_offset_y = {"type":"Float","name":"scene_offset_y","value":"0.0"},
    scene_offset_z = {"type":"Float","name":"scene_offset_z","value":"0.0"},
    scene_offset_yaw = {"type":"Float","name":"scene_offset_yaw","value":"0.0"},
    scene_offset_tilt = {"type":"Float","name":"scene_offset_tilt","value":str(FACTORY_SCENE_TILT_DEG)},
    # 60.0 matches rbx_rover.xml's own robot_camera fovy="60" -- the true
    # factory value, not a guess.
    camera_fov_deg = {"type":"Float","name":"camera_fov_deg","value":"60.0"},
    # Both default to enabled: a robot config that never touches these
    # settings behaves exactly as this driver did before this feature existed.
    autonomous_movement_enabled = {"type":"Discrete","name":"autonomous_movement_enabled","value":"TRUE"},
    camera_controls_enabled = {"type":"Discrete","name":"camera_controls_enabled","value":"TRUE"},
    # Empty = unrestricted -- see the CAPABILITY_SETTING_NAMES comment above.
    enabled_image_sources = {"type":"String","name":"enabled_image_sources","value":""}
  )

  FACTORY_SETTINGS_OVERRIDES = dict()

  RBX_STATES = []
  RBX_MODES = []
  # RESET_SIM is a REAL setup action here -- unlike rbx_webots_node.py's
  # honestly-accepted-but-not-physically-honored version (that Robot node
  # isn't a Supervisor), mujoco_rbx_bridge.py owns the physics state directly
  # and genuinely teleports the model back to its initial pose via
  # mujoco.mj_resetData. See resetSimAction below.
  RBX_SETUP_ACTIONS = ["RESET_SIM", "RETURN_HOME"]
  RBX_GO_ACTIONS = []

  GO_HOME_TIMEOUT_SEC = 60.0
  GO_HOME_POLL_INTERVAL_SEC = 0.2

  RECONNECT_INTERVAL_SEC = 3.0
  SOCKET_TIMEOUT_SEC = 5.0

  CONTROLLER_RATE_HZ = 20
  NAVPOSE_UPDATE_RATE = 10
  TELEMETRY_FRESH_SEC = 2.0

  # 4 independently-actuated wheels (see module docstring): [0]=front_left,
  # [1]=front_right, [2]=rear_left, [3]=rear_right -- same ordering
  # convention rbx_sim_node.py's/rbx_rover.xml's wheel1-4 use.
  # MOTOR_MAX_LINEAR_MPS/MOTOR_WHEEL_BASE_M match mujoco_rbx_bridge.py's own
  # WHEEL_RADIUS_M/WHEEL_TRACK_M-derived physical model -- this conversion
  # has to agree with the bridge's.
  MOTOR_MAX_LINEAR_MPS = 0.5
  MOTOR_WHEEL_BASE_M = 0.34

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
    # Image relay. Four always-live topics (robot/scene x color/depth-view),
    # matching rbx_sim_node.py's/rbx_webots_node.py's exact shape -- see this
    # file's own 2026-09-21 module-docstring update.
    self.image_topic_name = self.device_name + "/color_2d_image"
    self.robot_color_topic_name = self.image_topic_name + "/" + self.ROBOT_COLOR_TOPIC_SUFFIX
    self.scene_color_topic_name = self.image_topic_name + "/" + self.SCENE_COLOR_TOPIC_SUFFIX
    self.robot_depth_topic_name = self.image_topic_name + "/" + self.ROBOT_DEPTH_TOPIC_SUFFIX
    self.scene_depth_topic_name = self.image_topic_name + "/" + self.SCENE_DEPTH_TOPIC_SUFFIX
    self.image_pub_robot_color = nepi_sdk.create_publisher(self.robot_color_topic_name, Image, queue_size = 1)
    self.image_pub_scene_color = nepi_sdk.create_publisher(self.scene_color_topic_name, Image, queue_size = 1)
    self.image_pub_robot_depth = nepi_sdk.create_publisher(self.robot_depth_topic_name, Image, queue_size = 1)
    self.image_pub_scene_depth = nepi_sdk.create_publisher(self.scene_depth_topic_name, Image, queue_size = 1)

    self.sensor_topics = [
      (self.robot_color_topic_name, 'sensor_msgs/Image'),
      (self.scene_color_topic_name, 'sensor_msgs/Image'),
      (self.robot_depth_topic_name, 'sensor_msgs/Image'),
      (self.scene_depth_topic_name, 'sensor_msgs/Image'),
    ]

    ##############################
    # Goto controller state
    self.goto_target = None
    self.goto_target_lock = threading.Lock()
    self.stop_triggered = False

    ##############################
    # Manual motor-ratio state: 4 independently-actuated wheels (see module
    # docstring), [0]=front_left, [1]=front_right, [2]=rear_left, [3]=rear_right.
    self.motor_ratios = [0.0, 0.0, 0.0, 0.0]

    ##############################
    # Home position state: local ENU x/y/z meters (see rbx_gazebo_node.py's
    # getHome/setHome for why this reuses the GeoPoint plumbing)
    self.home_x_m = 0.0
    self.home_y_m = 0.0
    self.home_z_m = 0.0

    ##############################
    # Initialize RBX Settings
    # FIXED (2026-09-22): same bug as rbx_webots_node.py's identical fix --
    # a bare copy.deepcopy(self.FACTORY_SETTINGS) is missing 'options'/
    # 'bounds'/'default', so nepi_controls.get_clean_value() raises an
    # uncaught KeyError: 'options' on every Discrete-typed setting update
    # (e.g. "environment"), never reaching setEnvironmentAction. See
    # rbx_ardupilot_node.py's own initSettingsDict for the full reasoning.
    self.settings_dict = self.initSettingsDict()
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
    self.rbx_if.setImageTopicCb(String(data = self.robot_color_topic_name))

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

  def initSettingsDict(self):
    # Ported from rbx_ardupilot_node.py's own initSettingsDict, same as
    # rbx_webots_node.py's identical copy -- see that file's own comment
    # for the full reasoning.
    init_settings_dict = dict()
    for setting_name in self.CAP_SETTINGS.keys():
      cap_setting = self.CAP_SETTINGS[setting_name]
      setting_type = cap_setting['type']
      setting_dict = dict()
      setting_dict['type'] = setting_type
      if 'options' in cap_setting.keys():
        try:
          if setting_type == 'Int':
            setting_dict['bounds'] = [int(cap_setting['options'][0]), int(cap_setting['options'][1])]
          elif setting_type == 'Float':
            setting_dict['bounds'] = [float(cap_setting['options'][0]), float(cap_setting['options'][1])]
          else:
            setting_dict['options'] = [str(option) for option in cap_setting['options']]
        except Exception as e:
          self.msg_if.pub_warn("Invalid bounds/options for setting: " + setting_name + " : " + str(e))

      default = None
      if setting_name in self.FACTORY_SETTINGS.keys():
        default = self.FACTORY_SETTINGS[setting_name]['value']
      if setting_name in self.FACTORY_SETTINGS_OVERRIDES.keys():
        default = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
      if default is None:
        continue
      try:
        if setting_type == 'Int':
          default = int(float(default))
        elif setting_type == 'Float':
          default = float(default)
        elif setting_type == 'Toggle':
          default = (str(default) == 'True' or str(default) == 'true')
        else:
          default = str(default)
      except Exception as e:
        self.msg_if.pub_warn("Invalid factory value for setting: " + setting_name + " : " + str(e))
        continue
      setting_dict['default'] = default
      init_settings_dict[setting_name] = setting_dict

    settings_dict = nepi_controls.create_controls_dict(init_settings_dict)
    return settings_dict

  def getSettings(self):
    # Deep copy, not a bare reference -- see rbx_webots_node.py's identical
    # 2026-09-21 fix comment for the full cross-object mutation hazard this
    # avoids (SettingsIF assigns whatever this returns directly to its own
    # self.settings_dict, then mutates it in place).
    return copy.deepcopy(self.settings_dict)

  def settingUpdateFunction(self, setting_name, setting_value):
    # FIXED (2026-09-21): SettingsIF calls setSettingFunction(name, value,
    # [callback_arg]), not this function's old single combined-dict
    # argument, and expects a 3-tuple (success, msg, settings_dict) back --
    # see rbx_webots_node.py's identical fix (same incident, same root
    # cause: this driver was built from the same stale template).
    success = False
    setting_str = setting_name + ":" + str(setting_value)
    if setting_name not in self.settings_dict.keys():
      msg = (self.node_name + " Setting name " + setting_str + " is not supported")
      return success, msg, copy.deepcopy(self.settings_dict)
    if nepi_controls.get_clean_value(self.settings_dict, setting_name, setting_value) is None:
      msg = (self.node_name + " Setting data " + setting_str + " is not valid")
      return success, msg, copy.deepcopy(self.settings_dict)

    self.settings_dict = nepi_controls.set_control_value(self.settings_dict, setting_name, setting_value)
    success = True
    msg = (self.node_name + " UPDATED SETTINGS " + setting_str)
    if setting_name in self.CAMERA_SETTING_NAMES:
      self.sendCameraSettings()
    if setting_name in self.ENVIRONMENT_SETTING_NAMES:
      self.setEnvironmentAction(setting_value)
    return success, msg, copy.deepcopy(self.settings_dict)


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
    # depend on knowing the current position/heading.
    with self.sock_lock:
      return self.sock is not None

  def setMotorControlRatio(self, motor_ind, speed_ratio):
    if motor_ind < 0 or motor_ind >= len(self.motor_ratios):
      self.msg_if.pub_warn("Motor control ignored: motor index " + str(motor_ind) + " out of range")
      return
    # -1.0..1.0, not 0.0..1.0 -- rbx_rover.xml's wheels genuinely reverse.
    self.motor_ratios[motor_ind] = max(-1.0, min(1.0, speed_ratio))

  def getMotorControlRatios(self):
    return self.motor_ratios

  def autonomousControlsReady(self):
    # Gates all goto commands: require the Sim Connector's own
    # autonomous_movement_enabled toggle (checked here, not just hidden in
    # the RUI, so a client bypassing the RUI can't do what was turned off
    # either), plus a live bridge connection with fresh telemetry so goto
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
    # Genuine reset -- see module/class docstrings. mujoco_rbx_bridge.py
    # owns the physics state directly (no external Supervisor restriction the
    # way Webots' Robot node has), so this actually teleports the model back
    # to its initial pose. Returns True/False based on whether the command
    # was actually sent (same as every other driver here) -- the bridge
    # applies it synchronously once received.
    self.clearGotoTarget()
    self.sendVelocityCmd(0.0, 0.0)
    with self.sock_lock:
      connected = self.sock is not None
    if not connected:
      return False
    self.sendLineToBridge({'type': 'reset'}, "Reset sim")
    return True

  def setEnvironmentAction(self, environment_value):
    # Fire-and-forget, same as rbx_webots_node.py's -- the bridge now really
    # toggles the obstacle course on this message (see the
    # ENVIRONMENT_OPTIONS class comment).
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
    elif any(self.motor_ratios):
      lin, ang = self.motorControlToVelocity()
    self.sendVelocityCmd(lin, ang)

  def motorControlToVelocity(self):
    # [0]=front_left, [1]=front_right, [2]=rear_left, [3]=rear_right --
    # averaged per side since the wire protocol only ever carries one lin/ang
    # pair (see module docstring); each of the 4 sliders is still
    # individually movable and has a real effect on its own side's average.
    left = (self.motor_ratios[0] + self.motor_ratios[2]) / 2.0
    right = (self.motor_ratios[1] + self.motor_ratios[3]) / 2.0
    max_lin = float(self.settings_dict['max_linear_speed_mps']['value'])
    lin = (left + right) / 2.0 * max_lin
    ang = (right - left) / self.MOTOR_WHEEL_BASE_M * max_lin
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
    # LISTENS on bridge_port and mujoco_rbx_bridge.py dials in, instead of
    # this node dialing out to a configured host -- see rbx_webots_node.py's
    # own 2026-09-21 bridgeLoop comment for the full reasoning this mirrors
    # exactly (this VM cannot be reached from the device on any port).
    srv = None
    while srv is None and not nepi_sdk.is_shutdown():
      try:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(('0.0.0.0', int(self.bridge_port)))
        srv.listen(1)
      except Exception as e:
        self.msg_if.pub_warn("Bridge listen on 0.0.0.0:" + str(self.bridge_port) +
                             " failed: " + str(e))
        srv = None
        time.sleep(self.RECONNECT_INTERVAL_SEC)
    if srv is None:
      return
    self.msg_if.pub_info("Listening for mujoco bridge on 0.0.0.0:" + str(self.bridge_port))

    while not nepi_sdk.is_shutdown():
      try:
        sock, addr = srv.accept()
        sock.settimeout(self.SOCKET_TIMEOUT_SEC)
      except Exception:
        continue
      with self.sock_lock:
        self.sock = sock
      self.msg_if.pub_info("MuJoCo bridge connected from " + str(addr[0]))
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
      self.msg_if.pub_warn("MuJoCo bridge connection lost -- waiting for reconnect")

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
    # "camera" tag picks which of the four publishers this frame goes to --
    # see rbx_sim_node.py's/rbx_webots_node.py's identical processImageLine.
    try:
      camera = msg.get('camera', self.ROBOT_COLOR_TOPIC_SUFFIX)
      pub_attr = self.CAMERA_PUB_ATTR.get(camera, "image_pub_robot_color")
      image_pub = getattr(self, pub_attr)
      jpeg_bytes = base64.b64decode(msg['data'])
      arr = np.frombuffer(jpeg_bytes, dtype = np.uint8)
      cv2_img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
      if cv2_img is None:
        raise ValueError("cv2.imdecode returned None")
      ros_img = nepi_img.cv2img_to_rosimg(cv2_img, encoding = "bgr8")
      image_pub.publish(ros_img)
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
    # No view_mode -- this model's two cameras are always both live. Real,
    # applied settings (2026-09-21) -- mujoco_rbx_bridge.py's
    # applyCameraSettings writes model.cam_pos/model.cam_fovy directly, no
    # respawn needed.
    cmd = {
      'type': 'camera_settings',
      'offset_x': float(self.settings_dict['camera_offset_x']['value']),
      'offset_y': float(self.settings_dict['camera_offset_y']['value']),
      'offset_z': float(self.settings_dict['camera_offset_z']['value']),
      'offset_yaw': float(self.settings_dict['camera_offset_yaw']['value']),
      'offset_tilt': float(self.settings_dict['camera_offset_tilt']['value']),
      'scene_offset_x': float(self.settings_dict['scene_offset_x']['value']),
      'scene_offset_y': float(self.settings_dict['scene_offset_y']['value']),
      'scene_offset_z': float(self.settings_dict['scene_offset_z']['value']),
      'scene_offset_yaw': float(self.settings_dict['scene_offset_yaw']['value']),
      'scene_offset_tilt': float(self.settings_dict['scene_offset_tilt']['value']),
      'fov_deg': float(self.settings_dict['camera_fov_deg']['value']),
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
  MujocoNode()
