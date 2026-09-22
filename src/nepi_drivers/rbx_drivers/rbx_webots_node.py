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
# UPDATED (2026-09-21): rbx_rover.wbt (copied from sim_connector_rover.wbt,
# then extended) now has a real second (scene/chase) camera plus a
# RangeFinder depth pair on both cameras, matching Gazebo's generic_rover
# two-independently-posed-camera-link model -- see rbx_rover.wbt's own
# comment for the geometry. Four always-live topics now
# (robot_color/scene_color/robot_depth/scene_depth), no view-mode Setting,
# matching rbx_sim_node.py's exact convention. camera_offset_x/y/z and
# scene_offset_x/y/z are real, live-applied offsets now too (see
# webots_rbx_bridge.py's applyCameraSettings) -- reference frames are still
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
from nepi_sdk import nepi_controls
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

  # UPDATED (2026-09-21): rbx_rover.wbt now has a REAL second (scene/chase)
  # camera, rigidly mounted at the same offset generic_rover/model.sdf's own
  # camera_link_chase uses, plus a RangeFinder depth pair for both -- the
  # camera_view_mode-Setting mistake this comment used to describe (offering
  # a view that didn't exist) no longer applies; there are genuinely two
  # views now, always both live, matching rbx_sim_node.py's own four
  # always-live topics.
  ROBOT_COLOR_TOPIC_SUFFIX = "robot_color"
  SCENE_COLOR_TOPIC_SUFFIX = "scene_color"
  ROBOT_DEPTH_TOPIC_SUFFIX = "robot_depth"
  SCENE_DEPTH_TOPIC_SUFFIX = "scene_depth"

  # Which publisher each bridge "camera" tag routes to -- see
  # rbx_sim_node.py's identical CAMERA_PUB_ATTR for the full reasoning.
  CAMERA_PUB_ATTR = {
    "robot_color": "image_pub_robot_color",
    "scene_color": "image_pub_scene_color",
    "robot_depth": "image_pub_robot_depth",
    "scene_depth": "image_pub_scene_depth",
  }

  ROBOT_MAIN_REFERENCE_FRAME = "base_link"

  # OBSTACLE_COURSE really spawns/despawns now (2026-09-21) --
  # webots_rbx_bridge.py's setObstacleCourseEnabled imports/removes a
  # DEF OBSTACLE_COURSE Solid built from sim_container/models/
  # obstacle_course/dimensions.yaml (generate_environment_wbt.py), live via
  # Supervisor, in response to the same wire message this driver already
  # sends below. sim_connector_bridge_webots.py's own separate world/
  # protocol pair is unaffected -- its own documented gap stays as-is.
  ENVIRONMENT_OPTIONS = ["FLAT_GROUND", "OBSTACLE_COURSE"]
  OBSTACLE_COURSE_OPTION = "OBSTACLE_COURSE"

  # UPDATED (2026-09-21): rbx_rover.wbt now has a real second (scene/chase)
  # camera plus depth on both (see that file's own comment), and
  # webots_rbx_bridge.py's applyCameraSettings live-writes each camera's
  # translation field directly on every change -- no respawn needed, unlike
  # Gazebo. camera_offset_x/y/z (robot view) and scene_offset_x/y/z (scene
  # view) are therefore genuinely wired now, not a placeholder. camera_fov_deg
  # applies to both cameras' fieldOfView, matching rbx_sim_node.py's own
  # "both cameras share one FOV" convention. yaw/tilt are NOT included yet --
  # rotating a camera live needs real rotation composition on top of
  # camera_chase's existing pitch, not just a field write, and wasn't built
  # in this pass (see webots_rbx_bridge.py's own applyCameraSettings comment).
  CAMERA_SETTING_NAMES = ("camera_offset_x", "camera_offset_y", "camera_offset_z",
                          "scene_offset_x", "scene_offset_y", "scene_offset_z",
                          "camera_fov_deg")
  ENVIRONMENT_SETTING_NAMES = ("environment",)

  # Sim Connector's own per-robot-config "customize the capabilities that are
  # open" toggles -- same mechanism and same names as rbx_sim_node.py's own
  # CAPABILITY_SETTING_NAMES (see that file's comment for the full reasoning).
  # Enforced in autonomousControlsReady below, not just hidden in the RUI, so
  # a client bypassing the RUI can't do what was turned off either.
  CAPABILITY_SETTING_NAMES = ("autonomous_movement_enabled",
                              "camera_controls_enabled", "enabled_image_sources")

  CAP_SETTINGS = dict(
    max_linear_speed_mps = {"type":"Float","name":"max_linear_speed_mps","options":["0.05","2.0"]},
    max_angular_rate_dps = {"type":"Float","name":"max_angular_rate_dps","options":["5.0","180.0"]},
    environment = {"type":"Discrete","name":"environment","options":ENVIRONMENT_OPTIONS},
    camera_offset_x = {"type":"Float","name":"camera_offset_x","options":["-10.0","10.0"]},
    camera_offset_y = {"type":"Float","name":"camera_offset_y","options":["-10.0","10.0"]},
    camera_offset_z = {"type":"Float","name":"camera_offset_z","options":["-10.0","10.0"]},
    scene_offset_x = {"type":"Float","name":"scene_offset_x","options":["-10.0","10.0"]},
    scene_offset_y = {"type":"Float","name":"scene_offset_y","options":["-10.0","10.0"]},
    scene_offset_z = {"type":"Float","name":"scene_offset_z","options":["-10.0","10.0"]},
    camera_fov_deg = {"type":"Float","name":"camera_fov_deg","options":["10.0","150.0"]},
    autonomous_movement_enabled = {"type":"Discrete","name":"autonomous_movement_enabled","options":["TRUE","FALSE"]},
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
    # Real, wired offsets now (2026-09-21) -- zero means "at rbx_rover.wbt's
    # own factory mount point", same convention as rbx_sim_node.py's values
    # reproducing generic_rover/model.sdf's real camera_link poses.
    camera_offset_x = {"type":"Float","name":"camera_offset_x","value":"0.0"},
    camera_offset_y = {"type":"Float","name":"camera_offset_y","value":"0.0"},
    camera_offset_z = {"type":"Float","name":"camera_offset_z","value":"0.0"},
    scene_offset_x = {"type":"Float","name":"scene_offset_x","value":"0.0"},
    scene_offset_y = {"type":"Float","name":"scene_offset_y","value":"0.0"},
    scene_offset_z = {"type":"Float","name":"scene_offset_z","value":"0.0"},
    # 45.0 matches Webots' own Camera default fieldOfView (0.785398 rad),
    # which rbx_rover.wbt never overrides -- the true factory value, not a
    # guess.
    camera_fov_deg = {"type":"Float","name":"camera_fov_deg","value":"45.0"},
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
  # RESET_SIM is a real setup action, and (2026-09-21) a real physical
  # teleport too -- rbx_rover.wbt's Robot node is now `supervisor TRUE`,
  # so webots_rbx_bridge.py's resetSim() actually moves the body back to its
  # spawn pose instead of logging a no-op.
  RBX_SETUP_ACTIONS = ["RESET_SIM", "RETURN_HOME"]
  RBX_GO_ACTIONS = []

  GO_HOME_TIMEOUT_SEC = 60.0
  GO_HOME_POLL_INTERVAL_SEC = 0.2

  RECONNECT_INTERVAL_SEC = 3.0
  SOCKET_TIMEOUT_SEC = 5.0

  CONTROLLER_RATE_HZ = 20
  NAVPOSE_UPDATE_RATE = 10
  TELEMETRY_FRESH_SEC = 2.0

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
    # matching rbx_sim_node.py's exact ROBOT_COLOR_TOPIC_SUFFIX/etc shape --
    # see this file's own 2026-09-21 module-docstring update for why (rover
    # gained a real scene/chase camera plus depth on both cameras).
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
    # FIXED (2026-09-22): a bare copy.deepcopy(self.FACTORY_SETTINGS) means
    # every entry is missing 'options'/'bounds'/'default' -- the modern
    # controls-dict shape nepi_controls.get_clean_value() actually requires.
    # Confirmed live: toggling the "environment" Discrete setting raised an
    # uncaught KeyError: 'options' inside get_clean_value (never reaching
    # setEnvironmentAction), reported as "obstacle course spawning doesnt
    # work" -- rbx_ardupilot_node.py already has the fix (its own
    # initSettingsDict() docstring documents the identical bug, found there
    # first as an empty RUI Settings panel), just never ported to this
    # driver. See that file's initSettingsDict for the full reasoning.
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
    # CAP_SETTINGS/FACTORY_SETTINGS are a legacy, pre-controls-system shape
    # ({'type','name','options'} / {'type','name','value'}) that RBXRobotIF's
    # own capSettings=/factorySettings= constructor args accept but never
    # actually consume any more -- getSettingsFunction/setSettingFunction are
    # the only things SettingsIF actually reads, and it requires a real
    # nepi_controls controls dict (each entry carrying 'default', and
    # 'bounds'/'options' as its type requires), not this legacy shape.
    # Ported from rbx_ardupilot_node.py's own initSettingsDict -- that
    # driver's docstring documents the identical bug this fixes (there,
    # first noticed as an empty RUI Settings panel; here, as "environment"
    # setting updates raising an uncaught KeyError: 'options' inside
    # nepi_controls.get_clean_value, which never reached setEnvironmentAction
    # at all, reported as "obstacle course spawning doesnt work").
    init_settings_dict = dict()
    for setting_name in self.CAP_SETTINGS.keys():
      cap_setting = self.CAP_SETTINGS[setting_name]
      setting_type = cap_setting['type']
      setting_dict = dict()
      setting_dict['type'] = setting_type
      # The retired cap-settings form carried an Int/Float control's min and
      # max in an 'options' pair; a Selection/Discrete/String control's
      # actual option list also rode in 'options' -- the controls contract
      # splits these into 'bounds' (numeric) vs 'options' (named choices).
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
    # Deep copy, not a bare reference -- SettingsIF assigns whatever this
    # returns directly to its own self.settings_dict, then mutates individual
    # entries of it in place; a bare reference would alias the two, matching
    # the cross-object mutation hazard rbx_ardupilot_node.py's own
    # getSettings() already documents and rbx_sim_node.py's own copy already
    # avoids. FIXED (2026-09-21) -- was returning self.settings_dict directly.
    return copy.deepcopy(self.settings_dict)

  def settingUpdateFunction(self, setting_name, setting_value):
    # FIXED (2026-09-21): SettingsIF (system_if.py) calls
    # setSettingFunction(name, value, [callback_arg]) -- two/three plain
    # positional args, not the single combined {'name','type','value'} dict
    # this function's own body used to be written against (confirmed live:
    # "settingUpdateFunction() takes 2 positional arguments but 3 were
    # given" on every settings update attempt). Also was returning a 2-tuple
    # (success, msg) where SettingsIF unpacks 3 (success, msg, settings_dict).
    # Same incident, same fix, as rbx_ardupilot_node.py's own
    # settingUpdateFunction and rbx_sim_node.py's (this driver was built from
    # a stale template that predated that fix -- see
    # docs/WEBOTS_RBX_DRIVER_PLAN.md's own note on this).
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
    # -1.0..1.0, not 0.0..1.0 -- a wheeled rover's motors genuinely reverse,
    # same reasoning rbx_sim_node.py's/rbx_mujoco_node.py's own identical
    # methods already document. Reported live (2026-09-21): "making the
    # motors negative doesnt make it go backwards" -- root cause was this
    # clamp silently discarding every negative speed_ratio.
    self.motor_ratios[motor_ind] = max(-1.0, min(1.0, speed_ratio))

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
    # Fire-and-forget, same as rbx_gazebo_node.py's. UPDATED (2026-09-21):
    # rbx_rover.wbt's Robot node is now `supervisor TRUE`, so
    # webots_rbx_bridge.py's resetSim() performs a real teleport back to the
    # spawn pose (reported live: "the reset_sim button also doesnt work,
    # bringing the robot back to the starting point"). Still returns
    # True/False based on whether the command was actually SENT, not on
    # whether the physical reset completed -- this driver has no ack for
    # that, same as before.
    self.clearGotoTarget()
    self.sendVelocityCmd(0.0, 0.0)
    with self.sock_lock:
      connected = self.sock is not None
    if not connected:
      return False
    self.sendLineToBridge({'type': 'reset'}, "Reset sim")
    return True

  def setEnvironmentAction(self, environment_value):
    # Fire-and-forget, same as rbx_gazebo_node.py's -- the bridge now really
    # spawns/despawns the obstacle course on this message (see the
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
    # LISTENS on bridge_port and webots_rbx_bridge.py dials in, instead of
    # this node dialing out to a configured host (see rbx_sim_node.py's own
    # 2026-09-08 module-docstring note for the full reasoning this mirrors:
    # a device-dials-VM connection requires the VM to accept an unsolicited
    # inbound connection, which a very common real setup -- Windows + WSL2 --
    # blocks by default; outbound from the VM is never blocked). Confirmed
    # live (2026-09-21) that the VM cannot be reached at all from this
    # device on any port, so the old dial-out direction could never have
    # worked here regardless of host/tunnel configuration.
    #
    # Bind/listen once; accept in a loop so the VM side can restart
    # independently of this node -- any disconnect just goes back to
    # accept() and waits for the next connection.
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
    self.msg_if.pub_info("Listening for webots bridge on 0.0.0.0:" + str(self.bridge_port))

    while not nepi_sdk.is_shutdown():
      try:
        sock, addr = srv.accept()
        sock.settimeout(self.SOCKET_TIMEOUT_SEC)
      except Exception:
        continue
      with self.sock_lock:
        self.sock = sock
      self.msg_if.pub_info("Webots bridge connected from " + str(addr[0]))
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
      self.msg_if.pub_warn("Webots bridge connection lost -- waiting for reconnect")

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
    # see rbx_sim_node.py's identical processImageLine/CAMERA_PUB_ATTR.
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
    # No view_mode -- this world's two cameras are always both live. Real,
    # applied settings now (2026-09-21) -- webots_rbx_bridge.py's
    # applyCameraSettings live-writes each field, no respawn needed.
    cmd = {
      'type': 'camera_settings',
      'offset_x': float(self.settings_dict['camera_offset_x']['value']),
      'offset_y': float(self.settings_dict['camera_offset_y']['value']),
      'offset_z': float(self.settings_dict['camera_offset_z']['value']),
      'scene_offset_x': float(self.settings_dict['scene_offset_x']['value']),
      'scene_offset_y': float(self.settings_dict['scene_offset_y']['value']),
      'scene_offset_z': float(self.settings_dict['scene_offset_z']['value']),
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
  WebotsNode()
