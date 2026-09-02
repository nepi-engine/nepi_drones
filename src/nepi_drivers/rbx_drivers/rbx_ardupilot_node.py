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

import os

import time
import numpy as np
import math
import tf
import random
import sys
import socket
import cv2
import copy
import base64
import json
import threading

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_settings
from nepi_sdk import nepi_img

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Bool, String, Float32, Float64
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPose, GeoPoseStamped
from mavros_msgs.msg import State, AttitudeTarget, StatusText
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest, CommandHome, CommandHomeRequest, CommandLong, CommandLongRequest, StreamRate, StreamRateRequest

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, NavSatFix, BatteryState

from nepi_interfaces.msg import AxisControls

from nepi_api.device_if_rbx import RBXRobotIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'RBX_ARDUPILOT' # Use in display menus
FILE_TYPE = 'NODE'



#########################################
# Node Class
#########################################

#class ardupilot_rbx_node(object):
class ArdupilotNode:
  DEFAULT_NODE_NAME = "ardupilot" # connection port added once discovered

  # Camera-rig feature (Universal Simulator Bridge, ArduPilot SITL port):
  # two body-frame offset triples, one per always-live camera (robot/scene
  # view), following the exact same live CAP_SETTINGS/FACTORY_SETTINGS
  # pattern as rbx_sim_node.py's own camera settings -- no new mechanism.
  # No camera_view_mode any more (2026-08-18): both views used to be one
  # topic switched by this Setting, teleporting the SAME single Gazebo
  # camera model between two poses -- reworked so both are separate,
  # always-live Gazebo models (camera_rig_controller_ardupilot.py) and
  # separate ROS Image topics (see image_pub_robot_color/image_pub_scene_color
  # below), after a live report that a single reassignable topic and a
  # single teleported camera model meant the "third-person view" wasn't
  # really an independent thing a client could rely on. Further expanded
  # 2026-08-20: each of the two views now always publishes three
  # simultaneous topics (color, colorized-depth-view, raw-depth-map) instead
  # of switching between color and depth via a depth_map_enabled toggle --
  # see CAMERA_PUB_ATTR/DEPTH_MAP_CAMERAS below.
  CAMERA_SETTING_NAMES = ("camera_offset_x", "camera_offset_y", "camera_offset_z",
                          "scene_offset_x", "scene_offset_y", "scene_offset_z")

  # Sim Connector "customize the capabilities that are open" toggles -- same
  # mechanism and same three names as rbx_sim_node.py's own
  # CAPABILITY_SETTING_NAMES (see that file's comment for the full reasoning).
  # Added here for parity: the quadcopter's robot config (flight_robot_4_motor)
  # is exactly as much a Sim Connector-managed simulated robot as the rover's,
  # so it gets the same three configuration toggles, not a lesser set.
  # enabled_image_sources added for parity with rbx_sim_node.py's own
  # curation checklist Setting (NepiDeviceRBX.js's Image Source dropdown /
  # Nepi_IF_Sim-Controls.js's renderImageSourceCuration) -- was missing here
  # entirely, which meant that whole enable/disable-cameras UI section never
  # rendered for this driver at all (its gate is
  # rbxSettingsNamesList.includes("enabled_image_sources")), including the
  # physical camera candidate the curation list would otherwise have shown.
  CAPABILITY_SETTING_NAMES = ("autonomous_movement_enabled", "teleop_movement_enabled",
                              "camera_controls_enabled", "enabled_image_sources")

  CAP_SETTINGS = dict(
    takeoff_height_m = {"type":"Float","name":"takeoff_height_m","options":["0.0","100.0"]},
    takeoff_min_pitch_deg =  {"type":"Float","name":"takeoff_min_pitch_deg","options":["-90.0","90.0"]},
    motor_count = {"type":"Int","name":"motor_count","options":["1","16"]},
    motor_test_max_throttle_percent = {"type":"Float","name":"motor_test_max_throttle_percent","options":["0.0","100.0"]},
    motor_test_timeout_s = {"type":"Float","name":"motor_test_timeout_s","options":["1.0","300.0"]},
    camera_offset_x = {"type":"Float","name":"camera_offset_x","options":["-10.0","10.0"]},
    camera_offset_y = {"type":"Float","name":"camera_offset_y","options":["-10.0","10.0"]},
    camera_offset_z = {"type":"Float","name":"camera_offset_z","options":["-10.0","10.0"]},
    scene_offset_x = {"type":"Float","name":"scene_offset_x","options":["-10.0","10.0"]},
    scene_offset_y = {"type":"Float","name":"scene_offset_y","options":["-10.0","10.0"]},
    scene_offset_z = {"type":"Float","name":"scene_offset_z","options":["-10.0","10.0"]},
    autonomous_movement_enabled = {"type":"Discrete","name":"autonomous_movement_enabled","options":["TRUE","FALSE"]},
    teleop_movement_enabled = {"type":"Discrete","name":"teleop_movement_enabled","options":["TRUE","FALSE"]},
    camera_controls_enabled = {"type":"Discrete","name":"camera_controls_enabled","options":["TRUE","FALSE"]},
    # No fixed options -- the candidate topic set is per-deployment.
    enabled_image_sources = {"type":"String","name":"enabled_image_sources"}
  )

  FACTORY_SETTINGS = dict(
    takeoff_height_m = {"type":"Float","name":"takeoff_height_m","value":"5"},
    takeoff_min_pitch_deg =  {"type":"Float","name":"takeoff_min_pitch_deg","value":"10"},
    motor_count = {"type":"Int","name":"motor_count","value":"4"},
    motor_test_max_throttle_percent = {"type":"Float","name":"motor_test_max_throttle_percent","value":"20"},
    motor_test_timeout_s = {"type":"Float","name":"motor_test_timeout_s","value":"30"},
    # Matches camera_rig_controller_ardupilot.py's own defaults -- robot view
    # is forward and slightly below the body, a nose/belly-mounted
    # inspection-camera convention (distinct from the rover's flat
    # camera_link mount point since this is a multirotor, not a ground
    # vehicle); scene view is behind and above, a chase-cam convention
    # scaled down from the rover's own scene_offset_* defaults.
    camera_offset_x = {"type":"Float","name":"camera_offset_x","value":"0.15"},
    camera_offset_y = {"type":"Float","name":"camera_offset_y","value":"0.0"},
    camera_offset_z = {"type":"Float","name":"camera_offset_z","value":"-0.1"},
    scene_offset_x = {"type":"Float","name":"scene_offset_x","value":"-2.0"},
    scene_offset_y = {"type":"Float","name":"scene_offset_y","value":"0.0"},
    scene_offset_z = {"type":"Float","name":"scene_offset_z","value":"1.0"},
    # All default to enabled: a robot config that never touches these
    # settings behaves exactly as it did before this feature existed.
    autonomous_movement_enabled = {"type":"Discrete","name":"autonomous_movement_enabled","value":"TRUE"},
    teleop_movement_enabled = {"type":"Discrete","name":"teleop_movement_enabled","value":"TRUE"},
    camera_controls_enabled = {"type":"Discrete","name":"camera_controls_enabled","value":"TRUE"},
    # Empty = unrestricted -- see the CAPABILITY_SETTING_NAMES comment above.
    enabled_image_sources = {"type":"String","name":"enabled_image_sources","value":""}
  )

  FACTORY_SETTINGS_OVERRIDES = dict()

  # Camera-rig bridge (see camera_rig_controller_ardupilot.py and the
  # session summary for the full design). Fixed constants, not sourced from
  # DEVICE_DICT: unlike the rover's per-slot rbx_sim driver, ArduPilot SITL
  # is single-instance-only on this dev VM, so there is exactly one bridge
  # endpoint, matching how RESET_SIM_HOST/RESET_SIM_PORT are also fixed
  # constants on this same class rather than discovery-supplied.
  CAMERA_BRIDGE_HOST = "127.0.0.1"
  CAMERA_BRIDGE_PORT = 9026
  CAMERA_RECONNECT_INTERVAL_SEC = 3.0
  CAMERA_SOCKET_TIMEOUT_SEC = 5.0

  # Watchdog for a live-observed wedge: the bridge socket can stay
  # ESTABLISHED and keep accepting bytes at the TCP layer (confirmed via
  # `ss` byte counters climbing normally) while the recv/parse loop above it
  # never advances -- no frame ever reaches processCameraImageLine, camera
  # topics sit at 0 Hz indefinitely, and only a full node restart recovered
  # it (reported live as "the drone camera takes forever to show up" /
  # "the scene camera doesn't even exist" -- the exact case that motivated
  # this). The precise wedge mechanism wasn't pinned down (bytes were
  # provably arriving, so it isn't a dead/blocked recv()), so this doesn't
  # try to fix that mechanism directly -- it just notices "no frame has
  # gone out in far longer than the ~14 fps (2 cameras x 7 Hz) bridge should
  # ever go quiet for" and forces the socket closed, which unblocks
  # cameraBridgeLoop's recv() (if that's where it's actually stuck) or at
  # minimum forces a clean reconnect either way, mirroring the same
  # shutdown()-before-close() cross-thread unwedge already used by
  # camera_rig_controller_ardupilot.py's sendLineToClient for the
  # server-side half of this same bridge.
  CAMERA_STALE_TIMEOUT_SEC = 15.0
  CAMERA_WATCHDOG_INTERVAL_SEC = 5.0

  # Real-hardware camera relay: a real onboard camera (unlike the VM-side
  # simulator) is already a normal local ROS topic on this same ROS master --
  # no bridge protocol needed, just a subscriber. Runs independently of the
  # sim camera bridge above; whichever source is actually present feeds
  # image_pub. "idx/color_image" is the real IDX camera driver's own topic
  # convention (device_if_idx.py), distinct from the color_2d_image name this
  # driver publishes its own relayed output under -- searched for by
  # substring the same way the rest of this codebase finds live topics
  # (nepi_sdk.find_topic). SITL-only guard on where this thread is started
  # (see __init__): a SITL instance runs on a dev rig that can have an
  # unrelated physical camera plugged in (e.g. a USB webcam used for other
  # testing), and this substring match has no way to tell that apart from a
  # genuine onboard camera -- confirmed live hijacking robot_color away from
  # the sim bridge's own camera_rig feed the moment any such camera existed
  # on the device. Real hardware has no such ambiguity.
  REAL_CAMERA_TOPIC_PATTERN = "idx/color_image"
  REAL_CAMERA_WATCH_INTERVAL_SEC = 3.0


  # RBX State and Mode Dictionaries
  RBX_NAVPOSE_HAS_GPS = True
  RBX_NAVPOSE_HAS_ORIENTATION = True
  RBX_NAVPOSE_HAS_HEADING = True

  RBX_STATES = ["DISARM","ARM"]
  RBX_MODES = ["STABILIZE","LAND","RTL","LOITER","GUIDED","RESUME"]
  RBX_SETUP_ACTIONS = ["TAKEOFF","LAUNCH","RESET_SIM"]
  RBX_GO_ACTIONS = []

  RBX_STATE_FUNCTIONS = ["disarm","arm"]
  RBX_MODE_FUNCTIONS = ["stabilize","land","rtl","loiter","guided","resume"]
  RBX_SETUP_ACTION_FUNCTIONS = ["takeoff","launch","reset_sim"]
  RBX_GO_ACTION_FUNCTIONS = []

  # RESET_SIM reaches across the reverse SSH tunnel to a tiny listener
  # (~/.local/bin/gz_reset_listener.py) running on the dev VM where Gazebo
  # actually lives -- this driver runs on the NEPI device, not the VM.
  RESET_SIM_HOST = "127.0.0.1"
  RESET_SIM_PORT = 9021

  SETPOINT_PUBLISH_RATE_HZ = 50
  POSITION_UPDATE_RATE = 10

  # Teleop velocity setpoint loop -- deliberately its own rate/topic, not
  # reusing sendGotoCommandLoop/SETPOINT_PUBLISH_RATE_HZ (see
  # sendTeleopVelocityLoop's own comment for why). 20Hz keeps MAVROS's own
  # velocity-setpoint watchdog fed comfortably -- ArduPilot's SET_POSITION_
  # TARGET_LOCAL_NED handling treats a stream slower than roughly 2Hz as
  # stale and reverts to loiter, so 20Hz has wide margin. Conservative,
  # fixed caps rather than a Setting (unlike rbx_sim_node.py's
  # max_linear_speed_mps): this is new, unverified-in-flight code, and a
  # hard ceiling here is a real safety margin, not just a default.
  TELEOP_PUBLISH_RATE_HZ = 20
  TELEOP_CMD_TIMEOUT_SEC = 0.75
  # Confirmed live 2026-08-13: stopping the setpoint stream outright (no
  # further publishes at all) the instant TELEOP_CMD_TIMEOUT_SEC elapses is
  # NOT the same as stopping the vehicle -- ArduCopter's own GUIDED-mode
  # velocity controller keeps flying at the LAST commanded velocity until ITS
  # OWN internal setpoint-timeout expires (empirically a few seconds, not
  # governed by anything this driver controls), so the vehicle visibly kept
  # drifting for several seconds after this driver had already gone silent.
  # This grace window closes that gap: once idle, publish an EXPLICIT
  # (0,0,0,0) for TELEOP_STOP_GRACE_SEC more before actually going silent, so
  # a real stop reaches ArduCopter directly rather than relying on it timing
  # out on its own. Still bounded (not indefinite), so it does not go on
  # contesting a goto command that starts shortly after teleop ends.
  TELEOP_STOP_GRACE_SEC = 2.0
  TELEOP_MAX_LINEAR_MPS = 2.0
  TELEOP_MAX_ANGULAR_DPS = 30.0

  # MAV_CMD_DO_MOTOR_TEST (verified against pymavlink common.xml -- NOT 176,
  # which is MAV_CMD_DO_SET_MODE). ArduPilot auto-stops the motor after the
  # commanded duration (motor_test_timeout_s setting) even if no further
  # command is sent -- re-sliding re-issues the command and restarts the clock.
  MAV_CMD_DO_MOTOR_TEST = 209
  # MAV_CMD_COMPONENT_ARM_DISARM with the param2 "force" magic value (ArduPilot
  # convention) bypasses the in-flight/pre-arm safety interlock that otherwise
  # rejects a normal disarm while airborne -- required for RESET_SIM, since the
  # whole point is to reset a sim that may currently be flying.
  MAV_CMD_COMPONENT_ARM_DISARM = 400
  MAV_CMD_FORCE_MAGIC = 21196.0
  # Create shared class variables and thread locks 
  
  device_info_dict = dict(device_name = "",
                          path = "",
                          serial_number = "",
                          hw_version = "",
                          sw_version = "")

  # NavPose data provided as a single nepi_nav.BLANK_NAVPOSE_DICT, returned by getNavPoseCb
  navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)


  settings_dict = FACTORY_SETTINGS

  axis_controls = AxisControls()
  axis_controls.x = True
  axis_controls.y = True
  axis_controls.z = True
  axis_controls.roll = True
  axis_controls.pitch = True
  axis_controls.yaw = True

  state_ind = 0
  state_current = "None"
  state_last = "None"

  mode_ind = 0
  mode_current = "None"
  mode_last = "None"

  battery_percent = 0

  mavlink_state = None

  rbx_if = None

  port_id = None

  msg_list = ["","","","","",""]

  takeoff_complete = False
  # Set alongside takeoff_complete's timeout-path False below, never cleared
  # on success -- lets autonomousControlsReady()/teleopControlsReady() keep
  # checking real altitude against the actual goal even after the one-shot
  # completion wait in takeoff_action() has already given up (see the live
  # recheck there for why this matters).
  takeoff_goal_alt = None
  takeoff_error_bound_m = None
  takeoff_reset_modes = ["LAND","RTL"]

  home_loc = GeoPoint()
  home_loc.latitude = -999
  home_loc.longitude = -999
  home_loc.altitude = -999
  home_location = home_loc

  stop_triggered = False

  attitude_target = None
  position_target = None
  location_target = None

  att_sp_seq = 0
  pos_sp_seq = 0
  loc_sp_seq = 0

  gps_connected = False
  has_fake_gps = False

  # Most recent flight-controller STATUSTEXT (e.g. pre-arm/arm rejection reasons).
  # Recorded from the mavros statustext/recv topic and surfaced to the RBX status
  # so operators see the FCU's reason without digging through the mavros log.
  FCU_TEXT_REMIND_S = 10.0   # re-surface a persisting FCU message at most this often
  FCU_TEXT_RECENT_S = 15.0   # treat FCU text within this window as a command's failure reason
  last_fcu_text = ""
  last_fcu_severity = None
  last_fcu_text_time = 0.0
  _last_surfaced_fcu_text = ""
  _last_surfaced_fcu_time = 0.0

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"      
  drv_dict = dict()   
  ### LXS Driver NODE Initialization
  def __init__(self):
    ####  NODE Initialization ####
    nepi_sdk.init_node(name= self.DEFAULT_NODE_NAME)
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
        self.mav_node_name = self.drv_dict['DEVICE_DICT']['mavlink_node_name']
        self.has_fake_gps = self.drv_dict['DEVICE_DICT']['fake_gps']
        # device_path is connection_type + "_" + address (see discoveryFunction) --
        # a SITL connection is always "SITL_<addr>_<port>", every real connection
        # type (SERIAL/USB/TCP/UDP to an actual FCU) is not. Computed once here,
        # instance-wide, since it now gates three independent things: the
        # data_source_description passed to RBXRobotIF below, the real-camera
        # relay thread further down, and the existing Fake GPS reconcile logic.
        self.is_sitl = str(self.device_path).upper().startswith("SITL")
    except Exception as e:
        self.msg_if.pub_warn("Failed to load Device Dict " + str(e))
        nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
        return

    ##############################
    # Get Mavlink NameSpace
    self.msg_if.pub_info("Waiting for mavlink node that includes: " + self.mav_node_name)
    mav_node_name = nepi_sdk.wait_for_node(self.mav_node_name)
    MAVLINK_NAMESPACE = (mav_node_name + '/')
    self.msg_if.pub_info("Using mavlink namespace: " + MAVLINK_NAMESPACE)
    # Start Mavlink State Subscriber
    MAVLINK_STATE_TOPIC = MAVLINK_NAMESPACE + "state"
    # Wait for MAVLink State topic to publish then subscribe
    self.msg_if.pub_info("Waiting for topic: " + MAVLINK_STATE_TOPIC)
    nepi_sdk.wait_for_topic(MAVLINK_STATE_TOPIC)
    self.msg_if.pub_info("Starting state scubscriber callback")
    nepi_sdk.create_subscriber(MAVLINK_STATE_TOPIC, State, self.get_state_callback, queue_size = 1)
    while self.state_current == "None" and not nepi_sdk.is_shutdown():
      self.msg_if.pub_info("Waiting for mavlink state status to set")
      time.sleep(0.1)
    while self.mode_current == "None" and not nepi_sdk.is_shutdown():
      self.msg_if.pub_info("Waiting for mavlink mode status to set")
      time.sleep(0.1)
    self.msg_if.pub_info("Starting State: " + self.state_current)
    self.msg_if.pub_info("Starting Mode: " + self.mode_current)

    # MAVLINK Required Services
    self.msg_if.pub_info("Configuring interfaces for mavlink namespace: " + MAVLINK_NAMESPACE)
    ## Define Mavlink Services Calls
    MAVLINK_SET_HOME_SERVICE = MAVLINK_NAMESPACE + "cmd/set_home"
    MAVLINK_SET_MODE_SERVICE = MAVLINK_NAMESPACE + "set_mode"
    MAVLINK_ARMING_SERVICE = MAVLINK_NAMESPACE + "cmd/arming"
    MAVLINK_TAKEOFF_SERVICE = MAVLINK_NAMESPACE + "cmd/takeoff"
    MAVLINK_COMMAND_SERVICE = MAVLINK_NAMESPACE + "cmd/command"
    MAVLINK_SET_STREAM_RATE_SERVICE = MAVLINK_NAMESPACE + "set_stream_rate"

    # Waiting for these services to actually be advertised (not just
    # connect_service's bare rospy.ServiceProxy, which never checks the ROS
    # graph) matters because mavros's state TOPIC above starts flowing
    # before its service servers finish registering -- a Launch/Arm/Takeoff
    # click landing in that gap called call_service against a not-yet-live
    # service, which fails silently (nepi_sdk.call_service logs failures at
    # DEBUG only) and just retries every check_interval_s for the entire
    # cmd_timeout before anything is ever shown in the RUI. Waiting here
    # instead, before this node reports Initialization Complete, closes
    # that window at startup rather than on whichever command happens to be
    # the first one clicked.
    self.msg_if.pub_info("Waiting for mavlink services under: " + MAVLINK_NAMESPACE)
    nepi_sdk.wait_for_service(MAVLINK_SET_MODE_SERVICE)
    nepi_sdk.wait_for_service(MAVLINK_ARMING_SERVICE)
    nepi_sdk.wait_for_service(MAVLINK_TAKEOFF_SERVICE)

    # Remembered purely so a failed call can name the service it was trying to
    # reach. connect_service is a bare ServiceProxy that validates nothing, so
    # a wrong namespace produces a client that fails every call silently --
    # naming the path in the warning is what makes that case identifiable
    # instead of looking like an FCU problem.
    self.mode_client_name = MAVLINK_SET_MODE_SERVICE
    self.arming_client_name = MAVLINK_ARMING_SERVICE
    self.mavlink_namespace = MAVLINK_NAMESPACE

    self.set_home_client = nepi_sdk.connect_service(MAVLINK_SET_HOME_SERVICE, CommandHome)
    self.mode_client = nepi_sdk.connect_service(MAVLINK_SET_MODE_SERVICE, SetMode)
    self.arming_client = nepi_sdk.connect_service(MAVLINK_ARMING_SERVICE, CommandBool)
    self.takeoff_client = nepi_sdk.connect_service(MAVLINK_TAKEOFF_SERVICE, CommandTOL)
    self.command_client = nepi_sdk.connect_service(MAVLINK_COMMAND_SERVICE, CommandLong)
    self.set_stream_rate_client = nepi_sdk.connect_service(MAVLINK_SET_STREAM_RATE_SERVICE, StreamRate)

    # mavros reporting "connected" only means heartbeat/timesync are flowing --
    # ArduCopter must be explicitly asked to stream the rest (GPS, IMU,
    # global/local position, etc.) or global_position_wgs84_geo etc. below stay
    # permanently stale, silently breaking every altitude/position-based
    # completion check (takeoff climb, goto_location/goto_position) with a
    # timeout that looks like the vehicle never moved, even when it did.
    # STREAM_ALL (stream_id=0) at 10Hz mirrors the working manual fix confirmed
    # in SITL testing (rosservice call .../set_stream_rate).
    self.msg_if.pub_info("Requesting full MAVLink telemetry stream (STREAM_ALL @ 10Hz)")
    stream_rate_req = StreamRateRequest()
    stream_rate_req.stream_id = 0
    stream_rate_req.message_rate = 10
    stream_rate_req.on_off = True
    nepi_sdk.call_service(self.set_stream_rate_client, stream_rate_req)

    # Subscribe to MAVLink topics
    MAVLINK_BATTERY_TOPIC = MAVLINK_NAMESPACE + "battery"

    nepi_sdk.create_subscriber(MAVLINK_BATTERY_TOPIC, BatteryState, self.get_mavlink_battery_callback, queue_size = 1)

    MAVLINK_SOURCE_GPS_TOPIC = MAVLINK_NAMESPACE + "global_position/global"
    MAVLINK_SOURCE_ODOM_TOPIC = MAVLINK_NAMESPACE + "global_position/local"
    MAVLINK_SOURCE_HEADING_TOPIC = MAVLINK_NAMESPACE + "global_position/compass_hdg"

    nepi_sdk.create_subscriber(MAVLINK_SOURCE_GPS_TOPIC, NavSatFix, self.gps_topic_callback, queue_size = 1)
    nepi_sdk.create_subscriber(MAVLINK_SOURCE_ODOM_TOPIC, Odometry, self.odom_topic_callback, queue_size = 1)
    nepi_sdk.create_subscriber(MAVLINK_SOURCE_HEADING_TOPIC, Float64, self.heading_topic_callback, queue_size = 1)

    # FCU status text (pre-arm/arm rejections, EKF messages, failsafes, etc.)
    MAVLINK_STATUSTEXT_TOPIC = MAVLINK_NAMESPACE + "statustext/recv"
    nepi_sdk.create_subscriber(MAVLINK_STATUSTEXT_TOPIC, StatusText, self.get_statustext_callback, queue_size = 10)

    ## Define Mavlink Publishers
    MAVLINK_SETPOINT_ATTITUDE_TOPIC = MAVLINK_NAMESPACE + "setpoint_raw/attitude"
    MAVLINK_SETPOINT_POSITION_LOCAL_TOPIC = MAVLINK_NAMESPACE + "setpoint_position/local"
    MAVLINK_SETPOINT_LOCATION_GLOBAL_TOPIC = MAVLINK_NAMESPACE + "setpoint_position/global"

    self.setpoint_location_global_pub = nepi_sdk.create_publisher(MAVLINK_SETPOINT_LOCATION_GLOBAL_TOPIC, GeoPoseStamped, queue_size=1)
    self.setpoint_attitude_pub = nepi_sdk.create_publisher(MAVLINK_SETPOINT_ATTITUDE_TOPIC, AttitudeTarget, queue_size=1)
    self.setpoint_position_local_pub = nepi_sdk.create_publisher(MAVLINK_SETPOINT_POSITION_LOCAL_TOPIC, PoseStamped, queue_size=1)
    # Teleop's own velocity setpoint, deliberately separate from the three
    # position/attitude/location targets above -- see sendTeleopVelocityLoop.
    # cmd_vel_unstamped (plain Twist, not TwistStamped) is MAVROS's
    # setpoint_velocity plugin's simpler of its two topics; this driver has no
    # use for the stamped variant's extra header.
    MAVLINK_SETPOINT_VELOCITY_TOPIC = MAVLINK_NAMESPACE + "setpoint_velocity/cmd_vel_unstamped"
    self.setpoint_velocity_pub = nepi_sdk.create_publisher(MAVLINK_SETPOINT_VELOCITY_TOPIC, Twist, queue_size=1)

    self.msg_if.pub_info("... Connected to Mavlink!")

    ##############################
    # Camera-rig feature: publish decoded frames on a bare-relative topic
    # name that RBXRobotIF's find_topic()-based image subscriber is pointed
    # at via set_image_topic (see below). ArduPilot SITL is single-instance
    # on this device (no second rbx_ardupilot node can ever coexist), so
    # there is no cross-talk risk from another instance of THIS driver --
    # confirmed rather than assumed, since the rover's camera-rover-multi
    # phase found a real bug here (two rbx_sim instances' bare
    # "color_2d_image" colliding on the shared device-wide namespace, per
    # nepi_drvs.launchDriverNode only remapping __name, never __ns). This
    # driver still qualifies its topic with its own device_name, matching
    # rbx_sim_node.py's fix, as defense against the same shared-namespace
    # colliding with a DIFFERENT RBX driver (e.g. rbx_sim) that might be
    # running on this same device and left at RBXRobotIF's bare
    # "color_2d_image" factory default.
    # Six always-live topics (color/depth-view/raw-depth-map x robot/scene),
    # not one bare topic -- see CAMERA_SETTING_NAMES's own comment for why,
    # and rbx_sim_node.py's matching comment for the full 2026-08-20 redesign
    # (superseding the old depth_map_enabled toggle with simultaneous
    # publishing on all six). A real onboard camera (REAL_CAMERA_TOPIC_PATTERN
    # below) only ever feeds robot_color -- a real airframe has no chase-cam
    # concept and no depth stream, so scene_color/robot_depth/scene_depth/
    # robot_depth_map/scene_depth_map simply stay idle (topics exist, nothing
    # ever publishes to them) on real hardware, an honest reflection of
    # reality rather than a fabricated feed.
    self.image_topic_name = self.device_name + "/color_2d_image"
    self.robot_color_topic_name = self.image_topic_name + "/robot_color"
    self.scene_color_topic_name = self.image_topic_name + "/scene_color"
    self.robot_depth_topic_name = self.image_topic_name + "/robot_depth"
    self.scene_depth_topic_name = self.image_topic_name + "/scene_depth"
    self.robot_depth_map_topic_name = self.image_topic_name + "/robot_depth_map"
    self.scene_depth_map_topic_name = self.image_topic_name + "/scene_depth_map"
    self.image_pub_robot_color = nepi_sdk.create_publisher(self.robot_color_topic_name, Image, queue_size = 1)
    self.image_pub_scene_color = nepi_sdk.create_publisher(self.scene_color_topic_name, Image, queue_size = 1)
    self.image_pub_robot_depth = nepi_sdk.create_publisher(self.robot_depth_topic_name, Image, queue_size = 1)
    self.image_pub_scene_depth = nepi_sdk.create_publisher(self.scene_depth_topic_name, Image, queue_size = 1)
    self.image_pub_robot_depth_map = nepi_sdk.create_publisher(self.robot_depth_map_topic_name, Image, queue_size = 1)
    self.image_pub_scene_depth_map = nepi_sdk.create_publisher(self.scene_depth_map_topic_name, Image, queue_size = 1)

    # Camera bridge client state and connection thread -- see
    # camera_rig_controller_ardupilot.py and CAMERA_BRIDGE_HOST/PORT above.
    # MAVLink (via mavros) carries telemetry/commands for this driver
    # already; this is a second, independent persistent connection carrying
    # ONLY camera settings out and compressed frames in.
    self.camera_sock = None
    self.camera_sock_lock = threading.Lock()
    # CAMERA_STALE_TIMEOUT_SEC's own comment above explains why this is
    # tracked and watched from a separate thread rather than trusted to
    # self-detect from inside cameraBridgeLoop.
    self.camera_last_frame_time = 0.0
    self.camera_bridge_thread = threading.Thread(target = self.cameraBridgeLoop)
    self.camera_bridge_thread.daemon = True
    self.camera_bridge_thread.start()
    self.camera_watchdog_thread = threading.Thread(target = self.cameraBridgeWatchdogLoop)
    self.camera_watchdog_thread.daemon = True
    self.camera_watchdog_thread.start()

    # Real-hardware camera relay -- see REAL_CAMERA_TOPIC_PATTERN above.
    # Independent of the sim bridge thread; auto-detects a real onboard
    # camera the moment one appears and needs no separate enable step.
    # SITL guard: see REAL_CAMERA_TOPIC_PATTERN's own comment for why this
    # must not run against a SITL instance's dev rig.
    self.real_camera_sub = None
    if not self.is_sitl:
      self.real_camera_watch_thread = threading.Thread(target = self.realCameraWatchLoop)
      self.real_camera_watch_thread.daemon = True
      self.real_camera_watch_thread.start()

    # Initialize RBX Settings
    self.cap_settings = self.getCapSettings()
    '''
    self.msg_if.pub_warn("CAPS SETTINGS")
    for setting_name in self.cap_settings.keys():
        setting = self.cap_settings[setting_name]
        self.msg_if.pub_warn(str(setting))
    '''
    self.factory_settings = self.getFactorySettings()
    '''
    self.msg_if.pub_warn("FACTORY SETTINGS")
    for setting_name in self.factory_settings.keys():
        setting = self.factory_settings[setting_name]
        self.msg_if.pub_warn(str(setting))
    '''

    # Per-motor commanded speed ratios (0-1), tracked locally since ArduPilot's
    # DO_MOTOR_TEST is fire-and-forget and reports no ongoing per-motor state.
    self.motor_ratios = [0.0] * int(self.settings_dict['motor_count']['value'])

    # Teleop (keyboard-driven) velocity state -- already rotated into ENU and
    # scaled to m/s and rad/s, so sendTeleopVelocityLoop can publish it
    # directly with no further conversion. See setTeleopVelocity/
    # sendTeleopVelocityLoop. Locked because setTeleopVelocity (a subscriber
    # callback) and sendTeleopVelocityLoop (a timer callback) run on different
    # threads.
    self.teleop_lock = threading.Lock()
    self.teleop_linear_enu = [0.0, 0.0, 0.0]
    self.teleop_angular_z = 0.0
    self.teleop_last_cmd_time = 0.0


    # Define fake gps namespace and create fake_gps publishers.
    # Created unconditionally so the goto/home/mode callbacks never hit a missing
    # publisher; harmless when no fake_gps app is subscribed.
    # Target the Fake GPS app (nepi_app_fake_gps) -- a single instance at the base
    # namespace ("<base>/app_fake_gps") that injects HilGPS into its selected mavros
    # node. (Replaces the old per-device "<base>/fake_gps_<port>" node convention.)
    FAKE_GPS_APP_NODE_NAME = "app_fake_gps"
    FAKE_GPS_NAMESPACE = os.path.join(self.base_namespace, FAKE_GPS_APP_NODE_NAME)
    self.msg_if.pub_info("Setting up fake_gps pubs at namespace: " + FAKE_GPS_NAMESPACE)
    self.fake_gps_enable_pub = nepi_sdk.create_publisher(FAKE_GPS_NAMESPACE + "/enable", Bool, queue_size=1)
    self.fake_gps_reset_pub = nepi_sdk.create_publisher(FAKE_GPS_NAMESPACE + "/reset", GeoPoint, queue_size=1)
    self.fake_gps_go_stop_pub = nepi_sdk.create_publisher(FAKE_GPS_NAMESPACE + "/go_stop", Empty, queue_size=1)
    self.fake_gps_goto_position_pub = nepi_sdk.create_publisher(FAKE_GPS_NAMESPACE + "/goto_position", Point, queue_size=1)
    self.fake_gps_goto_location_pub = nepi_sdk.create_publisher(FAKE_GPS_NAMESPACE + "/goto_location", GeoPoint, queue_size=1)
    self.fake_gps_select_pub = nepi_sdk.create_publisher(FAKE_GPS_NAMESPACE + "/select_mavros_node", String, queue_size=1)

    # Reconcile the Fake GPS app against what kind of vehicle this actually is.
    # Placed here, after the fake_gps publishers exist, rather than earlier next
    # to the mavros service setup: an earlier attempt built its own publisher
    # from self.base_namespace + "app_fake_gps/" and silently published into
    # "/nepi/device1app_fake_gps/enable" -- get_base_namespace() returns the
    # namespace with NO trailing slash, which is exactly why the block above
    # uses os.path.join. Reusing these publishers means one namespace
    # convention, proven by the existing goto/home callbacks, instead of two.
    self.reconcileFakeGpsApp()



    # Launch the NEPI RBX interface -- this takes care of initializing all the rbx
    # settings from config, subscribing/advertising topics and services, etc.
    self.msg_if.pub_info("Launching NEPI RBX interface...")
    self.device_info_dict["device_name"] = self.device_name
    self.device_info_dict["path"] = self.device_path
    self.device_info_dict["serial_number"] = ""
    self.device_info_dict["hw_version"] = ""
    self.device_info_dict["sw_version"] = ""
    self.msg_if.pub_info(str(self.device_info_dict))


    self.rbx_if = RBXRobotIF(device_info = self.device_info_dict,
                                  # RBXRobotIF's factory default is
                                  # 'control_system' -- sim_connector_app_node.py's
                                  # own device discovery (simDiscoveryCb) only
                                  # matches DeviceRBXStatus publishers reporting
                                  # 'simulator' (SIM_SOURCE_DESCRIPTION) here, the
                                  # same fix rbx_sim_node.py already carries.
                                  # Conditional on is_sitl (unlike the rover, which
                                  # is never anything but a simulator) since this
                                  # same driver also serves genuine ArduPilot
                                  # hardware in the field. Without this, a SITL
                                  # instance never appears in that app's
                                  # available_simulators, so selected_simulator can
                                  # never resolve to it -- silently defeating every
                                  # Sim Connector control surface (image viewers,
                                  # camera offsets, capability toggles, keybind
                                  # editor) that depends on that discovery.
                                  data_source_description = ('simulator' if self.is_sitl else 'control_system'),
                                  capSettings = self.cap_settings,
                                  factorySettings = self.factory_settings,
                                  settingUpdateFunction = self.settingUpdateFunction,
                                  getSettingsFunction=self.getSettings,
                                  axisControls = self.axis_controls,
                                  getBatteryPercentFunction = self.getBatteryPercent,
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
                                  getHomeFunction = self.getHomeLocation,
                                  setHomeFunction = self.setHomeLocation,
                                  goHomeFunction = self.goHome,
                                  goStopFunction = self.goStop,
                                  gotoPoseFunction = self.gotoPose,
                                  gotoPositionFunction = self.gotoPosition,
                                  gotoLocationFunction = self.gotoLocation,
                                  getNavPoseCb = self.getNavPoseCb,
                                  navpose_update_rate = self.POSITION_UPDATE_RATE,
                                  msg_if = self.msg_if
                                )


    self.msg_if.pub_info("... RBX interface running")
    time.sleep(1)

    ## Point the interface's image-source search at this instance's own
    ## device-name-qualified robot_color topic by default (see the
    ## image_pub_robot_color comment above) -- overrides RBXRobotIF's plain
    ## "color_2d_image" factory default/any stale persisted config every
    ## startup, matching rbx_sim_node.py's own deterministic-per-startup
    ## rationale. This is "the" camera on real hardware too, so the default
    ## is correct there as well as in SITL; the operator can still switch to
    ## any of the other five topics (SITL only) any time via the ordinary
    ## Image Source dropdown.
    self.rbx_if.setImageTopicCb(String(data = self.robot_color_topic_name))

    ## Start goto setpoint check/send loop
    setpoint_pub_interval = float(1) / self.SETPOINT_PUBLISH_RATE_HZ
    nepi_sdk.start_timer_process(setpoint_pub_interval, self.sendGotoCommandLoop)
    ## Start teleop velocity setpoint loop -- independent of the goto loop
    ## above, see sendTeleopVelocityLoop's own comment for why.
    teleop_pub_interval = float(1) / self.TELEOP_PUBLISH_RATE_HZ
    nepi_sdk.start_timer_process(teleop_pub_interval, self.sendTeleopVelocityLoop)
    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    #Set up node shutdown
    nepi_sdk.on_shutdown(self.cleanup_actions)
    # Spin forever (until object is detected)
    nepi_sdk.spin()


  #**********************
  # Setting functions
  def getCapSettings(self):
    return self.CAP_SETTINGS

  def getFactorySettings(self):
    settings = self.getSettings()
    #Apply factory setting overides
    for setting_name in settings.keys():
      if setting_name in self.FACTORY_SETTINGS_OVERRIDES:
            settings[setting_name]['value'] = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
    return settings


  def getSettings(self):  
    return self.settings_dict

  def settingUpdateFunction(self,setting):
    success = False
    setting_str = str(setting)
    setting_name = setting['name']
    if nepi_settings.check_valid_setting(setting,self.cap_settings):
      if setting_name in self.settings_dict.keys():
        self.settings_dict[setting_name]['value'] = setting['value']
        success = True
      else:
        msg = (self.node_name  + " Setting name" + setting_str + " is not supported") 
      if success == True:
        msg = ( self.node_name  + " UPDATED SETTINGS " + setting_str)
        if setting_name in self.CAMERA_SETTING_NAMES:
          self.sendCameraSettings()
    else:
      msg = (self.node_name  + " Setting data" + setting_str + " is not valid")
    return success, msg

  ##########################
  # RBX Interface Functions

  def getStateInd(self):
    return self.state_ind

  def setStateInd(self,state_ind):
    state_last = self.state_current
    set_state_function = globals()[self.RBX_STATE_FUNCTIONS[state_ind]]
    success = set_state_function(self)
    if success:
      self.state_ind = state_ind
      self.state_current = self.RBX_STATES[state_ind]
      self.state_last = state_last
    return success

  def getModeInd(self):
    return self.mode_ind

  def setModeInd(self,mode_ind):
    mode_on_entry = self.mode_current
    set_mode_function = globals()[self.RBX_MODE_FUNCTIONS[mode_ind]]
    success = set_mode_function(self)
    if success:
      if self.RBX_MODES[mode_ind] == "RESUME":
        if self.mode_last != "RESUME":
          self.mode_current = self.mode_last
          self.mode_ind = self.RBX_MODES.index(self.mode_last)
          self.mode_last = mode_on_entry # Don't update last on resume
      else:
        if (mode_ind >= 0 and mode_ind <= (len(self.RBX_MODES)-1)):
          self.mode_ind = mode_ind
          self.mode_current = self.RBX_MODES[mode_ind]
          self.mode_last = mode_on_entry # Don't update last on resume
      #if self.mode_current in self.takeoff_reset_modes:
        #self.takeoff_complete = False
    return success
    
  def checkStopFunction(self):
    triggered = self.stop_triggered
    self.stop_triggered = False # Reset Stop Trigger
    return triggered

  def getBatteryPercent(self):
    return self.battery_percent

  def setHomeLocation(self,geo_point):
    self.set_home_location(geo_point)
    if self.has_fake_gps:
      self.fake_gps_reset_pub.publish(geo_point)

  def getHomeLocation(self):
    return self.home_location

  def setFakeGPSFunction(self,fake_gps_enabled):
    self.fake_gps_enable_pub.publish(data = fake_gps_enabled)


  def setMotorControlRatio(self,motor_ind,speed_ratio):
    if motor_ind < 0 or motor_ind >= len(self.motor_ratios):
      self.msg_if.pub_warn("Motor test ignored: motor index " + str(motor_ind + 1) + " out of range")
      return
    speed_ratio = max(0.0, min(1.0, speed_ratio))
    # Scale the 0-100% slider ratio onto [0, motor_test_max_throttle_percent]
    # rather than straight onto [0,100] -- e.g. a 20% max throttle setting
    # means the slider's 100% only ever commands 20% actual throttle, so the
    # cap integrates with the slider UI instead of silently overriding it.
    max_throttle_percent = float(self.settings_dict['motor_test_max_throttle_percent']['value'])
    throttle_percent = speed_ratio * max_throttle_percent
    timeout_s = float(self.settings_dict['motor_test_timeout_s']['value'])
    test_cmd = CommandLongRequest()
    test_cmd.broadcast = False
    test_cmd.command = self.MAV_CMD_DO_MOTOR_TEST
    test_cmd.confirmation = 0
    test_cmd.param1 = float(motor_ind + 1)  # ArduPilot motor test number is 1-based
    test_cmd.param2 = 0.0                    # MOTOR_TEST_THROTTLE_PERCENT
    test_cmd.param3 = throttle_percent if speed_ratio > 0.0 else 0.0
    test_cmd.param4 = timeout_s if speed_ratio > 0.0 else 0.0
    test_cmd.param5 = 1.0  # motor count: test only this one motor
    test_cmd.param6 = 0.0  # test order: default
    test_cmd.param7 = 0.0
    response = nepi_sdk.call_service(self.command_client, test_cmd)
    if response is not None and response.success:
      self.motor_ratios[motor_ind] = speed_ratio
    else:
      fail_msg = "Motor " + str(motor_ind + 1) + " test command rejected"
      reason = self.get_recent_fcu_reason()
      if reason != "":
        fail_msg = fail_msg + " (FCU: " + reason + ")"
      self.msg_if.pub_warn(fail_msg)
      if self.rbx_if is not None:
        self.rbx_if.update_error_msg(fail_msg)

  def getMotorControlRatios(self):
    return self.motor_ratios

  def setSetupActionInd(self,action_ind):
    set_action_function = globals()[self.RBX_SETUP_ACTION_FUNCTIONS[action_ind]]
    success = set_action_function(self)
    return success

  def setGoActionInd(self,action_ind):
    set_action_function = globals()[self.RBX_GO_ACTION_FUNCTIONS[action_ind]]
    success = set_action_function(self)
    return success

  def goStop(self):
    self.stop_triggered = True
    self.fake_gps_go_stop_pub.publish(Empty())
    return True

  def goHome(self):
    self.stop_triggered = True
    nepi_sdk.sleep(1,10)
    self.stop_triggered = False
    nepi_sdk.sleep(3,30)
    home_loc = self.home_location
    setpoint_location = [home_loc.latitude,home_loc.longitude,home_loc.altitude,-999]
    self.rbx_if.setpoint_location_global_wgs84(setpoint_location)
    self.fake_gps_goto_location_pub.publish(home_loc)
    return True


  def sendGotoCommandLoop(self,timer):
    if self.rbx_if.status_msg.ready == False:
      if self.attitude_target != None:
        # += self.att_sp_seq (previously) starting from 0 (line 317) is
        # always 0 -- header.seq never actually incremented. Harmless to
        # MAVROS/ArduCopter (they don't gate on it), but not what a
        # monotonic sequence counter is for; += 1 is the fix.
        self.att_sp_seq += 1
        self.attitude_target.header.stamp = nepi_sdk.get_msg_stamp()
        self.attitude_target.header.seq = self.att_sp_seq
        self.setpoint_attitude_pub.publish(self.attitude_target) # Publish Setpoint
      elif self.position_target != None:
        self.msg_if.pub_info("got position target valid")
        self.pos_sp_seq += 1
        self.position_target.header.stamp = nepi_sdk.get_msg_stamp()
        self.position_target.header.seq = self.pos_sp_seq
        self.setpoint_position_local_pub.publish(self.position_target) # Publish Setpoint
      elif self.location_target != None:
        self.loc_sp_seq += 1
        self.location_target.header.stamp = nepi_sdk.get_msg_stamp()
        self.location_target.header.seq = self.loc_sp_seq
        self.setpoint_location_global_pub.publish(self.location_target) # Publish Setpoint
    else:
      time.sleep(0.2)
      self.attitude_target = None
      self.position_target = None
      self.location_target = None

  def gotoPose(self,attitude_enu_degs):
    att_str = str(attitude_enu_degs)
    self.msg_if.pub_info("Recieved Pose setpoint command: " + att_str)
    # Create Setpoint Attitude Message
    attitude_enu_quat = nepi_nav.convert_rpy2quat(attitude_enu_degs)
    orientation_enu_quat = Quaternion()
    orientation_enu_quat.x = attitude_enu_quat[0]
    orientation_enu_quat.y = attitude_enu_quat[1]
    orientation_enu_quat.z = attitude_enu_quat[2]
    orientation_enu_quat.w = attitude_enu_quat[3]
    # Set other setpoint attitude message values
    body_rate = Vector3()
    body_rate.x = 0
    body_rate.y = 0
    body_rate.z = 0
    # MAVLink SET_ATTITUDE_TARGET type_mask bits 1/2/4 ignore body roll/
    # pitch/yaw RATE (Copter derives the rates itself from the orientation
    # target below, which is what every use of this command actually
    # wants -- an attitude to reach, not a rate to hold).
    #
    # Bit 64 (ignore throttle) is deliberately NOT set here, even though it
    # looks like the obvious fix for the bug below -- confirmed against
    # ArduCopter's own current source
    # (ArduCopter/GCS_MAVLink_Copter.cpp, handle_message_set_attitude_target):
    #   if (throttle_ignore) {
    #     // The throttle input is not defined
    #     copter.mode_guided.hold_position();
    #     return;
    #   }
    # Setting it makes Copter discard the ENTIRE message -- orientation
    # included -- and hold its current position instead. Confirmed live:
    # with bit 64 set, a 180-degree goto_pose held altitude perfectly but
    # never actually rotated; the command was a total no-op, not a safe
    # version of itself.
    type_mask = 1|2|4
    attitude_target_msg = AttitudeTarget()
    attitude_target_msg.orientation = orientation_enu_quat
    attitude_target_msg.body_rate = body_rate
    attitude_target_msg.type_mask = type_mask
    # thrust=0 (previously) is NOT "zero motor thrust" -- with bit 64 above
    # unset, Copter treats thrust as a CLIMB RATE command (same source as
    # above, the use_thrust=false branch): 0.5 means "hold altitude" (zero
    # climb rate, its own Z controller picks whatever throttle that takes),
    # 0.0 means "descend at the full WPNAV_SPEED_DN rate" for as long as
    # this setpoint keeps streaming, and 1.0 means "climb at WPNAV_SPEED_UP".
    # thrust=0 therefore commanded a continuous forced descent on every
    # single goto_pose call, including a pure yaw change with roll/pitch
    # left at 0 -- a small attitude change could look survivable if the
    # setpoint stream stopped (target reached) before the descent became
    # visible; a large one (confirmed live: 180 degrees of yaw, which takes
    # longer to converge) kept streaming that forced descent long enough to
    # reach the ground. 0.5 is the fix: zero commanded climb rate, altitude
    # held by Copter's own controller, while the attitude controller drives
    # toward the requested orientation independently. This is standard
    # ArduCopter GUIDED-mode behavior (identical on SITL and real hardware),
    # not a simulator-specific workaround.
    attitude_target_msg.thrust = 0.5
    ## Send Setpoint Message
    self.attitude_target = attitude_target_msg
    

  def gotoPosition(self,point_enu_m,orientation_enu_deg):
    pos_str = str(point_enu_m)
    self.msg_if.pub_info("Recieved Position setpoint command: " + pos_str)
    # RBXRobotIF (setpoint_position_local_body) only ever hands drivers an
    # ENU OFFSET -- the requested body-frame point rotated by current yaw,
    # NOT added to current position; its own docstring says "Commands the
    # robot to a target position in the LOCAL BODY FRAME" and its comment
    # explicitly notes each driver adds its own current position (see
    # rbx_sim_node.py's identical gotoPosition, "RBXRobotIF passes the goal
    # as an ENU offset point from the current position"). This driver
    # skipped that addition and published point_enu_m straight to mavros's
    # setpoint_position/local, which IS an absolute local-ENU target
    # relative to the EKF origin (home) -- so "move forward 2m" silently
    # became "fly to home+2m, ignoring wherever you currently are",
    # reported live as goto_position only ever working correctly from a
    # fresh spawn at the origin, and confirmed by a live test: commanding
    # +2m from x=8.4m sent the vehicle toward roughly x=2m (home-relative)
    # instead of x=10.4m (current-position-relative). Adding self.navpose_dict
    # here makes this match rbx_sim_node.py's rover exactly, giving the
    # quadcopter the same "move N units from wherever it is right now"
    # capability the rover already had.
    point_enu_m.x = point_enu_m.x + self.navpose_dict['x_m']
    point_enu_m.y = point_enu_m.y + self.navpose_dict['y_m']
    point_enu_m.z = point_enu_m.z + self.navpose_dict['z_m']
    # Create PoseStamped Setpoint Local ENU Message
    orientation_enu_q = nepi_nav.convert_rpy2quat(orientation_enu_deg)
    orientation_enu_quat = Quaternion()
    orientation_enu_quat.x = orientation_enu_q[0]
    orientation_enu_quat.y = orientation_enu_q[1]
    orientation_enu_quat.z = orientation_enu_q[2]
    orientation_enu_quat.w = orientation_enu_q[3]
    pose_enu=Pose()
    pose_enu.position = point_enu_m
    pose_enu.orientation = orientation_enu_quat
    position_local_target_msg = PoseStamped()
    position_local_target_msg.pose = pose_enu
    ## Send Message and Check for Setpoint Success
    self.position_target = position_local_target_msg
    self.fake_gps_goto_position_pub.publish(point_enu_m)

  def gotoLocation(self,geopoint_amsl,orientation_ned_deg):
    loc_str = str(geopoint_amsl)
    self.msg_if.pub_info("Recieved Location setpoint command: " + loc_str)
    # Create GeoPose Setpoint Global AMSL and Yaw NED Message
    orientation_ned_q = nepi_nav.convert_rpy2quat(orientation_ned_deg)
    orientation_ned_quat = Quaternion()
    orientation_ned_quat.x = orientation_ned_q[0]
    orientation_ned_quat.y = orientation_ned_q[1]
    orientation_ned_quat.z = orientation_ned_q[2]
    orientation_ned_quat.w = orientation_ned_q[3]
    geopose_enu=GeoPose()
    geopose_enu.position = geopoint_amsl
    geopose_enu.orientation = orientation_ned_quat
    location_global_target_msg = GeoPoseStamped()
    location_global_target_msg.pose = geopose_enu
    ##############################################
    ## Send Message and Check for Setpoint Success
    ##############################################
    self.location_target = location_global_target_msg
    geopoint_wsg84 = nepi_nav.convert_amsl_to_wgs84(geopoint_amsl)
    self.fake_gps_goto_location_pub.publish(geopoint_wsg84)

  ##########################
  # Control Ready Check Funcitons

  def manualControlsReady(self):
    # Always ready. ArduPilot's own DO_MOTOR_TEST handler arms the FC itself
    # as a side effect of running a motor test (see ArduCopter/motor_test.cpp
    # mavlink_motor_test_start()), which flips the mavros "armed" state we'd
    # otherwise gate on here. Gating manual/motor controls on DISARM created a
    # self-inflicted deadlock: testing one motor reported the vehicle as ARMed
    # until that test's timeout elapsed, locking out every other motor
    # command (including Turn Off) for the whole test duration. ArduPilot's
    # own mavlink_motor_control_check() (board initialized, motor_test_checks,
    # landed) is the real safety gate for motor tests -- no client-side arm
    # check is needed on top of it.
    return True

  def isAirborne(self):
    # takeoff_complete is set once, by takeoff_action()'s own bounded wait
    # loop -- if that loop times out while the vehicle is still genuinely
    # climbing (confirmed live 2026-09-01: a loaded VM can leave the vehicle
    # several meters short at the timeout, then keep climbing under its own
    # MAVLink TAKEOFF command and reach goal_alt a couple minutes later),
    # takeoff_complete stays False for the rest of the flight -- exactly the
    # "false timeout PERMANENTLY disabled every goto command" failure this
    # class of check already exists to avoid, just via a path the 2026-08-12
    # fix didn't cover (that one only handled converging AT the timeout
    # boundary, not well after it). Rather than widen the wait loop's own
    # timeout further (already raised once, still not always enough under
    # load), recheck the actual current altitude against the recorded goal
    # here on every readiness query -- cheap, and self-correcting the moment
    # the vehicle actually gets there, instead of trusting a flag that can
    # never become true again once it's False.
    if self.takeoff_complete:
      return True
    if self.takeoff_goal_alt is None or self.takeoff_error_bound_m is None:
      return False
    if self.rbx_if is None:
      return False
    alt_error = self.takeoff_goal_alt - self.rbx_if.current_location_wgs84_geo[2]
    if abs(alt_error) <= self.takeoff_error_bound_m:
      self.takeoff_complete = True
      return True
    return False

  def autonomousControlsReady(self):
    # Also requires autonomous_movement_enabled -- the Sim Connector's own
    # per-robot-config "automated movement" toggle, same as
    # rbx_sim_node.py's identical check. Checked HERE, not just in the RUI
    # (which hides the controls entirely), so disabling it actually blocks
    # the command for any client, not merely the RUI's own buttons.
    if self.settings_dict['autonomous_movement_enabled']['value'] != 'TRUE':
      return False
    ready = False
    if self.RBX_STATES[self.state_ind] == "ARM" and self.RBX_MODES[self.mode_ind] == "GUIDED" and self.isAirborne():
      ready = True
    return ready

  def teleopControlsReady(self):
    # Same ARM+GUIDED+airborne requirement as autonomousControlsReady --
    # sending body-frame velocity setpoints only means anything once the
    # vehicle is actually flying under GUIDED control; on the ground or in a
    # different mode ArduPilot ignores them anyway, but reporting "ready"
    # in that state would be misleading. Plus teleop_movement_enabled, the
    # Sim Connector's own toggle for this feature.
    if self.settings_dict['teleop_movement_enabled']['value'] != 'TRUE':
      return False
    return (self.RBX_STATES[self.state_ind] == "ARM"
            and self.RBX_MODES[self.mode_ind] == "GUIDED"
            and self.isAirborne())

  def setTeleopVelocity(self, linear_x, linear_y, linear_z, angular_z):
    # Inputs are body-frame ratios in [-1,1] (forward/right/up, yaw-rate) --
    # see device_if_rbx.py's setTeleopVelocityCb. MAVROS's velocity setpoint
    # topic (sendTeleopVelocityLoop below) is LOCAL ENU, a fixed world frame,
    # not body frame -- "forward" there means a fixed compass direction, not
    # "wherever the nose is pointing," which is not what a keyboard teleop
    # control is for. Rotating by the vehicle's own current yaw is what makes
    # "W" mean forward relative to the drone regardless of which way it is
    # facing, the same way a real quadcopter's stick inputs work.
    #
    # Scaled by max_linear_speed_mps-equivalent -- this driver has no such
    # Setting today (unlike rbx_sim_node.py), so a fixed, conservative cap is
    # used instead. TELEOP_MAX_LINEAR_MPS/TELEOP_MAX_ANGULAR_DPS below.
    max_lin = self.TELEOP_MAX_LINEAR_MPS
    max_ang = math.radians(self.TELEOP_MAX_ANGULAR_DPS)
    body_x = max(-1.0, min(1.0, linear_x)) * max_lin
    body_y = max(-1.0, min(1.0, linear_y)) * max_lin
    yaw_rad = math.radians(self.navpose_dict['yaw_deg'])
    enu_x = body_x * math.cos(yaw_rad) - body_y * math.sin(yaw_rad)
    enu_y = body_x * math.sin(yaw_rad) + body_y * math.cos(yaw_rad)
    with self.teleop_lock:
      self.teleop_linear_enu = [enu_x, enu_y, max(-1.0, min(1.0, linear_z)) * max_lin]
      self.teleop_angular_z = max(-1.0, min(1.0, angular_z)) * max_ang
      self.teleop_last_cmd_time = nepi_utils.get_time()

  def sendTeleopVelocityLoop(self, timer):
    # Independent of sendGotoCommandLoop -- that loop drives ONE-SHOT position/
    # attitude/location targets to convergence and stops (clears its targets)
    # once status_msg.ready flips back True, which is the wrong lifecycle for
    # a continuous joystick-style input that has no "reached" condition.
    #
    # Publishes ONLY while a teleop command has arrived within
    # TELEOP_CMD_TIMEOUT_SEC -- including the explicit (0,0,0,0) the RUI sends
    # on keyup, so a brief grace window still re-asserts that stop against a
    # dropped packet (the same "never trust a single packet" reasoning
    # rbx_sim_node.py's own TELEOP_CMD_TIMEOUT_SEC comment gives). Deliberately
    # stops publishing ENTIRELY once that window elapses, rather than settling
    # into an indefinite zero-velocity stream: unlike the rover (one Twist
    # channel total), this vehicle's velocity setpoint
    # (setpoint_velocity/cmd_vel_unstamped) and its position setpoint
    # (setpoint_position/local, sendGotoCommandLoop) are two INDEPENDENT
    # MAVROS channels -- a teleop session that ended minutes ago must not go
    # on contesting a goto command that starts later.
    with self.teleop_lock:
      time_since_cmd = nepi_utils.get_time() - self.teleop_last_cmd_time
      enu = list(self.teleop_linear_enu)
      ang_z = self.teleop_angular_z
    if time_since_cmd >= self.TELEOP_CMD_TIMEOUT_SEC + self.TELEOP_STOP_GRACE_SEC:
      # Truly done -- see TELEOP_STOP_GRACE_SEC's own comment for why this
      # isn't just "time_since_cmd >= TELEOP_CMD_TIMEOUT_SEC".
      return
    if time_since_cmd >= self.TELEOP_CMD_TIMEOUT_SEC:
      # Grace window: force an explicit stop rather than trusting whatever
      # was last commanded (which may itself have been a dropped stop).
      enu = [0.0, 0.0, 0.0]
      ang_z = 0.0
    twist = Twist()
    twist.linear.x = enu[0]
    twist.linear.y = enu[1]
    twist.linear.z = enu[2]
    twist.angular.z = ang_z
    self.setpoint_velocity_pub.publish(twist)

  ##############################
  # RBX NavPose Topic Publishers
  ### Callback to publish RBX navpose data
  


  def gps_topic_callback(self,navsatfix_msg):
      if navsatfix_msg.latitude != 0:
        self.gps_connected = True
      # geoid_height_m used to be read back from self.rbx_if.current_geoid_height_m,
      # which is itself only ever set from navpose_dict['geoid_height_meters']
      # below -- a self-referential loop with no real geoid-separation source
      # feeding it, permanently pinned at its initial value of 0 (confirmed
      # live 2026-08-27). No real geoid data is available from this SITL's
      # NavSatFix, so this is left an explicit, honest 0 instead of a fake
      # correction that only looked like it was doing something.
      geoid_height_m = 0
      altitude_wgs84 = navsatfix_msg.altitude - geoid_height_m
      time_ns = nepi_sdk.sec_from_msg_stamp(navsatfix_msg.header.stamp)
      # Location Lat,Long
      self.navpose_dict['has_location'] = True
      self.navpose_dict['time_location'] = time_ns
      self.navpose_dict['latitude'] = navsatfix_msg.latitude
      self.navpose_dict['longitude'] = navsatfix_msg.longitude
      self.navpose_dict['geoid_height_meters'] = geoid_height_m
      # Altitude positive meters WGS84
      self.navpose_dict['has_altitude'] = True
      self.navpose_dict['time_altitude'] = time_ns
      self.navpose_dict['altitude_m'] = altitude_wgs84


  ### Callback to update RBX odom (orientation + position) navpose data
  def odom_topic_callback(self,odom_msg):
      or_msg = odom_msg.pose.pose.orientation
      or_list = [or_msg.x, or_msg.y, or_msg.z, or_msg.w]
      pos_msg = odom_msg.pose.pose.position
      pos_list = [pos_msg.x, pos_msg.y, pos_msg.z]
      rpy = nepi_nav.convert_quat2rpy(or_list)
      # nav_msgs/Odometry's pose is always in a world-fixed frame (per
      # header.frame_id), never body-relative -- mavros's global_position/local
      # already reports an absolute ENU position here. Rotating it through
      # convert_point_body2enu() (meant for genuinely body-relative offsets)
      # double-applied the vehicle's own yaw on top of an already-absolute
      # position, so any x/y offset from the origin got rotated by the
      # vehicle's heading -- confirmed live 2026-08-27 as the root cause of
      # horizontal drift/"flew to some random place" during follow-object
      # missions (a goto to a real target coordinate got silently re-rotated
      # into the wrong direction as soon as the vehicle's yaw was nonzero).
      xyz = pos_list
      time_ns = nepi_sdk.sec_from_msg_stamp(odom_msg.header.stamp)

      # Orientation Degrees in selected 3d frame (roll,pitch,yaw)
      self.navpose_dict['has_orientation'] = True
      self.navpose_dict['time_orientation'] = time_ns
      self.navpose_dict['roll_deg'] = rpy[0]
      self.navpose_dict['pitch_deg'] = rpy[1]
      self.navpose_dict['yaw_deg'] = rpy[2]

      # Relative Position Meters in selected 3d frame (x,y,z) with x forward, y right/left, and z up/down
      self.navpose_dict['has_position'] = True
      self.navpose_dict['time_position'] = time_ns
      self.navpose_dict['x_m'] = xyz[0]
      self.navpose_dict['y_m'] = xyz[1]
      self.navpose_dict['z_m'] = xyz[2]


  ### Callback to update RBX heading navpose data
  def heading_topic_callback(self,heading_msg):
      # Heading in Degrees True North
      self.navpose_dict['has_heading'] = True
      self.navpose_dict['time_heading'] = nepi_utils.get_time()
      self.navpose_dict['heading_deg'] = heading_msg.data


  ### Callback for flight-controller status text (pre-arm/arm rejections, EKF, failsafes).
  ### The FCU reports why a command is refused (e.g. "Arm: Compass not healthy") only as
  ### STATUSTEXT, which otherwise lands in the mavros log. Surface warning-or-worse text to
  ### the RBX status so it shows in the UI, and record it for command handlers to report.
  def get_statustext_callback(self, statustext_msg):
      text = statustext_msg.text.strip()
      if text == "":
          return
      severity = statustext_msg.severity
      now = nepi_utils.get_time()
      # Record the latest so command handlers (e.g. arm) can report the FCU reason
      self.last_fcu_text = text
      self.last_fcu_severity = severity
      self.last_fcu_text_time = now
      # StatusText.WARNING == 4; lower severity value == more severe (MAV_SEVERITY).
      # Skip NOTICE/INFO/DEBUG to avoid log noise. Dedupe identical consecutive
      # messages, but re-remind every FCU_TEXT_REMIND_S while a condition persists.
      if severity <= StatusText.WARNING:
          is_new = (text != self._last_surfaced_fcu_text)
          is_stale = (now - self._last_surfaced_fcu_time) > self.FCU_TEXT_REMIND_S
          if is_new or is_stale:
              self._last_surfaced_fcu_text = text
              self._last_surfaced_fcu_time = now
              fcu_msg = "FCU: " + text
              self.msg_if.pub_warn(fcu_msg)
              if self.rbx_if is not None:
                  self.rbx_if.update_error_msg(fcu_msg)

  ### Returns the most recent FCU status text when it is a recent warning-or-worse
  ### message, used to annotate a failed command with the flight-controller's reason.
  def get_recent_fcu_reason(self):
      if self.last_fcu_text == "":
          return ""
      age = nepi_utils.get_time() - self.last_fcu_text_time
      recent = age <= self.FCU_TEXT_RECENT_S
      severe = (self.last_fcu_severity is None or self.last_fcu_severity <= StatusText.WARNING)
      if recent and severe:
          return self.last_fcu_text
      return ""


  ### Callback returning the full navpose dict to the RBX/NPX interface
  def getNavPoseCb(self):
    return self.navpose_dict

  #######################
  # Mavlink Interface Methods

  ### Callback to get current state message
  def get_state_callback(self,mavlink_state_msg):
    self.mavlink_state = mavlink_state_msg
    # Update state value
    arm_val = mavlink_state_msg.armed
    if arm_val == True:
      self.state_ind=1
    else:
      self.state_ind=0
    self.state_current = self.RBX_STATES[self.state_ind]
    # Update mode value
    mode_val = mavlink_state_msg.mode
    mode_ind=-999
    for ind, mode in enumerate(self.RBX_MODES):
      if mode == mode_val:
        mode_ind=ind
    self.mode_ind=mode_ind 
    if mode_ind >= 0 and mode_ind < len(self.RBX_MODES):
      self.mode_current = self.RBX_MODES[self.mode_ind]
    else:
      self.mode_current = "Undefined"


  ### Function to set mavlink armed state
  def set_mavlink_arm_state(self,arm_value):
    last_arm_value = self.mavlink_state.armed
    arm_cmd = CommandBoolRequest()
    arm_cmd.value = arm_value
    if arm_value == True and self.gps_connected == False:
      no_gps_msg = "Arm command ignored: no GPS connected"
      self.msg_if.pub_warn(no_gps_msg)
      if self.rbx_if is not None:
        self.rbx_if.update_error_msg(no_gps_msg)
    else:
      self.msg_if.pub_info("Updating State to: " + str(arm_value))
      # See set_mavlink_mode's identical removal for why this unconditional
      # 1s sleep is gone -- same unjustified per-command latency.
      self.msg_if.pub_info("Waiting for armed value to set to " + str(arm_value))
      timeout_sec = self.rbx_if.rbx_info.cmd_timeout
      check_interval_s = 0.25
      check_timer = 0
      # Same response-checking as set_mavlink_mode -- see its comment. mavros'
      # CommandBool replies success=False when the FCU rejects arming (failed
      # pre-arm checks being the usual reason), which is the single most useful
      # thing to tell an operator and was previously thrown away.
      no_response_count = 0
      refused_count = 0
      while self.mavlink_state.armed != arm_value and check_timer < timeout_sec and not nepi_sdk.is_shutdown():
        response = nepi_sdk.call_service(self.arming_client, arm_cmd)
        if response is None:
          no_response_count += 1
          if no_response_count == 1:
            self.msg_if.pub_warn("arming service call to " + str(self.arming_client_name) +
                                 " returned nothing -- the call itself is failing, not the FCU")
        elif getattr(response, 'success', True) == False:
          refused_count += 1
          if refused_count == 1:
            self.msg_if.pub_warn("FCU refused arming (success=False) -- usually a failed pre-arm check")
        time.sleep(check_interval_s)
        check_timer += check_interval_s
        #self.msg_if.pub_info("Waiting for armed value to set")
        #self.msg_if.pub_info("Set Value: " + str(arm_value))
        #self.msg_if.pub_info("Cur Value: " + str(self.mavlink_state.armed))
      if self.mavlink_state.armed == arm_value:
        # Reset Home Location on Arming
        if arm_value == True and arm_value != last_arm_value:
          home_loc = GeoPoint()
          home_loc.latitude = self.rbx_if.current_location_wgs84_geo[0]
          home_loc.longitude = self.rbx_if.current_location_wgs84_geo[1]
          home_loc.altitude = self.rbx_if.current_location_wgs84_geo[2]
          self.home_location = home_loc
      else:
        action = "Arm" if arm_value == True else "Disarm"
        fail_msg = action + " command timed out after " + str(timeout_sec) + "s"
        if no_response_count > 0:
          fail_msg = fail_msg + "; " + str(no_response_count) + " arming calls returned nothing"
        if refused_count > 0:
          fail_msg = fail_msg + "; " + str(refused_count) + " refused by the FCU (pre-arm check?)"
        reason = self.get_recent_fcu_reason()
        if reason != "":
          fail_msg = fail_msg + " (FCU: " + reason + ")"
        self.msg_if.pub_warn(fail_msg)
        if self.rbx_if is not None:
          self.rbx_if.update_error_msg(fail_msg)
      self.msg_if.pub_info("Armed value set to " + str(arm_value))
    return self.mavlink_state.armed == arm_value


  ### Function to set mavlink mode
  def set_mavlink_mode(self,mode_new):
    new_mode = SetModeRequest()
    new_mode.custom_mode = mode_new
    self.msg_if.pub_info("Updating mode")
    self.msg_if.pub_info(mode_new)
    # Was `time.sleep(1)` here ("give time for other process to see busy") --
    # removed. No process actually waits on this: RBXRobotIF's own busy/
    # process_current state is set before this function is ever called, not
    # inside it, and the poll loop below already only proceeds once the FCU
    # actually reports the new mode. A flat 1s of dead time before every
    # single mode change (and the identical one in set_mavlink_arm_state) is
    # exactly the reported "sending commands is really slow" -- a LAUNCH
    # setup action alone chains a mode change and an arm, so this was 2+
    # full seconds of unconditional sleep before anything even started, on
    # every command, every time.
    self.msg_if.pub_info("Waiting for mode to set to " + mode_new)
    timeout_sec = self.rbx_if.rbx_info.cmd_timeout
    check_interval_s = 0.25
    check_timer = 0
    # The service RESPONSE is checked now rather than discarded. mavros'
    # SetMode replies mode_sent=False when it refuses to forward the request
    # at all, and nepi_sdk.call_service returns None when the call itself
    # failed (it catches every exception and logs only at DEBUG, which ROS
    # suppresses by default). Ignoring both meant a mode change that was
    # never even sent looked exactly like one the FCU was still working on:
    # this loop just spun for the full cmd_timeout and the operator saw a
    # LAUNCH that sat there and then quietly gave up. Reported repeatedly as
    # "the launch command doesn't work" with nothing in last_error_message.
    #
    # Logged once per distinct outcome rather than every 0.25 s tick, so a
    # genuinely-slow-but-working mode change doesn't spam the message topic.
    no_response_count = 0
    refused_count = 0
    while self.mavlink_state.mode != mode_new and check_timer < timeout_sec and not nepi_sdk.is_shutdown():
      response = nepi_sdk.call_service(self.mode_client, new_mode)
      if response is None:
        no_response_count += 1
        if no_response_count == 1:
          self.msg_if.pub_warn("set_mode service call to " + str(self.mode_client_name) +
                               " returned nothing -- the service call itself is failing, "
                               "not the FCU (check the service exists and the request type matches)")
      elif getattr(response, 'mode_sent', True) == False:
        refused_count += 1
        if refused_count == 1:
          self.msg_if.pub_warn("mavros refused to send mode " + mode_new +
                               " (mode_sent=False) -- it is rejecting the request before the FCU sees it")
      time.sleep(check_interval_s)
      check_timer += check_interval_s
    if self.mavlink_state.mode == mode_new:
      self.msg_if.pub_info("Mode set to " + mode_new)
    else:
      # Include what the vehicle's mode ACTUALLY is, plus whether the calls
      # were even getting through -- "timed out" alone gave no way to tell a
      # rejected request from an unreachable service from an FCU that simply
      # would not change mode.
      fail_msg = ("Setting mode to " + mode_new + " timed out after " + str(timeout_sec) +
                  "s (vehicle still reports " + str(self.mavlink_state.mode) + ")")
      if no_response_count > 0:
        fail_msg = fail_msg + "; " + str(no_response_count) + " set_mode calls returned nothing"
      if refused_count > 0:
        fail_msg = fail_msg + "; " + str(refused_count) + " refused by mavros (mode_sent=False)"
      reason = self.get_recent_fcu_reason()
      if reason != "":
        fail_msg = fail_msg + " (FCU: " + reason + ")"
      self.msg_if.pub_warn(fail_msg)
      if self.rbx_if is not None:
        self.rbx_if.update_error_msg(fail_msg)
    return self.mavlink_state.mode == mode_new



  ### Callback to get current mavlink battery message
  def get_mavlink_battery_callback(self,battery_msg):
    self.battery_percent = battery_msg.percentage
 

  #######################
  # Mavlink Ardupilot Interface Methods

  ### Function for switching to arm state
  global arm
  def arm(self):
    return self.set_mavlink_arm_state(True)

  ### Function for switching to disarm state
  global disarm
  def disarm(self):
    return self.set_mavlink_arm_state(False)

  ## Action Function for setting arm state and sending takeoff command
  global launch
  def launch(self):
    # LAUNCH is guided-mode -> arm -> takeoff, and it aborts at the first step
    # that fails. Each abort now says WHICH step and reports it to the RUI.
    # Previously a failure anywhere here returned a bare False: the RUI showed
    # the action finish with an empty last_error_message and the vehicle simply
    # never moved, which is unactionable ("the launch command doesn't work").
    # The step functions surface their own reasons too (see set_mavlink_mode /
    # set_mavlink_arm_state); this adds which stage of LAUNCH was reached.
    self.msg_if.pub_info("Recieved Launch cmd")
    cmd_success = False

    if "guided" not in self.RBX_MODE_FUNCTIONS:
      self.reportLaunchFailure("this driver has no GUIDED mode function registered")
      return False
    if "arm" not in self.RBX_STATE_FUNCTIONS:
      self.reportLaunchFailure("this driver has no ARM state function registered")
      return False

    cmd_success = self.setModeInd(self.RBX_MODE_FUNCTIONS.index("guided"))
    if not cmd_success:
      self.reportLaunchFailure("could not switch to GUIDED mode -- not arming or taking off")
      return False

    cmd_success = self.setStateInd(self.RBX_STATE_FUNCTIONS.index("arm"))
    if not cmd_success:
      self.reportLaunchFailure("GUIDED mode was set but the vehicle would not ARM -- not taking off"
                               + self.describeArmRefusalCause())
      return False

    nepi_sdk.sleep(2,20)
    cmd_success = self.takeoff_action()
    if not cmd_success:
      self.reportLaunchFailure("armed in GUIDED mode but the takeoff did not complete")
    return cmd_success

  def reportLaunchFailure(self, why):
    msg = "LAUNCH failed: " + why
    self.msg_if.pub_warn(msg)
    if self.rbx_if is not None:
      self.rbx_if.update_error_msg(msg)

  def waitForPubConnection(self, pub, timeout_sec = 10.0):
    # Returns True once at least one subscriber is connected. Absence of a
    # subscriber is not an error -- the Fake GPS app simply may not be installed
    # or running on this deployment -- so a timeout just returns False and the
    # caller still publishes (latched, so a late subscriber picks it up).
    waited = 0.0
    while waited < timeout_sec:
      try:
        if pub.get_num_connections() > 0:
          return True
      except Exception:
        return False
      nepi_sdk.sleep(0.25,1)
      waited += 0.25
    return False

  def reconcileFakeGpsApp(self):
    # A simulated vehicle brings its own GPS; a real airframe on this deployment
    # may not have one at all. The Fake GPS app cannot know which it is looking
    # at, so this driver -- which does -- tells it.
    #
    # Why this exists: confirmed live 2026-08-12, app_fake_gps left enabled from
    # earlier real-hardware testing kept injecting GPS_INPUT at 41 Hz from its
    # configured start point (46.654, -122.319) while the ArduCopter SITL's own
    # simulated GPS sat at (-35.363, 149.165). Two GPS sources ~13000 km apart,
    # both claiming gps_id 0, so the EKF never converged on a position estimate,
    # ArduPilot answered "PreArm: Need Position Estimate", and every LAUNCH
    # aborted at the ARM step with the vehicle never leaving the ground.
    # Nothing in the system reconciled the two, and the app's enabled state
    # persists in /mnt/nepi_storage/user_cfg/app_fake_gps.yaml across reboots --
    # so once it was on, it stayed on and every subsequent sim silently could
    # not fly.
    #
    # Direction of the reconcile is decided by device_path, which discovery
    # builds as connection_type + "_" + address (see discoveryFunction): a SITL
    # connection is always "SITL_<addr>_<port>", every real connection type
    # (SERIAL/USB/TCP/UDP to an actual FCU) is not.
    #   SITL  -> disable Fake GPS (the sim has its own GPS; injection breaks it)
    #   real  -> point Fake GPS at this vehicle's mavros node and enable it
    #
    # Ordering matters on the enable path: select_mavros_node is published and
    # allowed to land BEFORE enable, otherwise the app can start publishing
    # against a stale/None selection. Both publishers are latched so the app
    # still receives them if it comes up after this driver.
    #
    # Timing note: this runs once, here, rather than on a timer. Discovery
    # relaunches this whole node whenever the vehicle is re-detected (see
    # checkOnDevice), so "once per detected vehicle" is exactly the right
    # granularity, and it deliberately does NOT fight a human who later toggles
    # Fake GPS by hand in the RUI for that same vehicle.
    is_sitl = self.is_sitl
    try:
      # Wait for the app's subscriber to actually connect before publishing.
      # These publishers are created immediately above, and a rospy publish
      # issued before the subscriber connection completes is dropped on the
      # floor with no error -- confirmed live 2026-08-12, an earlier version
      # published straight after create_publisher and the app's enabled flag
      # never changed. Absence of a subscriber is not an error (the Fake GPS app
      # need not be installed), so waitForPubConnection just times out and we
      # publish anyway.
      if is_sitl == False:
        self.waitForPubConnection(self.fake_gps_select_pub)
        nepi_sdk.publish_pub(self.fake_gps_select_pub, String(self.mavlink_namespace.rstrip('/')))
        nepi_sdk.sleep(1,10)
      self.waitForPubConnection(self.fake_gps_enable_pub)
      nepi_sdk.publish_pub(self.fake_gps_enable_pub, Bool(not is_sitl))
      if is_sitl:
        self.msg_if.pub_warn("Simulated vehicle detected (device_path " + str(self.device_path)
                             + ") -- disabling the Fake GPS app so its injected GPS_INPUT cannot"
                             + " prevent this vehicle's EKF from getting a position estimate")
      else:
        self.msg_if.pub_warn("Real vehicle detected (device_path " + str(self.device_path)
                             + ") -- pointing the Fake GPS app at " + str(self.mavlink_namespace)
                             + " and enabling it")
    except Exception as e:
      # Never fatal: a deployment without the Fake GPS app installed is entirely
      # legitimate, and this is a convenience reconcile, not a requirement for
      # this driver to operate.
      self.msg_if.pub_warn("Could not reconcile the Fake GPS app state: " + str(e))

  def describeArmRefusalCause(self):
    # ArduPilot refusing to ARM is almost never a NEPI-side problem, so the raw
    # refusal is unactionable on its own. The one cause this deployment has
    # actually hit -- and it cost most of a debugging session to find -- is the
    # Fake GPS app injecting GPS_INPUT into this same mavros node while the
    # vehicle (a SITL, or any FCU with a real GPS) already has its own GPS.
    # Confirmed live 2026-08-12: app_fake_gps published GPS_INPUT at 41 Hz with
    # its factory start point (46.654, -122.319, Washington State) while the
    # ArduCopter SITL's simulated GPS sat at (-35.363, 149.165, Canberra) --
    # two sources ~13000 km apart both claiming gps_id 0. The EKF never formed
    # a position estimate, ArduPilot answered "PreArm: Need Position Estimate",
    # and LAUNCH aborted here every time. Disabling Fake GPS was the only change
    # needed to make the vehicle arm, take off and hold 5 m.
    #
    # So: on an arm refusal, look for a foreign publisher on this mavros node's
    # gps_input topic and name it. Any failure to inspect the graph is swallowed
    # -- this is diagnostic text appended to an already-failing path and must
    # never be what breaks it.
    try:
      ns = getattr(self, 'mavlink_namespace', None)
      if ns is None:
        return ""
      gps_input_topic = ns + "gps_input/gps_input"
      # rosgraph.Master().getSystemState() rather than a nepi_sdk helper: the SDK
      # exposes topic/type listings and a has-subscribers check, but nothing that
      # returns a topic's PUBLISHER names, which is exactly what identifies the
      # offending node here. Same access path nepi_sdk uses internally.
      import rosgraph
      state = rosgraph.Master('/rbx_ardupilot_gps_conflict_check').getSystemState()
      pub_names = []
      for pub_topic, node_names in state[0]:
        if pub_topic == gps_input_topic:
          pub_names = list(node_names)
      if len(pub_names) > 0:
        return (". NOTE: " + str(pub_names) + " is publishing " + gps_input_topic
                + " -- an injected GPS fighting this vehicle's own GPS prevents the"
                + " EKF from forming a position estimate, which makes ArduPilot"
                + " refuse to arm (PreArm: Need Position Estimate). Disable the"
                + " Fake GPS app for a vehicle that already has its own GPS.")
    except Exception as e:
      self.msg_if.pub_debug("Could not check for a conflicting GPS_INPUT publisher: " + str(e))
    return ""

  ## Function for sending takeoff command
  global takeoff
  def takeoff(self):
    return self.takeoff_action()

  def takeoff_action(self):
    self.rbx_if.update_prev_errors()
    self.rbx_if.update_current_errors( [0,0,0,0,0,0,0] )
    cmd_success = False
    if self.state_current == "ARM":
      takeoff_height_m = float(self.settings_dict['takeoff_height_m']['value'])
      takeoff_min_pitch_deg = float(self.settings_dict['takeoff_min_pitch_deg']['value'])
      self.msg_if.pub_info("Sending Takeoff Command to altitude to " + str(takeoff_height_m) + " meters")
      takeoff_cmd = CommandTOLRequest()
      takeoff_cmd.min_pitch = takeoff_min_pitch_deg
      takeoff_cmd.altitude = takeoff_height_m
      nepi_sdk.call_service(self.takeoff_client, takeoff_cmd)
      # Command the fake GPS to climb straight up by takeoff_height_m using a
      # RELATIVE ENU move (east=0, north=0, up=+height) rather than an absolute
      # goto_location. Holding lat/lon and moving relative avoids two bench bugs:
      # (1) it never depends on current_location being populated (an absolute
      #     goto with a [0,0,0] current sent the vehicle to lat/lon 0,0), and
      # (2) it is immune to the ~20 m ellipsoid/AMSL geoid offset that mavros
      #     applies but the geoid-less NEPI pipeline does not - an absolute
      #     altitude goal computed from the mavros (ellipsoid) reading and then
      #     applied by the fake GPS as AMSL sends the climb the wrong way.
      # goal_alt stays in the mavros (ellipsoid) frame for the completion check
      # below, which converges because a +height AMSL climb is a +height
      # ellipsoid climb at a fixed lat/lon. See fakeGpsGoPosCb in the fake_gps app.
      start_alt = self.rbx_if.current_location_wgs84_geo[2]
      goal_alt = start_alt + takeoff_height_m
      self.takeoff_goal_alt = goal_alt
      climb_point = Point()
      climb_point.x = 0.0
      climb_point.y = 0.0
      climb_point.z = float(takeoff_height_m)
      self.fake_gps_goto_position_pub.publish(climb_point)

      error_bound_m = self.rbx_if.rbx_info.error_bounds.max_distance_error_m
      self.takeoff_error_bound_m = error_bound_m
      timeout_sec = self.rbx_if.rbx_info.cmd_timeout
      check_interval_s = float(timeout_sec) / 100
      check_timer = 0
      alt_error = (goal_alt - self.rbx_if.current_location_wgs84_geo[2])
      while (abs(alt_error) > error_bound_m and check_timer < timeout_sec):
        self.rbx_if.update_current_errors( [0,0,alt_error,0,0,0,0] )
        alt_error = (goal_alt - self.rbx_if.current_location_wgs84_geo[2])
        time.sleep(check_interval_s)
        check_timer += check_interval_s
      # Success is decided by whether the vehicle actually REACHED the altitude,
      # not by whether it got there fast enough. The loop above exits on either
      # condition, and this used to test `check_timer < timeout_sec` -- so a climb
      # that converged right at the timeout boundary was recorded as a failure
      # even though the vehicle was exactly on target.
      #
      # That was not cosmetic. takeoff_complete is one of the three things
      # autonomousControlsReady() requires (with ARM and GUIDED), so a false
      # timeout PERMANENTLY disabled every goto command for the rest of the
      # flight on a vehicle that was hovering at its target altitude and
      # perfectly capable of flying. Confirmed live 2026-08-12 running
      # drone_follow_object_mission_script.py against ArduCopter SITL: the
      # vehicle climbed 0.007 m -> 10.004 m and held it, while the RBX status
      # reported "takeoff did not complete", autonomous_control_mode_ready
      # stayed False, and the follow logic's goto_position commands were all
      # silently rejected -- local x/y never left 0.001 m. A loaded VM runs SITL
      # slower than realtime, so the ~20 s climb landed right on the script's
      # 20 s action timeout and lost the race by a fraction.
      #
      # A genuine failure (timed out while still metres away) still reports
      # failure, because that is judged on alt_error too.
      reached_altitude = abs(alt_error) <= error_bound_m
      if reached_altitude:
        cmd_success = True
        self.takeoff_complete = True
        if check_timer >= timeout_sec:
          self.msg_if.pub_warn("Takeoff reached " + str(takeoff_height_m)
                               + " m but only as the " + str(timeout_sec)
                               + "s timeout expired (error " + str(alt_error)
                               + " m) -- treating as complete since the vehicle is"
                               + " at altitude. Raise this command's timeout if the"
                               + " simulator is running slower than realtime.")
        else:
          self.msg_if.pub_info("Takeoff action completed with error: " + str(alt_error) + " meters")
      else:
        self.takeoff_complete = False
        fail_msg = "Takeoff action timed out with error: " + str(alt_error) + " meters"
        self.msg_if.pub_warn(fail_msg)
        if self.rbx_if is not None:
          self.rbx_if.update_error_msg(fail_msg)
    else:
      fail_msg = "Ignoring Takeoff command as system is not Armed"
      self.msg_if.pub_warn(fail_msg)
      if self.rbx_if is not None:
        self.rbx_if.update_error_msg(fail_msg)
    return cmd_success

  ## Action Function for teleporting the sim back to its original spawn
  ## position and time (via gz_reset_listener.py on the VM), then
  ## force-disarming now that it's actually there
  global reset_sim
  def reset_sim(self):
    """Reset the Gazebo sim's model poses and time to spawn, then force-disarm."""
    self.msg_if.pub_info("Recieved Reset Sim cmd")
    # Reset FIRST, disarm AFTER -- the original order (disarm, then reset)
    # cut motor thrust immediately while the vehicle was still at its
    # pre-reset altitude, so gravity took over and it visibly free-fell for
    # however long the gz_reset_listener round trip took before ever
    # snapping back, reported live as "kills all the motors and makes the
    # quadcopter just fall from the air" instead of an instant reset.
    # Disarming only once the vehicle is already back at its resting spawn
    # pose leaves nothing left to fall from.
    cmd_success = False
    try:
      sock = socket.create_connection((self.RESET_SIM_HOST, self.RESET_SIM_PORT), timeout=5)
      reply = sock.recv(200)
      sock.close()
      cmd_success = reply.startswith(b'OK')
    except Exception as e:
      self.msg_if.pub_warn("Reset Sim failed to reach gz_reset_listener: " + str(e))
    force_disarm_cmd = CommandLongRequest()
    force_disarm_cmd.broadcast = False
    force_disarm_cmd.command = self.MAV_CMD_COMPONENT_ARM_DISARM
    force_disarm_cmd.confirmation = 0
    force_disarm_cmd.param1 = 0.0                     # 0 = disarm
    force_disarm_cmd.param2 = self.MAV_CMD_FORCE_MAGIC # bypass in-flight safety interlock
    nepi_sdk.call_service(self.command_client, force_disarm_cmd)
    return cmd_success

  ### Function for switching to STABILIZE mode
  global stabilize
  def stabilize(self):
    cmd_success = self.set_mavlink_mode('STABILIZE')
    self.fake_gps_go_stop_pub.publish(Empty())
    return cmd_success
      
  ### Function for switching to LAND mode
  global land
  def land(self):
    cmd_success = False
    self.set_mavlink_mode('LAND')
    geo_point = GeoPoint()
    geo_point.latitude = self.rbx_if.current_location_wgs84_geo[0]
    geo_point.longitude = self.rbx_if.current_location_wgs84_geo[1]
    start_alt = self.rbx_if.current_location_wgs84_geo[2]
    goal_alt = 0
    geo_point.altitude = goal_alt
    self.fake_gps_goto_location_pub.publish(geo_point)
    self.msg_if.pub_info("Waiting for land process to complete and disarm")
    timeout_sec = self.rbx_if.rbx_info.cmd_timeout
    check_interval_s = float(timeout_sec) / 100
    check_timer = 0
    while (self.state_current == "ARM" and check_timer < timeout_sec):
      time.sleep(check_interval_s)
      check_timer += check_interval_s
    if self.state_current == "ARM":
      self.msg_if.pub_info("Land process complete")
      cmd_success = True
    else:
      self.msg_if.pub_info("Land process timed-out")
    return cmd_success


  ### Function for sending go home command
  global rtl
  def rtl(self):
    cmd_success = False
    self.set_mavlink_mode('RTL')
    self.fake_gps_goto_location_pub.publish(self.home_location)
    error_goal_m = self.rbx_if.rbx_info.error_bounds.max_distance_error_m
    last_loc = self.rbx_if.current_location_wgs84_geo
    timeout_sec = self.rbx_if.rbx_info.cmd_timeout
    check_interval_s = self.rbx_if.rbx_info.error_bounds.min_stabilize_time_s
    check_timer = 0
    stabilized_check = False
    while (stabilized_check is False and check_timer < timeout_sec):
      nepi_sdk.sleep(check_interval_s,100)
      check_timer += check_interval_s
      cur_loc = self.rbx_if.current_location_wgs84_geo
      max_distance_error_m = max(abs(np.subtract(cur_loc,last_loc)))
      stabilized_check = max_distance_error_m < error_goal_m
      last_loc = cur_loc
    if stabilized_check:
      self.msg_if.pub_info("RTL process complete")
      cmd_success = True
    else:
      self.msg_if.pub_info("RTL process timed-out")
    return cmd_success


  ### Function for switching to LOITER mode
  global loiter
  def loiter(self):
    cmd_success = self.set_mavlink_mode('LOITER')
    self.fake_gps_go_stop_pub.publish(Empty())
    return cmd_success


  ### Function for switching to Guided mode
  global guided
  def guided(self):
    cmd_success = self.set_mavlink_mode('GUIDED')
    self.fake_gps_go_stop_pub.publish(Empty())
    return cmd_success

  ### Function for switching back to current mission
  global resume
  def resume(self):
    # Reset mode to last
    self.msg_if.pub_info("Switching mavlink mode from " + self.mode_current + " back to " + self.mode_last)
    return self.set_mavlink_mode(self.mode_last)


  ### Function for setting home location
  def set_home_location(self,geo_point):
    self.msg_if.pub_info("Sending mavlink set home command")
    cmd_home = CommandHomeRequest()
    cmd_home.current_gps = False
    cmd_home.latitude = geo_point.latitude
    cmd_home.longitude = geo_point.longitude
    cmd_home.altitude = geo_point.altitude
    response = nepi_sdk.call_service(self.set_home_client, cmd_home)
    if response is not None:
      self.home_location = geo_point
      if self.has_fake_gps:
        self.fake_gps_reset_pub.publish(geo_point)
      cmd_success = True
    else:
      cmd_success = False
    return cmd_success




  #######################
  # Real Camera Relay (a real onboard camera, unlike the VM-side simulator,
  # is already a normal local ROS topic on this same ROS master -- no bridge
  # protocol needed). Runs independently of the sim camera bridge below;
  # whichever source is actually present feeds image_pub.

  def realCameraWatchLoop(self):
    # Polls for a live local IDX camera topic until one appears, then
    # subscribes once and stops watching. A topic disappearing/reappearing
    # mid-run isn't handled -- rare in practice, and left as a documented
    # future improvement rather than solved here.
    while not nepi_sdk.is_shutdown() and self.real_camera_sub is None:
      found_topic = nepi_sdk.find_topic(self.REAL_CAMERA_TOPIC_PATTERN)
      if found_topic != '' and found_topic != self.image_topic_name:
        self.msg_if.pub_info("Found real camera topic: " + found_topic + " -- relaying to " + self.image_topic_name)
        self.real_camera_sub = nepi_sdk.create_subscriber(found_topic, Image, self.realCameraImageCb, queue_size = 1)
        break
      time.sleep(self.REAL_CAMERA_WATCH_INTERVAL_SEC)

  def realCameraImageCb(self, image_msg):
    # Straight relay -- both ends are already sensor_msgs/Image, no
    # decode/re-encode needed (unlike the sim bridge's base64-JPEG frames).
    # Always robot_color: a real airframe has no chase-cam concept and no
    # depth stream, so there is nothing to relay to any of the other five
    # topics.
    self.image_pub_robot_color.publish(image_msg)

  #######################
  # Camera Bridge Processes (Universal Simulator Bridge camera feature,
  # ArduPilot SITL port -- see camera_rig_controller_ardupilot.py and the
  # session summary for the full design). Independent persistent connection
  # from the mavros/MAVLink path above: MAVLink already carries
  # telemetry/commands, this connection carries ONLY camera settings out and
  # compressed frames in.

  def cameraBridgeLoop(self):
    # Persistent client to the VM-side camera bridge server. The sim stack
    # (or the tunnel) can restart independently of this node -- any failure
    # tears the socket down and retries the connect on a fixed interval.
    buf = b''
    while not nepi_sdk.is_shutdown():
      sock = None
      try:
        sock = socket.create_connection((self.CAMERA_BRIDGE_HOST, self.CAMERA_BRIDGE_PORT),
                                        timeout = self.CAMERA_SOCKET_TIMEOUT_SEC)
        sock.settimeout(self.CAMERA_SOCKET_TIMEOUT_SEC)
      except Exception as e:
        self.msg_if.pub_warn("Camera bridge connect to " + self.CAMERA_BRIDGE_HOST + ":" +
                             str(self.CAMERA_BRIDGE_PORT) + " failed: " + str(e))
        time.sleep(self.CAMERA_RECONNECT_INTERVAL_SEC)
        continue
      with self.camera_sock_lock:
        self.camera_sock = sock
      # Baseline for the watchdog -- covers the "never got a single frame in
      # the first place" case, not just "frames stopped after a while".
      self.camera_last_frame_time = nepi_utils.get_time()
      self.msg_if.pub_info("Connected to camera bridge at " + self.CAMERA_BRIDGE_HOST +
                           ":" + str(self.CAMERA_BRIDGE_PORT))
      # Sync the VM side to this node's actual current camera settings on
      # every (re)connect -- a bare restart of this node resets
      # settings_dict to factory, but camera_rig_controller_ardupilot.py
      # keeps whatever settings it last had, so an explicit push avoids
      # relying on both sides coincidentally matching factory defaults.
      self.sendCameraSettings()
      buf = b''
      while not nepi_sdk.is_shutdown():
        try:
          data = sock.recv(65536)
        except socket.timeout:
          # Server pushes frames at ~7 Hz -- a quiet-but-open socket past
          # the timeout means the far side is gone (e.g. tunnel half-open)
          data = b''
        except Exception:
          data = b''
        if not data:
          break
        buf += data
        while b'\n' in buf:
          line, buf = buf.split(b'\n', 1)
          if line.strip():
            self.processCameraBridgeLine(line)
      with self.camera_sock_lock:
        self.camera_sock = None
      try:
        sock.close()
      except Exception:
        pass
      self.msg_if.pub_warn("Camera bridge connection lost -- retrying in " +
                           str(self.CAMERA_RECONNECT_INTERVAL_SEC) + "s")
      time.sleep(self.CAMERA_RECONNECT_INTERVAL_SEC)

  def processCameraBridgeLine(self, line):
    try:
      msg = json.loads(line)
    except Exception as e:
      self.msg_if.pub_warn("Bad line from camera bridge: " + str(e))
      return
    if msg.get('type') == 'image':
      self.processCameraImageLine(msg)
    else:
      self.msg_if.pub_warn("Unrecognized camera bridge line type: " + str(msg.get('type')))

  # "camera" (added alongside camera_rig_controller_ardupilot.py's six-topic
  # split) picks which of the six publishers a frame goes to; an older
  # sender with no "camera" field defaults to robot_color, matching the
  # original single-topic behavior. depth_map cameras carry a 16-bit PNG
  # (millimeters) instead of a JPEG -- see camera_rig_controller_ardupilot.py
  # and rbx_sim_node.py's matching CAMERA_PUB_ATTR/DEPTH_MAP_CAMERAS comment
  # for the full wire-format rationale (this is the numpy-array
  # reconstruction the platform actually needs to read depth data).
  CAMERA_PUB_ATTR = {
    "robot_color": "image_pub_robot_color",
    "scene_color": "image_pub_scene_color",
    "robot_depth": "image_pub_robot_depth",
    "scene_depth": "image_pub_scene_depth",
    "robot_depth_map": "image_pub_robot_depth_map",
    "scene_depth_map": "image_pub_scene_depth_map",
  }
  DEPTH_MAP_CAMERAS = ("robot_depth_map", "scene_depth_map")

  def processCameraImageLine(self, msg):
    # Bridge image frame -> decode the relayed frame and republish as a raw
    # sensor_msgs/Image on this instance's own namespaced image topic (see
    # the image_pub_robot_color/setImageTopicCb comments in __init__).
    try:
      camera = msg.get('camera', 'robot_color')
      pub_attr = self.CAMERA_PUB_ATTR.get(camera, "image_pub_robot_color")
      image_pub = getattr(self, pub_attr)
      encoded_bytes = base64.b64decode(msg['data'])
      arr = np.frombuffer(encoded_bytes, dtype = np.uint8)
      if camera in self.DEPTH_MAP_CAMERAS:
        depth_mm = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
        if depth_mm is None or depth_mm.dtype != np.uint16:
          raise ValueError("cv2.imdecode did not return a 16-bit depth map")
        depth_m = depth_mm.astype(np.float32) / 1000.0
        ros_img = nepi_img.cv2img_to_rosimg(depth_m, encoding = "32FC1")
      else:
        cv2_img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if cv2_img is None:
          raise ValueError("cv2.imdecode returned None")
        ros_img = nepi_img.cv2img_to_rosimg(cv2_img, encoding = "bgr8")
      image_pub.publish(ros_img)
      self.camera_last_frame_time = nepi_utils.get_time()
    except Exception as e:
      self.msg_if.pub_warn("Failed to process camera image frame: " + str(e))

  def cameraBridgeWatchdogLoop(self):
    # See CAMERA_STALE_TIMEOUT_SEC's comment for the failure this catches.
    # Runs independently of cameraBridgeLoop's own thread specifically so it
    # can act even if that thread's recv/parse loop is the thing that's
    # actually stuck.
    while not nepi_sdk.is_shutdown():
      time.sleep(self.CAMERA_WATCHDOG_INTERVAL_SEC)
      with self.camera_sock_lock:
        sock = self.camera_sock
      if sock is None:
        continue
      if (nepi_utils.get_time() - self.camera_last_frame_time) < self.CAMERA_STALE_TIMEOUT_SEC:
        continue
      self.msg_if.pub_warn("Camera bridge produced no frames for " +
                           str(self.CAMERA_STALE_TIMEOUT_SEC) +
                           "s despite an open connection -- forcing reconnect")
      # shutdown() before close(): cameraBridgeLoop's thread is almost
      # certainly blocked in (or about to re-enter) a recv() on this exact
      # socket -- closing a fd out from under a thread blocked in recv() on
      # it does not reliably unblock that recv() on Linux, matching
      # camera_rig_controller_ardupilot.py's own sendLineToClient fix for
      # the same underlying cross-thread unwedge problem on the server side
      # of this bridge.
      try:
        sock.shutdown(socket.SHUT_RDWR)
      except Exception:
        pass
      try:
        sock.close()
      except Exception:
        pass

  def sendCameraSettings(self):
    # All current values together, always -- camera_rig_controller_
    # ardupilot.py fills any missing key with its own default, so a partial
    # push (e.g. only the field that just changed) would silently reset the
    # rest to that default. No view_mode -- both views are always-live,
    # separate ROS topics now (see CAMERA_SETTING_NAMES's own comment),
    # nothing left to switch.
    cmd = {
      'type': 'camera_settings',
      'offset_x': float(self.settings_dict['camera_offset_x']['value']),
      'offset_y': float(self.settings_dict['camera_offset_y']['value']),
      'offset_z': float(self.settings_dict['camera_offset_z']['value']),
      'scene_offset_x': float(self.settings_dict['scene_offset_x']['value']),
      'scene_offset_y': float(self.settings_dict['scene_offset_y']['value']),
      'scene_offset_z': float(self.settings_dict['scene_offset_z']['value']),
    }
    self.sendLineToCameraBridge(cmd, "Camera settings")

  def sendLineToCameraBridge(self, line_dict, description):
    with self.camera_sock_lock:
      sock = self.camera_sock
    if sock is None:
      self.msg_if.pub_warn(description + " dropped -- camera bridge not connected")
      return
    try:
      sock.sendall((json.dumps(line_dict) + '\n').encode())
    except Exception as e:
      # cameraBridgeLoop's recv will fail on the same dead socket and reconnect
      self.msg_if.pub_warn("Failed to send " + description.lower() + " to camera bridge: " + str(e))

  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  ArdupilotNode()







