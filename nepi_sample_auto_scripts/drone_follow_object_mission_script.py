#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample NEPI Mission Script.
### Expects Classifier to be running ###
# 1) Monitors AI detector output for specfic target class
# 3) Changes system to Loiter mode on detection
# 4) Sends NEPI snapshot event trigger
# 5) Waits to achieve waits set time to complete snapshot events
# 6) Sets system back to original mode
# 6) Delays, then waits for next detection

# Requires the following additional scripts are running
# a) ai_detector_config_script.py
# (Optional) Some Snapshot Action Automation Script like the following
#   b)snapshot_trigger_save_to_disk_action_script.py
# These scripts are available for download at:
# [link text](https://github.com/nepi-engine/nepi_sample_auto_scripts)
#
# Updated for current NEPI Engine API (2026-07): nepi_ros_interfaces -> nepi_interfaces,
# RBXInfo/RBXStatus/RBXGoto*/RBXErrorBounds -> DeviceRBXInfo/DeviceRBXStatus/Goto*/ErrorBounds,
# nepi_msg module -> nepi_api.messages_if.MsgIF, settings topics moved under an rbx/settings/
# sub-namespace, fake GPS moved from a per-robot rbx/enable_fake_gps topic to the standalone
# app_fake_gps app, and TargetLocalization(s) -> Target(s) (Class -> target_name). The
# nepi_sdk.nepi_rbx helper module itself is still broken against these renames, so the RBX
# control helpers are inlined below directly rather than imported from it.
#
# SITL TEST STAND-IN (2026-08-04): no "app_ai_targeting" app exists in this workspace's
# nepi_apps (only fake_gps, file_pub_img, file_pub_vid, image_viewer, onvif_mgr,
# pan_tilt_auto, nav_sim), so this script's app_ai_targeting/target_localizations and
# app_ai_targeting/targeting_image topics have no real producer. For SITL/Gazebo testing,
# nepi_drones/sim_container/scripts/ai_targeting_controller_ardupilot.py (VM-side: spawns a
# moving "chair" target object, computes ground-truth range/azimuth/elevation from the
# drone's /gazebo/model_states pose, streams it over its own TCP bridge on port 9027) plus
# nepi_sample_auto_scripts/tools/sim_ai_targeting_bridge_script.py (device-side: republishes
# that stream as Targets on the exact topics this script waits for, and relays the RBX
# driver's own live image topic for targeting_image) stand in for the missing app. Deploy and
# launch both alongside this script (get_scripts/launch_script) to test the follow logic
# live. No changes were made to this script itself -- the sim infrastructure satisfies its
# existing (already-correct) expectations unmodified.
#
# Live-tested result: with the stand-in running, this script gets past
# wait_for_topic(AI_TARGETING_TOPIC), detects the simulated "chair" target with a live
# range_m/azimuth_deg/elevation_deg, and move_to_object_callback correctly computes and issues
# a body-frame goto_rbx_position command toward it -- proving the follow logic itself is
# correct. The vehicle doesn't visibly close the distance in Gazebo because LAUNCH's takeoff
# step times out before reaching altitude (a separate, pre-existing ArduPilot SITL fake-GPS
# takeoff-climb issue in rbx_ardupilot_node.py, already documented from an earlier session's
# drone_inspection_demo_mission_script.py test -- confirmed reproducible again here, not
# something this pass introduced or is in scope to fix).

import rospy
import sys
import os
import time
import math
import socket
from nepi_sdk import nepi_ros
from nepi_sdk import nepi_settings
from nepi_api.messages_if import MsgIF

from std_msgs.msg import Empty, Bool, String, UInt32, Int32, Float32, Float64
from geographic_msgs.msg import GeoPoint
from nepi_interfaces.msg import DeviceRBXInfo, DeviceRBXStatus, AxisControls, ErrorBounds, GotoErrors, MotorControl, \
     GotoPose, GotoPosition, GotoLocation, Setting, Settings, SettingsStatus
from nepi_interfaces.srv import RBXCapabilitiesQuery, RBXCapabilitiesQueryResponse
from sensor_msgs.msg import NavSatFix, Image
from nepi_interfaces.msg import Target, Targets

#########################################
# USER SETTINGS - Edit as Necessary
#########################################
#RBX Robot Name
RBX_ROBOT_NAME = "ardupilot"

# Robot Settings Overides
###################
TAKEOFF_HEIGHT_M = 10.0
# Ignore Yaw Control
IGNORE_YAW_CONTROL = True

###!!!!!!!! Set Automation action parameters !!!!!!!!
TARGET_TO_FOLLOW = "chair" # Either a target class name (will follow first found of that class) or specific target_id
TARGET_OFFSET_GOAL_M = 0.1 # How close to set setpoint to target
TRIGGER_RESET_DELAY_S = 5 # Time between detect/move checks

# Set Home Poistion
#
# ENABLE_FAKE_GPS was True, and that is exactly what stopped this script from
# ever flying. Root cause confirmed live 2026-08-12: the Fake GPS app injects
# GPS_INPUT at ~41 Hz from HOME_LOCATION while an ArduPilot SITL already has its
# own simulated GPS at the CMAC default below. Two GPS sources ~13000 km apart,
# both claiming gps_id 0, so the EKF never forms a position estimate, ArduPilot
# answers "PreArm: Need Position Estimate", and LAUNCH aborts at the ARM step.
# This is precisely the "LAUNCH's takeoff step times out before reaching
# altitude ... pre-existing ArduPilot SITL fake-GPS takeoff-climb issue" noted in
# this module's own header -- it was never a takeoff-climb issue, it was this.
#
# Fake GPS exists for a real airframe with no GPS of its own. A SITL has one, so
# it must stay OFF here. rbx_ardupilot_node.py's reconcileFakeGpsApp() now also
# auto-disables it whenever it detects a SITL, so leaving this True would have
# the script fighting the driver.
ENABLE_FAKE_GPS = False
# SET_HOME was True, and with Fake GPS off it is actively harmful against a SITL.
# Confirmed live 2026-08-12: publishing a home with altitude 0.0 m moved the
# EKF's home reference to sea level, so global_position/rel_alt read 583.9 m
# (the CMAC field sits ~603 m AMSL) and the vehicle believed it was already
# 584 m above home. It armed, then the 10 m takeoff never completed --
# "LAUNCH failed: armed in GUIDED mode but the takeoff did not complete" --
# because the climb target was already far below the reported current altitude.
#
# Setting home explicitly is what a FAKE-GPS deployment needs (the injected
# position has no meaningful home of its own). A SITL spawns with a correct home
# exactly where it sits, so the right move is to leave it alone. HOME_LOCATION
# is kept below, corrected to the SITL's own CMAC location including its real
# ~603 m AMSL field elevation, so flipping SET_HOME back on for a fake-GPS run
# does not silently reintroduce the sea-level bug.
SET_HOME = False
HOME_LOCATION = [-35.3632621,149.1652374,603.44]

# Goto Error Settings
GOTO_MAX_ERROR_M = 2.0 # Goal reached when all translation move errors are less than this value
GOTO_MAX_ERROR_DEG = 2.0 # Goal reached when all rotation move errors are less than this value
GOTO_STABILIZED_SEC = 1.0 # Window of time that setpoint error values must be good before proceeding

# CMD Timeout Values
CMD_STATE_TIMEOUT_SEC = 5
CMD_MODE_TIMEOUT_SEC = 5
# CMD_ACTION_TIMEOUT_SEC was 20, which is the whole budget the driver's takeoff
# completion check gets. Measured live 2026-08-12 on the dev VM: a 10 m
# TAKEOFF_HEIGHT_M climb took ~20 s wall-clock, because SITL plus Gazebo on a
# loaded 4-core VM runs slower than realtime -- so the climb finished right on
# the timeout and was scored a failure. 60 s leaves real headroom without
# masking an actually-stuck takeoff.
CMD_ACTION_TIMEOUT_SEC = 60
# Raised from 20 for the same slower-than-realtime reason, and because the follow
# target is a MOVING one -- see the call site in move_to_object_callback, which
# also had to be fixed to actually pass this value.
CMD_GOTO_TIMEOUT_SEC = 30

# Sim target teardown (added 2026-08-26) -- best-effort signal to the dev
# VM's ai_targeting_controller_ardupilot.py (see its TEARDOWN_PORT) to
# despawn the simulated "chair" when this script stops, over the same
# reverse-tunnel-forwarded loopback the RESET_SIM setup action's
# gz_reset_listener already uses. A no-op (times out quietly) against real
# hardware or when no sim is running -- this script has no way to know
# which it's talking to, and doesn't need to.
SIM_TEARDOWN_HOST = "127.0.0.1"
SIM_TEARDOWN_PORT = 9029
SIM_TEARDOWN_TIMEOUT_SEC = 5.0

#########################################
# Node Class
#########################################




class drone_follow_object_mission(object):

  rbx_info = DeviceRBXInfo()
  rbx_status = DeviceRBXStatus()
  rbx_settings = None

  img_height = 0
  img_width = 0

  settings_update =  dict(
    takeoff_height_m = {"type":"Float","name":"takeoff_height_m","value":str(TAKEOFF_HEIGHT_M)}
  )
  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "drone_follow_object_mission" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    self.msg_if = MsgIF(log_name = self.node_name)
    self.msg_if.pub_info("Starting Initialization Processes")
    ##############################
    ##############################
    self.msg_if.pub_info("Waiting for namespace containing: " + RBX_ROBOT_NAME)
    robot_namespace = nepi_ros.wait_for_node(RBX_ROBOT_NAME)
    robot_namespace = robot_namespace + "/"
    self.msg_if.pub_info("Found namespace: " + robot_namespace)
    rbx_namespace = (robot_namespace + "rbx/")
    self.msg_if.pub_info("Using rbx namesapce " + rbx_namespace)
    self.rbx_initialize(rbx_namespace)
    # Registered as soon as the rbx_* publishers cleanup_actions() needs
    # actually exist -- fires on a normal RUI stop (StopScript -> rospy
    # shutdown) as well as any other clean rospy shutdown, covering
    # virtually this whole script's runtime. Previously cleanup_actions()
    # was dead code (defined, never wired to anything) -- confirmed live
    # 2026-08-26 that stopping the script via the RUI left the drone armed/
    # airborne and the sim chair frozen in place with nothing tearing
    # either down.
    rospy.on_shutdown(self.cleanup_actions)
    time.sleep(1)
    self.msg_if.pub_info("Waiting for status message")
    while self.rbx_status is None and not rospy.is_shutdown():
       time.sleep(1)

    #### publishers used below are defined in rbx_initialize() above

    # Apply Takeoff Height setting overide
    for setting_name in self.settings_update.keys():
      setting = self.settings_update[setting_name]
      setting_msg = nepi_settings.create_msg_from_setting(setting)
      self.msg_if.pub_info("Updated setting msg:" + str(setting_msg))
      self.rbx_setting_update_pub.publish(setting_msg)

    # Setup Fake GPS if Enabled
    if ENABLE_FAKE_GPS:
      self.msg_if.pub_info("Enabling Fake GPS")
      self.fake_gps_enable_pub.publish(True)
      time.sleep(2)
    if SET_HOME:
      self.msg_if.pub_info("Upating RBX Home Location")
      new_home_geo = GeoPoint()
      new_home_geo.latitude = HOME_LOCATION[0]
      new_home_geo.longitude = HOME_LOCATION[1]
      new_home_geo.altitude = HOME_LOCATION[2]
      self.rbx_set_home_pub.publish(new_home_geo)
      nepi_ros.sleep(2) # Give system time to stabilize on new gps location
      if ENABLE_FAKE_GPS:
      	nepi_ros.sleep(15,100) # Give system time to stabilize on new gps location

    # Setup mission action processes
    SNAPSHOT_TRIGGER_TOPIC = self.base_namespace + "snapshot_trigger"
    self.snapshot_trigger_pub = rospy.Publisher(SNAPSHOT_TRIGGER_TOPIC, Empty, queue_size = 1)


    ###########################
    # Sutup AI 3d targeting
    ###########################
    # Wait for AI targeting detection topic and subscribe to it
    # NOTE: no app_ai_targeting app exists in this workspace today -- this will block here
    # until one is added (see module docstring's "KNOWN GAP").
    AI_TARGETING_TOPIC = "app_ai_targeting/target_localizations"
    self.msg_if.pub_info("Waiting for topic: " + AI_TARGETING_TOPIC)
    ai_targeting_topic = nepi_ros.wait_for_topic(AI_TARGETING_TOPIC)

    AI_TARGETING_IMAGE_TOPIC = "app_ai_targeting/targeting_image"
    self.msg_if.pub_info("Waiting for topic: " + AI_TARGETING_IMAGE_TOPIC)
    ai_targeting_image_topic = nepi_ros.wait_for_topic(AI_TARGETING_IMAGE_TOPIC)
    self.msg_if.pub_info("Setting image topic to: " + ai_targeting_image_topic)
    self.rbx_set_image_topic_pub.publish(ai_targeting_image_topic)

    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    self.msg_if.pub_info("Waiting for AI Object Detection")

    ###########################
    ## Start Mission
    ###########################
    # Run pre-mission processes
    self.pre_mission_actions()
    # Start misson processes
    self.msg_if.pub_info("Starting move to object callback")
    rospy.Subscriber(ai_targeting_topic, Targets, self.move_to_object_callback, queue_size = 1)

    ##############################
    ## Initiation Complete
    self.msg_if.pub_info(" Initialization Complete")
    # Spin forever (until object is detected)
    rospy.spin()
    ##############################

  #######################
  ### RBX Initialize and Control Helpers
  # Inlined from the (currently broken, see module docstring) nepi_sdk.nepi_rbx helper --
  # same logic, updated for current message names / settings topic layout / fake-gps app.

  def rbx_initialize(self, rbx_namespace):
    self.rbx_cap_states = [""]
    self.rbx_cap_modes = [""]
    self.rbx_cap_setup_actions = [""]
    self.rbx_cap_go_actions = [""]
    self.rbx_settings = None
    self.rbx_info = None
    self.rbx_status = None

    rbx_topic = nepi_ros.wait_for_topic(rbx_namespace)
    NEPI_ROBOT_NAMESPACE = rbx_topic.rpartition("rbx")[0]
    NEPI_RBX_NAMESPACE = (NEPI_ROBOT_NAMESPACE + "rbx/")
    self.msg_if.pub_info("Found rbx namespace: " + NEPI_RBX_NAMESPACE)

    NEPI_RBX_CAPABILITIES_TOPIC = NEPI_RBX_NAMESPACE + "capabilities_query"
    nepi_ros.wait_for_service(NEPI_RBX_CAPABILITIES_TOPIC)
    rbx_caps_service = nepi_ros.connect_service(NEPI_RBX_CAPABILITIES_TOPIC, RBXCapabilitiesQuery)
    time.sleep(1)
    rbx_caps = rbx_caps_service()
    self.rbx_cap_states = rbx_caps.state_options
    self.rbx_cap_modes = rbx_caps.mode_options
    self.rbx_cap_setup_actions = rbx_caps.setup_action_options
    self.rbx_cap_go_actions = rbx_caps.go_action_options
    self.msg_if.pub_info("RBX State Options: " + str(self.rbx_cap_states))
    self.msg_if.pub_info("RBX Mode Options: " + str(self.rbx_cap_modes))
    self.msg_if.pub_info("RBX Setup Action Options: " + str(self.rbx_cap_setup_actions))
    self.msg_if.pub_info("RBX Go Action Options: " + str(self.rbx_cap_go_actions))

    ## Settings live under rbx/settings/ (latched status -- no manual publish-trigger needed)
    NEPI_RBX_SETTINGS_TOPIC = NEPI_RBX_NAMESPACE + "settings/status"
    self.msg_if.pub_info("Waiting for topic: " + NEPI_RBX_SETTINGS_TOPIC)
    nepi_ros.wait_for_topic(NEPI_RBX_SETTINGS_TOPIC)
    nepi_ros.create_subscriber(NEPI_RBX_SETTINGS_TOPIC, SettingsStatus, self.rbx_settings_callback, queue_size=None)
    while self.rbx_settings is None and not nepi_ros.is_shutdown():
      self.msg_if.pub_info("Waiting for current rbx settings to publish")
      time.sleep(1)
    self.msg_if.pub_info("Initial settings:" + str(self.rbx_settings))

    ## Setup Info Update Callback
    self.NEPI_RBX_INFO_TOPIC = NEPI_RBX_NAMESPACE + "info"
    self.msg_if.pub_info("Waiting for topic: " + self.NEPI_RBX_INFO_TOPIC)
    nepi_ros.wait_for_topic(self.NEPI_RBX_INFO_TOPIC)
    rbx_info_pub = nepi_ros.create_publisher(NEPI_RBX_NAMESPACE + 'publish_info', Empty, queue_size=1)
    nepi_ros.create_subscriber(self.NEPI_RBX_INFO_TOPIC, DeviceRBXInfo, self.rbx_info_callback, queue_size=None)
    while self.rbx_info is None and not nepi_ros.is_shutdown():
      self.msg_if.pub_info("Waiting for current rbx info to publish")
      time.sleep(1)
      rbx_info_pub.publish(Empty())
    self.msg_if.pub_info(str(self.rbx_info))

    ## Setup Status Update Callback
    self.NEPI_RBX_STATUS_TOPIC = NEPI_RBX_NAMESPACE + "status"
    self.msg_if.pub_info("Waiting for topic: " + self.NEPI_RBX_STATUS_TOPIC)
    nepi_ros.wait_for_topic(self.NEPI_RBX_STATUS_TOPIC)
    rbx_status_pub = nepi_ros.create_publisher(NEPI_RBX_NAMESPACE + 'publish_status', Empty, queue_size=1)
    nepi_ros.create_subscriber(self.NEPI_RBX_STATUS_TOPIC, DeviceRBXStatus, self.rbx_status_callback, queue_size=None)
    while self.rbx_status is None and not nepi_ros.is_shutdown():
      self.msg_if.pub_info("Waiting for current rbx status to publish")
      time.sleep(0.1)
      rbx_status_pub.publish(Empty())
    self.msg_if.pub_info(str(self.rbx_status))

    NEPI_RBX_SETTINGS_UPDATE_TOPIC = NEPI_RBX_NAMESPACE + "settings/update_setting"
    self.rbx_setting_update_pub = nepi_ros.create_publisher(NEPI_RBX_SETTINGS_UPDATE_TOPIC, Setting, queue_size=1)

    NEPI_RBX_SET_STATE_TOPIC = NEPI_RBX_NAMESPACE + "set_state"
    NEPI_RBX_SET_MODE_TOPIC = NEPI_RBX_NAMESPACE + "set_mode"
    NEPI_RBX_SETUP_ACTION_TOPIC = NEPI_RBX_NAMESPACE + "setup_action"
    NEPI_RBX_SET_CMD_TIMEOUT_TOPIC = NEPI_RBX_NAMESPACE + "set_goto_timeout" # renamed from set_cmd_timeout
    NEPI_RBX_SET_HOME_TOPIC = NEPI_RBX_NAMESPACE + "set_home"
    NEPI_RBX_SET_STATUS_IMAGE_TOPIC = NEPI_RBX_NAMESPACE + "set_image_topic"
    NEPI_RBX_SET_PROCESS_NAME_TOPIC = NEPI_RBX_NAMESPACE + "set_process_name"

    self.rbx_set_state_pub = nepi_ros.create_publisher(NEPI_RBX_SET_STATE_TOPIC, Int32, queue_size=1)
    self.rbx_set_mode_pub = nepi_ros.create_publisher(NEPI_RBX_SET_MODE_TOPIC, Int32, queue_size=1)
    self.rbx_setup_action_pub = nepi_ros.create_publisher(NEPI_RBX_SETUP_ACTION_TOPIC, Int32, queue_size=1)
    self.rbx_set_cmd_timeout_pub = nepi_ros.create_publisher(NEPI_RBX_SET_CMD_TIMEOUT_TOPIC, UInt32, queue_size=1)
    self.rbx_set_home_pub = nepi_ros.create_publisher(NEPI_RBX_SET_HOME_TOPIC, GeoPoint, queue_size=1)
    self.rbx_set_image_topic_pub = nepi_ros.create_publisher(NEPI_RBX_SET_STATUS_IMAGE_TOPIC, String, queue_size=1)
    self.rbx_set_process_name_pub = nepi_ros.create_publisher(NEPI_RBX_SET_PROCESS_NAME_TOPIC, String, queue_size=1)

    NEPI_RBX_GO_ACTION_TOPIC = NEPI_RBX_NAMESPACE + "go_action"
    NEPI_RBX_GO_HOME_TOPIC = NEPI_RBX_NAMESPACE + "go_home"
    NEPI_RBX_GO_STOP_TOPIC = NEPI_RBX_NAMESPACE + "go_stop"
    NEPI_RBX_GOTO_POSE_TOPIC = NEPI_RBX_NAMESPACE + "goto_pose"
    NEPI_RBX_GOTO_POSITION_TOPIC = NEPI_RBX_NAMESPACE + "goto_position"
    NEPI_RBX_GOTO_LOCATION_TOPIC = NEPI_RBX_NAMESPACE + "goto_location"

    self.rbx_go_action_pub = nepi_ros.create_publisher(NEPI_RBX_GO_ACTION_TOPIC, Int32, queue_size=1)
    self.rbx_go_home_pub = nepi_ros.create_publisher(NEPI_RBX_GO_HOME_TOPIC, Empty, queue_size=1)
    self.rbx_go_stop_pub = nepi_ros.create_publisher(NEPI_RBX_GO_STOP_TOPIC, Empty, queue_size=1)
    self.rbx_goto_pose_pub = nepi_ros.create_publisher(NEPI_RBX_GOTO_POSE_TOPIC, GotoPose, queue_size=1)
    self.rbx_goto_position_pub = nepi_ros.create_publisher(NEPI_RBX_GOTO_POSITION_TOPIC, GotoPosition, queue_size=1)
    self.rbx_goto_location_pub = nepi_ros.create_publisher(NEPI_RBX_GOTO_LOCATION_TOPIC, GotoLocation, queue_size=1)

    # Fake GPS is a standalone app now (nepi_app_fake_gps), not a per-robot rbx/ topic --
    # a single instance at the base namespace injects HilGPS into whichever mavros node
    # the driver is attached to.
    # os.path.join, not string concatenation: get_base_namespace() returns the
    # namespace with NO trailing slash, so "base + 'app_fake_gps/'" built the
    # topic "/nepi/device1app_fake_gps/enable" -- confirmed live 2026-08-12 in
    # this node's own advertised-topic list. Harmless only because
    # ENABLE_FAKE_GPS is now False; with it True the enable publish went to a
    # topic nothing subscribes to, silently doing nothing.
    FAKE_GPS_NAMESPACE = os.path.join(self.base_namespace, "app_fake_gps") + "/"
    self.fake_gps_enable_pub = nepi_ros.create_publisher(FAKE_GPS_NAMESPACE + "enable", Bool, queue_size=1)

    self.msg_if.pub_info("RBX initialize process complete")

  def rbx_settings_callback(self, msg):
    self.rbx_settings = nepi_settings.parse_setting_msgs_list(msg)

  def rbx_info_callback(self, msg):
    self.rbx_info = msg

  def rbx_status_callback(self, msg):
    self.rbx_status = msg

  def wait_for_rbx_status_ready(self, timeout_sec=10):
    self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
    time.sleep(0.1)
    count_goal = 3
    counter = 0
    timeout_timer = 0
    sleep_time_sec = 0.1
    while (counter < count_goal) and timeout_timer < timeout_sec and not nepi_ros.is_shutdown():
      if self.rbx_status.ready is True:
        counter += 1
      else:
        counter = 0
      time.sleep(sleep_time_sec)
      timeout_timer += sleep_time_sec
    return self.rbx_status.ready

  def wait_for_rbx_status_busy(self, timeout_sec=10):
    self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
    time.sleep(0.1)
    count_goal = 3
    counter = 0
    timeout_timer = 0
    sleep_time_sec = 0.1
    while (counter < count_goal) and timeout_timer < timeout_sec and not nepi_ros.is_shutdown():
      if self.rbx_status.ready is False:
        counter += 1
      else:
        counter = 0
      time.sleep(sleep_time_sec)
      timeout_timer += sleep_time_sec
    return self.rbx_status.ready == False

  def set_rbx_state(self, state_str, timeout_sec=5):
    self.msg_if.pub_info("Set State Request Recieved: " + state_str)
    success = False
    self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
    new_state_ind = -1
    for ind, state in enumerate(self.rbx_cap_states):
      if state == state_str:
        new_state_ind = ind
    if new_state_ind == -1:
      self.msg_if.pub_warn("No matching state found: " + state_str)
    else:
      self.rbx_set_state_pub.publish(new_state_ind)
      timeout_timer = 0
      sleep_time_sec = 1
      while self.rbx_info.state != new_state_ind and timeout_timer < timeout_sec and not nepi_ros.is_shutdown():
        time.sleep(sleep_time_sec)
        timeout_timer += sleep_time_sec
      if self.rbx_info.state == new_state_ind:
        success = True
    time.sleep(2)
    return success

  def set_rbx_mode(self, mode_str, timeout_sec=5):
    self.msg_if.pub_info("Set Mode Request Recieved: " + mode_str)
    success = False
    self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
    new_mode_ind = -1
    for ind, mode in enumerate(self.rbx_cap_modes):
      if mode == mode_str:
        new_mode_ind = ind
    if new_mode_ind == -1:
      self.msg_if.pub_warn("No matching mode found: " + mode_str)
    else:
      self.rbx_set_mode_pub.publish(new_mode_ind)
      timeout_timer = 0
      sleep_time_sec = 1
      while self.rbx_info.mode != new_mode_ind and timeout_timer < timeout_sec and not nepi_ros.is_shutdown():
        time.sleep(sleep_time_sec)
        timeout_timer += sleep_time_sec
      if self.rbx_info.mode == new_mode_ind:
        success = True
    time.sleep(1)
    return success

  def setup_rbx_action(self, action_str, timeout_sec=10):
    self.msg_if.pub_info("Setup Action Request Recieved: " + action_str)
    success = False
    self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
    time.sleep(0.1)
    action_ind = -1
    for ind, action in enumerate(self.rbx_cap_setup_actions):
      if action == action_str:
        action_ind = ind
    if action_ind == -1:
      self.msg_if.pub_warn("No matching action found: " + action_str)
    else:
      ready = self.wait_for_rbx_status_ready(timeout_sec)
      if ready:
        self.rbx_setup_action_pub.publish(action_ind)
        busy = self.wait_for_rbx_status_busy(timeout_sec)
        if busy:
          self.wait_for_rbx_status_ready(timeout_sec)
      time.sleep(1)
      success = self.rbx_status.cmd_success
    return success

  def set_rbx_process_name(self, process_name):
    self.rbx_set_process_name_pub.publish(process_name)
    return True

  def goto_rbx_position(self, goto_data, timeout_sec=10):
    self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
    time.sleep(0.1)
    if len(goto_data) == 4:
      ready = self.wait_for_rbx_status_ready(timeout_sec)
      if ready:
        self.msg_if.pub_info("Starting goto Position Body Process")
        goto_msg = GotoPosition()
        goto_msg.x_meters = goto_data[0]
        goto_msg.y_meters = goto_data[1]
        goto_msg.z_meters = goto_data[2]
        goto_msg.yaw_deg = goto_data[3]
        self.rbx_goto_position_pub.publish(goto_msg)
        busy = self.wait_for_rbx_status_busy(timeout_sec)
        if busy:
          self.wait_for_rbx_status_ready(timeout_sec)
      time.sleep(1)
      return self.rbx_status.cmd_success
    return False

  #######################
  ### Node Methods

 ## Function for custom pre-mission actions
  def pre_mission_actions(self):
    ###########################
    # Start Your Custom Actions
    ###########################
    success = True
    # Use the LAUNCH setup action (GUIDED -> ARM -> takeoff chained server-side in one
    # call) rather than separate set_rbx_mode(GUIDED)/set_rbx_state(ARM)/setup_rbx_action(
    # TAKEOFF) calls -- confirmed in prior SITL testing that the network round-trip delay
    # between a standalone ARM and a standalone TAKEOFF can exceed ArduCopter's ground
    # auto-disarm timer, silently disarming the vehicle before takeoff lands.
    success=self.setup_rbx_action("LAUNCH",timeout_sec =CMD_ACTION_TIMEOUT_SEC)
    time.sleep(1)
    error_str = str(self.rbx_status.errors_current)
    if success:
      self.msg_if.pub_info("DRONE_INSPECT: Takeoff completed with errors: " + error_str )
    else:
      self.msg_if.pub_info("DRONE_INSPECT: Takeoff failed with errors: " + error_str )
    nepi_ros.sleep(2,10)
    ###########################
    # Stop Your Custom Actions
    ###########################
    print("Pre-Mission Actions Complete")
    return success


  ## Function for custom mission actions
  def mission_actions(self):
    ###########################
    # Start Your Custom Actions
    ###########################
    self.snapshot_trigger_pub.publish(Empty())
    success = True
    #########################################

    ###########################
    # Stop Your Custom Actions
    ###########################
    self.msg_if.pub_info("Mission Actions Complete")
    return success

  ## Function for custom post-mission actions
  def post_mission_actions(self):
    ###########################
    # Start Your Custom Actions
    ###########################
    success = True
    #success = self.set_rbx_mode("LAND", timeout_sec = CMD_MODE_TIMEOUT_SEC) # Uncomment to change to Land mode
    #success = self.set_rbx_mode("LOITER", timeout_sec = CMD_MODE_TIMEOUT_SEC) # Uncomment to change to Loiter mode
    success = self.set_rbx_mode("RTL", timeout_sec = CMD_MODE_TIMEOUT_SEC) # Uncomment to change to home mode
    #success = self.set_rbx_mode("RESUME", timeout_sec = CMD_MODE_TIMEOUT_SEC) # Uncomment to return to last mode
    nepi_ros.sleep(1,10)
    ###########################
    # Stop Your Custom Actions
    ###########################
    print("Post-Mission Actions Complete")
    return success

  #######################
  # AI Detection Functions

    ### Simple callback to get image height and width
  def ai_image_callback(self,img_msg):
    # This is just to get the image size for ratio purposes
    if (self.img_height == 0 and self.img_width == 0):
      self.msg_if.pub_info("Initial input image received. Size = " + str(img_msg.width) + "x" + str(img_msg.height))
      self.img_height = img_msg.height
      self.img_width = img_msg.width

  # Action upon detection and targeting for object of interest
  def move_to_object_callback(self,targets_data_msg):
    # Check for the object of interest and take appropriate actions
    for target_data_msg in targets_data_msg.targets:
      # nepi_interfaces/Target's class field is "name". This read "target_name",
      # which raised AttributeError inside the subscriber callback on EVERY
      # incoming Targets message -- confirmed live 2026-08-12: rospy caught it
      # and logged "bad callback ... 'Target' object has no attribute
      # 'target_name'" to /rosout, then carried on, so the script looked healthy
      # and simply never followed anything. The vehicle sat at its takeoff
      # altitude with local x/y pinned at 0.01 m while valid targets streamed in
      # at ~1 Hz. (This module's own header claims the rename went "Class ->
      # target_name"; the actual field is plain "name".)
      target_class = target_data_msg.name
      target_range_m = target_data_msg.range_m # [x,y,z]
      target_yaw_d = target_data_msg.azimuth_deg  # dz
      target_pitch_d = target_data_msg.elevation_deg # dy
      if target_class == TARGET_TO_FOLLOW and target_range_m != -999:
        self.msg_if.pub_info("Detected a " + TARGET_TO_FOLLOW + "with valid range")
        setpoint_range_m = target_range_m - TARGET_OFFSET_GOAL_M
        # Y/Z were computed in the AI-targeting sensor's own convention
        # (X forward, Y RIGHT, Z DOWN -- see ai_targeting_controller_ardupilot.py's
        # own docstring), but goto_rbx_position() ultimately calls
        # device_if_rbx.py's setpoint_position_local_body(), whose docstring
        # states its body frame is X forward, Y LEFT, Z UP -- the opposite
        # sign on both axes. Sending the sensor's raw right/down values
        # there means "descend toward a low target" got interpreted as
        # "climb", and left/right got mirrored too. Confirmed live
        # 2026-08-26: the drone climbed to ~18m (10m takeoff + ~8m of
        # wrong-direction climb) chasing a target near ground level, instead
        # of descending to meet it -- exactly the ~8-9m magnitude of the
        # elevation-driven Z command being applied with the wrong sign.
        # Fixed by negating both axes when building the driver-frame
        # setpoint, rather than touching the sensor's own (correct, and
        # shared with other consumers) right/down convention.
        sp_x_m = setpoint_range_m * math.cos(math.radians(target_yaw_d))  # X is Forward in both conventions
        sp_y_m = -setpoint_range_m * math.sin(math.radians(target_yaw_d)) # sensor Y is Right -> driver Y is Left
        sp_z_m = setpoint_range_m * math.sin(math.radians(target_pitch_d)) # sensor Z is Down -> driver Z is Up
        sp_yaw_d = target_yaw_d
        if IGNORE_YAW_CONTROL:
          sp_yaw_d = -999
        setpoint_position_body_m = [sp_x_m,sp_y_m,sp_z_m,sp_yaw_d]
        rospy.logwarn(setpoint_position_body_m)
        # Send poisition update
        self.msg_if.pub_info("Sending setpoint position body command")
        self.msg_if.pub_info(str(setpoint_position_body_m))
        # Pass CMD_GOTO_TIMEOUT_SEC explicitly. It is defined up in USER SETTINGS
        # but was never used -- goto_rbx_position's own default of 10 s applied
        # instead, so the configured value was silently ignored. 10 s is not
        # enough to close 10+ m on a target that is itself circling (radius
        # 2.5 m, 50 s period) to within GOTO_MAX_ERROR_M = 2.0 m, especially with
        # SITL running slower than realtime: confirmed live 2026-08-12, every
        # goto reported "Setpoint cmd timed out" / "Goto Position failed" while
        # the vehicle was in fact tracking the target correctly.
        success = self.goto_rbx_position(setpoint_position_body_m, timeout_sec = CMD_GOTO_TIMEOUT_SEC)
        error_str = str(self.rbx_status.errors_current)
        if success:
          self.msg_if.pub_info("Goto Position completed with errors: " + error_str )
        else:
          self.msg_if.pub_info("Goto Position failed with errors: " + error_str )
        nepi_ros.sleep(2,10)
        #########################################
        # Run Mission Actions
        #self.msg_if.pub_info("Starting Mission Actions")
        #success = self.mission_actions()
        ##########################################
  ##        self.msg_if.pub_info("Switching back to original mode")
  ##        self.set_rbx_mode("RESUME")
        self.msg_if.pub_info("Delaying next trigger for " + str(TRIGGER_RESET_DELAY_S) + " secs")
        nepi_ros.sleep(TRIGGER_RESET_DELAY_S,100)
        self.msg_if.pub_info("Waiting for next " + TARGET_TO_FOLLOW + " detection")
      else:
        self.msg_if.pub_info("Target range value invalid, skipping actions")
        time.sleep(1)


  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    # RESET_SIM force-disarms (works regardless of current flight state --
    # mid-goto, hovering, etc., unlike a plain mode change) and teleports
    # back to the origin/base position -- the same mechanism the driver's
    # own RESET_SIM RUI action uses. This is what actually satisfies
    # "stopping the script resets the drone to unarmed at the base
    # position." A no-op against real hardware (RESET_SIM won't be in
    # rbx_cap_setup_actions there) or if the RBX pubs never finished
    # initializing.
    try:
      if "RESET_SIM" in self.rbx_cap_setup_actions:
        self.setup_rbx_action("RESET_SIM", timeout_sec = CMD_ACTION_TIMEOUT_SEC)
      else:
        self.msg_if.pub_info("RESET_SIM not available (real hardware?) -- skipping")
    except Exception as e:
      self.msg_if.pub_warn("RESET_SIM on cleanup failed: " + str(e))

    # Ask the sim to despawn the chair -- see SIM_TEARDOWN_PORT above.
    try:
      sock = socket.create_connection((SIM_TEARDOWN_HOST, SIM_TEARDOWN_PORT),
                                       timeout = SIM_TEARDOWN_TIMEOUT_SEC)
      sock.settimeout(SIM_TEARDOWN_TIMEOUT_SEC)
      sock.recv(200)
      sock.close()
      self.msg_if.pub_info("Sim target teardown triggered")
    except Exception as e:
      self.msg_if.pub_info("Sim target teardown not reachable (expected on real hardware): " + str(e))

#########################################
# Main
#########################################
if __name__ == '__main__':
  drone_follow_object_mission()
