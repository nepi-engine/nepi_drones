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
# 1) Subscribes to NEPI nav_pose_current heading, orientation, position, location topics
# 2) Runs pre-mission processes
# 3) Runs mission goto command processes
# 4) Runs mission action processes
# 5) Runs post-mission processes

# Requires the following additional scripts are running
# a) ardupilot_rbx_driver_script.py
# (Optional) Some Snapshot Action Automation Script like the following
#   b)snapshot_event_save_to_disk_action_script.py
#   c)snapshot_event_send_to_cloud_action_script.py
# These scripts are available for download at:
# [link text](https://github.com/nepi-engine/nepi_sample_auto_scripts)
#
# Updated for current NEPI Engine API (2026-07): nepi_ros_interfaces -> nepi_interfaces,
# RBXInfo/RBXStatus/RBXGoto*/RBXErrorBounds -> DeviceRBXInfo/DeviceRBXStatus/Goto*/ErrorBounds,
# nepi_msg module -> nepi_api.messages_if.MsgIF, settings topics moved under an rbx/settings/
# sub-namespace, and fake GPS moved from a per-robot rbx/enable_fake_gps topic to the
# standalone app_fake_gps app. The nepi_sdk.nepi_rbx helper module itself is still broken
# against these renames, so the RBX control helpers are inlined below directly rather than
# imported from it (see nepi_drones/docs -- same fix applied to the sibling
# drone_follow_object_mission_script.py).

import rospy
import sys
import os
import time
from nepi_sdk import nepi_ros
from nepi_sdk import nepi_settings
from nepi_api.messages_if import MsgIF

from std_msgs.msg import Empty, Bool, String, UInt32, Int32, Float32, Float64
from geographic_msgs.msg import GeoPoint
from nepi_interfaces.msg import DeviceRBXInfo, DeviceRBXStatus, AxisControls, ErrorBounds, GotoErrors, MotorControl, \
     GotoPose, GotoPosition, GotoLocation, Setting, Settings, SettingsStatus
from nepi_interfaces.srv import RBXCapabilitiesQuery, RBXCapabilitiesQueryResponse

#########################################
# USER SETTINGS - Edit as Necessary
#########################################
#RBX Robot Name
RBX_ROBOT_NAME = "ardupilot"

# Robot Settings Overides
###################
TAKEOFF_HEIGHT_M = 10.0

# GoTo Position Global Settings
###################
# goto_location is [LAT, LONG, ALT_WGS84, YAW_NED_DEGREES]
# Altitude is specified as meters above the WGS-84 and converted to AMSL before sending
# Yaw is specified in NED frame degrees 0-360 or +-180
#
# These were Seattle-area coordinates (~47.65,-122.32) with a real-hardware,
# fake-GPS deployment in mind -- thousands of km from the SITL's actual CMAC
# spawn point (see HOME_LOCATION below, matching the sibling
# drone_follow_object_mission_script.py's corrected value). Against a SITL
# this sent goto_location commands to fly there literally, which is not a
# testable mission. Replaced with the same small triangular patrol pattern
# (identical lat/long deltas from home as the original Seattle waypoints),
# just re-centered on CMAC so this script is actually testable against the
# dev VM's SITL.
#
# GOTO_LOCATION_CORNERS' altitude is an EXPLICIT value (613.44 = the CMAC
# field's 603.44 m AMSL + TAKEOFF_HEIGHT_M), not -999 (hold current).
# Confirmed live 2026-08-27: chaining multiple -999-altitude goto_location
# calls back to back climbed the vehicle from ~10 m AGL after the first
# call to ~48 m AGL by the third corner (613 -> 632 -> 633 -> 651 m AMSL
# commanded, each call nominally just "holding" whatever altitude the
# previous one left it at) -- confirmed via Gazebo ground truth, not just
# a telemetry artifact. Root cause not fully pinned down (traced deep into
# device_if_rbx.py's setpoint_location_global_wgs84()/current_geoid_height_m
# plumbing without finding a clean, confident mechanism -- likely a race
# between the GPS-fix callback and the goto-processing loop reading
# current_geoid_height_m/current_location_wgs84_geo mid-update), so rather
# than risk a shared-driver-level fix without fully understanding it,
# sidestepped here at the script level: an explicit altitude removes the
# repeated-current-altitude-read path entirely, since it was verified in
# this same test that a SINGLE -999-altitude call (the main GOTO_LOCATION
# below, called only once right after takeoff) is NOT affected. Worth
# revisiting device_if_rbx.py's geoid-height handling directly if any other
# script needs to safely chain multiple -999-altitude goto_locations.
GOTO_LOCATION = [-35.3632241, 149.1653332, -999, -999] # [Lat, Long, Alt WGS84, Yaw NED Frame], Enter -999 to use current value
GOTO_LOCATION_CORNERS =  [[-35.3632187,149.1651804, 613.44, -999],[-35.3632244,149.1652420, 613.44, -999],[-35.3632292,149.1651589, 613.44, -999]]

# Set Home Poistion
#
# ENABLE_FAKE_GPS was True, and that is exactly what stops this script from
# ever flying against a SITL -- same root cause already diagnosed and fixed
# in the sibling drone_follow_object_mission_script.py (2026-08-12): the
# Fake GPS app injects GPS_INPUT from HOME_LOCATION while the SITL already
# has its own simulated GPS at CMAC, thousands of km apart, both claiming
# gps_id 0. The EKF never forms a position estimate, ArduPilot answers
# "PreArm: Need Position Estimate", and LAUNCH aborts at the ARM step.
#
# Fake GPS exists for a real airframe with no GPS of its own. A SITL has
# one, so it must stay OFF here. rbx_ardupilot_node.py's
# reconcileFakeGpsApp() also auto-disables it whenever it detects a SITL,
# so leaving this True would just have the script fighting the driver.
ENABLE_FAKE_GPS = False
# SET_HOME was True, and with Fake GPS off it is actively harmful against a
# SITL -- same bug already fixed in the sibling script: a home altitude of
# 0.0 m moves the EKF's home reference to sea level while the CMAC field
# sits ~603 m AMSL, so the vehicle believes it's already ~584 m above home
# and the takeoff climb target is already below current altitude, causing
# "armed in GUIDED mode but the takeoff did not complete". A SITL spawns
# with a correct home exactly where it sits, so the right move is to leave
# it alone. HOME_LOCATION is kept below, corrected to the SITL's own CMAC
# location including its real ~603 m AMSL field elevation (identical value
# to drone_follow_object_mission_script.py's), so flipping SET_HOME back on
# for a real fake-GPS deployment does not silently reintroduce the
# sea-level bug.
SET_HOME = False
HOME_LOCATION = [-35.3632621,149.1652374,603.44]

# Goto Error Settings
GOTO_MAX_ERROR_M = 2.0 # Goal reached when all translation move errors are less than this value
GOTO_MAX_ERROR_DEG = 2.0 # Goal reached when all rotation move errors are less than this value
GOTO_STABILIZED_SEC = 1.0 # Window of time that setpoint error values must be good before proceeding

# CMD Timeout Values
CMD_STATE_TIMEOUT_SEC = 5
CMD_MODE_TIMEOUT_SEC = 5
# Both raised from 20 -- confirmed live in the sibling script that SITL plus
# Gazebo on a loaded VM runs slower than realtime (a 10 m takeoff climb took
# ~20 s wall-clock), so a 20 s budget lands right on the timeout and scores
# an actually-succeeding action as a failure. 60/40 leave real headroom.
CMD_ACTION_TIMEOUT_SEC = 60
CMD_GOTO_TIMEOUT_SEC = 40

#########################################
# Node Class
#########################################

class drone_inspection_demo_mission(object):

  settings_update =  dict(
    takeoff_height_m = {"type":"Float","name":"takeoff_height_m","value":str(TAKEOFF_HEIGHT_M)}
  )

  rbx_info = DeviceRBXInfo()
  rbx_status = DeviceRBXStatus()
  rbx_settings = None
  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "drone_inspection_demo_mission" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    self.msg_if = MsgIF(log_name = self.node_name)
    self.msg_if.pub_info("Starting Initialization Processes")
    ##############################
    self.msg_if.pub_info("Waiting for namespace containing: " + RBX_ROBOT_NAME)
    robot_namespace = nepi_ros.wait_for_node(RBX_ROBOT_NAME)
    robot_namespace = robot_namespace + "/"
    self.msg_if.pub_info("Found namespace: " + robot_namespace)
    rbx_namespace = (robot_namespace + "rbx/")
    self.msg_if.pub_info("Using rbx namesapce " + rbx_namespace)
    self.rbx_initialize(rbx_namespace)
    # Registered as soon as the rbx_* publishers cleanup_actions() needs
    # actually exist. This mission normally reaches post_mission_actions()'s
    # own RTL on its own linear path, but that never runs if the RUI stops
    # this script mid-mission -- same gap found and fixed in the sibling
    # drone_follow_object_mission_script.py (cleanup_actions() previously
    # existed but was never wired to anything, so a mid-mission stop could
    # leave the vehicle armed/flying with nothing supervising it).
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

    ##############################
    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    ##############################

    #########################################
    # Run Pre-Mission Custom Actions
    self.msg_if.pub_info("Starting Mission Actions")
    success = self.pre_mission_actions()
    if success:
      #########################################
      # Start Mission
      #########################################
      # Send goto Location Command
      self.msg_if.pub_info("Starting Mission Processes")
      success = self.mission()
      #########################################
    # End Mission
    #########################################
    # Run Post-Mission Actions
    self.msg_if.pub_info("Starting Post-Goto Actions")
    success = self.post_mission_actions()
    nepi_ros.sleep(10,100)
    #########################################
    #Mission Complete, Shutting Down
    rospy.signal_shutdown("Mission Complete, Shutting Down")


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
    # namespace with NO trailing slash, so "base + 'app_fake_gps/'" would build
    # the topic "/nepi/device1app_fake_gps/enable" -- same bug found and fixed
    # in the sibling drone_follow_object_mission_script.py. Dormant here too
    # (ENABLE_FAKE_GPS is False), fixed anyway so re-enabling it for a real
    # fake-GPS deployment doesn't silently publish to a topic nothing subscribes to.
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

  def goto_rbx_location(self, goto_data, timeout_sec=10):
    self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
    time.sleep(0.1)
    if len(goto_data) == 4:
      ready = self.wait_for_rbx_status_ready(timeout_sec)
      if ready:
        self.msg_if.pub_info("Starting goto Location Global Process")
        goto_msg = GotoLocation()
        goto_msg.lat = goto_data[0]
        goto_msg.long = goto_data[1]
        goto_msg.altitude_meters = goto_data[2]
        goto_msg.yaw_deg = goto_data[3]
        self.rbx_goto_location_pub.publish(goto_msg)
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
    time.sleep(2)
    error_str = str(self.rbx_status.errors_current)
    if success:
      self.msg_if.pub_info("Takeoff completed with errors: " + error_str )
    else:
      self.msg_if.pub_info("Takeoff failed with errors: " + error_str )
    nepi_ros.sleep(2,10)
    ###########################
    # Stop Your Custom Actions
    ###########################
    self.msg_if.pub_info("Pre-Mission Actions Complete")
    return success

  ## Function for custom mission
  def mission(self):
    ###########################
    # Start Your Custom Process
    ###########################
    success = True
    ##########################################
    # Send goto Location Command
    self.msg_if.pub_info("Starting goto Location Process")
    success = self.goto_rbx_location(GOTO_LOCATION,timeout_sec =CMD_GOTO_TIMEOUT_SEC)
    error_str = str(self.rbx_status.errors_current)
    if success:
      self.msg_if.pub_info("Goto Location completed with errors: " + error_str )
    else:
      self.msg_if.pub_info("Goto Location failed with errors: " + error_str )
    nepi_ros.sleep(2,10)
    #########################################
    # Run Mission Actions
    self.msg_if.pub_info("Starting Mission Actions")
    success = self.mission_actions()
   #########################################
    # Send goto Location Loop Command
    for ind in range(3):
      # Send goto Location Command
      self.msg_if.pub_info("Starting goto Location Corners Process")
      success = self.goto_rbx_location(GOTO_LOCATION_CORNERS[ind],timeout_sec =CMD_GOTO_TIMEOUT_SEC)
      error_str = str(self.rbx_status.errors_current)
      if success:
        self.msg_if.pub_info("Goto Location Corner " + str(ind) + " completed with errors: " + error_str )
      else:
        # Confirmed live 2026-08-27: ArduPilot SITL can climb well past a
        # correctly and continuously re-sent altitude setpoint after a
        # TAKEOFF->GUIDED handoff, causing this goto to time out here
        # instead of silently reporting success. A fresh re-issue of the
        # same goto occasionally breaks that stuck condition, so retry once
        # before moving on rather than compounding the error into the next
        # corner uncorrected.
        self.msg_if.pub_info("Goto Location Corner " + str(ind) + " failed with errors: " + error_str + " -- retrying once")
        success = self.goto_rbx_location(GOTO_LOCATION_CORNERS[ind],timeout_sec =CMD_GOTO_TIMEOUT_SEC)
        error_str = str(self.rbx_status.errors_current)
        if success:
          self.msg_if.pub_info("Goto Location Corner " + str(ind) + " retry completed with errors: " + error_str )
        else:
          self.msg_if.pub_info("Goto Location Corner " + str(ind) + " retry failed with errors: " + error_str + " -- proceeding anyway")
      # Run Mission Actions
      self.msg_if.pub_info("Starting Mission Actions")
      success = self.mission_actions()

    ###########################
    # Stop Your Custom Process
    ###########################
    self.msg_if.pub_info("Mission Processes Complete")
    return success

  ## Function for custom mission actions
  def mission_actions(self):
    ###########################
    # Start Your Custom Actions
    ###########################
    ## Send Snapshot Trigger
    success = True
    success = self.set_rbx_process_name("SNAPSHOT EVENT")
    self.msg_if.pub_info("Sending snapshot event trigger")
    self.snapshot()
    nepi_ros.sleep(2,10)
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
    #success = self.set_rbx_mode("LAND", timeout_sec =CMD_MODE_TIMEOUT_SEC) # Uncomment to change to Land mode
    #success = self.set_rbx_mode("LOITER", timeout_sec =CMD_MODE_TIMEOUT_SEC) # Uncomment to change to Loiter mode
    success = self.set_rbx_mode("RTL", timeout_sec =CMD_MODE_TIMEOUT_SEC) # Uncomment to change to home mode
    #success = self.set_rbx_mode("RESUME", timeout_sec =CMD_MODE_TIMEOUT_SEC) # Uncomment to return to last mode
    nepi_ros.sleep(1,10)
    ###########################
    # Stop Your Custom Actions
    ###########################
    self.msg_if.pub_info("Post-Mission Actions Complete")
    return success

  #######################
  # Mission Action Functions

  ### Function to send snapshot event trigger and wait for completion
  def snapshot(self):
    self.snapshot_trigger_pub.publish(Empty())
    self.msg_if.pub_info("Snapshot trigger sent")

  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    # Best-effort safety net for a mid-mission stop (e.g. via the RUI) --
    # post_mission_actions() already does this on the script's own normal
    # completion path, so only act here if that never got a chance to run
    # (vehicle still reports ARM).
    try:
      if self.rbx_info is not None and self.rbx_cap_states:
        armed_ind = -1
        for ind, state in enumerate(self.rbx_cap_states):
          if state == "ARM":
            armed_ind = ind
        if armed_ind != -1 and self.rbx_info.state == armed_ind:
          self.msg_if.pub_info("Vehicle still armed on shutdown -- sending RTL")
          self.set_rbx_mode("RTL", timeout_sec = CMD_MODE_TIMEOUT_SEC)
    except Exception as e:
      self.msg_if.pub_warn("RTL on cleanup failed: " + str(e))

#########################################
# Main
#########################################
if __name__ == '__main__':
  drone_inspection_demo_mission()
