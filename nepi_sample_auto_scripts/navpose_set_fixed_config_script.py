#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


# Sample NEPI Config Script.
# If your NEPI system does not have an attached GPS/IMU/Compass or other
# NavPose source, this script can be set to run at startup setting fixed
# NavPose values on your system.
# 1. Sets a fixed NavPose Solution (Lat,Long,Alt,Heading,Roll,Pitch,Yaw)
#
# Updated for current NEPI Engine API (2026-07): nepi_msg module -> nepi_api.messages_if.MsgIF.
#
# nav_pose_mgr was renamed to navpose_mgr AND its whole topic architecture changed. The old
# manager exposed flat "point at a source topic" setters (set_init_gps_fix / set_init_heading /
# set_init_orientation, plus a reinit_solution trigger) -- none of these exist anymore. The
# current navpose_mgr (read in full: src/nepi_engine/nepi_managers/scripts/navpose_mgr.py)
# instead manages a dict of named NavPose "frames", each with a connect_dict of components
# (location, heading, orientation, position, altitude, depth, pan_tilt), where each component's
# init/update/offset/reset stage points at either a live source topic or the literal string
# 'Fixed'. When a component's init_topic is set to 'Fixed', navpose_mgr continuously applies that
# frame's stored fixed-navpose values for that component on every publish cycle (see
# updateNavposesData() / setFrameCompTopic() in navpose_mgr.py) instead of listening to a topic --
# there is no more one-shot "reinit" step needed; the fixed values are live-applied for as long as
# the component stays set to 'Fixed'.
#
# This script targets the reserved 'base_frame' frame (NavPoseMgr.NAVPOSE_BASE_FRAME) by default,
# which always exists at startup and cannot be renamed or deleted (see the renameNavpose()/
# removeNavpose() guards in navpose_mgr.py) -- it is the frame the rest of the system reads as its
# primary navpose solution, and is the direct current-API equivalent of "the system's" navpose
# used by the old script. The port:
#   1) Publishes one UpdateNavPose message to navpose_mgr/set_frame_fixed_navpose with the fixed
#      lat/long/altitude/heading/roll/pitch/yaw values for the target frame.
#   2) Publishes UpdateString messages to navpose_mgr/set_frame_comp_topic for the location,
#      altitude, heading, and orientation components of that frame, each with value 'Fixed', so
#      the frame actually uses the fixed data above instead of any live source topic.
#
# NavPose.msg carries orientation directly as roll_deg/pitch_deg/yaw_deg (float32, degrees) rather
# than a quaternion -- confirmed by reading nepi_interfaces/msg/NavPose.msg and
# nepi_nav.convert_navpose_dict2msg()/convert_navpose_msg2dict() in nepi_sdk/nepi_nav.py. This
# removes the need for the old nepi_nav.convert_rpy2quat() + geometry_msgs/QuaternionStamped path
# entirely, so nepi_nav and tf are no longer imported here -- not because convert_rpy2quat changed
# (it did not; it's confirmed unchanged) but because the current fixed-navpose API no longer takes
# a quaternion at all.

import rospy
import time
import sys
from nepi_sdk import nepi_ros
from nepi_api.messages_if import MsgIF

from nepi_interfaces.msg import UpdateNavPose, UpdateString, NavPose

#########################################
# USER SETTINGS - Edit as Necessary
#########################################

# Set Start Fixed NavPose Values
#Numurus Office
START_GEOPOINT = [47.6540828,-122.3187578,0.0] # [Lat, Long, Altitude_AMSL_M]
START_HEADING_DEG = 88.0 # Global True North, or 0 for Body Relative
START_ORIENTATION_DEGS = [10.0,20.0,30.0]

# navpose_mgr frame these fixed values are applied to. 'base_frame' is the reserved default
# frame -- always present at startup, cannot be renamed or deleted -- and is what the rest of
# the system treats as its primary navpose solution. This matches the old script's behavior of
# setting "the system's" navpose. Only change this if targeting a different, custom frame that
# has already been created via navpose_mgr's add_frame topic.
NAVPOSE_TARGET_FRAME = "base_frame"


#########################################
# Node Class
#########################################

class navpose_set_fixed_config(object):

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "navpose_set_fixed_config" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    self.msg_if = MsgIF(log_name = self.node_name)
    self.msg_if.pub_info("Starting Initialization Processes")
    ##############################
    ## Initialize Class Variables
    ## Define Class Namespaces
    NAVPOSE_MGR_NAMESPACE = self.base_namespace + "navpose_mgr/"
    SET_FRAME_FIXED_NAVPOSE_TOPIC = NAVPOSE_MGR_NAMESPACE + "set_frame_fixed_navpose"
    SET_FRAME_COMP_TOPIC_TOPIC = NAVPOSE_MGR_NAMESPACE + "set_frame_comp_topic"
    ## Define Class Services Calls
    ## Create Class Sevices
    ## Create Class Publishers
    ## Start Class Subscribers
    ## Start Node Processes
    self.msg_if.pub_info("Waiting for topic: " + NAVPOSE_MGR_NAMESPACE + "status")
    nepi_ros.wait_for_topic(NAVPOSE_MGR_NAMESPACE + "status")

    # Make sure to use the correct message type: "rostopic info" can help identify it.
    fixed_navpose_pub = rospy.Publisher(SET_FRAME_FIXED_NAVPOSE_TOPIC, UpdateNavPose, queue_size=1)
    rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe
    comp_topic_pub = rospy.Publisher(SET_FRAME_COMP_TOPIC_TOPIC, UpdateString, queue_size=5)
    rospy.sleep(1) # VERY IMPORTANT - Sleep a bit between declaring a publisher and using it subscribers have time to subscribe

    lat = START_GEOPOINT[0]
    long = START_GEOPOINT[1]
    alt = START_GEOPOINT[2]
    heading = START_HEADING_DEG
    # Orientation is native roll/pitch/yaw degrees in the current NavPose message -- no
    # quaternion conversion needed (see module docstring).
    roll = START_ORIENTATION_DEGS[0]
    pitch = START_ORIENTATION_DEGS[1]
    yaw = START_ORIENTATION_DEGS[2]

    fixed_navpose_msg = NavPose()
    fixed_navpose_msg.has_location = True
    fixed_navpose_msg.latitude = lat
    fixed_navpose_msg.longitude = long
    fixed_navpose_msg.has_altitude = True
    fixed_navpose_msg.altitude_m = alt
    fixed_navpose_msg.has_heading = True
    fixed_navpose_msg.heading_deg = heading
    fixed_navpose_msg.has_orientation = True
    fixed_navpose_msg.roll_deg = roll
    fixed_navpose_msg.pitch_deg = pitch
    fixed_navpose_msg.yaw_deg = yaw

    self.msg_if.pub_info("Setting fixed navpose values for frame: " + NAVPOSE_TARGET_FRAME)
    fixed_navpose_pub.publish(name = NAVPOSE_TARGET_FRAME, navpose = fixed_navpose_msg)

    # At this point the frame's fixed-navpose values are stored, but the frame's components are
    # not yet pointed at them. Point the location/altitude/heading/orientation components' init
    # stage at 'Fixed' so the frame actually applies the fixed values set above.
    rospy.sleep(1) # Give the fixed navpose values time to get captured
    for comp_name in ["location", "altitude", "heading", "orientation"]:
      comp_topic_pub.publish(name = NAVPOSE_TARGET_FRAME, name2 = comp_name, name3 = "init", value = "Fixed")
      rospy.sleep(0.2)
    self.msg_if.pub_info("Fixed navpose components set to 'Fixed' for frame: " + NAVPOSE_TARGET_FRAME)

    ##############################
    ## Initiation Complete
    self.msg_if.pub_info(" Initialization Complete")
    # Spin forever (until object is detected)
    rospy.spin()
    ##############################

  #######################
  ### Node Methods


  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  navpose_set_fixed_config()
