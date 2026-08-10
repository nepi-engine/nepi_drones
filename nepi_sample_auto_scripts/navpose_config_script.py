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
# 1. Connect NEPI NavPose base_frame components to appropriate driver source topics
# 2. (See KNOWN GAP below regarding GPS clock sync)
#
# Updated for current NEPI Engine API (2026-07): nepi_ros_interfaces -> nepi_interfaces,
# nepi_msg module -> nepi_api.messages_if.MsgIF.
#
# ARCHITECTURE CHANGE (confirmed by reading navpose_mgr.py, src/nepi_engine/nepi_managers/
# scripts/navpose_mgr.py, in full): nav_pose_mgr was renamed navpose_mgr, and its old
# per-robot "point NEPI at a source topic" interface (set_gps_fix_topic/
# set_heading_topic/set_orientation_topic/enable_gps_clock_sync) no longer exists at
# all. It was replaced by a multi-frame transform system. One navpose frame exists by
# default out of the box: NAVPOSE_BASE_FRAME = "base_frame" (see
# NavPoseMgr.navposes_init_frames / NAVPOSE_BASE_FRAME). Each frame has named
# "components" (location, heading, orientation, position, altitude, depth, pan_tilt),
# and each component can be pointed at a live ROS topic via the generic
# navpose_mgr/set_frame_comp_topic topic (nepi_interfaces/UpdateString: name=frame_name,
# name2=comp_name, name3='init'|'update'|'offset'|'reset', value=topic_name -- confirmed
# via NavPoseMgr._setFrameCompTopicCb / setFrameCompTopic).
#
# This script now uses set_frame_comp_topic against the "base_frame" frame as the
# direct current-API equivalent of the old set_gps_fix_topic/set_orientation_topic/
# set_heading_topic calls:
#   - GPS fix topic  -> comp_name="location",    type_name="update"
#   - Odom topic     -> comp_name="orientation", type_name="update" (the old script only
#                        ever wired the odom topic to the orientation setter, never to
#                        position, even though Odometry carries both -- that mapping is
#                        preserved here as-is rather than "improved")
#   - Heading topic  -> comp_name="heading",     type_name="update"
# "update" (continuous re-integration) is used rather than "init" (one-shot/periodic
# reference reset) because the old script's intent was to continuously track a live
# driver output topic, not just seed a one-time reference -- "update" is the current
# mechanism for that (see NavPoseMgr.setFrameCompTopic's 'update' branch).
#
# IMPORTANT CAVEAT (confirmed in navpose_mgr.py's setFrameCompTopic() and
# _updateAvailTopicsCb()): a set_frame_comp_topic call only takes effect if the target
# topic (a) is currently live on the ROS graph and (b) publishes one of the message
# types navpose_mgr recognizes for that component -- checked on a 5-second rescan cycle
# against nepi_sdk/nepi_nav.py's NAVPOSE_MSG_DICT:
#   location:    nepi_interfaces/NavPoseLocation, sensor_msgs/NavSatFix, geographic_msgs/GeoPoint
#   orientation: nepi_interfaces/NavPoseOrientation, nav_msgs/Odometry, geometry_msgs/Pose, geometry_msgs/Quaternion
#   heading:     nepi_interfaces/NavPoseHeading only
# If your heading source topic is not a NavPoseHeading message, navpose_mgr will
# silently ignore the set_frame_comp_topic call for it (the topic never appears in its
# avail_topics_dict, so setFrameCompTopic no-ops) -- no error is raised anywhere. Verify
# your driver's actual heading message type against the list above before relying on
# this script to wire it up.
#
# KNOWN GAP (not fixed by this pass): the old SYNC_NEPI_CLOCK / enable_gps_clock_sync
# behavior -- syncing NEPI's system clock to a GPS-fix message's embedded timestamp --
# has no current equivalent. Reading time_mgr.py (src/nepi_engine/nepi_managers/
# scripts/time_mgr.py) in full: the only clock-sync mechanism left in the current
# engine is auto_sync_clocks, which is NTP/Chrony-based (syncs to network time
# servers), not GPS-fix-topic based -- there is no code path anywhere in navpose_mgr.py
# or time_mgr.py that reads a timestamp off a nav topic and steps the system clock from
# it. This is an architecture removal, not a rename, so no replacement call is made
# here; the old SYNC_NEPI_CLOCK setting and its set_gps_timesync_pub publisher have
# been removed entirely rather than guessed at.

import rospy
import time
from nepi_sdk import nepi_ros
from nepi_api.messages_if import MsgIF

from nepi_interfaces.msg import UpdateString


#########################################
# USER SETTINGS - Edit as Necessary
#########################################

# Set NEPI NavPose Source Topics Names, or Enter "" to Ignore
NEPI_NAVPOSE_SOURCE_GPS_TOPIC = "rbx/gps_fix"  # Enter "" to Ignore
NEPI_NAVPOSE_SOURCE_ODOM_TOPIC = "rbx/odom" # Enter "" to Ignore
NEPI_NAVPOSE_SOURCE_HEADING_TOPIC = "rbx/heading" # Enter "" to Ignore

# NavPose frame to wire these source topics into -- "base_frame" is the one frame that
# exists by default in navpose_mgr (see NavPoseMgr.NAVPOSE_BASE_FRAME / navposes_init_frames)
NEPI_NAVPOSE_FRAME_NAME = "base_frame"


#########################################
# Node Class
#########################################

class navpose_config(object):

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "navpose_config" # Can be overwitten by luanch command
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
    NEPI_SET_FRAME_COMP_TOPIC = self.base_namespace + "navpose_mgr/set_frame_comp_topic"
    ## Define Class Services Calls
    self.msg_if.pub_info("Waiting for topic: " + NEPI_SET_FRAME_COMP_TOPIC)
    nepi_ros.wait_for_topic(NEPI_SET_FRAME_COMP_TOPIC)
    self.set_frame_comp_topic_pub = rospy.Publisher(NEPI_SET_FRAME_COMP_TOPIC, UpdateString, queue_size=1)
    # GPS Topic -> location component source
    self.gps_topic = None
    if NEPI_NAVPOSE_SOURCE_GPS_TOPIC != "":
      self.msg_if.pub_info("Waiting for topic: " + NEPI_NAVPOSE_SOURCE_GPS_TOPIC)
      self.gps_topic = nepi_ros.wait_for_topic(NEPI_NAVPOSE_SOURCE_GPS_TOPIC)
    # Odom Topic -> orientation component source (matches original script's own wiring)
    self.odom_topic = None
    if NEPI_NAVPOSE_SOURCE_ODOM_TOPIC != "":
      self.msg_if.pub_info("Waiting for topic: " + NEPI_NAVPOSE_SOURCE_ODOM_TOPIC)
      self.odom_topic = nepi_ros.wait_for_topic(NEPI_NAVPOSE_SOURCE_ODOM_TOPIC)
    # Heading Topic -> heading component source
    self.heading_topic = None
    if NEPI_NAVPOSE_SOURCE_HEADING_TOPIC != "":
      self.msg_if.pub_info("Waiting for topic: " + NEPI_NAVPOSE_SOURCE_HEADING_TOPIC)
      self.heading_topic = nepi_ros.wait_for_topic(NEPI_NAVPOSE_SOURCE_HEADING_TOPIC)
    ##############################
    self.msg_if.pub_info("Setup complete")
    ## Create Class Sevices
    ## Create Class Publishers
    ## Start Class Subscribers
    ## Start Node Processes
    self.msg_if.pub_info("Starting set navpose topics timer callback")
    rospy.Timer(rospy.Duration(5.0), self.set_nepi_navpose_topics_callback)

    ##############################
    ## Initiation Complete
    self.msg_if.pub_info(" Initialization Complete")
    # Spin forever (until object is detected)
    rospy.spin()
    ##############################

  #######################
  ### Node Methods


  ### Callback to set NEPI navpose base_frame component source topics
  def set_nepi_navpose_topics_callback(self,timer):
    if self.gps_topic is not None:
      # Set Location Component Topic
      update_msg = UpdateString()
      update_msg.name = NEPI_NAVPOSE_FRAME_NAME
      update_msg.name2 = "location"
      update_msg.name3 = "update"
      update_msg.value = self.gps_topic
      self.set_frame_comp_topic_pub.publish(update_msg)
      self.msg_if.pub_info("Location Topic Set to: " + self.gps_topic)
    if self.odom_topic is not None:
      # Set Orientation Component Topic
      update_msg = UpdateString()
      update_msg.name = NEPI_NAVPOSE_FRAME_NAME
      update_msg.name2 = "orientation"
      update_msg.name3 = "update"
      update_msg.value = self.odom_topic
      self.set_frame_comp_topic_pub.publish(update_msg)
      self.msg_if.pub_info("Orientation Topic Set to: " + self.odom_topic)
    if self.heading_topic is not None:
      # Set Heading Component Topic
      update_msg = UpdateString()
      update_msg.name = NEPI_NAVPOSE_FRAME_NAME
      update_msg.name2 = "heading"
      update_msg.name3 = "update"
      update_msg.value = self.heading_topic
      self.set_frame_comp_topic_pub.publish(update_msg)
      self.msg_if.pub_info("Heading Topic Set to: " + self.heading_topic)


    #######################
    # Node Cleanup Function

  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  navpose_config()

