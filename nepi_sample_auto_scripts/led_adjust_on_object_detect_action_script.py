#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample NEPI Process Script.
# 1. Waits for LED system
# 2. Waits for the NEPI AI detection bounding-boxes topic
# 3. Adjust LED level based on target location in image
#
# Updated for current NEPI Engine API (2026-07): nepi_msg module -> nepi_api.messages_if.MsgIF.
# The lsx/turn_on_off, lsx/blink_on_off, lsx/set_blink_interval topics and
# nepi_ros.find_topic/wait_for_topic calls are unchanged and left as-is.
#
# FIXED (2026-08-06): the intensity lookup searched for "lsx/set_intensity" (no "_ratio"),
# which nepi_ros.find_topic() deliberately rejects as a match against the real topic
# lsx/set_intensity_ratio -- it excludes any candidate immediately followed by "_" after the
# search string, precisely to avoid this kind of accidental substring match, but that means an
# exact-except-for-the-suffix search string never matches at all. Confirmed live: has_intensity
# stayed False and no intensity command was ever sent until this was corrected to
# "lsx/set_intensity_ratio" -- the same fix led_auto_level_process_script.py already had.
#
# RE-PORTED (2026-08-06) against the real current architecture -- this script's detection
# trigger previously depended on an "ai_detector_mgr" node publishing
# ai_detector_mgr/bounding_boxes (darknet_ros_msgs/BoundingBoxes) and
# ai_detector_mgr/found_object (darknet_ros_msgs/ObjectCount). Neither exists any more:
# ai_detector_mgr was replaced by ai_models_mgr (see ai_detector_config_script.py's own
# module docstring for the full framework/model-enable architecture change), and
# darknet_ros_msgs never existed in this workspace at all.
#
# RE-PORTED AGAIN (2026-08-06, later same day) -- the re-port above still targeted
# nepi_interfaces/AiBoundingBoxes, which does not exist in this checkout (confirmed: no
# AiBoundingBoxes.msg anywhere in nepi_interfaces) -- it was removed by the controls-pipeline
# refactor that replaced it with nepi_interfaces/Detections before this script was ever
# actually run against it. Confirmed by reading nepi_api/node_if_ai_detector.py in full:
# every detection node (regardless of framework -- yolov8, yolov11, yolo26, hailo) publishes
# an aggregated nepi_interfaces/Detections message at <base_namespace>/all/detections (each
# detector also has its own <node>/detections copy; the base_namespace one lets this script
# watch regardless of which model produced the detection). Detections.detections is a
# Detection[] with xmin/ymin/xmax/ymax (int64, same pixel meaning as before) and .name
# instead of .Class -- so the per-box matching logic below is structurally unchanged.
#
# One real gap the old AiBoundingBoxes message apparently used to paper over: neither
# Detection nor Detections carries the source image's width/height, which this script needs
# to turn a box's pixel center into a 0-1 ratio across the frame. Detections.source_topic
# does carry the exact image topic the detection came from, though, so
# updateImageDimsFromSourceCb below grabs width/height from that topic directly (once, then
# cached) instead of assuming a bounding-box message field that no longer exists.
# See ai_detector_config_script.py to actually get a model running and publishing to this
# topic in the first place -- this script only reacts to detections, it doesn't start them.

import os
import time
import sys
import rospy
import statistics
import numpy as np
from nepi_sdk import nepi_ros
from nepi_api.messages_if import MsgIF

from std_msgs.msg import Bool, Empty, Float32
from sensor_msgs.msg import Image
from nepi_interfaces.msg import Detections


#########################################
# USER SETTINGS - Edit as Necessary 
#########################################

OBJECT_LABEL_OF_INTEREST = "bottle"
LOST_COUNT_THRESHOLD = 5
LED_LEVEL_MAX = 0.3
LED_BLINK_RATE = 0.5
LED_BLINK_THRESHOLD = 0.5
WATCHDOG_TIME = 4
AVG_LENGTH = 2





#########################################
# Node Class
#########################################

class led_adjust_on_object_detect(object):

  has_intensity = False
  has_blink = False
  is_blinking = False
  set_intensity = 0
  lost_count = 0

  
  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "led_adjust_on_object_detect" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    self.msg_if = MsgIF(log_name = self.node_name)
    self.msg_if.pub_info("Starting Initialization Processes")
    ##############################
    ## Initialize Class Variables
    self.led_intensity_pub = None
    self.object_label_of_interest = OBJECT_LABEL_OF_INTEREST
    self.led_level_max = LED_LEVEL_MAX
    self.wd_timeout_sec = WATCHDOG_TIME
    self.wd_check_interval_sec = 1
    self.wd_timer = 0
    self.intensity_history = np.zeros(AVG_LENGTH)


    led_control_topic_name = "lsx/turn_on_off"
    self.msg_if.pub_info("Waiting for topic name: " + led_control_topic_name)
    led_control_topic=nepi_ros.wait_for_topic(led_control_topic_name)
    if led_control_topic != "":
      self.led_on_off_pub = rospy.Publisher(led_control_topic, Bool, queue_size = 1)

    led_control_topic_name = "lsx/set_intensity_ratio"
    self.msg_if.pub_info("Looking for topic name: " + led_control_topic_name)
    led_control_topic=nepi_ros.find_topic(led_control_topic_name)
    if led_control_topic != "":
      self.has_intensity = True
      self.led_intensity_pub = rospy.Publisher(led_control_topic, Float32, queue_size = 1)

    led_control_topic_name = "lsx/blink_on_off"
    self.msg_if.pub_info("Looking for topic name: " + led_control_topic_name)
    led_control_topic=nepi_ros.find_topic(led_control_topic_name)
    if led_control_topic != "":
      self.has_blink = True
      self.led_blink_on_off_pub = rospy.Publisher(led_control_topic, Bool, queue_size = 1)
      led_control_topic = led_control_topic.replace("blink_on_off","set_blink_interval")
      self.led_blink_interval_pub = rospy.Publisher(led_control_topic, Float32, queue_size = 1)


    if self.has_intensity or self.has_blink:
      time.sleep(1)
      if not rospy.is_shutdown():
        if self.has_intensity:
          self.led_intensity_pub.publish(data = 0)
        if self.has_blink:
          self.led_blink_on_off_pub.publish(False)
        self.led_on_off_pub.publish(True)
          


      self.img_width = 0 # Updated from the detection's own source image topic, once
      self.img_height = 0 # (see updateImageDimsFromSourceCb / module docstring)
      self.object_detected = False
      ## Define Class Namespaces
      # Aggregated AI detection output -- see module docstring for why this is one topic now
      AI_DETECTIONS_TOPIC = os.path.join(self.base_namespace, "all", "detections")
      ## Class subscribers

      # Wait for the AI detections topic to publish -- requires a model already enabled
      # and running (see ai_detector_config_script.py)
      self.msg_if.pub_info("Waiting for topic: " + AI_DETECTIONS_TOPIC)
      found_topic = nepi_ros.wait_for_topic(AI_DETECTIONS_TOPIC)
      if found_topic == "":
        self.msg_if.pub_warn("Detections topic not found yet -- continuing anyway, " +
                              "will start reacting once a detector is enabled and publishing")
      else:
        self.msg_if.pub_info("Found topic: " + found_topic)
      ## Start Class Subscribers
      # Set up object detector subscriber -- one message now carries what the old
      # BoundingBoxes + ObjectCount pair split across two topics (see module docstring)
      self.msg_if.pub_info("Starting object detection subscriber: Object of interest = " + self.object_label_of_interest + "...")
      rospy.Subscriber(AI_DETECTIONS_TOPIC, Detections, self.object_detected_callback, queue_size = 1)
      ## Start Node Processes
      # Setup LED process
      self.msg_if.pub_info("Setting up LED timer")
      rospy.Timer(rospy.Duration(self.wd_check_interval_sec), self.led_timer_callback)

      ##############################
      ## Initiation Complete
      self.msg_if.pub_info(" Initialization Complete")
      # Spin forever (until object is detected)
      rospy.spin()
      ##############################



  #######################
  ### Node Methods

  # One-shot fetch of the source image's width/height, cached thereafter -- Detections
  # carries source_topic but not the frame dimensions themselves (see module docstring).
  # A short timeout so a slow/dead image topic degrades to "skip ratio math this frame"
  # rather than blocking the whole detections callback.
  def updateImageDims(self, image_topic):
    if not image_topic:
      return
    try:
      img_msg = rospy.wait_for_message(image_topic, Image, timeout = 2.0)
      self.img_width = img_msg.width
      self.img_height = img_msg.height
    except Exception as e:
      self.msg_if.pub_warn("Failed to get image dimensions from " + str(image_topic) + ": " + str(e))

  # Action upon detection of object of interest
  def object_detected_callback(self,detections_msg):
    self.wd_timer = 0
    if self.img_width == 0 or self.img_height == 0:
      self.updateImageDims(detections_msg.source_topic)
    object_detected = False
    # Iterate over all of the objects reported by the detector -- Detections carries
    # every currently-detected box in one message (replaces the old separate ObjectCount
    # message; see module docstring), so "found" is just "did any box match this callback"
    for box in detections_msg.detections:
      # Check for the object of interest and take appropriate actions -- skip ratio math if
      # dimensions aren't known yet (updateImageDims above just failed or hasn't run)
      if box.name == self.object_label_of_interest and self.img_width > 0 and self.img_height > 0:
        box_of_interest=box
        #nepi_msg.publishMsgInfo(box_of_interest.Class)
        # Calculate the box center in image ratio terms
        object_loc_y_pix = box_of_interest.ymin + ((box_of_interest.ymax - box_of_interest.ymin)  / 2)
        object_loc_x_pix = box_of_interest.xmin + ((box_of_interest.xmax - box_of_interest.xmin)  / 2)
        object_loc_y_ratio = float(object_loc_y_pix) / self.img_height
        object_loc_x_ratio = float(object_loc_x_pix) / self.img_width
        #nepi_msg.publishMsgInfo("Object Detected " + self.object_label_of_interest + " with box center (" + str(object_loc_x_ratio) + ", " + str(object_loc_y_ratio) + ")")
        # check if we are AIose enough to center in either dimension to stop motion: Hysteresis band
        box_abs_error_x_ratio = 2.0 * abs(object_loc_x_ratio - 0.5)
        box_abs_error_y_ratio = 2.0 * abs(object_loc_y_ratio - 0.5)
        #nepi_msg.publishMsgInfo("Object Detection Error Ratios Horz: " "%.2f" % (box_abs_error_x_ratio) + " Vert: " + "%.2f" % (box_abs_error_y_ratio))
        # Sending LED level update
        center_ratios = [1-box_abs_error_x_ratio] # ignore vertical
        mean_center_ratio = statistics.mean(center_ratios)
        #self.msg_if.pub_info("Target center ratio: " + "%.2f" % (mean_center_ratio))
        intensity = self.led_level_max *  mean_center_ratio**4
        self.intensity_history = np.roll(self.intensity_history,1)
        self.intensity_history[0]=intensity
        self.set_intensity = np.mean(self.intensity_history)
        if mean_center_ratio > LED_BLINK_THRESHOLD:
          self.set_blink_interval = LED_BLINK_RATE
        else:
          self.set_blink_interval = 0
        object_detected = True
        break  # Only need the first matching box per message

    # Lost-count debounce: once per message, not once per non-matching box (the old
    # per-box loop here double-counted on any frame with multiple detected objects).
    # A miss doesn't immediately clear self.object_detected -- LOST_COUNT_THRESHOLD
    # consecutive misses are allowed first, so a couple of dropped detection frames
    # don't flicker the LED off and back on.
    if object_detected:
      self.lost_count = 0
      self.object_detected = True
    else:
      self.lost_count += 1
      if self.lost_count > LOST_COUNT_THRESHOLD:
        self.object_detected = False


  ### Setup a regular background scan process based on timer callback
  def led_timer_callback(self,timer):
    # Called periodically no matter what as a Timer object callback
    self.msg_if.pub_warn("LED timer: " + str(self.wd_timer))
    if self.wd_timer > self.wd_timeout_sec:
      self.msg_if.pub_info("Past timeout time, turning lights off")
      if self.has_intensity:
        self.led_intensity_pub.publish(data = 0)
      if self.has_blink and self.is_blinking == True:
        self.led_blink_on_off_pub.publish(False)
        self.is_blinking = False
      self.led_on_off_pub.publish(False)
    else:
      self.wd_timer += self.wd_check_interval_sec
      #self.led_on_off_pub.publish(True)
      if self.object_detected:
        if not rospy.is_shutdown():
          #self.led_on_off_pub.publish(True)
          if self.has_intensity:
            self.msg_if.pub_info("Setting intensity level to: " + "%.2f" % (self.set_intensity))
            self.led_intensity_pub.publish(data = self.set_intensity)
          self.msg_if.pub_info("Have blink interval of: " + "%.2f" % (self.set_blink_interval))
          if self.has_blink and self.set_blink_interval > 0:
            if self.is_blinking == False:
              self.msg_if.pub_info("Setting blink interval to: " + "%.2f" % (self.set_blink_interval))
              self.led_blink_on_off_pub.publish(True)
              self.led_blink_interval_pub.publish(data = self.set_blink_interval)
              self.is_blinking = True
          else:
            self.led_blink_on_off_pub.publish(False)
            self.is_blinking = False
      elif not rospy.is_shutdown():
          if self.has_intensity:
            self.led_intensity_pub.publish(data = 0)
          if self.has_blink and self.is_blinking == True:
            self.led_blink_on_off_pub.publish(False)
            self.is_blinking = False




  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    global led_intensity_pub
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    self.led_intensity_pub.publish(data = 0)



#########################################
# Main
#########################################
if __name__ == '__main__':
  led_adjust_on_object_detect()


