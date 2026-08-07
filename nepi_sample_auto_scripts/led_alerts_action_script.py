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
# 3. Takes actions based on whether the object of interest is currently detected
#
# Updated for current NEPI Engine API (2026-07): nepi_ros_interfaces -> nepi_interfaces,
# LSXStatus -> DeviceLSXStatus (field names unchanged), LSXCapabilitiesQuery unchanged,
# nepi_msg module -> nepi_api.messages_if.MsgIF. The nepi_app_ai_alerts.msg import
# (AiAlertsStatus, AiAlerts) was dead code -- neither class was referenced anywhere in this
# script -- and has been removed.
#
# RE-PORTED (2026-08-06) against the real current architecture -- this script's whole
# purpose previously depended on an "app_ai_alerts" app publishing a
# base_namespace/app_ai_alerts/alert_state Bool topic, which never existed in this
# workspace's nepi_apps (only fake_gps, file_pub_img, file_pub_vid, image_viewer, onvif_mgr,
# pan_tilt_auto, nav_sim) and blocked here forever waiting for it. Rather than invent a
# fake stand-in app, this now derives the exact same True/False "alert" signal from the
# real current AI detection output: any enabled model (see ai_detector_config_script.py)
# publishes <base_namespace>/all/detections; "alert" is simply "is OBJECT_LABEL_OF_INTEREST
# present in the latest message" -- the same source led_adjust_on_object_detect_action_script.py's
# own re-port uses, just collapsed to a boolean here instead of an intensity ramp. See that
# script's module docstring for the full detail on why this is the correct current
# replacement, not a workaround.
#
# RE-PORTED AGAIN (2026-08-06, later same day) -- the previous re-port above still targeted
# nepi_interfaces/AiBoundingBoxes, which does not exist in this checkout (confirmed: no
# AiBoundingBoxes.msg anywhere in nepi_interfaces) -- it was removed by the controls-pipeline
# refactor that replaced it with nepi_interfaces/Detections before this script was ever
# actually run against it. The real aggregated feed (confirmed by reading
# nepi_api/node_if_ai_detector.py) is <base_namespace>/all/detections
# (nepi_interfaces/Detections, field .detections is a Detection[]), and each entry's class
# name is .name, not .Class. Fixed below; this script needs no image dimensions (unlike
# led_adjust_on_object_detect_action_script.py) so the fix is just the import/topic/field
# rename, nothing structural.

import os
#### ROS namespace setup
#NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1] # remove to run as automation script
import rospy
import time
import sys
from nepi_sdk import nepi_ros
from nepi_api.messages_if import MsgIF

from std_msgs.msg import Empty, Int8, UInt8, UInt32, Int32, Bool, String, Float32, Float64
from nepi_interfaces.msg import DeviceLSXStatus, Detections
from nepi_interfaces.srv import LSXCapabilitiesQuery, LSXCapabilitiesQueryResponse


#########################################
# USER SETTINGS - Edit as Necessary
#########################################

LED_LEVEL_MAX = 0.3 # 0-1 ratio
LED_LEVEL_STEP = 0.05 # 0-1 ratio
LED_STEP_SEC = 1.0

#Set LED Control ROS Topic Name (or partial name)
LED_STATUS_TOPIC_NAME = "lsx/status"

# Object class name that triggers the "alert true" LED actions below. Requires a model
# already enabled and running (see ai_detector_config_script.py) publishing this class on
# <base_namespace>/all/detections.
OBJECT_LABEL_OF_INTEREST = "bottle"
# Consecutive missed detection messages allowed before flipping back to "alert false" --
# same debounce idea as led_adjust_on_object_detect_action_script.py's LOST_COUNT_THRESHOLD.
ALERT_LOST_COUNT_THRESHOLD = 5

# Start and Alert Actions List [on_off_state,intensity_val,blink_state,blink_time_sec,color_string] Use -999 to ignore (not set)
# States or actions for unsupported capabilities will be ignored
START_STATE = [True,0.2,False,0.0,"GREEN"]
ALERT_TRUE_ACTIONS = [True,0.2,True,0.5,"RED"]
ALERT_FALSE_ACTIONS = [True,0.4,False,-999,"GREEN"]

#########################################
# Node Class
#########################################

class led_alert_actions(object):

  led_on_off_pub = None
  led_intensity_pub = None
  led_color_pub = None
  led_blink_interval_pub = None
  led_on_off_pub = None

  led_state = None
  last_led_state = None

  alert_state = False
  last_alert_state = None

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "led_alert_acations" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    self.msg_if = MsgIF(log_name = self.node_name)
    self.msg_if.pub_info("Starting Initialization Processes")
    ##############################
    ## Initialize Class Variables
    ## Connect to LED
    led_status_topic_name = LED_STATUS_TOPIC_NAME
    self.msg_if.pub_info("Waiting for status topic name: " + led_status_topic_name)
    led_status_topic=nepi_ros.wait_for_topic(led_status_topic_name)
    self.msg_if.pub_info("Found status topic: " + led_status_topic)
    rospy.Subscriber(led_status_topic, DeviceLSXStatus, self.ledStatusCb, queue_size = 1)
    self.msg_if.pub_info("Waiting for status msg to publish")
    while self.led_state == None:
      time.sleep(1)
    led_namespace = led_status_topic.split('/lsx')[0]
    # Get LED device capabilities
    led_capabilities_service_topic = os.path.join( led_namespace,"lsx/capabilities_query")
    try:
      led_caps_service = rospy.ServiceProxy(led_capabilities_service_topic, LSXCapabilitiesQuery)
      time.sleep(1)
      led_caps = led_caps_service()
      self.has_standby_mode = led_caps.has_standby_mode
      self.has_on_off_control = led_caps.has_on_off_control
      self.has_intensity_control = led_caps.has_intensity_control
      self.has_color_control = led_caps.has_color_control
      self.color_options_list = led_caps.color_options_list
      self.has_kelvin_control = led_caps.has_kelvin_control
      self.kelvin_min = led_caps.kelvin_min
      self.kelvin_max = led_caps.kelvin_max
      self.has_blink_control = led_caps.has_blink_control
    except Exception as e:
      self.msg_if.pub_warn("Failed to call led capabilities service: " + led_capabilities_service_topic + " " + str(e))
      self.has_standby_mode = False
      self.has_on_off_control = False
      self.has_intensity_control = False
      self.has_color_control = False
      self.has_blink_control = False
      self.color_options_list = ["None"]
      self.has_kelvin_control = False
      self.kelvin_min = 0
      self.kelvin_max = 1
    # Check for action controls support
    if  self.has_on_off_control == False and self.has_intensity_control == False \
        and self.has_color_control == False and self.has_blink_control == False:
        rospy.signal_shutdown("LED has no capabilities need for actions, Shutting Down")
    else:
      ## Create Class Publishers
      if self.has_on_off_control == True:
        self.led_on_off_pub = rospy.Publisher(os.path.join(led_namespace,"lsx/turn_on_off"), Bool, queue_size = 1)
      if self.has_intensity_control == True:
        self.led_intensity_pub = rospy.Publisher(os.path.join(led_namespace,"lsx/set_intensity_ratio"), Float32, queue_size = 1)
      if self.has_blink_control == True:
        self.led_blink_on_off_pub = rospy.Publisher(os.path.join(led_namespace,"lsx/blink_on_off"), Bool, queue_size = 1)
        self.led_blink_interval_pub = rospy.Publisher(os.path.join(led_namespace,"lsx/set_blink_interval"), Float32, queue_size = 1)
      if self.has_color_control == True:
        self.led_color_pub = rospy.Publisher(os.path.join(led_namespace,"lsx/set_color"), String, queue_size = 1)
      time.sleep(1)
      # Initialize LED
      self.msg_if.pub_info("Initializaing LED state")
      self.updateLedState(START_STATE)

      ## Connect to AI detection output -- see module docstring for why this replaces the
      ## old, never-existent app_ai_alerts app
      self.lost_count = ALERT_LOST_COUNT_THRESHOLD + 1  # Start in the "no alert" state
      detections_topic = os.path.join(self.base_namespace,"all","detections")
      self.msg_if.pub_info("Waiting for detections topic: " + detections_topic)
      found_topic = nepi_ros.wait_for_topic(detections_topic)
      if found_topic == "":
        self.msg_if.pub_warn("Detections topic not found yet -- continuing anyway, " +
                              "will start reacting once a detector is enabled and publishing")
      else:
        self.msg_if.pub_info("Found detections topic: " + found_topic)
      rospy.Subscriber(detections_topic, Detections, self.detectionsCb, queue_size = 1)

    ##############################
    ## Initiation Complete
    self.msg_if.pub_info(" Initialization Complete")
    # Spin forever (until object is detected)
    rospy.spin()
    ##############################


  def updateLedState(self,new_led_state):
      if new_led_state != self.last_led_state:
        if self.led_on_off_pub != None and new_led_state[0] != -999:
          self.led_on_off_pub.publish(new_led_state[0])
        if self.led_intensity_pub != None and new_led_state[1] != -999:
          self.led_intensity_pub.publish(new_led_state[1])
        if self.led_blink_on_off_pub and new_led_state[2] != -999:
          self.led_blink_on_off_pub.publish(new_led_state[2])
        if self.led_blink_interval_pub != None and new_led_state[3] != -999:
          self.led_blink_interval_pub.publish(new_led_state[3])
        if self.led_color_pub != None and new_led_state[4] != -999:
          self.led_color_pub.publish(new_led_state[4])
      self.last_led_state = new_led_state

  def ledStatusCb(self,msg):
    self.led_state = [msg.on_off_state,msg.intensity_ratio,msg.blink_state,msg.blink_interval,msg.color_setting]
    if self.last_led_state == None:
      self.last_led_state = [msg.on_off_state,msg.intensity_ratio,msg.blink_state,msg.blink_interval,msg.color_setting]

  def detectionsCb(self,detections_msg):
    # Alert is True exactly when OBJECT_LABEL_OF_INTEREST appears in this message; debounced
    # over ALERT_LOST_COUNT_THRESHOLD consecutive misses the same way
    # led_adjust_on_object_detect_action_script.py's own re-port debounces its lost_count,
    # so a couple of dropped detection frames don't flicker the LED actions on and off.
    found = False
    for det in detections_msg.detections:
      if det.name == OBJECT_LABEL_OF_INTEREST:
        found = True
        break
    if found:
      self.lost_count = 0
      self.alert_state = True
    else:
      self.lost_count += 1
      if self.lost_count > ALERT_LOST_COUNT_THRESHOLD:
        self.alert_state = False

    if self.alert_state != self.last_alert_state or self.last_alert_state == None:
      self.msg_if.pub_info("Alert State updated to: " + str(self.alert_state))
      if self.alert_state == True:
        self.updateLedState(ALERT_TRUE_ACTIONS)
      else:
        self.updateLedState(ALERT_FALSE_ACTIONS)
    self.last_alert_state = self.alert_state


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    self.updateLedState(START_STATE)


#########################################
# Main
#########################################
if __name__ == '__main__':
  led_alert_actions()

