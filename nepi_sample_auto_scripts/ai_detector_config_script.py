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
# 1. Checks if AI input image topic exists
# 2. Enables the model's AI framework, then the model itself, via ai_models_mgr
# 3. Points the now-running detection node at the input image topic and starts detection
# 4. Stops AI detection process on shutdown

# Updated for current NEPI Engine API (2026-07): nepi_ros_interfaces -> nepi_interfaces,
# nepi_msg module -> nepi_api.messages_if.MsgIF.
#
# RE-PORTED (2026-08-06) against the real current architecture -- previously left as a
# non-functional reference pending a replacement mechanism; that mechanism was found by
# reading ai_models_mgr.py, nepi_api/ai_if_detector.py, and an installed framework adapter
# (nepi_ai_frameworks/nepi_aif_yolov8/api/aif_yolov8_if.py) in full:
#
#   - ai_detector_mgr's old single-call start_classifier/stop_classifier (ClassifierSelection)
#     is gone -- ai_models_mgr splits the same job into two steps instead:
#       1) ai_models_mgr/update_framework_state (UpdateBool: name=<framework>, value=True) --
#          enables the named AI framework (e.g. "yolov8"). A model's update_model_state call
#          is silently ignored until its framework is enabled this way first (confirmed by
#          reading updateModelStateCb).
#       2) ai_models_mgr/update_model_state (UpdateBool: name=<model_name>, value=True) --
#          enables the named model, which is what actually launches its detection node
#          (aif_class.launchModel(model_dict) -> nepi_sdk.nepi_aifs.launchModelNode()).
#   - The launched detection node's own name is the model's display_name (confirmed in
#     nepi_aifs.py's launchModelNode/getModelsDict), launched directly under the base
#     namespace (ai_models_mgr passes launch_namespace = self.base_namespace) -- so
#     nepi_ros.wait_for_node(DETECTION_MODEL) finds it, exactly like every RBX driver script
#     in this folder already does for its own robot node.
#   - Every detection node hosts the same generic nepi_api.ai_if_detector.AiDetectorIF
#     interface (confirmed by reading it in full) -- this is what actually accepts a
#     caller-supplied image topic, replacing the old ClassifierSelection.img_topic field:
#       <node>/set_img_topic (std_msgs/String) -- point this instance's detector at a topic
#       <node>/set_threshold (std_msgs/Float32) -- replaces the old detection_threshold field
#       <node>/enable         (std_msgs/Bool)    -- starts/stops detection on that topic
#   - DETECTION_MODEL below must be the display_name of a model actually installed under
#     /mnt/nepi_storage/ai_models/<framework>/ on the device (see AI_FRAMEWORK_NAME) --
#     "darknet_common_object_detection_fast" was the old darknet-based sample value and no
#     longer means anything; check what's actually installed (RUI's AI Models panel, or
#     `rosservice call .../ai_models_mgr/model_status_query`) before relying on this script.

import time
import sys
import rospy
from nepi_sdk import nepi_ros
from nepi_api.messages_if import MsgIF

from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, Empty, String, Bool, Float32
from nepi_interfaces.msg import UpdateBool, StringArray

#########################################
# USER SETTINGS - Edit as Necessary
#########################################

#Set AI Detector Image ROS Topic Name or Partial Name
IMAGE_INPUT_TOPIC_NAME = "color_2d_image"

# Fallback searched alongside IMAGE_INPUT_TOPIC_NAME above (not instead of it) if that one
# isn't found -- "idx/color_image" is a real IDX camera driver's own topic convention
# (device_if_idx.py), distinct from the sim/RBX relay convention IMAGE_INPUT_TOPIC_NAME
# defaults to. Lets this script find either a simulated or a real camera without editing.
IMAGE_INPUT_TOPIC_FALLBACK = "idx/color_image"

# Set AI Framework + Model. AI_FRAMEWORK_NAME must be an installed framework's name (e.g.
# "yolov8", "yolov11", "yolo26", "hailo" -- see src/nepi_ai_frameworks/). DETECTION_MODEL
# must be a model display_name actually installed under that framework's model folder on
# the device -- there is no factory-default model shipped with the engine, so this MUST be
# changed to match something real on your device before this script can do anything.
AI_FRAMEWORK_NAME = "yolov8"
DETECTION_MODEL = "CHANGE_ME_to_an_installed_model_display_name"
DETECTION_THRESHOLD = 0.5

# How long to wait for the model's detection node to come up after enabling it
MODEL_LAUNCH_TIMEOUT_SEC = 30


def find_image_topic(candidates, timeout = 10):
  # Polls nepi_ros.find_topic() across all candidates each tick (rather than trying each
  # one for a full `timeout` in turn), so total worst-case wait is still just `timeout`,
  # not timeout * len(candidates).
  start_time = time.time()
  while (time.time() - start_time) < timeout and not nepi_ros.is_shutdown():
    for candidate in candidates:
      found = nepi_ros.find_topic(candidate)
      if found != "":
        return found
    time.sleep(0.1)
  return ""


#########################################
# Node Class
#########################################

class ai_detector_config(object):

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "ai_detector_config" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    self.msg_if = MsgIF(log_name = self.node_name)
    self.msg_if.pub_info("Starting Initialization Processes")
    ##############################
    ## Define Class Namespaces
    AI_MGR_NAMESPACE = self.base_namespace + "ai_models_mgr/"
    UPDATE_FRAMEWORK_STATE_TOPIC = AI_MGR_NAMESPACE + "update_framework_state"
    UPDATE_MODEL_STATE_TOPIC = AI_MGR_NAMESPACE + "update_model_state"

    ## Create Class Publishers
    self.update_framework_state_pub = rospy.Publisher(UPDATE_FRAMEWORK_STATE_TOPIC, UpdateBool, queue_size=1)
    self.update_model_state_pub = rospy.Publisher(UPDATE_MODEL_STATE_TOPIC, UpdateBool, queue_size=1)
    time.sleep(1)  # Give the publishers a moment to connect before the first publish

    ## Confirm the input image topic actually exists -- log-only, doesn't block: the
    ## detector node will keep watching for it once pointed at it below, exactly like
    ## every other image-consuming script in this folder.
    self.msg_if.pub_info("Checking for input image topic: " + IMAGE_INPUT_TOPIC_NAME +
                          " (or " + IMAGE_INPUT_TOPIC_FALLBACK + ")")
    image_topic = find_image_topic([IMAGE_INPUT_TOPIC_NAME, IMAGE_INPUT_TOPIC_FALLBACK], timeout = 10)
    if image_topic == "":
      self.msg_if.pub_warn("Image topic " + IMAGE_INPUT_TOPIC_NAME + " not found yet -- " +
                            "continuing anyway, the detector will pick it up once it appears")
      image_topic = IMAGE_INPUT_TOPIC_NAME
    else:
      self.msg_if.pub_info("Found image topic: " + image_topic)

    ## Enable the model's AI framework, then the model itself -- update_model_state is a
    ## no-op until the framework is active (see module docstring)
    self.msg_if.pub_info("Enabling AI framework: " + AI_FRAMEWORK_NAME)
    self.update_framework_state_pub.publish(UpdateBool(name = AI_FRAMEWORK_NAME, value = True))
    time.sleep(1)
    self.msg_if.pub_info("Enabling AI model: " + DETECTION_MODEL)
    self.update_model_state_pub.publish(UpdateBool(name = DETECTION_MODEL, value = True))

    ## Wait for the model's detection node to actually come up
    self.msg_if.pub_info("Waiting for detection node: " + DETECTION_MODEL)
    detector_node = nepi_ros.wait_for_node(DETECTION_MODEL, timeout = MODEL_LAUNCH_TIMEOUT_SEC)
    if detector_node == "":
      self.msg_if.pub_warn("Detection node " + DETECTION_MODEL + " did not come up within " +
                            str(MODEL_LAUNCH_TIMEOUT_SEC) + "s -- check that AI_FRAMEWORK_NAME/" +
                            "DETECTION_MODEL match an installed framework/model on this device " +
                            "(RUI AI Models panel). Node will idle rather than configure a " +
                            "detector that isn't running.")
      self.detector_namespace = None
    else:
      self.msg_if.pub_info("Found detection node: " + detector_node)
      self.detector_namespace = detector_node + "/"

      ## Point the running detector at the input image topic and start it
      set_img_topic_pub = rospy.Publisher(self.detector_namespace + "set_img_topic", String, queue_size=1)
      set_threshold_pub = rospy.Publisher(self.detector_namespace + "set_threshold", Float32, queue_size=1)
      self.enable_pub = rospy.Publisher(self.detector_namespace + "enable", Bool, queue_size=1)
      time.sleep(1)
      set_img_topic_pub.publish(String(data = image_topic))
      set_threshold_pub.publish(Float32(data = DETECTION_THRESHOLD))
      self.enable_pub.publish(Bool(data = True))
      self.msg_if.pub_info("Detector " + DETECTION_MODEL + " enabled on " + image_topic +
                            " at threshold " + str(DETECTION_THRESHOLD))

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
    if self.detector_namespace is not None:
      self.enable_pub.publish(Bool(data = False))
    self.update_model_state_pub.publish(UpdateBool(name = DETECTION_MODEL, value = False))



#########################################
# Main
#########################################
if __name__ == '__main__':
  ai_detector_config()
