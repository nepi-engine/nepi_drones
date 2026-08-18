#!/usr/bin/env python3
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

# Camera view relay (Universal Simulator Bridge camera feature).
#
# Both the first-person and chase cameras are now rigid links welded
# directly onto generic_rover itself (camera_link / camera_link_chase in
# generic_rover/model.sdf) -- there is no more standalone "rig" model to
# reposition. Previously (see git history) this node drove a separate
# camera_rig model at 20 Hz via /gazebo/set_model_state, computing a
# look-at pose from the rover's latest odometry every tick; that had an
# inherent lag, since the camera pose update, the odometry update, and the
# rover's own physics update were three independent async loops instead of
# one physics step. A fixed joint has no such lag -- both cameras move in
# lockstep with the rover, in the same physics step, for free.
#
# What's left for this node to do is much smaller: relay BOTH onboard camera
# topics simultaneously, each on its own compressed topic. This used to relay
# only whichever one matched a currently-selected camera_view_mode RBX
# setting (rbx_sim_node.py pushed it here via sim_bridge_node.py, which set
# it as a plain ROS param this node polled every publish tick) -- reworked
# (2026-08-18) per the same live report that made both the "third-person view
# doesn't really exist" and "only one instance" complaints about the
# quadcopter driver: a single topic that gets reassigned content depending on
# a mode setting isn't a second view a client can rely on, it's the same
# topic sometimes showing something else. Both feeds are now always live,
# each its own honestly-named topic -- rbx_sim_node.py exposes them as two
# separate ROS Image topics, and the existing Image Source dropdown's
# find_topics_by_msg('Image') discovery picks up both with no new RUI
# plumbing needed (see NepiDeviceRBX.js's createImageOptions).
#
# Image compression happens here, not in sim_bridge_node.py: this node is
# already Gazebo/cv2-facing (it subscribes the onboard cameras' raw
# images), and ros-noetic-compressed-image-transport (which would have
# given a free .../compressed topic from the gazebo_ros_camera plugin) is
# not installed on this VM; explicit cv2.imencode keeps this
# dependency-free and gives direct control over quality/rate rather than
# assuming a system package is present.

import threading

import cv2
import rospy

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

PKG_NAME = 'CAMERA_RIG_CONTROLLER'
NODE_NAME = 'camera_rig_controller'

ROBOT_VIEW_IMAGE_TOPIC = '/rover/camera/image_raw'
SCENE_VIEW_IMAGE_TOPIC = '/rover/camera_chase/image_raw'

# The NEPI device receiving this feed is a Raspberry Pi (NEPI_HW_TYPE=rpi):
# every frame that arrives here still has to be JPEG-decoded and republished
# as a raw Image by rbx_sim_node.py, then JPEG-RE-encoded again by
# web_video_server for the browser (see streamingImageQuality/
# streamingImageRate in NepiDeviceRBX.js, which turned out to be the
# dominant cost -- that hop runs live, per viewer, at whatever quality/rate
# it's given regardless of these numbers). Kept moderate rather than pushed
# further, so this stage doesn't add its own bottleneck on top of that one.
IMAGE_RATE_HZ = 10.0
JPEG_QUALITY = 65

# Camera offsets (offset_x/y/z / scene_offset_x/y/z) are applied by
# sim_bridge_node.py respawning generic_rover/model.sdf with new camera
# <pose> values -- not this node's concern; it only ever relays whatever
# frames arrive on the two subscribed topics, regardless of their current pose.


class CameraRigController:

  def __init__(self):
    rospy.init_node(NODE_NAME)
    rospy.loginfo(PKG_NAME + ": Starting Node Initialization Processes")

    self.bridge = CvBridge()

    self.image_lock = threading.Lock()
    self.latest_robot_view_img = None
    self.latest_scene_view_img = None

    self.robot_view_pub = rospy.Publisher('/camera_rig/robot_view/image_compressed',
                                          CompressedImage, queue_size = 1)
    self.scene_view_pub = rospy.Publisher('/camera_rig/scene_view/image_compressed',
                                          CompressedImage, queue_size = 1)

    self.robot_view_sub = rospy.Subscriber(ROBOT_VIEW_IMAGE_TOPIC, Image,
                                           self.robotViewImageCb)
    self.scene_view_sub = rospy.Subscriber(SCENE_VIEW_IMAGE_TOPIC, Image,
                                           self.sceneViewImageCb)

    self.image_timer = rospy.Timer(rospy.Duration(1.0 / IMAGE_RATE_HZ), self.imagePublishCb)

    rospy.loginfo(PKG_NAME + ": Camera view relay initialized")
    rospy.loginfo(PKG_NAME + ": Relaying " + ROBOT_VIEW_IMAGE_TOPIC + " and " +
                  SCENE_VIEW_IMAGE_TOPIC + " simultaneously, each its own topic")

  def run(self):
    """Block until ROS shutdown, servicing the image relay timer."""
    rospy.spin()

  def robotViewImageCb(self, msg):
    self.storeImage(msg, is_scene_view = False)

  def sceneViewImageCb(self, msg):
    self.storeImage(msg, is_scene_view = True)

  def storeImage(self, msg, is_scene_view):
    try:
      cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
    except Exception as e:
      rospy.logwarn_throttle(5.0, PKG_NAME + ": Image conversion failed: " + str(e))
      return
    with self.image_lock:
      if is_scene_view:
        self.latest_scene_view_img = cv_img
      else:
        self.latest_robot_view_img = cv_img

  def imagePublishCb(self, timer_event):
    with self.image_lock:
      robot_img = self.latest_robot_view_img
      scene_img = self.latest_scene_view_img
    self.encodeAndPublish(robot_img, self.robot_view_pub)
    self.encodeAndPublish(scene_img, self.scene_view_pub)

  def encodeAndPublish(self, cv_img, pub):
    if cv_img is None:
      return
    ok, encoded = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
      rospy.logwarn_throttle(5.0, PKG_NAME + ": JPEG encode failed")
      return
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = 'jpeg'
    msg.data = encoded.tobytes()
    pub.publish(msg)


#########################################
# Main
#########################################

if __name__ == '__main__':
  node = CameraRigController()
  node.run()
