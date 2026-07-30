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
# What's left for this node to do is much smaller: just relay whichever of
# the two onboard camera topics matches the currently-selected view mode.
# View mode is configured on the remote NEPI device as an rbx_sim RBX
# setting (camera_view_mode); rbx_sim_node.py pushes it over the existing
# bridge TCP connection to sim_bridge_node.py, which sets it as a plain ROS
# param on this VM's local master (/sim/camera/view_mode) -- the cleanest
# path since the two machines have separate ROS masters and this node has
# no visibility into the remote device's process memory otherwise.
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

FIRST_PERSON_IMAGE_TOPIC = '/rover/camera/image_raw'
THIRD_PERSON_IMAGE_TOPIC = '/rover/camera_chase/image_raw'

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

# Param namespace sim_bridge_node.py writes into on receipt of a
# camera_settings line from the remote device's rbx_sim_node.py. The
# offset params (offset_x/y/z) that used to live alongside this are gone --
# the camera offsets are now fixed joint poses baked into
# generic_rover/model.sdf, not a runtime setting, since a rigid attachment
# can't have a freely-adjustable offset without either driving an actuator
# or going back to the laggy kinematic-follow approach this change removes.
PARAM_VIEW_MODE = '/sim/camera/view_mode'

# Matches rbx_sim_node.py's FACTORY_SETTINGS default -- fallback if the
# bridge hasn't pushed anything yet, e.g. right after a (re)connect.
DEFAULT_VIEW_MODE = 'FIRST_PERSON'


class CameraRigController:

  def __init__(self):
    rospy.init_node(NODE_NAME)
    rospy.loginfo(PKG_NAME + ": Starting Node Initialization Processes")

    self.bridge = CvBridge()

    self.image_lock = threading.Lock()
    self.latest_first_person_img = None
    self.latest_third_person_img = None

    self.compressed_pub = rospy.Publisher('/camera_rig/image_compressed',
                                          CompressedImage, queue_size = 1)

    self.first_person_sub = rospy.Subscriber(FIRST_PERSON_IMAGE_TOPIC, Image,
                                             self.firstPersonImageCb)
    self.third_person_sub = rospy.Subscriber(THIRD_PERSON_IMAGE_TOPIC, Image,
                                             self.thirdPersonImageCb)

    self.image_timer = rospy.Timer(rospy.Duration(1.0 / IMAGE_RATE_HZ), self.imagePublishCb)

    rospy.loginfo(PKG_NAME + ": Camera view relay initialized")
    rospy.loginfo(PKG_NAME + ": Relaying " + FIRST_PERSON_IMAGE_TOPIC + " / " +
                  THIRD_PERSON_IMAGE_TOPIC + " per " + PARAM_VIEW_MODE)

  def run(self):
    """Block until ROS shutdown, servicing the image relay timer."""
    rospy.spin()

  def firstPersonImageCb(self, msg):
    self.storeImage(msg, is_third_person = False)

  def thirdPersonImageCb(self, msg):
    self.storeImage(msg, is_third_person = True)

  def storeImage(self, msg, is_third_person):
    try:
      cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
    except Exception as e:
      rospy.logwarn_throttle(5.0, PKG_NAME + ": Image conversion failed: " + str(e))
      return
    with self.image_lock:
      if is_third_person:
        self.latest_third_person_img = cv_img
      else:
        self.latest_first_person_img = cv_img

  def imagePublishCb(self, timer_event):
    view_mode = rospy.get_param(PARAM_VIEW_MODE, DEFAULT_VIEW_MODE)
    with self.image_lock:
      cv_img = self.latest_third_person_img if view_mode == 'THIRD_PERSON' else self.latest_first_person_img
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
    self.compressed_pub.publish(msg)


#########################################
# Main
#########################################

if __name__ == '__main__':
  node = CameraRigController()
  node.run()
