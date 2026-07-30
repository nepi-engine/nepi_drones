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

# Camera-rig follow controller, ArduPilot SITL port (Universal Simulator
# Bridge camera feature). New file, not an edit to the rover's
# camera_rig_controller.py: the pose source, the vehicle's motion (full 3D,
# not planar), and the network path are all materially different here, per
# this project's convention of a separate file per distinct
# simulator/workflow (see camera_rig_controller_multi.py for the same
# reasoning applied to the rover's multi-robot port).
#
# Differences from the rover version:
#   - Pose source: /gazebo/model_states (filtered for the "iris_demo" model),
#     not /rover/odom -- ArduPilot SITL has no ROS-native odom topic of its
#     own on this VM; Gazebo's ground-truth model state is the only pose feed
#     available here (confirmed working, including real roll/pitch, by
#     direct test while the SITL vehicle was armed and flying -- see the
#     session summary).
#   - Vehicle motion is full 3D (multirotor), not planar -- the drone's
#     altitude varies, so cam_z now tracks drone_z + offset_z rather than a
#     fixed offset_z as the rover used (rover never left z=0).
#   - FIRST_PERSON is yaw-only / gimbal-stabilized (camera stays level
#     regardless of airframe roll/pitch), not rigidly slaved to the full
#     airframe attitude. Chosen because NEPI's target use cases (inspection,
#     survey -- the VideoRay/OceanAero/WESMAR field deployments the platform
#     is built around) match real commercial drones' 3-axis gimbals, not FPV
#     racing rigs: a camera that banked/pitched with every stabilization
#     twitch would be a poor default for an inspection data product. It is
#     also the cheapest extension of the rover's own FIRST_PERSON semantic
#     (cam_yaw = vehicle_yaw, pitch/roll = 0) -- same formula, now reading a
#     real quaternion's yaw component instead of assuming pitch/roll were
#     already zero (true for the flat rover, not for a multirotor). The
#     rigidly-slaved alternative is a legitimate convention too (nose-mounted
#     FPV camera) but is not built here -- see the session summary for the
#     full reasoning and how cheaply it could be added later (a per-request
#     stabilized/unstabilized toggle) if ever needed.
#   - THIRD_PERSON's look-at is extended to real 3D: pitch is computed from
#     the real altitude difference (drone_z - cam_z), not assumed zero.
#   - Bridge: no separate sim_bridge_node.py exists for this workflow (the
#     ArduPilot driver's only other channel is raw MAVLink, which already
#     carries telemetry/commands and has no camera channel at all) so this
#     node runs its own minimal TCP JSON-lines server directly, combining the
#     roles the rover version split across two processes/files. Settings
#     applied directly to local instance state (no ROS-param handoff needed
#     -- there is no second process here to hand off to). Port 9026 (next
#     free slot in the 902x sim-utility block after the rover's 9021-9025);
#     forwarded by nepi_tunnel in nepi_sitl_dev_env.sh.

import base64
import json
import math
import socket
import threading
import time

import cv2
import rospy

from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelState, ModelStates
from cv_bridge import CvBridge

PKG_NAME = 'CAMERA_RIG_CONTROLLER_ARDUPILOT'
NODE_NAME = 'camera_rig_controller_ardupilot'

MODEL_STATES_TOPIC = '/gazebo/model_states'
VEHICLE_MODEL_NAME = 'iris_demo'
IMAGE_TOPIC = '/camera_rig/camera/image_raw'
MODEL_STATE_TOPIC = '/gazebo/set_model_state'
CAMERA_MODEL_NAME = 'camera_rig'

# Pose-follow update rate: matches the rover controller's own rationale --
# smooth relative to the ~1-10 Hz MAVLink telemetry rate elsewhere in this
# project, and the camera pose is purely visual so a higher rate costs little.
CONTROL_RATE_HZ = 20.0
# Modest frame rate/quality per the same tunnel-bandwidth convention used by
# every prior phase of this feature.
IMAGE_RATE_HZ = 7.0
JPEG_QUALITY = 60

# Camera bridge TCP server: next free port in the 902x sim-utility block
# (9021 gz_reset, 9022-9025 rover heartbeat/bridge slots). Loopback-only,
# reached from the remote NEPI device solely through nepi_tunnel's reverse
# forward (see nepi_sitl_dev_env.sh).
BRIDGE_PORT = 9026

# Matches rbx_ardupilot_node.py's FACTORY_SETTINGS for these (kept in sync by
# eye -- both sides fall back to the same values before the first
# camera_settings line arrives, e.g. right after a (re)connect). Forward and
# slightly below the body: a nose/belly-mounted inspection-camera convention,
# distinct from the rover's flat mount point since this is a multirotor.
DEFAULT_VIEW_MODE = 'FIRST_PERSON'
DEFAULT_OFFSET_X = 0.15
DEFAULT_OFFSET_Y = 0.0
DEFAULT_OFFSET_Z = -0.1


class CameraRigControllerArdupilot:

  def __init__(self):
    rospy.init_node(NODE_NAME)
    rospy.loginfo(PKG_NAME + ": Starting Node Initialization Processes")

    self.bridge = CvBridge()

    self.pose_lock = threading.Lock()
    self.drone_x = 0.0
    self.drone_y = 0.0
    self.drone_z = 0.0
    self.drone_yaw = 0.0
    self.have_pose = False

    self.settings_lock = threading.Lock()
    self.view_mode = DEFAULT_VIEW_MODE
    self.offset_x = DEFAULT_OFFSET_X
    self.offset_y = DEFAULT_OFFSET_Y
    self.offset_z = DEFAULT_OFFSET_Z

    self.image_lock = threading.Lock()
    self.latest_cv_img = None

    self.client_lock = threading.Lock()
    self.client_conn = None

    self.state_pub = rospy.Publisher(MODEL_STATE_TOPIC, ModelState, queue_size = 1)

    self.model_states_sub = rospy.Subscriber(MODEL_STATES_TOPIC, ModelStates, self.modelStatesCb)
    self.image_sub = rospy.Subscriber(IMAGE_TOPIC, Image, self.imageCb)

    self.control_timer = rospy.Timer(rospy.Duration(1.0 / CONTROL_RATE_HZ), self.controlCb)
    self.image_timer = rospy.Timer(rospy.Duration(1.0 / IMAGE_RATE_HZ), self.imagePublishCb)

    self.server_thread = threading.Thread(target = self.bridgeServerLoop)
    self.server_thread.daemon = True
    self.server_thread.start()

    rospy.loginfo(PKG_NAME + ": Camera rig controller initialized")
    rospy.loginfo(PKG_NAME + ": Following " + VEHICLE_MODEL_NAME + " (via " +
                  MODEL_STATES_TOPIC + ") -> " + CAMERA_MODEL_NAME +
                  " via " + MODEL_STATE_TOPIC)
    rospy.loginfo(PKG_NAME + ": Camera bridge server on 127.0.0.1:" + str(BRIDGE_PORT))

  def run(self):
    """Block until ROS shutdown, servicing the control/image timers and the
    bridge server thread."""
    rospy.spin()

  def modelStatesCb(self, msg):
    try:
      idx = msg.name.index(VEHICLE_MODEL_NAME)
    except ValueError:
      return
    pos = msg.pose[idx].position
    q = msg.pose[idx].orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    with self.pose_lock:
      self.drone_x = pos.x
      self.drone_y = pos.y
      self.drone_z = pos.z
      self.drone_yaw = yaw
      self.have_pose = True

  def imageCb(self, msg):
    try:
      cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
    except Exception as e:
      rospy.logwarn_throttle(5.0, PKG_NAME + ": Image conversion failed: " + str(e))
      return
    with self.image_lock:
      self.latest_cv_img = cv_img

  def controlCb(self, timer_event):
    with self.pose_lock:
      if not self.have_pose:
        return
      drone_x = self.drone_x
      drone_y = self.drone_y
      drone_z = self.drone_z
      drone_yaw = self.drone_yaw

    with self.settings_lock:
      view_mode = self.view_mode
      off_x = self.offset_x
      off_y = self.offset_y
      off_z = self.offset_z

    # Rotate the body-frame offset into the drone's current yaw only (not its
    # full 3D attitude) so the rig's position doesn't jitter with small
    # roll/pitch stabilization oscillations -- only the aim direction differs
    # by view mode, matching a real gimbal mount's decoupled position.
    cos_y = math.cos(drone_yaw)
    sin_y = math.sin(drone_yaw)
    world_dx = off_x * cos_y - off_y * sin_y
    world_dy = off_x * sin_y + off_y * cos_y

    cam_x = drone_x + world_dx
    cam_y = drone_y + world_dy
    cam_z = drone_z + off_z

    if view_mode == 'THIRD_PERSON':
      # Chase-cam: real look-at (yaw AND pitch) toward the drone's current
      # position, extended to 3D via the real altitude difference.
      dx = drone_x - cam_x
      dy = drone_y - cam_y
      dz = drone_z - cam_z
      horiz_dist = math.hypot(dx, dy)
      cam_yaw = math.atan2(dy, dx)
      cam_pitch = math.atan2(dz, horiz_dist) if horiz_dist > 1e-6 else 0.0
    else:
      # FIRST_PERSON: yaw-only, gimbal-stabilized -- stays level regardless
      # of the airframe's own roll/pitch (see module docstring for why).
      cam_yaw = drone_yaw
      cam_pitch = 0.0

    qx, qy, qz, qw = self.eulerToQuat(0.0, cam_pitch, cam_yaw)

    state = ModelState()
    state.model_name = CAMERA_MODEL_NAME
    state.pose.position.x = cam_x
    state.pose.position.y = cam_y
    state.pose.position.z = cam_z
    state.pose.orientation.x = qx
    state.pose.orientation.y = qy
    state.pose.orientation.z = qz
    state.pose.orientation.w = qw
    state.reference_frame = 'world'
    self.state_pub.publish(state)

  def eulerToQuat(self, roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw

  def imagePublishCb(self, timer_event):
    with self.image_lock:
      cv_img = self.latest_cv_img
    if cv_img is None:
      return
    ok, encoded = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
      rospy.logwarn_throttle(5.0, PKG_NAME + ": JPEG encode failed")
      return
    line = {
      'type': 'image',
      'data': base64.b64encode(encoded.tobytes()).decode('ascii'),
      'stamp': rospy.Time.now().to_sec(),
    }
    self.sendLineToClient(line)

  def applyCameraSettings(self, cmd):
    with self.settings_lock:
      self.view_mode = str(cmd.get('view_mode', DEFAULT_VIEW_MODE))
      self.offset_x = float(cmd.get('offset_x', DEFAULT_OFFSET_X))
      self.offset_y = float(cmd.get('offset_y', DEFAULT_OFFSET_Y))
      self.offset_z = float(cmd.get('offset_z', DEFAULT_OFFSET_Z))

  def sendLineToClient(self, line_dict):
    with self.client_lock:
      conn = self.client_conn
    if conn is None:
      return
    try:
      conn.sendall((json.dumps(line_dict) + '\n').encode())
    except Exception as e:
      rospy.logwarn_throttle(5.0, PKG_NAME + ": Failed to send line to client: " + str(e))
      with self.client_lock:
        if self.client_conn is conn:
          self.client_conn = None
      try:
        conn.close()
      except Exception:
        pass

  def bridgeServerLoop(self):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # rospy sets a process-global socket.setdefaulttimeout(60) on
    # init_node(), which accept() applies to every accepted connection. The
    # settings side of this channel is legitimately idle for long stretches
    # (settings change rarely), so a recv timeout here must not be treated as
    # client death -- clear it and block instead; a real disconnect still
    # unblocks recv with EOF, and the 7 Hz image send loop independently
    # detects a dead client via its own sendall failure.
    srv.settimeout(None)
    srv.bind(('127.0.0.1', BRIDGE_PORT))
    srv.listen(1)
    while not rospy.is_shutdown():
      try:
        conn, _ = srv.accept()
        conn.settimeout(None)
      except Exception:
        continue
      rospy.loginfo(PKG_NAME + ": Bridge client connected")
      with self.client_lock:
        self.client_conn = conn
      self.serveClient(conn)
      with self.client_lock:
        if self.client_conn is conn:
          self.client_conn = None
      try:
        conn.close()
      except Exception:
        pass
      rospy.loginfo(PKG_NAME + ": Bridge client disconnected")

  def serveClient(self, conn):
    buf = b''
    while not rospy.is_shutdown():
      try:
        data = conn.recv(4096)
      except Exception as e:
        rospy.logwarn(PKG_NAME + ": Bridge client recv error: " + repr(e))
        return
      if not data:
        rospy.loginfo(PKG_NAME + ": Bridge client closed connection (EOF)")
        return
      buf += data
      while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        if not line.strip():
          continue
        try:
          cmd = json.loads(line)
        except Exception as e:
          rospy.logwarn(PKG_NAME + ": Bad bridge command line: " + str(e))
          continue
        if cmd.get('type') == 'camera_settings':
          self.applyCameraSettings(cmd)
        else:
          rospy.logwarn_throttle(5.0, PKG_NAME + ": Unrecognized bridge line type: " +
                                 str(cmd.get('type')))


#########################################
# Main
#########################################

if __name__ == '__main__':
  node = CameraRigControllerArdupilot()
  node.run()
