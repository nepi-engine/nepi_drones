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

# Simulator-side bridge entry point (Universal Simulator Bridge, Phase 1/3a).
# Runs against this dev VM's own local roscore -- it is a plain ROS node with
# no nepi_sdk dependency, since the NEPI SDK is not installed on the sim VM.
# Publishes a liveness heartbeat, relays NEPI-namespace velocity commands
# to the Gazebo diff-drive plugin's topic, and (Phase 3a) serves the
# cross-machine command/telemetry bridge on a plain TCP port: the remote NEPI
# device's rbx_sim_node.py cannot see this VM's ROS graph (separate masters,
# only raw TCP ports survive the reverse SSH tunnel), so it connects here
# instead. Protocol is newline-delimited JSON both ways on one persistent
# connection: commands in ({"linear_x", "angular_z"} -> /nepi/sim/cmd_vel,
# feeding the existing relay), odometry out (pushed at a fixed rate from the
# latest /rover/odom -- push, not poll, keeps the client a bare line reader
# and avoids a tunnel round-trip per sample).
#
# Camera-rover feature addition: two more line shapes on the same socket,
# distinguished from the above by key presence rather than a mandatory "type"
# tag (kept backward compatible with the already-verified velocity/telemetry
# shapes above, which carry none):
#   in  -- {"type":"camera_settings","view_mode":...} from rbx_sim_node.py's
#           settings mechanism. This VM has no visibility into the remote
#           device's process memory, so the only way camera_rig_controller.py
#           (a separate local ROS node) learns the current view mode is via
#           this line -> a plain ROS param on this VM's own local master
#           (rospy get_param/set_param, not a new message type -- simplest
#           mechanism available to two plain scripts with no nepi_sdk).
#           Offsets are no longer part of this shape: both camera views are
#           now rigid links welded onto generic_rover itself (fixed joint
#           poses in generic_rover/model.sdf), not a repositionable rig, so
#           there is nothing left to configure at runtime but which one is
#           active.
#   out -- {"type":"image","data":"<base64 jpeg>","stamp":...} relayed
#           straight through from camera_rig_controller.py's own
#           /camera_rig/image_compressed topic (it owns the Gazebo-facing
#           compression/throttling; this node only owns the network relay,
#           same division of labor as the existing odom -> telemetry path).
#
# RESET_SIM go-action addition: a third line shape, {"type":"reset"} in, no
# reply out. Unlike ArduPilot's RESET_SIM (which reaches gz_reset_listener.py
# directly over its own socket), this rover has no autopilot/FDM in the way,
# so the reset is just an instant /gazebo/set_model_state teleport of the
# rover model back to its generic_rover.world spawn pose -- the same
# non-blocking-topic mechanism camera_rig_controller.py already uses to move
# the follow-cam smoothly, confirmed there to not fight the physics solver.
#
# OBSTACLE_COURSE_ON/OFF setup-action addition: a fourth line shape,
# {"type":"obstacle_course","enabled":bool} in, no reply out. Swapping the
# whole world file would mean tearing down and relaunching gzserver (drops
# every existing ROS connection, heavyweight for what's meant to be a live
# RUI toggle), so instead this spawns/deletes the standalone
# models/obstacle_course/model.sdf (chicane walls + ramp bump) into the
# already-running generic_rover.world session via the stock
# /gazebo/spawn_sdf_model and /gazebo/delete_model services -- the same
# geometry `generic_rover_obstacle_course.world` includes for standalone
# testing, single source of truth, just spawned live instead of baked into
# the world file. `sim_rover_gazebo` (the default "basic room" command) still
# always launches plain generic_rover.world with no obstacles.
#
# Manual per-motor control (RBX_EXTERNAL_HARDWARE_INTERFACES.md worked
# example, section 6) is implemented entirely on the rbx_sim_node.py side:
# it folds the left/right-ratio-to-Twist conversion into the SAME
# continuously-running 20Hz control loop that already sends goto/idle
# velocity commands (gotoControlCb), then sends the result over this
# bridge's existing velocity-command shape. A one-shot motor_cmd message
# here would have been immediately overwritten by that loop's next
# (0,0)-when-idle tick, since it always sends every 20Hz regardless of
# whether a goto is active -- confirmed live during development. No new
# bridge message shape needed as a result.

import base64
import json
import math
import os
import re
import socket
import threading
import time

import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, DeleteModel

PKG_NAME = 'SIM_BRIDGE'  # Use in display menus
FILE_TYPE = 'NODE'

NODE_NAME = 'sim_bridge_node'

HEARTBEAT_TOPIC = '/sim/heartbeat'
HEARTBEAT_FRAME_ID = 'gazebo_simulation'
HEARTBEAT_INTERVAL_SEC = 1.0

NEPI_CMD_VEL_TOPIC = '/nepi/sim/cmd_vel'
GAZEBO_CMD_VEL_TOPIC = '/rover/cmd_vel'
GAZEBO_ODOM_TOPIC = '/rover/odom'

# Command/telemetry bridge port: next free port after the 9022 heartbeat
# listener in the 902x sim-utility block (9021 gz reset, 9022 heartbeat),
# clear of the 576x MAVLink ports. Forwarded by nepi_tunnel alongside 9022.
BRIDGE_PORT = 9023
TELEMETRY_RATE_HZ = 10.0

# Local ROS param namespace camera_rig_controller.py reads for the current
# view mode -- see the module docstring above for why a param, not a new
# topic/message, is the bridge point between this node and that one.
CAMERA_COMPRESSED_TOPIC = '/camera_rig/image_compressed'
PARAM_VIEW_MODE = '/sim/camera/view_mode'

# RESET_SIM target: generic_rover.world's containing <model> name and its
# (unmodified, default) spawn pose -- world origin, identity orientation.
MODEL_STATE_TOPIC = '/gazebo/set_model_state'
ROVER_MODEL_NAME = 'generic_rover_demo'

# OBSTACLE_COURSE_ON/OFF target: models/obstacle_course/model.sdf, read once
# at startup and spawned/deleted whole by model name via the stock Gazebo
# services. Path resolved relative to this script, not GAZEBO_MODEL_PATH --
# this node reads the raw SDF text itself rather than asking Gazebo to
# resolve a model:// URI, so it works whether or not the model happens to be
# on that path.
OBSTACLE_COURSE_MODEL_NAME = 'obstacle_course'
OBSTACLE_COURSE_SDF_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'models',
    'obstacle_course', 'model.sdf')
SPAWN_MODEL_SERVICE = '/gazebo/spawn_sdf_model'
DELETE_MODEL_SERVICE = '/gazebo/delete_model'

# camera_offset_*/scene_offset_* settings (rbx_sim_node.py's own robot-view /
# scene-view camera offset controls, sent here in the same camera_settings
# line as view_mode): applied by editing generic_rover/model.sdf's camera_link
# / camera_link_chase <pose> and respawning the whole rover model.
#
# Why a respawn rather than moving the camera at runtime -- every runtime
# option was tried live on this VM (Gazebo 11.15.1) and rejected:
#   - A cross-model fixed joint (spawning the camera as its own model welded
#     to generic_rover_demo::base_link) is SILENTLY IGNORED. Confirmed: the
#     spawn reports success, but the welded link stayed at its spawn pose
#     while the rover drove 13 m away.
#   - /gazebo/set_model_configuration and /gazebo/set_link_state (to drive a
#     3-DOF prismatic camera-mount chain built for this) both report success
#     while moving nothing -- tried with and without physics paused, and with
#     every joint-name scoping variant get_model_properties actually reports.
#   - gazebo_ros_joint_pose_trajectory (the one plugin that DID move the
#     joints) fights the physics engine's own integration of an unactuated
#     joint and drove all six camera joint states to nan within a few ticks --
#     usable for a kinematic-only model, not for links riding on a
#     physically-simulated rover.
# The two cameras stay genuinely rigid (fixed joints, zero per-tick follow
# lag) between offset changes, which is the property worth keeping; the
# respawn is the one moment they are not, and it is instant. This is viable
# specifically because generic_rover/model.sdf's diff_drive plugin uses
# <odometrySource>world</odometrySource> -- odom is read back from Gazebo's
# own model pose, so it resumes correctly after the model is recreated with
# no encoder state to lose.
ROVER_MODEL_SDF_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'models',
    'generic_rover', 'model.sdf')
# Matches a <link name="LINKNAME"> immediately followed by its own <pose>
# element. Group 1 is everything up to and including "<pose>"; group 2 is the
# existing rotation triple plus "</pose>" -- substituting keeps group 1 and
# group 2 and replaces only what sits between them (the position triple), so
# the existing roll/pitch/yaw survives untouched. (camera_link_chase carries a
# pitch for its downward look; camera_link does not offer rotation as a
# setting at all, so preserving whatever is already there is correct for
# both.)
CAMERA_LINK_POSE_RE = {
  'camera_link': re.compile(
      r'(<link name="camera_link">\s*<pose>)\s*'
      r'[-0-9.eE]+\s+[-0-9.eE]+\s+[-0-9.eE]+\s*'
      r'([-0-9.eE]+\s+[-0-9.eE]+\s+[-0-9.eE]+\s*</pose>)'),
  'camera_link_chase': re.compile(
      r'(<link name="camera_link_chase">\s*<pose>)\s*'
      r'[-0-9.eE]+\s+[-0-9.eE]+\s+[-0-9.eE]+\s*'
      r'([-0-9.eE]+\s+[-0-9.eE]+\s+[-0-9.eE]+\s*</pose>)'),
}
GAZEBO_SERVICE_WAIT_SEC = 5.0


#########################################
# Node Class
#########################################

class SimBridgeNode:

  def __init__(self):
    rospy.init_node(NODE_NAME)
    rospy.loginfo(PKG_NAME + ": Starting Node Initialization Processes")

    self.heartbeat_pub = rospy.Publisher(HEARTBEAT_TOPIC, Header, queue_size=1)
    self.gazebo_cmd_pub = rospy.Publisher(GAZEBO_CMD_VEL_TOPIC, Twist, queue_size=1)
    self.nepi_cmd_pub = rospy.Publisher(NEPI_CMD_VEL_TOPIC, Twist, queue_size=1)
    self.cmd_sub = rospy.Subscriber(NEPI_CMD_VEL_TOPIC, Twist, self.cmdCb)
    self.odom_sub = rospy.Subscriber(GAZEBO_ODOM_TOPIC, Odometry, self.odomCb)
    # Camera-rover feature: camera_rig_controller.py owns compression/rate
    # throttling on its own topic; this node only relays whatever arrives.
    self.image_sub = rospy.Subscriber(CAMERA_COMPRESSED_TOPIC, CompressedImage,
                                      self.imageCompressedCb)
    self.model_state_pub = rospy.Publisher(MODEL_STATE_TOPIC, ModelState, queue_size=1)

    # Obstacle-course model, read once here rather than per-toggle -- the
    # file never changes at runtime, no reason to re-read it on every
    # OBSTACLE_COURSE_ON. self.obstacle_course_spawned tracks which state
    # Gazebo is actually in so repeated ON/ON or OFF/OFF toggles (e.g. a
    # stale RUI button double-click) don't send a doomed second spawn (name
    # collision) or a doomed second delete (already gone) to the service.
    try:
      with open(OBSTACLE_COURSE_SDF_PATH, 'r') as f:
        self.obstacle_course_sdf = f.read()
    except Exception as e:
      rospy.logwarn(PKG_NAME + ": Failed to read obstacle course SDF at " +
                    OBSTACLE_COURSE_SDF_PATH + ": " + str(e))
      self.obstacle_course_sdf = None
    self.obstacle_course_spawned = False

    # Rover model SDF, read once here for the same reason as the obstacle
    # course file above (the file's structure never changes at runtime -- only
    # the two camera <pose> values get substituted per offset change, see
    # applyCameraSettings). self.applied_camera_offsets tracks the last offsets
    # actually baked into the live model, so a camera_settings line that only
    # changed view_mode (view_mode and the offsets share one wire message,
    # see rbx_sim_node.py's sendCameraSettings) does not trigger a pointless
    # respawn.
    try:
      with open(ROVER_MODEL_SDF_PATH, 'r') as f:
        self.rover_sdf_template = f.read()
    except Exception as e:
      rospy.logwarn(PKG_NAME + ": Failed to read rover SDF at " +
                    ROVER_MODEL_SDF_PATH + ": " + str(e) +
                    " -- camera offset changes will be ignored")
      self.rover_sdf_template = None
    self.applied_camera_offsets = None

    # Latest odom snapshot for the telemetry push loop, and the single
    # active bridge client (one robot, one remote node -- serve one
    # connection at a time; a reconnect is picked up after the dead one
    # is detected and torn down).
    self.latest_telemetry = None
    self.client_conn = None
    self.client_lock = threading.Lock()

    # Idle-hold anchor for holdStill() below -- captured ONCE when cmd_vel
    # first goes to exactly zero, then re-asserted unchanged on every
    # subsequent idle tick. Deliberately NOT re-read from self.latest_telemetry
    # each time: an earlier version did that and still drifted, because
    # re-anchoring to "whatever odom says right now" just locks in that
    # tick's tiny residual-velocity creep instead of preventing it -- the
    # anchor has to stay fixed across the whole idle period to actually hold
    # position, not merely re-zero velocity every tick.
    self.held_pose = None

    # Wall-clock thread, not rospy.Timer: with /use_sim_time set (the
    # gazebo_ros launcher sets it), a ROS timer tracks sim time -- it slows
    # with the real-time factor and stops entirely if the sim is paused,
    # which would falsely read as "simulator dead" to a liveness consumer.
    self.heartbeat_thread = threading.Thread(target=self.heartbeatLoop)
    self.heartbeat_thread.daemon = True
    self.heartbeat_thread.start()

    # Bridge server + telemetry push threads (wall-clock for the same
    # reason as the heartbeat: the push doubles as connection liveness).
    self.server_thread = threading.Thread(target=self.bridgeServerLoop)
    self.server_thread.daemon = True
    self.server_thread.start()
    self.telemetry_thread = threading.Thread(target=self.telemetryPushLoop)
    self.telemetry_thread.daemon = True
    self.telemetry_thread.start()

    rospy.loginfo(PKG_NAME + ": Simulator Bridge Node initialized")
    rospy.loginfo(PKG_NAME + ": Heartbeat on " + HEARTBEAT_TOPIC)
    rospy.loginfo(PKG_NAME + ": Relaying " + NEPI_CMD_VEL_TOPIC +
                  " -> " + GAZEBO_CMD_VEL_TOPIC)
    rospy.loginfo(PKG_NAME + ": Bridge server on 127.0.0.1:" +
                  str(BRIDGE_PORT))

  def run(self):
    """Block until ROS shutdown, servicing the heartbeat timer and the
    command relay subscriber."""
    rospy.spin()

  def heartbeatLoop(self):
    while not rospy.is_shutdown():
      hdr = Header()
      hdr.stamp = rospy.Time.now()
      hdr.frame_id = HEARTBEAT_FRAME_ID
      self.heartbeat_pub.publish(hdr)
      time.sleep(HEARTBEAT_INTERVAL_SEC)

  def cmdCb(self, msg):
    self.gazebo_cmd_pub.publish(msg)
    # rbx_sim_node.py's gotoControlCb sends a fresh velocity command every
    # control tick regardless of active/idle, defaulting to (0,0) when there's
    # no goto in progress -- a zero Twist here means "hold position", not just
    # "no active command yet". See holdStill() for why that needs enforcing
    # at the model level, not just left to the diff-drive plugin's wheel motors.
    if msg.linear.x == 0.0 and msg.linear.y == 0.0 and msg.angular.z == 0.0:
      if self.held_pose is None:
        self.held_pose = self.captureCurrentPose()
      self.holdStill()
    else:
      # Actively driving again -- drop any stale anchor so the next time
      # cmd_vel returns to zero, a fresh one gets captured at wherever the
      # rover actually stopped, not wherever it was idle before this move.
      self.held_pose = None

  def captureCurrentPose(self):
    if self.latest_telemetry is None:
      return None
    return {
      'x': self.latest_telemetry['x'],
      'y': self.latest_telemetry['y'],
      'yaw': self.latest_telemetry['yaw'],
    }

  def odomCb(self, msg):
    pos = msg.pose.pose.position
    q = msg.pose.pose.orientation
    # Planar rover: yaw is all the remote scaffold needs (math, not tf,
    # to keep this node's dependencies minimal)
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    self.latest_telemetry = {
      'x': pos.x,
      'y': pos.y,
      'yaw': yaw,
      'linear_x': msg.twist.twist.linear.x,
      'angular_z': msg.twist.twist.angular.z,
      'stamp': msg.header.stamp.to_sec(),
    }

  def imageCompressedCb(self, msg):
    # Relayed straight through to whichever client is connected right now;
    # dropped silently if none is (matches the existing "no client" behavior
    # of sendVelocityCmd's counterpart on the remote-device side).
    line = {
      'type': 'image',
      'data': base64.b64encode(bytes(msg.data)).decode('ascii'),
      'stamp': msg.header.stamp.to_sec(),
    }
    self.sendLineToClient(line)

  def sendLineToClient(self, line_dict):
    # Holds client_lock across the actual sendall, not just the self.client_conn
    # read -- imageCompressedCb (image subscriber thread) and telemetryPushLoop
    # (its own thread) both write to the same TCP stream socket. Without one
    # lock serializing the writes themselves, two sendall calls landing at
    # once can interleave their bytes on the wire, corrupting the
    # newline-delimited JSON stream (the receiver's json.loads then silently
    # drops the mangled line -- see rbx_sim_node.py's processBridgeLine).
    with self.client_lock:
      conn = self.client_conn
      if conn is None:
        return
      try:
        conn.sendall((json.dumps(line_dict) + '\n').encode())
      except Exception as e:
        rospy.logwarn_throttle(5.0, PKG_NAME + ": Failed to send line to client: " + str(e))
        if self.client_conn is conn:
          self.client_conn = None
        try:
          conn.close()
        except Exception:
          pass

  def resetRover(self):
    # Stop first so the teleported pose isn't immediately fought by the
    # diff-drive plugin still applying the last commanded velocity.
    self.gazebo_cmd_pub.publish(Twist())
    state = ModelState()
    state.model_name = ROVER_MODEL_NAME
    state.pose.orientation.w = 1.0
    state.reference_frame = 'world'
    self.model_state_pub.publish(state)
    # Drop any idle anchor from before the reset -- otherwise the very next
    # idle tick's holdStill() would immediately re-teleport the rover back to
    # its pre-reset position, fighting the reset it's supposed to respect.
    self.held_pose = None

  def holdStill(self):
    # Confirmed live (get_joint_properties on all 4 wheel joints, with
    # cmd_vel verified rock-solid at exactly zero) that the rover still drifts
    # slowly in position and yaw even fully idle: each wheel joint's
    # velocity-controlled motor settles to a small residual angular velocity
    # instead of exactly zero, and the four residuals aren't even
    # symmetric side-to-side. Four simultaneous wheel-ground friction
    # contacts against a perfectly flat plane is a slightly over-determined
    # constraint problem for ODE's iterative "quick" solver, which doesn't
    # converge to exact zero every step; that tiny per-step error integrates
    # into a real, slowly-growing position/heading drift over time even
    # though the commanded velocity never leaves zero. Bumping solver
    # iterations made this worse, not better (tested live), and there's no
    # persistent-SDF equivalent of the ODE auto-disable-bodies knob to fall
    # back on.
    #
    # Re-asserts self.held_pose -- a FIXED anchor captured once when cmd_vel
    # first went to zero (see cmdCb), not a fresh read of self.latest_telemetry
    # each call. An earlier version re-read live telemetry every tick and
    # still drifted just as much: re-anchoring to "whatever odom says right
    # now" only locks in that tick's residual-velocity creep as the new
    # baseline instead of preventing it, so the drift kept accumulating one
    # tiny confirmed step at a time. Holding one fixed value across the whole
    # idle period is what actually stops it. Same non-blocking
    # /gazebo/set_model_state mechanism resetRover (above) and
    # camera_rig_controller.py's follow-cam already use, confirmed elsewhere
    # in this codebase not to fight the physics solver.
    if self.held_pose is None:
      return
    state = ModelState()
    state.model_name = ROVER_MODEL_NAME
    state.pose.position.x = self.held_pose['x']
    state.pose.position.y = self.held_pose['y']
    yaw = self.held_pose['yaw']
    state.pose.orientation.z = math.sin(yaw / 2.0)
    state.pose.orientation.w = math.cos(yaw / 2.0)
    state.reference_frame = 'world'
    self.model_state_pub.publish(state)

  def applyCameraSettings(self, cmd):
    # From rbx_sim_node.py's RBX settings mechanism (view_mode), pushed here
    # because this VM has no other visibility into the remote device's
    # process state. Stored as a plain ROS param for camera_rig_controller.py
    # to read each image-publish tick.
    rospy.set_param(PARAM_VIEW_MODE, str(cmd.get('view_mode', 'FIRST_PERSON')))

    # offset_x/y/z (robot view) and scene_offset_x/y/z (scene/chase view) are
    # optional in this wire message: absent on any deployment still running
    # an older rbx_sim_node.py that only ever sent view_mode. get() with None
    # sentinels, then bail without touching anything already applied, rather
    # than defaulting to 0.0 and silently snapping both cameras to the origin
    # the first time an old sender's message arrives.
    keys = ('offset_x', 'offset_y', 'offset_z',
            'scene_offset_x', 'scene_offset_y', 'scene_offset_z')
    if any(cmd.get(k) is None for k in keys):
      return
    try:
      offsets = tuple(float(cmd[k]) for k in keys)
    except (TypeError, ValueError) as e:
      rospy.logwarn(PKG_NAME + ": Ignoring malformed camera offsets: " + str(e))
      return
    if offsets == self.applied_camera_offsets:
      return  # Already live -- e.g. this message only changed view_mode.
    self.respawnRoverWithCameraOffsets(offsets)

  def respawnRoverWithCameraOffsets(self, offsets):
    if self.rover_sdf_template is None:
      rospy.logwarn(PKG_NAME + ": No rover SDF loaded, cannot apply camera offsets")
      return
    (off_x, off_y, off_z, scene_x, scene_y, scene_z) = offsets

    sdf = self.rover_sdf_template
    sdf, n1 = CAMERA_LINK_POSE_RE['camera_link'].subn(
        lambda m: m.group(1) + ("%.6f %.6f %.6f " % (off_x, off_y, off_z)) + m.group(2), sdf)
    sdf, n2 = CAMERA_LINK_POSE_RE['camera_link_chase'].subn(
        lambda m: m.group(1) + ("%.6f %.6f %.6f " % (scene_x, scene_y, scene_z)) + m.group(2), sdf)
    if n1 != 1 or n2 != 1:
      # A structural change to generic_rover/model.sdf (renamed link, reordered
      # pose) could make this regex stop matching -- fail loudly rather than
      # silently spawning the rover with its OLD/default camera poses, which
      # would look exactly like "the offset setting doesn't do anything".
      rospy.logerr(PKG_NAME + ": Camera pose substitution matched " + str(n1) +
                   "/1 camera_link and " + str(n2) + "/1 camera_link_chase -- "
                   "refusing to respawn with an unverified model")
      return

    # Capture the rover's current pose so the respawn doesn't teleport it back
    # to the world origin -- odometrySource=world means Gazebo's own model pose
    # IS the odom source, so this is the one piece of state that must survive.
    pose = self.captureCurrentPose()
    initial_pose = Pose()
    if pose is not None:
      initial_pose.position.x = pose['x']
      initial_pose.position.y = pose['y']
      initial_pose.orientation.z = math.sin(pose['yaw'] / 2.0)
      initial_pose.orientation.w = math.cos(pose['yaw'] / 2.0)
    else:
      initial_pose.orientation.w = 1.0

    # Stop first, same reasoning as resetRover: an in-flight cmd_vel would
    # otherwise be applied to the new model the instant it exists.
    self.gazebo_cmd_pub.publish(Twist())
    try:
      rospy.wait_for_service(DELETE_MODEL_SERVICE, timeout=GAZEBO_SERVICE_WAIT_SEC)
      rospy.ServiceProxy(DELETE_MODEL_SERVICE, DeleteModel)(ROVER_MODEL_NAME)
      rospy.wait_for_service(SPAWN_MODEL_SERVICE, timeout=GAZEBO_SERVICE_WAIT_SEC)
      spawn = rospy.ServiceProxy(SPAWN_MODEL_SERVICE, SpawnModel)
      resp = spawn(ROVER_MODEL_NAME, sdf, '', initial_pose, 'world')
      if not resp.success:
        rospy.logerr(PKG_NAME + ": Respawn with new camera offsets failed: " +
                     resp.status_message)
        return
    except Exception as e:
      rospy.logerr(PKG_NAME + ": Respawn with new camera offsets failed: " + str(e))
      return

    self.applied_camera_offsets = offsets
    self.held_pose = None  # Stale anchor from before the respawn -- see resetRover.
    rospy.loginfo(PKG_NAME + ": Applied camera offsets, robot=(%.2f,%.2f,%.2f) scene=(%.2f,%.2f,%.2f)"
                  % offsets)

  def setObstacleCourse(self, enabled):
    if self.obstacle_course_sdf is None:
      rospy.logwarn(PKG_NAME + ": Obstacle course SDF not loaded, ignoring toggle")
      return
    if enabled == self.obstacle_course_spawned:
      return  # Already in the requested state -- avoid a doomed duplicate spawn/delete
    if enabled:
      try:
        rospy.wait_for_service(SPAWN_MODEL_SERVICE, timeout=GAZEBO_SERVICE_WAIT_SEC)
        spawn = rospy.ServiceProxy(SPAWN_MODEL_SERVICE, SpawnModel)
        resp = spawn(OBSTACLE_COURSE_MODEL_NAME, self.obstacle_course_sdf,
                    '', Pose(), 'world')
        if resp.success:
          self.obstacle_course_spawned = True
          rospy.loginfo(PKG_NAME + ": Obstacle course spawned")
        else:
          rospy.logwarn(PKG_NAME + ": Obstacle course spawn failed: " + resp.status_message)
      except Exception as e:
        rospy.logwarn(PKG_NAME + ": Obstacle course spawn service call failed: " + str(e))
    else:
      try:
        rospy.wait_for_service(DELETE_MODEL_SERVICE, timeout=GAZEBO_SERVICE_WAIT_SEC)
        delete = rospy.ServiceProxy(DELETE_MODEL_SERVICE, DeleteModel)
        resp = delete(OBSTACLE_COURSE_MODEL_NAME)
        if resp.success:
          self.obstacle_course_spawned = False
          rospy.loginfo(PKG_NAME + ": Obstacle course removed")
        else:
          rospy.logwarn(PKG_NAME + ": Obstacle course delete failed: " + resp.status_message)
      except Exception as e:
        rospy.logwarn(PKG_NAME + ": Obstacle course delete service call failed: " + str(e))

  def bridgeServerLoop(self):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # rospy sets a process-global socket.setdefaulttimeout(60), which
    # accept() applies to every accepted connection. The command stream is
    # legitimately idle for long stretches (commands are sporadic), so a
    # recv timeout must not be treated as client death -- clear the timeout
    # and block instead; a real disconnect still unblocks recv with EOF, and
    # a half-open client is caught by the 10 Hz telemetry push failing.
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
    # Blocking recv loop on the one active client: newline-delimited JSON
    # commands in. Returns (back to accept) on client close or any error,
    # so the remote node can restart independently and reconnect.
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
        # Dispatch by key presence, not a mandatory "type" tag: the existing
        # velocity command shape ({"linear_x","angular_z"}) predates this and
        # is left untouched. Only the new camera_settings shape carries a
        # "type" field.
        if cmd.get('type') == 'camera_settings':
          self.applyCameraSettings(cmd)
          continue
        if cmd.get('type') == 'reset':
          self.resetRover()
          continue
        if cmd.get('type') == 'obstacle_course':
          self.setObstacleCourse(bool(cmd.get('enabled', False)))
          continue
        twist = Twist()
        twist.linear.x = float(cmd.get('linear_x', 0.0))
        twist.angular.z = float(cmd.get('angular_z', 0.0))
        self.nepi_cmd_pub.publish(twist)

  def telemetryPushLoop(self):
    interval = 1.0 / TELEMETRY_RATE_HZ
    while not rospy.is_shutdown():
      time.sleep(interval)
      if self.latest_telemetry is None:
        continue
      # Same client_lock-around-sendall rationale as sendLineToClient above --
      # this loop and imageCompressedCb's sendLineToClient both write the one
      # client socket from different threads and must not interleave.
      with self.client_lock:
        conn = self.client_conn
        if conn is None:
          continue
        try:
          conn.sendall((json.dumps(self.latest_telemetry) + '\n').encode())
        except Exception as e:
          # Dead client: closing here unblocks serveClient's recv, which
          # returns the server loop to accept for the reconnect
          rospy.logwarn(PKG_NAME + ": Telemetry push failed, dropping client: " + repr(e))
          if self.client_conn is conn:
            self.client_conn = None
          try:
            conn.close()
          except Exception:
            pass


#########################################
# Main
#########################################

if __name__ == '__main__':
  node = SimBridgeNode()
  node.run()
