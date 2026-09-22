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

# Webots bridge for rbx_webots_node.py -- the RBX-driver path, NOT the generic
# sim_connector path (that one is sim_connector_bridge_webots.py, kept intact
# and unmodified as its own separate controller/world pair).
#
# Runs as a Webots ROBOT CONTROLLER (launched by Webots itself, declared via
# rbx_rover.wbt's `controller "webots_rbx_bridge"` field). Speaks the SAME
# simple wire protocol sim_bridge_node.py does for Gazebo (see that file):
# raw velocity in ({"linear_x","angular_z"}), bare telemetry + relayed camera
# frames out, plus {"type":"camera_settings"/"reset"/"environment_option"}
# handled as documented no-ops below -- NOT the generic sim_connector
# protocol's goto_position/motor_control/sensor_topics/goto_result messages.
#
# This bridge is deliberately SIMPLER than sim_connector_bridge_webots.py: the
# RBX driver (rbx_webots_node.py) runs its own closed-loop goto controller and
# only ever sends raw velocity downstream, exactly like sim_bridge_node.py does
# for rbx_gazebo_node.py -- so there is no goto-target/proportional-control
# logic in this file at all, only sensor reading and direct velocity-to-wheel
# conversion.
#
# CLIENT, not server (2026-09-21, direction reversed -- see
# sim_heartbeat_listener.py's own docstring for the full reasoning this
# mirrors exactly): both the heartbeat ping and the bridge connection now
# DIAL OUT to the NEPI device instead of waiting to be dialed. Confirmed live
# that this dev VM cannot be reached from the device on any port at all (a
# bare SSH connect attempt to the VM's real LAN IP times out with no
# response, matching the classic WSL2-behind-NAT case), so the old
# listen-and-wait-for-a-tunnel model could never have worked here. Outbound
# from the VM is never blocked, so dialing the device instead needs no
# tunnel and no firewall configuration on any OS.
#
# Robot: sim_container/bridges/webots/worlds/rbx_rover.wbt -- started as a
# copy of sim_connector_rover.wbt with only the controller field changed,
# same wheel1-4/GPS/IMU devices.
#
# UPDATED (2026-09-21): reported live that Webots had no scene (chase) view
# and no depth cameras at all, unlike every Gazebo world in this project
# (generic_rover/model.sdf's camera_link + camera_link_chase, each color+
# depth via a Kinect-style sensor). rbx_rover.wbt now has four camera-family
# devices -- "camera"/"camera_depth" (robot view) and "camera_chase"/
# "camera_chase_depth" (scene/chase view, rigidly mounted at the same
# offset+pitch as camera_link_chase) -- captured and relayed as four
# separately-tagged image lines ("robot_color"/"robot_depth"/"scene_color"/
# "scene_depth"), matching sim_bridge_node.py's own wire protocol exactly so
# rbx_webots_node.py can reuse rbx_sim_node.py's CAMERA_PUB_ATTR routing
# unchanged. RangeFinder has no color channel, so "depth" frames here are
# colorized (normalized + a JET colormap) for viewing, same as what Gazebo's
# camera_rig_controller.py already produces for its own depth topics.
#
# Also RESET is a real Supervisor teleport now (reported live: "the
# reset_sim button also doesnt work, bringing the robot back to the
# starting point") -- rbx_rover.wbt's Robot node is `supervisor TRUE` as of
# the same fix, matching rbx_quadcopter.wbt's own Robot node.
# environment_option stays an honest no-op: this world still has no
# obstacle-course model.

import base64
import json
import math
import os
import socket
import sys
import threading
import time

import numpy as np
import cv2

from controller import Supervisor

DEFAULT_HEARTBEAT_PORT = 9041
DEFAULT_BRIDGE_PORT = 9046
ALIVE_REPLY = b'ALIVE\n'

# The NEPI device's own reachable address -- same env var and same default
# ("nepi") as sim_heartbeat_listener.py/sim_bridge_node.py's own DEVICE_HOST.
DEVICE_HOST = os.environ.get('NEPI_DEVICE_SSH_HOST', 'nepi')

WHEEL_RADIUS_M = 0.04
WHEEL_TRACK_M = 0.12
MAX_WHEEL_RADPS = 8.0

RECONNECT_INTERVAL_SEC = 3.0
SOCKET_TIMEOUT_SEC = 5.0
TELEMETRY_RATE_HZ = 10.0
IMAGE_RATE_HZ = 5.0
JPEG_QUALITY = 60
# Well under rbx_webots_discovery.py's HEARTBEAT_LISTEN_TIMEOUT_SEC (4s) so
# one dropped ping or one slow connection attempt doesn't read as "gone" --
# matches sim_heartbeat_listener.py's own PING_INTERVAL_SEC reasoning.
HEARTBEAT_PING_INTERVAL_SEC = 2.0


class WebotsRbxBridge:

  def __init__(self, heartbeat_port, bridge_port):
    self.heartbeat_port = heartbeat_port
    self.bridge_port = bridge_port

    self.robot = Supervisor()
    self.timestep = int(self.robot.getBasicTimeStep())

    # Supervisor-only: this node's own handle, and its spawn pose -- what
    # resetSim() below teleports back to. Captured once at startup so it
    # stays correct even if rbx_rover.wbt's own translation field changes.
    self.self_node = self.robot.getSelf()
    self.spawn_translation = list(self.self_node.getField("translation").getSFVec3f())
    self.spawn_rotation = list(self.self_node.getField("rotation").getSFRotation())

    # wheel1/wheel3 = left (anchor y=+0.06), wheel2/wheel4 = right (y=-0.06) --
    # matches the .wbt file's HingeJoint anchors exactly, same grouping
    # sim_connector_bridge_webots.py already uses for this same robot body.
    self.left_motors = [self.robot.getDevice("wheel1"), self.robot.getDevice("wheel3")]
    self.right_motors = [self.robot.getDevice("wheel2"), self.robot.getDevice("wheel4")]
    for m in self.left_motors + self.right_motors:
      m.setPosition(float("inf"))  # velocity-control mode
      m.setVelocity(0.0)

    self.gps = self.robot.getDevice("gps")
    self.gps.enable(self.timestep)
    self.imu = self.robot.getDevice("imu")
    self.imu.enable(self.timestep)
    self.camera = self.robot.getDevice("camera")
    self.camera.enable(self.timestep)
    self.camera_depth = self.robot.getDevice("camera_depth")
    self.camera_depth.enable(self.timestep)
    self.camera_chase = self.robot.getDevice("camera_chase")
    self.camera_chase.enable(self.timestep)
    self.camera_chase_depth = self.robot.getDevice("camera_chase_depth")
    self.camera_chase_depth.enable(self.timestep)

    # Node/field handles for live camera_offset_x/y/z + scene_offset_x/y/z +
    # camera_fov_deg (2026-09-21, requested live: "make sure the camera
    # offset stuff works... just like they do in gazebo"). Gazebo achieves
    # this by respawning the whole rover model with new camera_link poses
    # (see rbx_sim_node.py's own CAMERA_SETTING_NAMES comment); Webots needs
    # no respawn at all here -- a Supervisor can write a device node's own
    # translation/fieldOfView field directly and it takes effect immediately.
    # DEF names (ROBOT_CAM/ROBOT_CAM_DEPTH/SCENE_CAM/SCENE_CAM_DEPTH) are
    # rbx_rover.wbt's own, added for exactly this. Factory translations
    # captured here (not hardcoded) so a future .wbt mount-point change stays
    # correct automatically, matching self.spawn_translation's own reasoning
    # above. Rotation (yaw/tilt) is NOT live-adjustable yet -- position
    # offsets and FOV are the concrete, tested part of this fix; composing a
    # runtime yaw/tilt delta on top of camera_chase's existing pitch needs
    # real rotation-matrix composition (scipy.spatial.transform.Rotation),
    # which is a reasonable follow-up but wasn't verified live this pass.
    self.robot_cam_translation_field = self.robot.getFromDef("ROBOT_CAM").getField("translation")
    self.robot_cam_depth_translation_field = self.robot.getFromDef("ROBOT_CAM_DEPTH").getField("translation")
    self.scene_cam_translation_field = self.robot.getFromDef("SCENE_CAM").getField("translation")
    self.scene_cam_depth_translation_field = self.robot.getFromDef("SCENE_CAM_DEPTH").getField("translation")
    self.factory_robot_cam_translation = list(self.robot_cam_translation_field.getSFVec3f())
    self.factory_scene_cam_translation = list(self.scene_cam_translation_field.getSFVec3f())
    self.robot_cam_fov_field = self.robot.getFromDef("ROBOT_CAM").getField("fieldOfView")
    self.scene_cam_fov_field = self.robot.getFromDef("SCENE_CAM").getField("fieldOfView")
    self.factory_fov_rad = self.robot_cam_fov_field.getSFFloat()

    self.pose_lock = threading.Lock()
    self.x_m = 0.0
    self.y_m = 0.0
    self.yaw_rad = 0.0
    self.lin_mps = 0.0
    self.ang_radps = 0.0
    self._last_x, self._last_y, self._last_t = 0.0, 0.0, None

    # Commanded velocity, set directly by the RBX driver's own closed-loop
    # controller -- no goto-target/proportional-control state here at all,
    # unlike sim_connector_bridge_webots.py (see module docstring).
    self.cmd_lock = threading.Lock()
    self.cmd_linear_x = 0.0
    self.cmd_angular_z = 0.0

    # Four named frames (robot_color/robot_depth/scene_color/scene_depth) --
    # see the module docstring's 2026-09-21 update. One lock/dict for all
    # four rather than four separate attributes, since they're always
    # captured and sent together as a batch.
    self.frame_lock = threading.Lock()
    self.latest_frames = {}

    self.sock = None
    self.sock_lock = threading.Lock()

    threading.Thread(target = self.heartbeatLoop, daemon = True).start()
    threading.Thread(target = self.bridgeClientLoop, daemon = True).start()

    print("webots_rbx_bridge: controller started, dialing device %s heartbeat %d / "
          "bridge %d" % (DEVICE_HOST, self.heartbeat_port, self.bridge_port), flush = True)

  #**********************
  # Webots simulation-step loop -- runs on the MAIN thread, as Webots requires.

  def run(self):
    while self.robot.step(self.timestep) != -1:
      self.updatePoseFromSensors()
      self.applyCommandedVelocity()

  def updatePoseFromSensors(self):
    x, y, _z = self.gps.getValues()
    roll, pitch, yaw = self.imu.getRollPitchYaw()
    now = self.robot.getTime()
    lin_mps = 0.0
    ang_radps = 0.0
    if self._last_t is not None:
      dt = now - self._last_t
      if dt > 1e-6:
        lin_mps = math.hypot(x - self._last_x, y - self._last_y) / dt
        ang_radps = self.normalizeAngle(yaw - self.yaw_rad) / dt
    self._last_x, self._last_y, self._last_t = x, y, now
    with self.pose_lock:
      self.x_m, self.y_m, self.yaw_rad = x, y, yaw
      self.lin_mps, self.ang_radps = lin_mps, ang_radps

    if now - getattr(self, "_last_image_capture", -999.0) >= 1.0 / IMAGE_RATE_HZ:
      self._last_image_capture = now
      self.captureFrame()

  def captureFrame(self):
    try:
      robot_color = self._captureColorFrame(self.camera)
      scene_color = self._captureColorFrame(self.camera_chase)
      robot_depth = self._captureDepthFrame(self.camera_depth)
      scene_depth = self._captureDepthFrame(self.camera_chase_depth)
    except Exception as e:
      print("webots_rbx_bridge: bad camera frame: %s" % str(e), flush = True)
      return
    with self.frame_lock:
      if robot_color is not None:
        self.latest_frames["robot_color"] = robot_color
      if scene_color is not None:
        self.latest_frames["scene_color"] = scene_color
      if robot_depth is not None:
        self.latest_frames["robot_depth"] = robot_depth
      if scene_depth is not None:
        self.latest_frames["scene_depth"] = scene_depth

  def _captureColorFrame(self, camera):
    width, height = camera.getWidth(), camera.getHeight()
    raw = camera.getImage()
    if raw is None:
      return None
    arr = np.frombuffer(raw, dtype = np.uint8).reshape((height, width, 4))
    bgr = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    return encoded.tobytes() if ok else None

  def _captureDepthFrame(self, range_finder):
    # RangeFinder has no color channel -- getRangeImage() returns a flat
    # list of meters (inf/nan past maxRange), reshaped to height x width
    # below. Colorized here (normalize against this device's own min/max
    # range, then a JET colormap) purely for viewing, matching what Gazebo's
    # camera_rig_controller.py already produces for its own *_depth topics --
    # no raw 32FC1 depth_map data product exists in this project any more
    # (see rbx_sim_node.py's own CAMERA_SETTING_NAMES comment: removed
    # 2026-09-14, no consumer ever used it).
    width, height = range_finder.getWidth(), range_finder.getHeight()
    raw = range_finder.getRangeImage()  # data_type='list' (default) -- plain Python floats
    if raw is None or len(raw) < width * height:
      return None
    arr = np.array(raw, dtype = np.float32).reshape((height, width))
    min_range, max_range = range_finder.getMinRange(), range_finder.getMaxRange()
    finite = np.isfinite(arr)
    clipped = np.where(finite, np.clip(arr, min_range, max_range), max_range)
    span = max(max_range - min_range, 1e-6)
    normalized = ((clipped - min_range) / span * 255.0).astype(np.uint8)
    colorized = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
    ok, encoded = cv2.imencode(".jpg", colorized, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    return encoded.tobytes() if ok else None

  def normalizeAngle(self, angle_rad):
    while angle_rad > math.pi:
      angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
      angle_rad += 2.0 * math.pi
    return angle_rad

  def applyCommandedVelocity(self):
    # No goto math here -- rbx_webots_node.py already computed lin/ang and
    # sends it every control tick (including (0,0) when idle), the same
    # self-healing-against-dropped-packets design sim_bridge_node.py relies on
    # for Gazebo. This just converts to per-wheel velocity and applies it.
    with self.cmd_lock:
      lin, ang = self.cmd_linear_x, self.cmd_angular_z

    left_radps = (lin - ang * WHEEL_TRACK_M / 2.0) / WHEEL_RADIUS_M
    right_radps = (lin + ang * WHEEL_TRACK_M / 2.0) / WHEEL_RADIUS_M
    left_radps = max(-MAX_WHEEL_RADPS, min(MAX_WHEEL_RADPS, left_radps))
    right_radps = max(-MAX_WHEEL_RADPS, min(MAX_WHEEL_RADPS, right_radps))
    for m in self.left_motors:
      m.setVelocity(left_radps)
    for m in self.right_motors:
      m.setVelocity(right_radps)

  #**********************
  # Heartbeat pinger -- matches sim_heartbeat_listener.py exactly (dials the
  # device instead of waiting to be dialed). No separate "is the sim really
  # alive" check needed here the way that file has one for gzserver: this
  # loop only runs at all while Webots is running this controller, and
  # Webots kills its controller process the instant the world/simulation
  # stops, so the process being alive already IS "the sim is alive".

  def heartbeatLoop(self):
    while True:
      try:
        with socket.create_connection((DEVICE_HOST, self.heartbeat_port), timeout = 3) as sock:
          sock.sendall(ALIVE_REPLY)
      except Exception:
        # Device listener not up yet / momentarily unreachable -- harmless,
        # matches every other reconnect-style loop in this codebase; just
        # try again next cycle.
        pass
      time.sleep(HEARTBEAT_PING_INTERVAL_SEC)

  #**********************
  # rbx_webots_node.py TCP client -- matches sim_bridge_node.py's dial-out
  # role (this bridge dials the device's own listener, not the other way
  # around).

  def bridgeClientLoop(self):
    while True:
      try:
        conn = socket.create_connection((DEVICE_HOST, self.bridge_port), timeout = 5)
      except Exception as e:
        print("webots_rbx_bridge: bridge connect to %s:%d failed: %s" %
              (DEVICE_HOST, self.bridge_port, str(e)), flush = True)
        time.sleep(RECONNECT_INTERVAL_SEC)
        continue
      conn.settimeout(SOCKET_TIMEOUT_SEC)
      with self.sock_lock:
        self.sock = conn
      print("webots_rbx_bridge: connected to device bridge at %s:%d" %
            (DEVICE_HOST, self.bridge_port), flush = True)

      sender_stop = threading.Event()
      sender = threading.Thread(target = self.senderLoop, args = (conn, sender_stop), daemon = True)
      sender.start()

      buf = b""
      while True:
        try:
          data = conn.recv(4096)
        except socket.timeout:
          continue
        except Exception:
          data = b""
        if not data:
          break
        buf += data
        while b"\n" in buf:
          line, buf = buf.split(b"\n", 1)
          if line.strip():
            self.processLineFromNode(line)

      sender_stop.set()
      with self.sock_lock:
        self.sock = None
      try:
        conn.close()
      except Exception:
        pass
      print("webots_rbx_bridge: device bridge connection lost, reconnecting", flush = True)
      time.sleep(RECONNECT_INTERVAL_SEC)

  def senderLoop(self, conn, stop_event):
    last_image = 0.0
    while not stop_event.is_set():
      now = time.time()
      self.sendLine(conn, self.buildTelemetryLine())

      if now - last_image >= 1.0 / IMAGE_RATE_HZ:
        with self.frame_lock:
          frames = dict(self.latest_frames)
        # One line per camera, each tagged -- matches sim_bridge_node.py's
        # wire protocol exactly (see rbx_sim_node.py's CAMERA_PUB_ATTR),
        # rather than the old single untagged image line.
        for camera_name, frame in frames.items():
          self.sendLine(conn, {
              "type": "image",
              "camera": camera_name,
              "data": base64.b64encode(frame).decode("ascii"),
          })
        last_image = now

      time.sleep(1.0 / TELEMETRY_RATE_HZ)

  def buildTelemetryLine(self):
    with self.pose_lock:
      x_m, y_m, yaw_rad = self.x_m, self.y_m, self.yaw_rad
      lin_mps, ang_radps = self.lin_mps, self.ang_radps
    # Matches sim_bridge_node.py's bare-telemetry shape exactly: x/y/yaw plus
    # linear_x/angular_z, no "type" key (a line with no type key IS telemetry,
    # per rbx_webots_node.py's processBridgeLine dispatch).
    return {
        "x": x_m, "y": y_m, "yaw": yaw_rad,
        "linear_x": lin_mps, "angular_z": ang_radps,
    }

  def sendLine(self, conn, line_dict):
    # Locked around the actual send, not just self.sock's assignment -- same
    # reasoning as every other bridge in this project: two threads (this
    # sender loop and nothing else here, since there's no separate goto-result
    # sender) could otherwise interleave sendall() calls on the same socket.
    with self.sock_lock:
      try:
        conn.sendall((json.dumps(line_dict) + "\n").encode())
      except Exception:
        pass

  #**********************
  # Commands from rbx_webots_node.py

  def processLineFromNode(self, line):
    try:
      msg = json.loads(line)
    except Exception as e:
      print("webots_rbx_bridge: bad line from node: %s" % str(e), flush = True)
      return
    if not isinstance(msg, dict):
      return
    if 'linear_x' in msg and 'type' not in msg:
      with self.cmd_lock:
        self.cmd_linear_x = float(msg.get('linear_x', 0.0))
        self.cmd_angular_z = float(msg.get('angular_z', 0.0))
      return
    msg_type = msg.get("type")
    if msg_type == "camera_settings":
      self.applyCameraSettings(msg)
    elif msg_type == "reset":
      self.resetSim()
    elif msg_type == "environment_option":
      print("webots_rbx_bridge: environment_option not supported on this world, ignoring",
            flush = True)

  def applyCameraSettings(self, msg):
    # Position offsets are a plain delta from each camera's factory mount
    # point (matching rbx_sim_node.py's own offset_x/y/z convention), applied
    # directly to the live translation field -- no respawn needed, unlike
    # Gazebo. camera_depth/camera_chase_depth (the RangeFinder pair) move
    # together with their paired Camera so the color/depth views stay
    # co-located, same as their factory-mounted pairing.
    try:
      rx = self.factory_robot_cam_translation[0] + float(msg.get('offset_x', 0.0))
      ry = self.factory_robot_cam_translation[1] + float(msg.get('offset_y', 0.0))
      rz = self.factory_robot_cam_translation[2] + float(msg.get('offset_z', 0.0))
      self.robot_cam_translation_field.setSFVec3f([rx, ry, rz])
      self.robot_cam_depth_translation_field.setSFVec3f([rx, ry, rz])

      sx = self.factory_scene_cam_translation[0] + float(msg.get('scene_offset_x', 0.0))
      sy = self.factory_scene_cam_translation[1] + float(msg.get('scene_offset_y', 0.0))
      sz = self.factory_scene_cam_translation[2] + float(msg.get('scene_offset_z', 0.0))
      self.scene_cam_translation_field.setSFVec3f([sx, sy, sz])
      self.scene_cam_depth_translation_field.setSFVec3f([sx, sy, sz])

      if 'fov_deg' in msg:
        fov_rad = math.radians(float(msg['fov_deg']))
        self.robot_cam_fov_field.setSFFloat(fov_rad)
        self.scene_cam_fov_field.setSFFloat(fov_rad)
    except Exception as e:
      print("webots_rbx_bridge: failed to apply camera settings: %s" % str(e), flush = True)

  def resetSim(self):
    # Real Supervisor teleport now (2026-09-21) -- rbx_rover.wbt's Robot node
    # is `supervisor TRUE` as of the same fix. resetPhysics() clears
    # accumulated velocity/momentum from the teleport itself, matching
    # standard Webots practice for repositioning a physics-simulated body
    # (without it, the body would keep whatever velocity it had the instant
    # before teleporting and immediately drift again).
    with self.cmd_lock:
      self.cmd_linear_x = 0.0
      self.cmd_angular_z = 0.0
    for m in self.left_motors + self.right_motors:
      m.setVelocity(0.0)
    self.self_node.getField("translation").setSFVec3f(self.spawn_translation)
    self.self_node.getField("rotation").setSFRotation(self.spawn_rotation)
    self.self_node.resetPhysics()
    print("webots_rbx_bridge: reset to spawn pose", flush = True)


def main():
  heartbeat_port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_HEARTBEAT_PORT
  bridge_port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BRIDGE_PORT
  bridge = WebotsRbxBridge(heartbeat_port, bridge_port)
  bridge.run()


if __name__ == "__main__":
  main()
