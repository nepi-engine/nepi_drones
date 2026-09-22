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

# MuJoCo bridge for rbx_mujoco_node.py -- the RBX-driver "simple protocol"
# path, same shape as sim_bridge_node.py (Gazebo) and webots_rbx_bridge.py
# (Webots): raw velocity in ({"linear_x","angular_z"}), bare telemetry +
# relayed camera frames out, plus {"type":"camera_settings"/"reset"/
# "environment_option"} handled below.
#
# Unlike Gazebo/Webots, there is no separate simulator binary here -- MuJoCo
# is a plain Python physics library, so this SAME process owns the physics
# loop AND serves the bridge/heartbeat sockets. It DOES need a real DISPLAY/
# XAUTHORITY, same as Gazebo/Webots (see this repo's nepi_sim_display_env.sh,
# sourced by this target's own launch_command) -- a visible
# mujoco.viewer.launch_passive window is opened in run() below for the same
# reason gazebo/webots pop open their own GUI, on top of (not instead of) the
# camera image relay, which renders through a separate offscreen Renderer
# context and works whether or not a real display is available.
#
# Found the hard way while wiring the viewer up: mujoco.viewer.launch_passive
# segfaults if closed (or the process exits) immediately after creation with
# no step/sync loop run in between -- harmless here since run() below always
# steps+syncs in a loop for the life of the process, but worth knowing before
# writing a short-lived test script against this file's classes.
#
# CLIENT, not server (2026-09-21, direction reversed -- see
# webots_rbx_bridge.py's own comment for the full reasoning this mirrors
# exactly): both the heartbeat ping and the bridge connection dial OUT to
# the NEPI device instead of waiting to be dialed. Confirmed live that this
# dev VM cannot be reached from the device on any port at all (a bare SSH
# connect attempt to the VM's real LAN IP times out with no response,
# classic WSL2-behind-NAT), so the old listen-and-wait model could never
# have worked here regardless of tunnel/firewall configuration.
#
# Model: models/rbx_rover.xml -- 4 independently-actuated wheels (unlike
# Webots' 2-side-only rbx_rover.wbt), so this bridge converts a single
# lin/ang command into per-side wheel velocities exactly like
# webots_rbx_bridge.py's applyCommandedVelocity, just written to 4 actuators
# instead of 2 motor devices.
#
# RESET is genuine here (unlike Webots' rbx_rover.wbt before its own
# 2026-09-21 Supervisor fix) -- this process owns MuJoCo's physics state
# directly, so a reset request calls mujoco.mj_resetData and the model is
# immediately back at its initial pose.
#
# UPDATED (2026-09-21): rbx_rover.xml now has a real second (scene/chase)
# camera plus depth rendering on both cameras (requested live: "get mujoco
# to that same point too, with all the same features i asked for with
# webots" -- see webots_rbx_bridge.py's own identical same-day fix for the
# full reasoning). Captured/relayed as four separately-tagged image lines
# ("robot_color"/"robot_depth"/"scene_color"/"scene_depth"), matching
# sim_bridge_node.py's wire protocol so rbx_mujoco_node.py could reuse
# rbx_sim_node.py's CAMERA_PUB_ATTR routing unchanged. MuJoCo's Renderer has
# no combined RGBD mode -- depth frames are colorized (normalize + a JET
# colormap) for viewing, same as webots_rbx_bridge.py produces. Camera
# offsets (camera_offset_x/y/z, scene_offset_x/y/z) and camera_fov_deg are
# real, live-applied writes to model.cam_pos/model.cam_fovy now too -- no
# respawn needed, simpler even than Webots' Supervisor field writes.

import os
import base64
import math
import json
import socket
import sys
import threading
import time

import numpy as np
import cv2
import mujoco
import mujoco.viewer

DEFAULT_HEARTBEAT_PORT = 9051
DEFAULT_BRIDGE_PORT = 9056
ALIVE_REPLY = b"ALIVE\n"

# The NEPI device's own reachable address -- same env var/default as every
# other VM-side script in this project.
DEVICE_HOST = os.environ.get('NEPI_DEVICE_SSH_HOST', 'nepi')
HEARTBEAT_PING_INTERVAL_SEC = 2.0

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "rbx_rover.xml")

# Matches rbx_rover.xml's physical spec (also rbx_mujoco_node.py's
# MOTOR_WHEEL_BASE_M/MOTOR_MAX_LINEAR_MPS -- this conversion has to agree
# with the driver's own).
WHEEL_RADIUS_M = 0.1
WHEEL_TRACK_M = 0.34
MAX_WHEEL_RADPS = 15.0

RECONNECT_ACCEPT_TIMEOUT_SEC = 5.0
RECONNECT_INTERVAL_SEC = 3.0
SOCKET_TIMEOUT_SEC = 5.0
TELEMETRY_RATE_HZ = 10.0
IMAGE_RATE_HZ = 5.0
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
JPEG_QUALITY = 60


class MujocoRbxBridge:

  def __init__(self, heartbeat_port, bridge_port):
    self.heartbeat_port = heartbeat_port
    self.bridge_port = bridge_port

    self.model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    self.data = mujoco.MjData(self.model)
    mujoco.mj_forward(self.model, self.data)
    self.renderer = mujoco.Renderer(self.model, height = IMAGE_HEIGHT, width = IMAGE_WIDTH)
    self.camera_name = "robot_camera"
    self.scene_camera_name = "scene_camera"

    # Camera ids + factory mount points for live camera_offset_x/y/z and
    # scene_offset_x/y/z (2026-09-21, requested live: "make sure the camera
    # offset stuff works... just like they do in gazebo" -- see
    # webots_rbx_bridge.py's own identical fix). model.cam_pos/model.cam_fovy
    # are plain mutable per-model arrays MuJoCo re-reads every step via
    # mj_kinematics -- no respawn needed, simpler even than Webots'
    # Supervisor field writes. yaw/tilt not wired yet, same as Webots.
    self.robot_cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name)
    self.scene_cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.scene_camera_name)
    self.factory_robot_cam_pos = self.model.cam_pos[self.robot_cam_id].copy()
    self.factory_scene_cam_pos = self.model.cam_pos[self.scene_cam_id].copy()

    # Visible window, same reason Gazebo/Webots pop open their own GUI --
    # needs a real DISPLAY/XAUTHORITY (see module docstring). Failure here
    # (no X server reachable, e.g. a genuinely headless deployment) degrades
    # to a warning, not a crash: the physics loop, bridge, and camera relay
    # all work identically either way, so a missing display shouldn't take
    # the whole simulator down.
    self.viewer = None
    try:
      self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
    except Exception as e:
      print("mujoco_rbx_bridge: could not open a viewer window (%s) -- "
            "continuing headless" % str(e), flush = True)

    self.pose_lock = threading.Lock()
    self.x_m = 0.0
    self.y_m = 0.0
    self.yaw_rad = 0.0
    self.lin_mps = 0.0
    self.ang_radps = 0.0
    self._last_x, self._last_y, self._last_yaw, self._last_t = 0.0, 0.0, 0.0, None

    # Commanded velocity, set directly by the RBX driver's own closed-loop
    # controller -- no goto-target/proportional-control state here at all,
    # same reasoning as webots_rbx_bridge.py.
    self.cmd_lock = threading.Lock()
    self.cmd_linear_x = 0.0
    self.cmd_angular_z = 0.0

    # Four named frames (robot_color/robot_depth/scene_color/scene_depth) --
    # see the module docstring's 2026-09-21 update.
    self.frame_lock = threading.Lock()
    self.latest_frames = {}

    self.sock = None
    self.sock_lock = threading.Lock()

    self._last_image_capture = -999.0

    threading.Thread(target = self.heartbeatLoop, daemon = True).start()
    threading.Thread(target = self.bridgeClientLoop, daemon = True).start()

    print("mujoco_rbx_bridge: started, dialing device %s heartbeat %d / "
          "bridge %d" % (DEVICE_HOST, self.heartbeat_port, self.bridge_port), flush = True)

  #**********************
  # Physics loop -- runs on the calling (main) thread, paced to real time
  # (MuJoCo has no built-in blocking step-and-wait the way Webots' robot.step()
  # provides, so this does its own wall-clock pacing).

  def run(self):
    next_tick = time.time()
    while True:
      self.applyCommandedVelocity()
      mujoco.mj_step(self.model, self.data)
      self.updatePoseFromSensors()
      if self.viewer is not None and self.viewer.is_running():
        self.viewer.sync()

      next_tick += self.model.opt.timestep
      sleep_s = next_tick - time.time()
      if sleep_s > 0:
        time.sleep(sleep_s)
      else:
        # Fell behind (e.g. slow image render) -- resync rather than
        # accumulating an ever-growing backlog of steps to catch up on.
        next_tick = time.time()

  def updatePoseFromSensors(self):
    x, y, _z = self.data.qpos[0:3]
    qw, qx, qy, qz = self.data.qpos[3:7]
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    now = self.data.time

    lin_mps = 0.0
    ang_radps = 0.0
    if self._last_t is not None:
      dt = now - self._last_t
      if dt > 1e-6:
        lin_mps = math.hypot(x - self._last_x, y - self._last_y) / dt
        ang_radps = self.normalizeAngle(yaw - self._last_yaw) / dt
    self._last_x, self._last_y, self._last_yaw, self._last_t = x, y, yaw, now

    with self.pose_lock:
      self.x_m, self.y_m, self.yaw_rad = x, y, yaw
      self.lin_mps, self.ang_radps = lin_mps, ang_radps

    if now - self._last_image_capture >= 1.0 / IMAGE_RATE_HZ:
      self._last_image_capture = now
      self.captureFrame()

  def captureFrame(self):
    try:
      robot_color = self._captureColorFrame(self.camera_name)
      robot_depth = self._captureDepthFrame(self.camera_name)
      scene_color = self._captureColorFrame(self.scene_camera_name)
      scene_depth = self._captureDepthFrame(self.scene_camera_name)
    except Exception as e:
      print("mujoco_rbx_bridge: bad camera frame: %s" % str(e), flush = True)
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

  def _captureColorFrame(self, camera_name):
    self.renderer.disable_depth_rendering()
    self.renderer.update_scene(self.data, camera = camera_name)
    rgb = self.renderer.render()
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    return encoded.tobytes() if ok else None

  def _captureDepthFrame(self, camera_name):
    # MuJoCo's Renderer returns a plain height x width float32 array of
    # meters once depth rendering is enabled, same scene/camera as the last
    # update_scene call -- re-called here anyway since _captureColorFrame's
    # own disable_depth_rendering() call for the OTHER camera runs in
    # between and Renderer only ever holds one mode/camera at a time.
    # Colorized (normalize against this model's own camera clipping range,
    # then a JET colormap) purely for viewing, matching
    # webots_rbx_bridge.py's identical depth-colorization approach -- no raw
    # depth data product exists in this project (see rbx_sim_node.py's own
    # CAMERA_SETTING_NAMES comment: removed 2026-09-14, no consumer ever
    # used it).
    self.renderer.update_scene(self.data, camera = camera_name)
    self.renderer.enable_depth_rendering()
    depth = self.renderer.render()
    self.renderer.disable_depth_rendering()
    # MuJoCo's model.vis.map.znear/zfar are relative multipliers of the
    # model's own extent, not absolute meters, so a fixed sane depth-view
    # range is used instead -- matching webots_rbx_bridge.py's own
    # RangeFinder minRange/maxRange convention (0.05/100), scaled down to
    # this smaller model's actual scene size.
    min_range, max_range = 0.05, 20.0
    finite = np.isfinite(depth)
    clipped = np.where(finite, np.clip(depth, min_range, max_range), max_range)
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
    # No goto math here -- rbx_mujoco_node.py already computed lin/ang and
    # sends it every control tick (including (0,0) when idle), same
    # self-healing-against-dropped-packets design every other bridge here
    # relies on. This just converts to per-wheel velocity and writes ctrl.
    with self.cmd_lock:
      lin, ang = self.cmd_linear_x, self.cmd_angular_z

    left_radps = (lin - ang * WHEEL_TRACK_M / 2.0) / WHEEL_RADIUS_M
    right_radps = (lin + ang * WHEEL_TRACK_M / 2.0) / WHEEL_RADIUS_M
    left_radps = max(-MAX_WHEEL_RADPS, min(MAX_WHEEL_RADPS, left_radps))
    right_radps = max(-MAX_WHEEL_RADPS, min(MAX_WHEEL_RADPS, right_radps))
    # wheel1=front_left, wheel2=front_right, wheel3=rear_left, wheel4=rear_right
    self.data.ctrl[0] = left_radps
    self.data.ctrl[1] = right_radps
    self.data.ctrl[2] = left_radps
    self.data.ctrl[3] = right_radps

  def resetSim(self):
    # Genuine reset -- see module docstring. Clears commanded velocity and
    # finite-difference state too, so telemetry doesn't report a stale
    # lin/ang spike computed against the pre-reset pose.
    with self.cmd_lock:
      self.cmd_linear_x = 0.0
      self.cmd_angular_z = 0.0
    mujoco.mj_resetData(self.model, self.data)
    mujoco.mj_forward(self.model, self.data)
    self._last_x, self._last_y, self._last_yaw, self._last_t = 0.0, 0.0, 0.0, None
    print("mujoco_rbx_bridge: reset to initial pose", flush = True)

  def applyCameraSettings(self, msg):
    # Position offsets are a plain delta from each camera's factory mount
    # point, written directly into model.cam_pos -- mj_kinematics re-reads
    # this every step, so no respawn/reset is needed for it to take effect
    # (simpler than webots_rbx_bridge.py's Supervisor field write, since
    # MuJoCo already exposes this as a mutable per-model array).
    try:
      self.model.cam_pos[self.robot_cam_id] = self.factory_robot_cam_pos + np.array([
          float(msg.get('offset_x', 0.0)), float(msg.get('offset_y', 0.0)), float(msg.get('offset_z', 0.0))])
      self.model.cam_pos[self.scene_cam_id] = self.factory_scene_cam_pos + np.array([
          float(msg.get('scene_offset_x', 0.0)), float(msg.get('scene_offset_y', 0.0)), float(msg.get('scene_offset_z', 0.0))])
      if 'fov_deg' in msg:
        fov_deg = float(msg['fov_deg'])
        self.model.cam_fovy[self.robot_cam_id] = fov_deg
        self.model.cam_fovy[self.scene_cam_id] = fov_deg
    except Exception as e:
      print("mujoco_rbx_bridge: failed to apply camera settings: %s" % str(e), flush = True)

  #**********************
  # Heartbeat pinger -- matches sim_heartbeat_listener.py exactly (dials the
  # device instead of waiting to be dialed).

  def heartbeatLoop(self):
    while True:
      try:
        with socket.create_connection((DEVICE_HOST, self.heartbeat_port), timeout = 3) as sock:
          sock.sendall(ALIVE_REPLY)
      except Exception:
        pass
      time.sleep(HEARTBEAT_PING_INTERVAL_SEC)

  #**********************
  # rbx_mujoco_node.py TCP client -- dials the device's own listener instead
  # of waiting to be dialed (2026-09-21, matches webots_rbx_bridge.py's own
  # bridgeClientLoop).

  def bridgeClientLoop(self):
    while True:
      try:
        conn = socket.create_connection((DEVICE_HOST, self.bridge_port), timeout = 5)
      except Exception as e:
        print("mujoco_rbx_bridge: bridge connect to %s:%d failed: %s" %
              (DEVICE_HOST, self.bridge_port, str(e)), flush = True)
        time.sleep(RECONNECT_INTERVAL_SEC)
        continue
      conn.settimeout(SOCKET_TIMEOUT_SEC)
      with self.sock_lock:
        self.sock = conn
      print("mujoco_rbx_bridge: connected to device bridge at %s:%d" %
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
      print("mujoco_rbx_bridge: device bridge connection lost, reconnecting", flush = True)
      time.sleep(RECONNECT_INTERVAL_SEC)

  def senderLoop(self, conn, stop_event):
    last_image = 0.0
    while not stop_event.is_set():
      now = time.time()
      self.sendLine(conn, self.buildTelemetryLine())

      if now - last_image >= 1.0 / IMAGE_RATE_HZ:
        with self.frame_lock:
          frames = dict(self.latest_frames)
        # One line per camera, each tagged -- matches sim_bridge_node.py's/
        # webots_rbx_bridge.py's wire protocol exactly.
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
    # Matches sim_bridge_node.py/webots_rbx_bridge.py's bare-telemetry shape
    # exactly: x/y/yaw plus linear_x/angular_z, no "type" key.
    return {
        "x": x_m, "y": y_m, "yaw": yaw_rad,
        "linear_x": lin_mps, "angular_z": ang_radps,
    }

  def sendLine(self, conn, line_dict):
    with self.sock_lock:
      try:
        conn.sendall((json.dumps(line_dict) + "\n").encode())
      except Exception:
        pass

  #**********************
  # Commands from rbx_mujoco_node.py

  def processLineFromNode(self, line):
    try:
      msg = json.loads(line)
    except Exception as e:
      print("mujoco_rbx_bridge: bad line from node: %s" % str(e), flush = True)
      return
    if not isinstance(msg, dict):
      return
    if "linear_x" in msg and "type" not in msg:
      with self.cmd_lock:
        self.cmd_linear_x = float(msg.get("linear_x", 0.0))
        self.cmd_angular_z = float(msg.get("angular_z", 0.0))
      return
    msg_type = msg.get("type")
    if msg_type == "camera_settings":
      self.applyCameraSettings(msg)
    elif msg_type == "reset":
      self.resetSim()
    elif msg_type == "environment_option":
      # Honest no-op for this pass -- no obstacle-course MJCF model built
      # yet, same documented gap as webots_rbx_bridge.py's own.
      print("mujoco_rbx_bridge: environment_option not supported yet, ignoring", flush = True)


def main():
  heartbeat_port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_HEARTBEAT_PORT
  bridge_port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BRIDGE_PORT
  bridge = MujocoRbxBridge(heartbeat_port, bridge_port)
  bridge.run()


if __name__ == "__main__":
  main()
