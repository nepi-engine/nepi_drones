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
# (Webots): raw velocity in ({"linear_x","linear_y","angular_z"}), bare
# telemetry + relayed camera frames out, plus {"type":"camera_settings"/
# "reset"/"set_environment"/"refresh_environment"/"wheel_independence"}
# handled below.
#
# UPDATED (2026-09-21, "get mujoco to that same point too, with all the
# same features i asked for with webots"): rbx_rover.xml is now a
# GENERATED file (generate_rover_xml.py, run once here at startup from
# sim_container/models/generic_rover/dimensions.yaml). Unlike Webots'
# Supervisor import/remove, every environment's geoms are compiled into
# rbx_rover.xml from the start (generate_environment_xml.py) and
# setEnvironment picks exactly one live via each geom's rgba alpha +
# contype/conaffinity, since a compiled MjModel can't have bodies added/
# removed at runtime -- see that generator's own module docstring.
#
# UPDATED (2026-09-25, "make mujoco the main sim, test everything"):
# added the crab-steer swerve modules (see generate_rover_xml.py), a
# CUSTOM_OBSTACLES environment alongside OBSTACLE_COURSE, and
# refreshEnvironment -- a live rebuild of model/data (_bindModel) so an
# edited environment dimensions config takes effect without a relaunch,
# preserving the rover's own pose/velocity across the rebuild.
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

import generate_rover_xml
import generate_environment_xml
from generate_environment_xml import OBSTACLE_COURSE_GEOM_NAMES

DEFAULT_HEARTBEAT_PORT = 9051
DEFAULT_BRIDGE_PORT = 9056
ALIVE_REPLY = b"ALIVE\n"

# The NEPI device's own reachable address -- same env var/default as every
# other VM-side script in this project.
DEVICE_HOST = os.environ.get('NEPI_DEVICE_SSH_HOST', 'nepi')
HEARTBEAT_PING_INTERVAL_SEC = 2.0

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models", "rbx_rover.xml")

# Fallback only, if generic_rover/dimensions.yaml can't be read at startup
# (see loadWheelDimensions below) -- matches rbx_rover.xml's own factory
# defaults (also rbx_mujoco_node.py's MOTOR_WHEEL_BASE_M/
# MOTOR_MAX_LINEAR_MPS, which this conversion has to agree with).
WHEEL_RADIUS_M = 0.1
WHEEL_TRACK_M = 0.34
MAX_WHEEL_RADPS = 15.0

WHEEL_NAMES = [name for name, _x, _y in generate_rover_xml.ROVER_WHEELS]
# Below this wheel ground speed a wheel keeps its last steer angle instead
# of re-deriving atan2(~0, ~0), so it doesn't flick straight on every pause.
STEER_HOLD_SPEED_MPS = 0.01
STEER_ALIGN_GATE_RAD = math.radians(25.0)
# Rate limit on the steer command. Snapping all four wheels 90 degrees at
# once with a stiff steer actuator kicks the chassis ~6.5 degrees in yaw from
# the reaction torque; 20 rad/s cuts that to ~2.6 while a 90-degree swing
# still takes under 0.1 s.
STEER_SLEW_RADPS = 20.0
# Same active heading hold as crab_steer_plugin.cpp (Gazebo): with no
# rotation commanded, servo back to the heading captured when rotation
# stopped, capped so the correction can never spin the rover.
YAW_HOLD_EPS = 0.01
YAW_HOLD_KP = 2.0
MAX_YAW_HOLD_RATE = 1.0


def yawTiltToQuat(yaw_deg, tilt_deg):
    # Both rbx_rover.xml cameras' own factory xyaxes fix local X (right) at
    # (0,-1,0) and tilt local Y (up) in the XZ plane -- robot_camera's own
    # "0 -1 0 0 0 1" is exactly this at tilt=0, scene_camera's own
    # "0 -1 0 0.5514 0 0.8342" is exactly this at tilt=FACTORY_SCENE_TILT_DEG
    # (rbx_mujoco_node.py's own constant). MuJoCo cameras view along local
    # -Z, so local Z (back) = X cross Y reproduces "look toward +X" at
    # yaw=tilt=0, matching Gazebo/Webots' own yaw=0 convention. yaw further
    # rotates this whole base frame about the parent body's own +Z (up)
    # axis. Returns MuJoCo's own (w,x,y,z) quaternion convention for
    # model.cam_quat, via the standard rotation-matrix -> quaternion
    # formula (Shepperd's method).
    yaw = math.radians(yaw_deg)
    tilt = math.radians(tilt_deg)
    bx = np.array([0.0, -1.0, 0.0])
    by = np.array([math.sin(tilt), 0.0, math.cos(tilt)])
    bz = np.cross(bx, by)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    x = rz @ bx
    y = rz @ by
    z = rz @ bz
    r = np.column_stack([x, y, z])
    trace = np.trace(r)
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        w = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        w = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        w = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        w = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s
    return np.array([w, qx, qy, qz])


def loadWheelDimensions():
    # Best-effort, same reasoning as webots_rbx_bridge.py's identical
    # helper: a missing/malformed dimensions.yaml degrades to the module-
    # level defaults above rather than crashing the whole bridge.
    try:
        dims = generate_rover_xml.loadDimensions("generic_rover", generate_rover_xml.DEFAULT_DIMENSIONS)
        return float(dims["wheel_radius_m"]), float(dims["track_width_m"])
    except Exception as e:
        print("mujoco_rbx_bridge: could not read dimensions.yaml (%s) -- using defaults "
              "wheel_radius_m=%.3f track_width_m=%.3f" %
              (str(e), WHEEL_RADIUS_M, WHEEL_TRACK_M), flush = True)
        return WHEEL_RADIUS_M, WHEEL_TRACK_M

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

    # Regenerate rbx_rover.xml from the current dimensions.yaml files before
    # every launch -- "next launch, not live" contract, identical to
    # generate_rover_wbt.py's own for Webots (see generate_rover_xml.py's
    # module docstring for why MuJoCo can't do this live). Calling
    # generate_rover_xml.main() directly would misread sys.argv -- that's
    # THIS process's own heartbeat/bridge port argv, not a model name -- so
    # this replicates main()'s body instead of calling it.
    self.camera_name = "robot_camera"
    self.scene_camera_name = "scene_camera"
    self.wheel_radius_m, self.wheel_track_m = loadWheelDimensions()
    self.current_environment = "FLAT_GROUND"
    self.last_camera_settings = None
    self.held_yaw = None
    self._bindModel(initial = True)

    # Visible window, same reason Gazebo/Webots pop open their own GUI --
    # needs a real DISPLAY/XAUTHORITY (see module docstring). Failure here
    # (no X server reachable, e.g. a genuinely headless deployment) degrades
    # to a warning, not a crash: the physics loop, bridge, and camera relay
    # all work identically either way, so a missing display shouldn't take
    # the whole simulator down.
    self.viewer = None
    self._openViewer()

    self.pose_lock = threading.Lock()
    self.x_m = 0.0
    self.y_m = 0.0
    self.yaw_rad = 0.0
    self.lin_mps = 0.0
    self.ang_radps = 0.0
    self.vx_world = 0.0
    self.vy_world = 0.0
    self._last_x, self._last_y, self._last_yaw, self._last_t = 0.0, 0.0, 0.0, None

    # Commanded velocity, set directly by the RBX driver's own closed-loop
    # controller -- no goto-target/proportional-control state here at all,
    # same reasoning as webots_rbx_bridge.py.
    self.cmd_lock = threading.Lock()
    self.cmd_linear_x = 0.0
    self.cmd_linear_y = 0.0
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

  def _bindModel(self, initial):
    # Regenerate rbx_rover.xml from the current dimensions.yaml files, then
    # (re)build model/data and re-resolve every id/handle that lives on this
    # specific MjModel instance. Called once at construction and again by
    # refreshEnvironment, which is why every one of those handles has to be
    # re-derived here rather than assumed to still be valid.
    generate_rover_xml.regenerate("generic_rover")
    self.model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    self.data = mujoco.MjData(self.model)
    mujoco.mj_forward(self.model, self.data)
    self.renderer = mujoco.Renderer(self.model, height = IMAGE_HEIGHT, width = IMAGE_WIDTH)

    # Obstacle-course / custom-obstacles geoms are always compiled into the
    # model (see generate_environment_xml.py's own docstring for why a
    # compiled MjModel can't have geoms added/removed at runtime); setEnvironment
    # toggles visibility/collision live instead of spawning/removing anything.
    self.obstacle_course_geom_ids = [
        mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in OBSTACLE_COURSE_GEOM_NAMES
    ]
    self.custom_obstacles_geom_ids = [
        mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in generate_environment_xml.CUSTOM_OBSTACLES_GEOM_NAMES
    ]
    self.aerial_obstacle_course_geom_ids = [
        mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in generate_environment_xml.AERIAL_OBSTACLE_COURSE_GEOM_NAMES
    ]
    # Compiled colliding (see generate_environment_xml.py's _boxGeom for
    # why); force all off, then setEnvironment (called by our own caller --
    # __init__ for a fresh FLAT_GROUND start, refreshEnvironment to restore
    # whatever was selected) applies the real state.
    self._setGeomGroupEnabled(self.obstacle_course_geom_ids, False)
    self._setGeomGroupEnabled(self.custom_obstacles_geom_ids, False)
    self._setGeomGroupEnabled(self.aerial_obstacle_course_geom_ids, False)

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

    # Per-wheel swerve module handles (see generate_rover_xml.py): spin and
    # steer actuator ids, the steer joint's qpos address, and the module's
    # (x, y) offset in the chassis frame, all read from the compiled model so
    # they always match whatever dimensions.yaml generated.
    def _id(obj, name):
      return mujoco.mj_name2id(self.model, obj, name)
    self.wheels = []
    for name in WHEEL_NAMES:
      steer_body = _id(mujoco.mjtObj.mjOBJ_BODY, name + "_steer")
      self.wheels.append({
          'spin_act': _id(mujoco.mjtObj.mjOBJ_ACTUATOR, name + "_vel"),
          'steer_act': _id(mujoco.mjtObj.mjOBJ_ACTUATOR, name + "_steer_pos"),
          'steer_qadr': self.model.jnt_qposadr[_id(mujoco.mjtObj.mjOBJ_JOINT, name + "_steer")],
          'x': float(self.model.body_pos[steer_body][0]),
          'y': float(self.model.body_pos[steer_body][1]),
          'steer_target': 0.0,
          'steer_cmd': 0.0,
      })
    self.steer_range_rad = generate_rover_xml.STEER_RANGE_RAD

    if initial:
      # Starting mode comes from the launched robot config's dimensions
      # (e.g. crabrover has wheel_independence_enabled: 1); reported back in
      # telemetry so rbx_mujoco_node.py's Setting matches, and still
      # switchable live afterward by that Setting. Not re-read on a live
      # refresh -- that would silently revert an operator's own live toggle.
      try:
        dims = generate_rover_xml.loadDimensions("generic_rover", generate_rover_xml.DEFAULT_DIMENSIONS)
        self.wheel_independence = bool(int(float(dims.get("wheel_independence_enabled", 0))))
      except (TypeError, ValueError):
        self.wheel_independence = False
      print("mujoco_rbx_bridge: starting with wheel independence %s" %
            ("on" if self.wheel_independence else "off"), flush = True)

  def _openViewer(self):
    # Visible window, same reason Gazebo/Webots pop open their own GUI --
    # needs a real DISPLAY/XAUTHORITY (see module docstring). Failure here
    # (no X server reachable, e.g. a genuinely headless deployment) degrades
    # to a warning, not a crash: the physics loop, bridge, and camera relay
    # all work identically either way, so a missing display shouldn't take
    # the whole simulator down. Also (re)called by refreshEnvironment: a
    # passive viewer is bound to one specific model/data pair at creation,
    # so a rebuilt model needs a fresh window, not a mutation of the old one.
    if self.viewer is not None:
      try:
        self.viewer.close()
      except Exception:
        pass
      self.viewer = None
    try:
      self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
    except Exception as e:
      print("mujoco_rbx_bridge: could not open a viewer window (%s) -- "
            "continuing headless" % str(e), flush = True)

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
    vx_world = 0.0
    vy_world = 0.0
    if self._last_t is not None:
      dt = now - self._last_t
      if dt > 1e-6:
        vx_world = (x - self._last_x) / dt
        vy_world = (y - self._last_y) / dt
        lin_mps = math.hypot(vx_world, vy_world)
        ang_radps = self.normalizeAngle(yaw - self._last_yaw) / dt
    self._last_x, self._last_y, self._last_yaw, self._last_t = x, y, yaw, now

    with self.pose_lock:
      self.x_m, self.y_m, self.yaw_rad = x, y, yaw
      self.lin_mps, self.ang_radps = lin_mps, ang_radps
      self.vx_world, self.vy_world = vx_world, vy_world

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
      lin, lin_y, ang = self.cmd_linear_x, self.cmd_linear_y, self.cmd_angular_z
      wheel_independence = self.wheel_independence

    if wheel_independence:
      self.applySwerveVelocity(lin, lin_y, ang)
      return

    left_radps = (lin - ang * self.wheel_track_m / 2.0) / self.wheel_radius_m
    right_radps = (lin + ang * self.wheel_track_m / 2.0) / self.wheel_radius_m
    left_radps = max(-MAX_WHEEL_RADPS, min(MAX_WHEEL_RADPS, left_radps))
    right_radps = max(-MAX_WHEEL_RADPS, min(MAX_WHEEL_RADPS, right_radps))
    # wheel1=front_left, wheel2=front_right, wheel3=rear_left, wheel4=rear_right
    # Straight ahead (0, or +-pi reversed if that's where the wheel already
    # is after crab mode) -- so switching modes never swings a wheel round.
    for w, radps in zip(self.wheels, (left_radps, right_radps, left_radps, right_radps)):
      self.driveWheel(w, 0.0, radps * self.wheel_radius_m, force_steer = True)

  def driveWheel(self, w, theta, speed, force_steer = False):
    # Point wheel w along theta (chassis frame) and roll it at speed (m/s,
    # signed). Of the equivalent angles (theta, or theta +- pi with the wheel
    # reversed), takes the one nearest the wheel's current angle that is
    # still inside the joint's range, so a wheel never swings the long way
    # round and a 180-degree direction change is a reverse, not a half-turn.
    current = float(self.data.qpos[w['steer_qadr']])
    if force_steer or abs(speed) > STEER_HOLD_SPEED_MPS:
      best, best_sign = None, 1.0
      for cand, sign in ((theta, 1.0), (theta + math.pi, -1.0), (theta - math.pi, -1.0)):
        if abs(cand) > self.steer_range_rad:
          continue
        if best is None or abs(cand - current) < abs(best - current):
          best, best_sign = cand, sign
      w['steer_target'] = best
      speed = best_sign * speed
    else:
      # Nearly stopped: hold the last angle instead of re-deriving it from
      # a ~zero vector, so wheels don't flick straight on every pause.
      speed = 0.0
    # Don't push while the wheel is still well off its target angle, or it
    # drives the chassis the wrong way during the swing.
    err = abs(w['steer_target'] - current)
    alignment = math.cos(err) if err < STEER_ALIGN_GATE_RAD else 0.0
    radps = speed * alignment / self.wheel_radius_m
    step = STEER_SLEW_RADPS * self.model.opt.timestep
    w['steer_cmd'] += max(-step, min(step, w['steer_target'] - w['steer_cmd']))
    self.data.ctrl[w['steer_act']] = w['steer_cmd']
    self.data.ctrl[w['spin_act']] = max(-MAX_WHEEL_RADPS, min(MAX_WHEEL_RADPS, radps))

  def applySwerveVelocity(self, vx, vy, vyaw):
    # Crab steer: body-frame (vx, vy, vyaw) -> each wheel's own required
    # ground velocity v = v_body + omega x r, r = the module's (x, y) offset.
    # The chassis is never driven directly -- it moves only because the
    # wheels push it, so it still collides with obstacles like any body.
    qw, qx, qy, qz = self.data.qpos[3:7]
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    if abs(vyaw) < YAW_HOLD_EPS:
      if self.held_yaw is None:
        self.held_yaw = yaw
      err = self.normalizeAngle(self.held_yaw - yaw)
      vyaw = max(-MAX_YAW_HOLD_RATE, min(MAX_YAW_HOLD_RATE, YAW_HOLD_KP * err))
    else:
      self.held_yaw = None

    for w in self.wheels:
      wvx = vx - vyaw * w['y']
      wvy = vy + vyaw * w['x']
      self.driveWheel(w, math.atan2(wvy, wvx), math.hypot(wvx, wvy))

  def resetSim(self):
    # Genuine reset -- see module docstring. Clears commanded velocity and
    # finite-difference state too, so telemetry doesn't report a stale
    # lin/ang spike computed against the pre-reset pose.
    with self.cmd_lock:
      self.cmd_linear_x = 0.0
      self.cmd_linear_y = 0.0
      self.cmd_angular_z = 0.0
    self.held_yaw = None
    for w in self.wheels:
      w['steer_target'] = 0.0
      w['steer_cmd'] = 0.0
    mujoco.mj_resetData(self.model, self.data)
    mujoco.mj_forward(self.model, self.data)
    self._last_x, self._last_y, self._last_yaw, self._last_t = 0.0, 0.0, 0.0, None
    print("mujoco_rbx_bridge: reset to initial pose", flush = True)

  def _setGeomGroupEnabled(self, geom_ids, enabled):
    # Idempotent, same reasoning as webots_rbx_bridge.py's own toggle.
    # geom_rgba/contype/conaffinity live on the MODEL (not mjData), so
    # resetSim's mj_resetData never touches this -- a sim reset doesn't
    # silently remove/restore the environment, matching Gazebo/Webots' own
    # behavior (RESET_SIM only teleports the robot, never the environment).
    alpha = 1.0 if enabled else 0.0
    contype = 1 if enabled else 0
    for gid in geom_ids:
      self.model.geom_rgba[gid][3] = alpha
      self.model.geom_contype[gid] = contype
      self.model.geom_conaffinity[gid] = contype

  def setEnvironment(self, environment_value):
    # Exactly one environment geom group visible/colliding at a time --
    # rbx_mujoco_node.py's own environment Setting is a single-select, same
    # as every other simulator here. Idempotent early-return -- the device
    # now resends this periodically as a connection-drop self-heal (see
    # rbx_mujoco_node.py's settingsResyncCb), not just on a real change.
    if environment_value == self.current_environment:
      return
    self.current_environment = environment_value
    self._setGeomGroupEnabled(self.obstacle_course_geom_ids,
                              environment_value == "OBSTACLE_COURSE")
    self._setGeomGroupEnabled(self.custom_obstacles_geom_ids,
                              environment_value == "CUSTOM_OBSTACLES")
    self._setGeomGroupEnabled(self.aerial_obstacle_course_geom_ids,
                              environment_value == "AERIAL_OBSTACLE_COURSE")
    print("mujoco_rbx_bridge: environment set to %s" % environment_value, flush = True)

  def refreshEnvironment(self):
    # Live edits to the environment dimensions (Obstacle Course's walls/
    # baffles/ramp, or the Custom Obstacles list) only reach the compiled
    # geoms by rebuilding the model -- see generate_environment_xml.py's
    # own docstring. Rebuilds a fresh MjModel/MjData and re-resolves every
    # id/handle that pointed into the old one (_bindModel), but keeps the
    # rover's current pose/velocity and commanded state, so this reads as
    # "the environment changed," not an unexpected RESET_SIM.
    saved_environment = self.current_environment
    old_qpos, old_qvel = self.data.qpos.copy(), self.data.qvel.copy()
    old_cmd = (self.cmd_linear_x, self.cmd_linear_y, self.cmd_angular_z)
    old_wheel_state = [(w['steer_target'], w['steer_cmd']) for w in self.wheels]
    self._bindModel(initial = False)
    # _bindModel force-disables every environment geom group on the fresh
    # model, but setEnvironment below is now idempotent (see its own
    # comment) and would otherwise treat "still saved_environment" as
    # "nothing to do" and skip re-enabling them -- forcing a mismatch here
    # guarantees the reapply actually runs.
    self.current_environment = None
    if self.data.qpos.shape == old_qpos.shape and self.data.qvel.shape == old_qvel.shape:
      self.data.qpos[:] = old_qpos
      self.data.qvel[:] = old_qvel
      mujoco.mj_forward(self.model, self.data)
    if len(old_wheel_state) == len(self.wheels):
      for w, (target, cmd) in zip(self.wheels, old_wheel_state):
        w['steer_target'], w['steer_cmd'] = target, cmd
    with self.cmd_lock:
      self.cmd_linear_x, self.cmd_linear_y, self.cmd_angular_z = old_cmd
    self.setEnvironment(saved_environment)
    if self.last_camera_settings is not None:
      self.applyCameraSettings(self.last_camera_settings)
    self._openViewer()
    print("mujoco_rbx_bridge: environment refreshed", flush = True)

  def applyCameraSettings(self, msg):
    # Cached so refreshEnvironment can reapply it after rebuilding the
    # model -- _bindModel recomputes factory_robot_cam_pos/
    # factory_scene_cam_pos from the FRESH model's own un-offset XML
    # values, so without this a refresh silently snapped both cameras
    # back to their factory position/FOV, discarding whatever offset was
    # live at the time.
    self.last_camera_settings = dict(msg)
    # Position offsets are a plain delta from each camera's factory mount
    # point, written directly into model.cam_pos -- mj_kinematics re-reads
    # this every step, so no respawn/reset is needed for it to take effect
    # (simpler than webots_rbx_bridge.py's Supervisor field write, since
    # MuJoCo already exposes this as a mutable per-model array).
    #
    # offset_yaw/offset_tilt/scene_offset_yaw/scene_offset_tilt ADDED
    # (2026-09-22) -- ABSOLUTE angles (degrees), not deltas, same convention
    # rbx_sim_node.py/sim_bridge_node.py already use for Gazebo. Written
    # into model.cam_quat via yawTiltToQuat (module level, below), which
    # reproduces both cameras' own factory xyaxes exactly at yaw=0: (0,-1,0)/
    # (0,0,1) for robot_camera (tilt=0) and (0,-1,0)/(0.5514,0,0.8342) for
    # scene_camera (tilt=FACTORY_SCENE_TILT_DEG) -- see that function's own
    # comment for the derivation.
    try:
      self.model.cam_pos[self.robot_cam_id] = self.factory_robot_cam_pos + np.array([
          float(msg.get('offset_x', 0.0)), float(msg.get('offset_y', 0.0)), float(msg.get('offset_z', 0.0))])
      self.model.cam_quat[self.robot_cam_id] = yawTiltToQuat(
          float(msg.get('offset_yaw', 0.0)), float(msg.get('offset_tilt', 0.0)))
      self.model.cam_pos[self.scene_cam_id] = self.factory_scene_cam_pos + np.array([
          float(msg.get('scene_offset_x', 0.0)), float(msg.get('scene_offset_y', 0.0)), float(msg.get('scene_offset_z', 0.0))])
      self.model.cam_quat[self.scene_cam_id] = yawTiltToQuat(
          float(msg.get('scene_offset_yaw', 0.0)), float(msg.get('scene_offset_tilt', 0.0)))
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
      vx_world, vy_world = self.vx_world, self.vy_world
    with self.cmd_lock:
      wheel_independence = self.wheel_independence
    # Matches sim_bridge_node.py/webots_rbx_bridge.py's bare-telemetry shape
    # exactly: x/y/yaw plus linear_x/angular_z, no "type" key.
    return {
        "x": x_m, "y": y_m, "yaw": yaw_rad,
        "linear_x": lin_mps, "angular_z": ang_radps,
        # Extra keys (rbx_mujoco_node.py reads them if present): real
        # world-frame velocity, and the current crab-steer mode.
        "vx_world": vx_world, "vy_world": vy_world,
        "wheel_independence": wheel_independence,
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
        self.cmd_linear_y = float(msg.get("linear_y", 0.0))
        self.cmd_angular_z = float(msg.get("angular_z", 0.0))
      return
    msg_type = msg.get("type")
    if msg_type == "wheel_independence":
      enabled = bool(msg.get("enabled", False))
      with self.cmd_lock:
        changed = (enabled != self.wheel_independence)
        self.wheel_independence = enabled
      if changed:
        self.held_yaw = None
        print("mujoco_rbx_bridge: wheel independence %s" % ("on" if enabled else "off"), flush = True)
    elif msg_type == "camera_settings":
      self.applyCameraSettings(msg)
    elif msg_type == "reset":
      self.resetSim()
    elif msg_type == "set_environment":
      self.setEnvironment(msg.get("environment", "FLAT_GROUND"))
    elif msg_type == "refresh_environment":
      self.refreshEnvironment()


def main():
  heartbeat_port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_HEARTBEAT_PORT
  bridge_port = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BRIDGE_PORT
  bridge = MujocoRbxBridge(heartbeat_port, bridge_port)
  bridge.run()


if __name__ == "__main__":
  main()
