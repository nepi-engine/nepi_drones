#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_drivers) repo
# (see https://https://github.com/nepi-engine/nepi_drivers)
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

# RBX driver node for the Gazebo simulated rover (RBX_SIM), following
# rbx_ardupilot_node.py's RBXRobotIF integration pattern. The simulator runs
# on a dev VM with its own ROS master, reachable from this device solely
# through a reverse SSH tunnel forwarding raw TCP ports, so this node holds a
# persistent TCP connection to the VM's sim_bridge_node.py (host/port from
# DEVICE_DICT) speaking newline-delimited JSON: velocity commands out
# ({"linear_x","angular_z"}, republished onto the VM's /nepi/sim/cmd_vel),
# odometry telemetry in ({"x","y","yaw","linear_x","angular_z","stamp"},
# pushed at 10 Hz from the VM's /rover/odom).
#
# Unlike the ArduPilot driver there is no onboard autopilot to delegate goto
# setpoints to: the rover only understands instantaneous velocity, so this
# node implements its own closed-loop 2D controller (gotoControlCb) that
# drives toward the RBXRobotIF-supplied position/yaw target at 10 Hz until
# within the RBX error bounds. The rover has no ARM/DISARM or flight-mode
# equivalent, no battery, and no geographic (WGS84) location/goto, so
# states, modes, gotoLocation, and the battery callback are legitimately
# empty/None (RBXRobotIF reports the matching has_* capabilities False).
# Home/set_home *is* wired (getHome/setHome below), reusing RBXRobotIF's
# GeoPoint-shaped home plumbing with its three floats reinterpreted as
# local ENU x/y/z meters instead of lat/long/alt degrees -- the only home
# concept this rover has, since it has no WGS84 reference of its own.

import base64
import copy
import json
import math
import socket
import threading
import time

import numpy as np
import cv2

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_settings
from nepi_sdk import nepi_img

from std_msgs.msg import UInt32, String
from sensor_msgs.msg import Image

from nepi_interfaces.msg import AxisControls
from geographic_msgs.msg import GeoPoint

from nepi_api.device_if_rbx import RBXRobotIF
from nepi_api.messages_if import MsgIF

PKG_NAME = 'RBX_SIM' # Use in display menus
FILE_TYPE = 'NODE'


#########################################
# Node Class
#########################################

class SimNode:

  # Camera-rover feature. Both cameras are rigid links welded onto
  # generic_rover itself (fixed joint poses in generic_rover/model.sdf), not a
  # single repositionable rig -- that's what makes them lag-free. Both are now
  # relayed simultaneously as six always-live ROS Image topics (see the
  # *_TOPIC_SUFFIX constants below: color, colorized depth view, and raw
  # depth map, for each of robot/scene), not switched between via a
  # camera_view_mode Setting: reworked (2026-08-18) after a live report that
  # a single topic whose content gets reassigned depending on a mode setting
  # isn't a real second view a client can rely on ("the third-person view
  # doesn't really exist"), then widened again (2026-08-20) so color and
  # depth are simultaneously available instead of a depth_map_enabled toggle
  # swapping one topic's content. The existing Image Source dropdown's
  # find_topics_by_msg('Image') discovery picks up all of them with no new
  # RUI plumbing needed (see NepiDeviceRBX.js's createImageOptions).
  #
  # Camera offsets ARE still runtime-adjustable, a separate concept from which
  # view a client is looking at: an offset change is applied by respawning the
  # rover model with new camera link poses, at its current world pose. That is
  # viable specifically because the diff_drive plugin uses
  # <odometrySource>world</odometrySource>, so odom is read back from the
  # model's Gazebo pose and survives the respawn unbroken. See
  # respawnRoverWithCameraOffsets in sim_bridge_node.py, which also records
  # every Gazebo mechanism that does NOT work here.
  #
  # camera_offset_* is the ROBOT view (same names the ArduPilot driver uses, so
  # the RUI's existing offset block renders it unchanged); scene_offset_* is the
  # scene/chase view. Factory values reproduce generic_rover/model.sdf's
  # hard-coded camera_link (0.2, 0, 0.65) and camera_link_chase
  # (-2.5, 0, 1.65) poses exactly, so the default view is unchanged.
  #
  # *_yaw/*_tilt added (2026-09-03) -- reported live: "for the camera offsets
  # as well, yaw and tilt should also be editable. right now its just x y and
  # z." Both in DEGREES, matching this app's own existing angle convention
  # (camera_horizontal_fov_deg, max_angular_rate_dps) -- sim_bridge_node.py
  # converts to radians only when building the SDF <pose> string. "tilt"
  # (the RUI-facing name the operator used) maps internally to SDF pitch;
  # roll stays fixed at 0 for both cameras, unchanged, not exposed as a
  # setting -- matches generic_rover/model.sdf's own pose convention, where
  # neither camera has ever had roll. Factory values reproduce
  # generic_rover/model.sdf's hard-coded rotations exactly: camera_link has
  # none (0, 0, 0) -- FACTORY_CAMERA_LINK_TILT_DEG stays 0.0 -- and
  # camera_link_chase's own hard-coded downward-look pitch (0.5404195 rad)
  # is what FACTORY_SCENE_TILT_DEG below reproduces in degrees.
  CAMERA_SETTING_NAMES = ("camera_offset_x", "camera_offset_y", "camera_offset_z",
                          "camera_offset_yaw", "camera_offset_tilt",
                          "scene_offset_x", "scene_offset_y", "scene_offset_z",
                          "scene_offset_yaw", "scene_offset_tilt")

  # Suffixes for the six always-live ROS topics published off
  # self.image_topic_name -- see processImageLine's routing by the bridge
  # line's "camera" field. color/depth (colorized, for viewing) replace the
  # old robot_view/scene_view pair once depth_map_enabled was removed
  # (2026-08-20) in favor of always publishing both simultaneously rather
  # than toggling which one a single topic shows; depth_map is the raw
  # 32FC1-meters data, for later processing rather than viewing.
  ROBOT_COLOR_TOPIC_SUFFIX = "robot_color"
  SCENE_COLOR_TOPIC_SUFFIX = "scene_color"
  ROBOT_DEPTH_TOPIC_SUFFIX = "robot_depth"
  SCENE_DEPTH_TOPIC_SUFFIX = "scene_depth"
  ROBOT_DEPTH_MAP_TOPIC_SUFFIX = "robot_depth_map"
  SCENE_DEPTH_MAP_TOPIC_SUFFIX = "scene_depth_map"

  # Environment: was originally two RBX_SETUP_ACTIONS entries
  # (OBSTACLE_COURSE_ON/OFF), then a dedicated "Environment" dropdown was
  # added to the RUI that still just sent those two setup actions under the
  # hood -- redundant with itself once that dropdown existed, and the setup
  # actions kept cluttering the generic Setup Actions dropdown alongside
  # RESET_SIM/RETURN_HOME. Modeled as a Setting instead: one clean value, one
  # place it lives.
  ENVIRONMENT_SETTING_NAMES = ("environment",)

  # Which control SURFACES this deployment wants exposed, as opposed to which
  # ones this robot TYPE structurally supports (that's still
  # has_autonomous_controls/has_manual_controls etc on the capabilities query,
  # untouched). These are the Sim Connector's own robot-config "customize the
  # capabilities that are open" controls. Modeled as Settings, same mechanism
  # as camera_offset_x, so:
  #   - the RUI needs no new capability-query plumbing, just the existing
  #     settingsNamesList/settingsValuesDict gate (see NepiDeviceRBX-Controls.js)
  #   - the value is "always editable" (the user's own requirement) rather
  #     than a one-shot capability decided at construction and frozen
  #   - toggling it takes real effect at the DRIVER too (autonomousControlsReady
  #     below), not just in the RUI, so a client that bypasses the RUI can't
  #     do what was turned off either.
  # camera_controls_enabled: same toggle mechanism, gating whether
  # camera_offset_*/scene_offset_* show up on the RUI at all (see
  # NepiDeviceRBX.js's has_camera_offsets/has_scene_offsets, which AND this
  # in alongside the Setting-existence
  # check). Unlike autonomous movement, this has no driver-side
  # enforcement point to add -- there is no "camera controls ready" concept,
  # positioning a Gazebo camera can't fail the way a goto or a motor command
  # can -- so it is a pure visibility toggle, RUI-side only.
  #
  # enabled_image_sources: comma-separated allowlist of image topic names for
  # NepiDeviceRBX.js's own Image_Source dropdown (createImageOptions) --
  # "choose what image sources are good and what aren't". Empty (the factory
  # value) means unrestricted -- every image topic createImageOptions would
  # otherwise offer stays offered, so a robot config that never touches this
  # Setting is completely unaffected. A String Setting, not a Discrete list,
  # since the candidate topic set is per-deployment and can't be enumerated as
  # fixed Discrete options the way TRUE/FALSE can.
  CAPABILITY_SETTING_NAMES = ("autonomous_movement_enabled",
                              "camera_controls_enabled", "enabled_image_sources")

  # generic_rover/model.sdf's own hard-coded camera_link_chase pitch, in
  # degrees -- FACTORY_SETTINGS' scene_offset_tilt reproduces this exactly so
  # an unconfigured robot config's default view is unchanged. camera_link
  # itself has no rotation at all (0 rad), so no equivalent constant is
  # needed for camera_offset_tilt's own factory value (just "0.0" below).
  FACTORY_SCENE_TILT_DEG = math.degrees(0.5404195)

  # generic_rover/model.sdf's own hard-coded camera_link_chase POSITION
  # (body-frame, relative to the rover's own origin) -- requested live
  # (2026-09-04): "for the scene view cam, the 0 0 0 point should be set
  # to wherever the cam is by default, not where the center of the robot
  # is, so its easier for viewers to refer off that." scene_offset_x/y/z
  # is now a DELTA from this factory mount point, not an absolute
  # robot-frame coordinate -- FACTORY_SETTINGS below is "0.0" for all
  # three (no delta = stock position), and sim_bridge_node.py's own
  # respawnRoverWithCameraOffsets adds these same three constants back in
  # before writing the actual SDF pose (see that function's own comment).
  FACTORY_SCENE_OFFSET_X = -2.5
  FACTORY_SCENE_OFFSET_Y = 0.0
  FACTORY_SCENE_OFFSET_Z = 1.65

  CAP_SETTINGS = dict(
    max_linear_speed_mps = {"type":"Float","name":"max_linear_speed_mps","options":["0.05","5.0"]},
    max_angular_rate_dps = {"type":"Float","name":"max_angular_rate_dps","options":["5.0","180.0"]},
    environment = {"type":"Discrete","name":"environment","options":["FLAT_GROUND","OBSTACLE_COURSE"]},
    camera_offset_x = {"type":"Float","name":"camera_offset_x","options":["-10.0","10.0"]},
    camera_offset_y = {"type":"Float","name":"camera_offset_y","options":["-10.0","10.0"]},
    camera_offset_z = {"type":"Float","name":"camera_offset_z","options":["-10.0","10.0"]},
    camera_offset_yaw = {"type":"Float","name":"camera_offset_yaw","options":["-180.0","180.0"]},
    camera_offset_tilt = {"type":"Float","name":"camera_offset_tilt","options":["-90.0","90.0"]},
    scene_offset_x = {"type":"Float","name":"scene_offset_x","options":["-10.0","10.0"]},
    scene_offset_y = {"type":"Float","name":"scene_offset_y","options":["-10.0","10.0"]},
    scene_offset_z = {"type":"Float","name":"scene_offset_z","options":["-10.0","10.0"]},
    scene_offset_yaw = {"type":"Float","name":"scene_offset_yaw","options":["-180.0","180.0"]},
    scene_offset_tilt = {"type":"Float","name":"scene_offset_tilt","options":["-90.0","90.0"]},
    autonomous_movement_enabled = {"type":"Discrete","name":"autonomous_movement_enabled","options":["TRUE","FALSE"]},
    camera_controls_enabled = {"type":"Discrete","name":"camera_controls_enabled","options":["TRUE","FALSE"]},
    # No fixed options -- the candidate topic set is per-deployment.
    enabled_image_sources = {"type":"String","name":"enabled_image_sources"}
  )

  FACTORY_SETTINGS = dict(
    max_linear_speed_mps = {"type":"Float","name":"max_linear_speed_mps","value":"0.5"},
    max_angular_rate_dps = {"type":"Float","name":"max_angular_rate_dps","value":"45.0"},
    environment = {"type":"Discrete","name":"environment","value":"FLAT_GROUND"},
    # Reproduces generic_rover/model.sdf's hard-coded camera_link pose exactly.
    camera_offset_x = {"type":"Float","name":"camera_offset_x","value":"0.2"},
    camera_offset_y = {"type":"Float","name":"camera_offset_y","value":"0.0"},
    camera_offset_z = {"type":"Float","name":"camera_offset_z","value":"0.65"},
    camera_offset_yaw = {"type":"Float","name":"camera_offset_yaw","value":"0.0"},
    camera_offset_tilt = {"type":"Float","name":"camera_offset_tilt","value":"0.0"},
    # Zero delta from generic_rover/model.sdf's own hard-coded
    # camera_link_chase mount point (FACTORY_SCENE_OFFSET_X/Y/Z) -- see that
    # constant's own comment for why this is 0.0 now instead of the
    # absolute -2.5/0.0/1.65 pose.
    scene_offset_x = {"type":"Float","name":"scene_offset_x","value":"0.0"},
    scene_offset_y = {"type":"Float","name":"scene_offset_y","value":"0.0"},
    scene_offset_z = {"type":"Float","name":"scene_offset_z","value":"0.0"},
    scene_offset_yaw = {"type":"Float","name":"scene_offset_yaw","value":"0.0"},
    scene_offset_tilt = {"type":"Float","name":"scene_offset_tilt","value":str(FACTORY_SCENE_TILT_DEG)},
    # Both default to enabled: a robot config that never touches these
    # settings behaves exactly as every robot config did before this feature
    # existed.
    autonomous_movement_enabled = {"type":"Discrete","name":"autonomous_movement_enabled","value":"TRUE"},
    camera_controls_enabled = {"type":"Discrete","name":"camera_controls_enabled","value":"TRUE"},
    # Empty = unrestricted -- see the CAPABILITY_SETTING_NAMES comment above.
    enabled_image_sources = {"type":"String","name":"enabled_image_sources","value":""}
  )

  FACTORY_SETTINGS_OVERRIDES = dict()

  # A generic differential-drive rover has no ARM/DISARM or flight-mode
  # machinery -- RBXRobotIF handles empty lists correctly (bounds checks
  # reject any set_state/set_mode index, status shows "Not Set"), so no
  # placeholder entries are invented. The get*Ind functions must still be
  # real callables (RBXRobotIF calls them unconditionally).
  RBX_STATES = []
  RBX_MODES = []
  # Setup Actions dropdown (RUI's "Setup Controls" section, wired to
  # setSetupActionInd below): a rover has no ARM/mode state machine, but
  # does need two one-shot commands: an instant physics teleport back to
  # the Gazebo spawn pose (RESET_SIM, mirroring rbx_ardupilot_node.py's
  # RESET_SIM setup action, but here reached over the sim_bridge TCP
  # connection instead of the ArduPilot gz_reset_listener socket -- this
  # driver has no direct route to the VM's Gazebo otherwise), and driving
  # back to the user-settable home position under closed-loop control
  # (RETURN_HOME, also wired as the RBXRobotIF goHomeFunction so the
  # standard "Go Home" control works too). Obstacle-course switching used to
  # live here too (OBSTACLE_COURSE_ON/OFF) -- moved to the environment
  # Setting above once a dedicated RUI dropdown made the setup-action version
  # redundant with itself.
  RBX_SETUP_ACTIONS = ["RESET_SIM", "RETURN_HOME"]
  # Go-actions dropdown (RUI's "selected_go_action" control, wired to
  # setGoActionInd below). Camera view switching used to live here
  # (VIEW_FIRST_PERSON/VIEW_THIRD_PERSON), then moved to a camera_view_mode
  # Settings-tab toggle -- both are gone now that robot/scene view are two
  # always-live ROS Image topics selected via the ordinary Image Source
  # dropdown, same as any other camera topic (see CAMERA_SETTING_NAMES's own
  # comment).
  RBX_GO_ACTIONS = []

  # GO_HOME polls the same goto_target the gotoPosition/gotoPose setpoint
  # functions drive (gotoControlCb clears it on arrival) rather than
  # returning immediately, matching how RBXRobotIF's go_action dispatch
  # blocks the calling topic callback on this function's return value to
  # determine success/failure.
  GO_HOME_TIMEOUT_SEC = 60.0
  GO_HOME_POLL_INTERVAL_SEC = 0.2

  RECONNECT_INTERVAL_SEC = 3.0
  SOCKET_TIMEOUT_SEC = 5.0

  # Bounded window, at startup only, for the VM to report its available
  # environment models (see processEnvironmentOptionsLine) before this
  # driver's capabilities are baked into RBXRobotIF -- RBX capabilities have
  # no live-refresh path once constructed (see
  # docs/SCAN_TO_SIM_ENVIRONMENT_PLAN.md section 2.1), so this is the only
  # window this process gets. Same scale as SOCKET_TIMEOUT_SEC; never blocks
  # startup indefinitely -- if the VM doesn't answer in time, CAP_SETTINGS'
  # environment.options (below) and ENVIRONMENT_VALUE_TO_MODEL (just below
  # that) simply keep the two literal values they already had, unchanged
  # from this driver's pre-scanned-environments behavior.
  ENVIRONMENT_OPTIONS_WAIT_SEC = 5.0

  # Discrete-option-value -> spawnable model name, updated in place by
  # processEnvironmentOptionsLine when the VM reports scanned models
  # (environment_models.py, VM-side). Starts as this literal two-entry
  # mapping (FLAT_GROUND -> nothing spawned, OBSTACLE_COURSE -> the one
  # model this driver has always known about) so setEnvironmentAction always
  # has something to look up even if the VM never answers.
  ENVIRONMENT_VALUE_TO_MODEL = {"FLAT_GROUND": None, "OBSTACLE_COURSE": "obstacle_course"}

  # Raised from 10 -- at the higher end of the now-settable max_linear_speed_mps
  # range (up to 5.0 m/s), a 10Hz control tick lets the rover travel up to 0.5m
  # between heading corrections, which is coarse enough to overshoot and
  # oscillate. Finer ticks reduce that per-tick travel distance, independent of
  # (and in addition to) the wheelTorque headroom bump in generic_rover/model.sdf.
  CONTROLLER_RATE_HZ = 20
  NAVPOSE_UPDATE_RATE = 10
  TELEMETRY_FRESH_SEC = 2.0

  # Manual motor-ratio tank-drive conversion (RBX_EXTERNAL_HARDWARE_INTERFACES.md
  # worked example, section 6): motor 0 = left, motor 1 = right. Converted to
  # the same linear/angular Twist gotoControlCb already sends, via standard
  # differential-drive kinematics -- no new Gazebo model/plugin or bridge
  # message needed, since this device's controller loop already owns sending
  # velocity every tick. Per MotorControl.msg, speed_ratio is a 0-1 magnitude
  # with no reverse/direction bit, so this can drive straight and
  # differentially steer but cannot reverse or spin in place -- an honest
  # limitation of the wire format, not this conversion. MOTOR_WHEEL_BASE_M is
  # an approximation of generic_rover's actual wheel track width.
  #
  # No MOTOR_MAX_LINEAR_MPS constant here on purpose (removed 2026-08-20) --
  # motorControlToVelocity reads settings_dict['max_linear_speed_mps'] live,
  # same as gotoControlCb already does, so 100% motor always means "whatever
  # max_linear_speed_mps currently is," and changing that Setting rescales
  # manual motor control instantly too. A hardcoded 0.5 here used to silently
  # match only the Setting's OWN factory default -- changing
  # max_linear_speed_mps rescaled goto but left manual motor control's 100%
  # pinned at the old value, found live 2026-08-20.
  MOTOR_WHEEL_BASE_M = 0.4

  # Closed-loop goto controller shape: proportional gains, plus a
  # turn-in-place gate so the rover rotates toward the target bearing
  # before driving (differential drive can pivot on the spot).
  GOTO_KP_LIN = 0.5       # (m/s per m of distance error)
  GOTO_KP_ANG = 1.5       # (rad/s per rad of heading error)
  GOTO_TURN_GATE_RAD = math.radians(30.0)
  # Stop inside half the RBX max_distance/rotation error bounds so the
  # interface's own convergence check (error < bound sustained for the
  # stabilize window) passes cleanly instead of hovering at the edge.
  GOTO_TOL_FRACTION = 0.5
  # Fallback tolerances if the RBX interface is not up yet (matches the
  # GOTO_TOL_FRACTION of RBXRobotIF's factory 2.0m / 2.0deg error bounds)
  FACTORY_GOTO_TOL_M = 1.0
  FACTORY_GOTO_TOL_RAD = math.radians(1.0)

  # RBXRobotIF's blocking setpoint wait (setpoint_position_local_body) uses a
  # single fixed cmd_timeout for both the drive and final-yaw phases. A
  # non-holonomic differential-drive rover can legitimately need close to a
  # full 360 degrees of cumulative turning for a single goto (turn to face
  # the bearing, drive, then turn again to the requested final yaw if it
  # differs from the bearing -- confirmed this can require two ~180 degree
  # turns back to back) plus the full commanded travel distance at the
  # capped max_linear_speed_mps. The RBXRobotIF factory default (25s) was
  # observed to be too short for this: a 6m body-frame goto requiring two
  # ~180 degree turns logged "Setpoint cmd timed out" (cmd_success False)
  # at 25s even though the driver's own gotoControlCb went on to converge
  # and log "Goto target reached" ~2.5s later using its own (independently
  # correct) tolerance check. Raised here to comfortably cover a full
  # reorientation (360deg at the factory max_angular_rate_dps of 45 = 8s)
  # plus a generous single-command travel distance for a Gazebo world (20m
  # at the factory max_linear_speed_mps of 0.5 = 40s) with margin -- a
  # heuristic sized to this driver's factory speed defaults and a
  # "reasonably-sized simulated world", not a guarantee for arbitrarily
  # long single commands or a user who raises max speeds. See the Phase 5
  # session summary for the full investigation and the broader, deferred
  # design question this points to (RBXRobotIF's fixed-timeout model
  # doesn't scale with maneuver complexity for non-holonomic ground
  # vehicles in general).
  GOTO_CMD_TIMEOUT_SEC = 60

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = PKG_NAME.lower() + "_node"
  drv_dict = dict()

  def __init__(self):
    ####  NODE Initialization ####
    nepi_sdk.init_node(name = self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_sdk.get_base_namespace()
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()

    ##############################
    # Create Msg Class
    self.msg_if = MsgIF(log_name = self.class_name)
    self.msg_if.pub_info("Starting Node Initialization Processes")

    ##############################
    # Gather Driver Settings from param server drv_dict -- rbx_sim_discovery.py
    # sets this param on the SAME master an instant before launching this
    # node, but right after a fresh reboot (cold rosmaster, cold DNS/hostname
    # resolution for this container's own hostname) that set can still be in
    # flight when this process's own get_param call lands, reading back the
    # empty default and self-shutting-down over what is really just a
    # startup race, not a genuinely missing/invalid dict (confirmed live,
    # 2026-09-01: reproduced a full-node restart loop this way immediately
    # after a device+VM reboot; the SAME launch succeeded instantly once the
    # system had been up a few minutes). Retries a few times with a short
    # backoff before giving up for real.
    DRV_DICT_READ_RETRIES = 5
    DRV_DICT_READ_RETRY_SEC = 0.5
    self.drv_dict = dict()
    last_error = None
    for attempt in range(DRV_DICT_READ_RETRIES):
      self.drv_dict = nepi_sdk.get_param('~drv_dict', dict())
      try:
        self.device_name = self.drv_dict['DEVICE_DICT']['device_name']
        self.device_path = self.drv_dict['DEVICE_DICT']['device_path']
        self.sim_host = self.drv_dict['DEVICE_DICT']['sim_host']
        self.bridge_port = self.drv_dict['DEVICE_DICT']['bridge_port']
        last_error = None
        break
      except Exception as e:
        last_error = e
        nepi_sdk.sleep(DRV_DICT_READ_RETRY_SEC)
    if last_error is not None:
        self.msg_if.pub_warn("Failed to load Device Dict " + str(last_error))
        nepi_sdk.signal_shutdown(self.node_name + ": Shutting down because no valid Device Dict")
        return

    ##############################
    # Bridge connection and telemetry state
    self.sock = None
    self.sock_lock = threading.Lock()
    self.last_telemetry_time = 0.0
    self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)

    ##############################
    # Camera-rover feature: publish decoded frames on a bare-relative topic
    # name that RBXRobotIF's find_topic()-based image subscriber is pointed
    # at via set_image_topic (see below).
    #
    # Deliberately a BARE relative name, not
    # nepi_sdk.create_namespace(self.node_namespace, ...): confirmed by direct
    # test (rostopic info after deploy) that RBXRobotIF's own subscribe --
    # also a bare relative rospy.Subscriber(...), since find_topic() returns
    # the search string itself rather than the matched full topic path --
    # resolves against a namespace SHARED BY EVERY DRIVER NODE ON THIS DEVICE
    # (/nepi/<device>, not this node's own self.node_namespace), because
    # nepi_drvs.launchDriverNode only remaps __name, never __ns -- every
    # rbx_sim node process inherits the same default ROS namespace.
    #
    # Single-robot camera-rover phase used RBXRobotIF's plain factory default
    # ("color_2d_image", its FACTORY_IMAGE_INPUT_TOPIC_NAME) unmodified,
    # since with only one instance nothing else could collide with it.
    # Camera-rover-multi phase found (real cross-talk, not assumed): with two
    # live sim_rover instances, BOTH processes' bare "color_2d_image" publish
    # AND both RBXRobotIF instances' bare "color_2d_image" find_topic()
    # subscribe resolve to the exact same shared /nepi/<device>/color_2d_image
    # topic (confirmed via rosnode info on the live single-instance deploy,
    # which already showed /nepi/device1/sim_rover1 publishing AND
    # subscribing that unnamespaced topic) -- a second instance would publish
    # to and read from the identical topic, so each robot's frames would
    # bleed into the other's image data product. Fix: bake this node's own
    # device_name (sim_rover1/sim_rover2, already unique per
    # SIM_ROBOT_SLOTS/DEVICE_DICT) into the bare-relative string itself, on
    # both the publish and the find_topic() search string (via
    # set_image_topic below) -- still a bare relative name (still resolves
    # against the shared device-wide namespace), but now distinguished by
    # content, so /nepi/<device>/sim_rover1/color_2d_image and
    # .../sim_rover2/color_2d_image never collide.
    # Six always-live topics (robot/scene x color/depth-view/depth_map), not
    # one bare topic -- see CAMERA_SETTING_NAMES's own comment for why.
    # image_topic_name stays the shared, device-name-qualified BASE all six
    # are built from, so the cross-instance collision fix above still
    # applies identically to all of them.
    self.image_topic_name = self.device_name + "/color_2d_image"
    self.robot_color_topic_name = self.image_topic_name + "/" + self.ROBOT_COLOR_TOPIC_SUFFIX
    self.scene_color_topic_name = self.image_topic_name + "/" + self.SCENE_COLOR_TOPIC_SUFFIX
    self.robot_depth_topic_name = self.image_topic_name + "/" + self.ROBOT_DEPTH_TOPIC_SUFFIX
    self.scene_depth_topic_name = self.image_topic_name + "/" + self.SCENE_DEPTH_TOPIC_SUFFIX
    self.robot_depth_map_topic_name = self.image_topic_name + "/" + self.ROBOT_DEPTH_MAP_TOPIC_SUFFIX
    self.scene_depth_map_topic_name = self.image_topic_name + "/" + self.SCENE_DEPTH_MAP_TOPIC_SUFFIX
    self.image_pub_robot_color = nepi_sdk.create_publisher(self.robot_color_topic_name, Image, queue_size = 1)
    self.image_pub_scene_color = nepi_sdk.create_publisher(self.scene_color_topic_name, Image, queue_size = 1)
    self.image_pub_robot_depth = nepi_sdk.create_publisher(self.robot_depth_topic_name, Image, queue_size = 1)
    self.image_pub_scene_depth = nepi_sdk.create_publisher(self.scene_depth_topic_name, Image, queue_size = 1)
    # Raw depth maps: 32FC1 meters, not compressed-and-decompressed color --
    # published straight through once decoded (see processImageLine), for
    # later processing rather than live viewing.
    self.image_pub_robot_depth_map = nepi_sdk.create_publisher(self.robot_depth_map_topic_name, Image, queue_size = 1)
    self.image_pub_scene_depth_map = nepi_sdk.create_publisher(self.scene_depth_map_topic_name, Image, queue_size = 1)

    ##############################
    # Goto controller state
    self.goto_target = None       # dict(x_m, y_m, yaw_deg or None) in sim ENU world frame
    self.goto_target_lock = threading.Lock()
    self.stop_triggered = False

    ##############################
    # Manual motor-ratio state: RBX_EXTERNAL_HARDWARE_INTERFACES.md worked
    # example (section 6) originally modeled a CAN-bus-style two-motor tank
    # drive (0=left, 1=right). Widened to 4 (2026-08-20): generic_rover/
    # model.sdf is a genuine 4-joint skid-steer (front_left, front_right,
    # rear_left, rear_right -- see its own diff_drive_controller plugin
    # comment), and RBXRobotIF's own motor_count capability is derived
    # generically as len(getMotorControlRatios()), the same mechanism that
    # already makes the ArduPilot driver "automatically show all 4 motors"
    # (its own self.motor_ratios is sized from motor_count) -- hardcoding 2
    # here regardless of the selected robot config's declared motor_count
    # left the Motor Controls panel showing only 2 sliders even for
    # ground_robot_4_wheel. Order matches the SDF's own joint order: [0]
    # front_left, [1] front_right, [2] rear_left, [3] rear_right.
    # libgazebo_ros_diff_drive only ever takes ONE velocity per side (see
    # that plugin's repeated <leftJoint>/<rightJoint> tags driving both
    # front+rear identically), so genuinely independent per-wheel speeds
    # aren't physically realizable here -- motorControlToVelocity below
    # averages each side's pair, which means all 4 sliders are individually
    # movable and each has a real effect, but front/rear on the same side
    # can't diverge from each other.
    self.motor_ratios = [0.0, 0.0, 0.0, 0.0]

    ##############################
    # Home position state: local ENU x/y/z meters (this rover has no WGS84
    # reference), carried over RBXRobotIF's existing GeoPoint-based
    # set_home/get_home/set_home_current plumbing -- see getHome/setHome
    # below for why reusing that mechanism (rather than adding a new
    # message/topic) is safe here.
    self.home_x_m = 0.0
    self.home_y_m = 0.0
    self.home_z_m = 0.0

    ##############################
    # Initialize RBX Settings
    self.settings_dict = copy.deepcopy(self.FACTORY_SETTINGS)
    self.cap_settings = self.getCapSettings()
    self.factory_settings = self.getFactorySettings()

    self.axis_controls = AxisControls()
    self.axis_controls.x = True
    self.axis_controls.y = True
    self.axis_controls.z = False
    self.axis_controls.roll = False
    self.axis_controls.pitch = False
    self.axis_controls.yaw = True

    ##############################
    # Bridge client thread: connects, reads telemetry, reconnects on failure
    self.environment_options_received_event = threading.Event()
    self.bridge_thread = threading.Thread(target = self.bridgeLoop)
    self.bridge_thread.daemon = True
    self.bridge_thread.start()
    # Bounded wait (see ENVIRONMENT_OPTIONS_WAIT_SEC's own comment) so that,
    # when the VM answers quickly, CAP_SETTINGS['environment']['options'] and
    # ENVIRONMENT_VALUE_TO_MODEL already reflect its scanned models by the
    # time RBXRobotIF bakes capabilities in below -- and when it doesn't,
    # startup proceeds anyway with the two-entry fallback unchanged.
    self.environment_options_received_event.wait(timeout = self.ENVIRONMENT_OPTIONS_WAIT_SEC)

    ##############################
    # Launch the NEPI RBX interface -- this takes care of initializing all
    # the rbx settings from config, subscribing/advertising topics and
    # services, publishing status/navpose, and running the blocking goto
    # convergence checks against the navpose data supplied by getNavPoseCb.
    self.msg_if.pub_info("Launching NEPI RBX interface...")
    self.device_info_dict = dict(device_name = self.device_name,
                                 path = self.device_path,
                                 serial_number = "",
                                 hw_version = "",
                                 sw_version = "")
    self.msg_if.pub_info(str(self.device_info_dict))

    self.rbx_if = RBXRobotIF(device_info = self.device_info_dict,
                                  # RBXRobotIF's factory default is
                                  # 'control_system' -- sim_connector_app_node.py's
                                  # own device discovery (simDiscoveryCb) only
                                  # matches DeviceRBXStatus publishers reporting
                                  # 'simulator' (SIM_SOURCE_DESCRIPTION) here.
                                  # Without this override, this rover never
                                  # appears in that app's available_simulators
                                  # list, so its own selected_simulator can never
                                  # resolve to this device's namespace -- silently
                                  # defeating any Sim Connector control surface
                                  # that depends on that discovery, including the
                                  # camera-rig controls added 2026-08-13.
                                  data_source_description = 'simulator',
                                  capSettings = self.cap_settings,
                                  factorySettings = self.factory_settings,
                                  settingUpdateFunction = self.settingUpdateFunction,
                                  getSettingsFunction = self.getSettings,
                                  axisControls = self.axis_controls,
                                  getBatteryPercentFunction = None,
                                  states = self.RBX_STATES,
                                  getStateIndFunction = self.getStateInd,
                                  setStateIndFunction = self.setStateInd,
                                  modes = self.RBX_MODES,
                                  getModeIndFunction = self.getModeInd,
                                  setModeIndFunction = self.setModeInd,
                                  checkStopFunction = self.checkStopFunction,
                                  setup_actions = self.RBX_SETUP_ACTIONS,
                                  setSetupActionIndFunction = self.setSetupActionInd,
                                  go_actions = self.RBX_GO_ACTIONS,
                                  setGoActionIndFunction = self.setGoActionInd,
                                  manualControlsReadyFunction = self.manualControlsReady,
                                  getMotorControlRatios = self.getMotorControlRatios,
                                  setMotorControlRatio = self.setMotorControlRatio,
                                  autonomousControlsReadyFunction = self.autonomousControlsReady,
                                  getHomeFunction = self.getHome,
                                  setHomeFunction = self.setHome,
                                  goHomeFunction = self.returnHomeAction,
                                  goStopFunction = self.goStop,
                                  gotoPoseFunction = self.gotoPose,
                                  gotoPositionFunction = self.gotoPosition,
                                  gotoLocationFunction = None,
                                  getNavPoseCb = self.getNavPoseCb,
                                  navpose_update_rate = self.NAVPOSE_UPDATE_RATE,
                                  msg_if = self.msg_if
                                )

    self.msg_if.pub_info("... RBX interface running")
    time.sleep(1)

    ## Raise the interface's setpoint-wait cmd_timeout above its 25s factory
    ## default -- see GOTO_CMD_TIMEOUT_SEC above for why a non-holonomic
    ## rover needs more headroom than the factory value provides.
    self.rbx_if.setCmdTimeoutCb(UInt32(data = self.GOTO_CMD_TIMEOUT_SEC))

    ## Point the interface's image-source search at this instance's own
    ## per-device-name-qualified robot_color topic by default (matching the
    ## old FIRST_PERSON factory default, back when there was one switchable
    ## topic instead of six always-live ones -- see the image_pub_robot_color
    ## comment above) -- overrides RBXRobotIF's plain "color_2d_image"
    ## factory default/any stale persisted config every startup, same
    ## rationale as the cmd_timeout override just above: deterministic
    ## per-instance behavior regardless of what a previous run left in
    ## config. The operator can still switch to any of the other five any
    ## time via the ordinary Image Source dropdown, same as picking any other
    ## camera topic.
    self.rbx_if.setImageTopicCb(String(data = self.robot_color_topic_name))

    ## Start the closed-loop goto controller
    controller_interval = float(1) / self.CONTROLLER_RATE_HZ
    nepi_sdk.start_timer_process(controller_interval, self.gotoControlCb)

    ## Initiation Complete
    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.on_shutdown(self.cleanup_actions)
    nepi_sdk.spin()


  #**********************
  # Setting functions

  def getCapSettings(self):
    return self.CAP_SETTINGS

  def getFactorySettings(self):
    settings = self.getSettings()
    #Apply factory setting overides
    for setting_name in settings.keys():
      if setting_name in self.FACTORY_SETTINGS_OVERRIDES:
            settings[setting_name]['value'] = self.FACTORY_SETTINGS_OVERRIDES[setting_name]
    return settings

  def getSettings(self):
    return self.settings_dict

  def settingUpdateFunction(self,setting):
    success = False
    setting_str = str(setting)
    setting_name = setting['name']
    if nepi_settings.check_valid_setting(setting,self.cap_settings):
      if setting_name in self.settings_dict.keys():
        self.settings_dict[setting_name]['value'] = setting['value']
        success = True
      else:
        msg = (self.node_name  + " Setting name" + setting_str + " is not supported")
      if success == True:
        msg = ( self.node_name  + " UPDATED SETTINGS " + setting_str)
        if setting_name in self.CAMERA_SETTING_NAMES:
          self.sendCameraSettings()
        if setting_name in self.ENVIRONMENT_SETTING_NAMES:
          self.setEnvironmentAction(setting['value'])
    else:
      msg = (self.node_name  + " Setting data" + setting_str + " is not valid")
    return success, msg

  ##########################
  # RBX Interface Functions

  def getStateInd(self):
    # No robot states (empty RBX_STATES) -- RBXRobotIF still calls this
    # unconditionally and displays "Not Set" for the empty list
    return 0

  def setStateInd(self,state_ind):
    # Unreachable with empty RBX_STATES (RBXRobotIF bounds-checks first)
    return False

  def getModeInd(self):
    return 0

  def setModeInd(self,mode_ind):
    # Unreachable with empty RBX_MODES
    return False

  def checkStopFunction(self):
    triggered = self.stop_triggered
    self.stop_triggered = False # Reset Stop Trigger
    return triggered

  def manualControlsReady(self):
    # Gates manual motor-ratio commands the same way autonomousControlsReady
    # gates goto commands: require a live bridge connection. Fresh telemetry
    # is not required here (unlike goto) since a direct motor command doesn't
    # depend on knowing the current position/heading.
    with self.sock_lock:
      return self.sock is not None

  def setMotorControlRatio(self, motor_ind, speed_ratio):
    # Only updates local state -- gotoControlCb (already running continuously
    # at CONTROLLER_RATE_HZ) is the single authoritative sender of velocity
    # commands to the bridge. A one-shot send from here was tried first and
    # confirmed live to be immediately overwritten by that loop's next
    # (0,0)-when-idle tick, since it sends every tick regardless of whether a
    # goto or manual command is active. See motorControlToVelocity.
    if motor_ind < 0 or motor_ind >= len(self.motor_ratios):
      self.msg_if.pub_warn("Motor control ignored: motor index " + str(motor_ind) + " out of range")
      return
    # -1.0..1.0, not 0.0..1.0 -- a wheeled rover's motors genuinely reverse,
    # unlike the ArduPilot driver's motor TEST (a prop spin-up check, which
    # has no meaningful reverse direction and stays 0..1 on its own class).
    self.motor_ratios[motor_ind] = max(-1.0, min(1.0, speed_ratio))

  def getMotorControlRatios(self):
    return self.motor_ratios

  def autonomousControlsReady(self):
    # Gates all goto commands: require a live bridge connection with fresh
    # telemetry so goto targets are computed from a real current position.
    # Also require autonomous_movement_enabled -- the Sim Connector's own
    # per-robot-config "automated movement" toggle. Checked HERE, not just in
    # the RUI (which hides the controls entirely), so disabling it actually
    # blocks the command for any client, not merely the RUI's own buttons.
    if self.settings_dict['autonomous_movement_enabled']['value'] != 'TRUE':
      return False
    with self.sock_lock:
      connected = self.sock is not None
    fresh = (nepi_utils.get_time() - self.last_telemetry_time) < self.TELEMETRY_FRESH_SEC
    return connected and fresh

  def goStop(self):
    self.stop_triggered = True
    self.clearGotoTarget()
    self.sendVelocityCmd(0.0, 0.0)
    return True

  def gotoPose(self,attitude_enu_degs):
    # RBXRobotIF passes target attitude ENU [roll,pitch,yaw] degrees; only
    # yaw is achievable on a ground rover (roll/pitch stay ~0 on flat
    # ground, so the interface's roll/pitch error checks converge). Hold
    # position, turn in place.
    self.msg_if.pub_info("Recieved Pose setpoint command: " + str(attitude_enu_degs))
    with self.goto_target_lock:
      self.goto_target = {'x_m': self.navpose_dict['x_m'],
                          'y_m': self.navpose_dict['y_m'],
                          'yaw_deg': attitude_enu_degs[2]}

  def gotoPosition(self,point_enu_m,orientation_enu_deg):
    # RBXRobotIF passes the goal as an ENU offset point from the current
    # position plus an absolute target orientation (its own convergence
    # check uses current + offset), so the controller target is computed
    # the same way from the same navpose source. z is ignored: the rover
    # moves in the ground plane (z stays 0, matching a z=0 offset input).
    self.msg_if.pub_info("Recieved Position setpoint command: " + str(point_enu_m))
    with self.goto_target_lock:
      self.goto_target = {'x_m': self.navpose_dict['x_m'] + point_enu_m.x,
                          'y_m': self.navpose_dict['y_m'] + point_enu_m.y,
                          'yaw_deg': orientation_enu_deg[2]}

  def getNavPoseCb(self):
    return self.navpose_dict

  #######################
  ### Setup-Action Functions

  def setSetupActionInd(self, action_ind):
    # action_ind is already bounds-checked against RBX_SETUP_ACTIONS by
    # RBXRobotIF before this is called.
    action = self.RBX_SETUP_ACTIONS[action_ind]
    if action == "RESET_SIM":
      return self.resetSimAction()
    elif action == "RETURN_HOME":
      return self.returnHomeAction()
    return False

  #######################
  ### Go-Action Functions

  def setGoActionInd(self, action_ind):
    # Unreachable with empty RBX_GO_ACTIONS (RBXRobotIF bounds-checks first)
    return False

  #######################
  ### Home Functions

  def getHome(self):
    # No GPS/WGS84 reference on a ground rover -- RBXRobotIF's home
    # mechanism only carries a plain 3-float GeoPoint though, so this reuses
    # that same plumbing (set_home/get_home/set_home_current topics, config
    # persistence via RBXRobotIF's own 'home_location' param) with the
    # three floats reinterpreted as local ENU x/y/z meters instead of
    # lat/long/alt degrees. See processTelemetryLine's navpose_dict
    # latitude/longitude/altitude_m fields for the matching reinterpretation
    # that makes RBXRobotIF's set_home_current ("use current position")
    # path work correctly for this driver with zero shared-API changes.
    home = GeoPoint()
    home.latitude = self.home_x_m
    home.longitude = self.home_y_m
    home.altitude = self.home_z_m
    return home

  def setHome(self, geo_point):
    self.home_x_m = geo_point.latitude
    self.home_y_m = geo_point.longitude
    self.home_z_m = geo_point.altitude
    return True

  def returnHomeAction(self):
    # Drives to the stored home position the same way a gotoPosition
    # setpoint is: set the goto_target and let gotoControlCb's existing
    # closed-loop controller take it from there. No forced final yaw --
    # home is just a position here, not a position+heading. Blocks until
    # arrival (or timeout) since both the go_action/setup_action dispatch
    # and RBXRobotIF's own goHomeFunction call treat this call's return
    # value as success/failure, not just "accepted".
    if not self.autonomousControlsReady():
      return False
    with self.goto_target_lock:
      self.goto_target = {'x_m': self.home_x_m, 'y_m': self.home_y_m, 'yaw_deg': None}
    start_time = nepi_utils.get_time()
    while (nepi_utils.get_time() - start_time) < self.GO_HOME_TIMEOUT_SEC:
      with self.goto_target_lock:
        reached = self.goto_target is None
      if reached:
        return True
      time.sleep(self.GO_HOME_POLL_INTERVAL_SEC)
    return False

  def resetSimAction(self):
    # Unlike GO_HOME (drive there under the closed-loop controller), this is
    # an instant physics teleport back to the Gazebo spawn pose -- mirrors
    # rbx_ardupilot_node.py's RESET_SIM setup action, but reached over the
    # existing sim_bridge TCP connection (this driver has no direct route to
    # the VM's Gazebo/gz_reset_listener otherwise).
    self.clearGotoTarget()
    # Clearing motor_ratios (not just sending one zero velocity command) is
    # required, not cosmetic -- reported live (2026-09-02): "if the rover is
    # a little discombobulated... it doesn't stabilize it, it continues to
    # glitch around". gotoControlCb runs continuously at CONTROLLER_RATE_HZ
    # and falls back to `elif any(self.motor_ratios): ... motorControlToVelocity()`
    # whenever there's no active goto -- if an operator had left manual motor
    # sliders non-zero, the very NEXT control tick after this method's own
    # sendVelocityCmd(0.0, 0.0) would see motor_ratios still non-zero and
    # immediately resend that stale non-zero command, undoing the reset
    # within a single tick (this method's own zero-send would look like it
    # "didn't work" even though it ran correctly). RUI motor sliders read
    # getMotorControlRatios() (via rbx/status's current_motor_control_settings),
    # so clearing the array here is also what makes them visibly snap back
    # to 0 the moment Reset Sim is hit, not just internally stop driving.
    self.motor_ratios = [0.0] * len(self.motor_ratios)
    self.sendVelocityCmd(0.0, 0.0)
    with self.sock_lock:
      connected = self.sock is not None
    if not connected:
      return False
    self.sendLineToBridge({'type': 'reset'}, "Reset sim")
    return True

  def setEnvironmentAction(self, environment_value):
    # Fire-and-forget over the bridge, same pattern as resetSimAction: the VM
    # side (sim_bridge_node.py) owns the actual Gazebo spawn/delete service
    # calls and its own currently-spawned bookkeeping (environment_models.py's
    # EnvironmentModelSpawner), so this side just needs a live connection to
    # send the request on. environment_value is a Discrete option string
    # (FLAT_GROUND, OBSTACLE_COURSE, or an uppercased scanned model name);
    # ENVIRONMENT_VALUE_TO_MODEL maps it to the real model directory name to
    # spawn, or None for "nothing spawned" (FLAT_GROUND). Formerly
    # setObstacleCourseAction(enabled: bool), generalized from a single
    # hardcoded model to any of the models environment_options reports (see
    # docs/SCAN_TO_SIM_ENVIRONMENT_PLAN.md section 5.6).
    with self.sock_lock:
      connected = self.sock is not None
    if not connected:
      return False
    model_name = self.ENVIRONMENT_VALUE_TO_MODEL.get(environment_value)
    self.sendLineToBridge({'type': 'environment', 'model_name': model_name},
                          "Environment set to " + environment_value)
    return True

  #######################
  ### Goto Controller Processes

  def clearGotoTarget(self):
    with self.goto_target_lock:
      self.goto_target = None

  def gotoControlCb(self,timer):
    # 10 Hz closed-loop differential-drive controller: turn toward the
    # target bearing, drive when roughly aligned, then rotate to the final
    # yaw goal. A velocity command is sent on EVERY tick, active goto or
    # not (defaulting to (0, 0) when idle) -- never rely on a single
    # one-shot zero command to stop the rover. The sim bridge is a plain
    # TCP link with no delivery guarantee/ack (see sendLineToBridge): if a
    # single stop packet -- or the last drive-phase command right at
    # convergence -- is ever dropped, Gazebo's diff-drive plugin latches
    # whatever velocity it last received and the rover keeps drifting
    # (typically a slow, small residual yaw rotation) indefinitely, since
    # nothing would ever re-send the correction. Re-asserting every 100ms
    # makes that self-healing: the next tick after any drop still lands.
    with self.goto_target_lock:
      target = self.goto_target

    lin = 0.0
    ang = 0.0
    if target is not None:
      cur_x = self.navpose_dict['x_m']
      cur_y = self.navpose_dict['y_m']
      cur_yaw_rad = math.radians(self.navpose_dict['yaw_deg'])

      max_lin = float(self.settings_dict['max_linear_speed_mps']['value'])
      max_ang = math.radians(float(self.settings_dict['max_angular_rate_dps']['value']))
      tol_m = self.FACTORY_GOTO_TOL_M
      tol_rad = self.FACTORY_GOTO_TOL_RAD
      if self.rbx_if is not None:
        tol_m = self.rbx_if.rbx_info.error_bounds.max_distance_error_m * self.GOTO_TOL_FRACTION
        tol_rad = math.radians(self.rbx_if.rbx_info.error_bounds.max_rotation_error_deg) * self.GOTO_TOL_FRACTION

      dx = target['x_m'] - cur_x
      dy = target['y_m'] - cur_y
      dist = math.hypot(dx, dy)

      if dist > tol_m:
        # Drive phase: point at the target, drive when roughly aligned
        bearing_err = self.normalizeAngle(math.atan2(dy, dx) - cur_yaw_rad)
        ang = max(-max_ang, min(max_ang, self.GOTO_KP_ANG * bearing_err))
        if abs(bearing_err) < self.GOTO_TURN_GATE_RAD:
          lin = max(0.0, min(max_lin, self.GOTO_KP_LIN * dist))
      else:
        # Final yaw phase (skipped if no yaw goal)
        yaw_err = 0.0
        if target['yaw_deg'] is not None:
          yaw_err = self.normalizeAngle(math.radians(target['yaw_deg']) - cur_yaw_rad)
        if abs(yaw_err) > tol_rad:
          ang = max(-max_ang, min(max_ang, self.GOTO_KP_ANG * yaw_err))
        else:
          # Target reached: clear (lin/ang stay 0.0 -- rover stops)
          self.clearGotoTarget()
          self.msg_if.pub_info("Goto target reached")
    elif any(self.motor_ratios):
      # No active goto -- an active manual motor command takes over this same
      # tick, so there is exactly one authoritative sender rather than a race
      # between this loop and a separate one-shot command.
      lin, ang = self.motorControlToVelocity()
    self.sendVelocityCmd(lin, ang)

  def motorControlToVelocity(self):
    # [0]=front_left, [1]=front_right, [2]=rear_left, [3]=rear_right --
    # averaged per side since the diff-drive plugin only takes one command
    # per side (see self.motor_ratios' own comment).
    left = (self.motor_ratios[0] + self.motor_ratios[2]) / 2.0
    right = (self.motor_ratios[1] + self.motor_ratios[3]) / 2.0
    max_lin = float(self.settings_dict['max_linear_speed_mps']['value'])
    lin = (left + right) / 2.0 * max_lin
    ang = (right - left) / self.MOTOR_WHEEL_BASE_M * max_lin
    return lin, ang

  def normalizeAngle(self,angle_rad):
    while angle_rad > math.pi:
      angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
      angle_rad += 2.0 * math.pi
    return angle_rad

  #######################
  ### Bridge Processes

  def bridgeLoop(self):
    # Persistent client to the VM-side bridge server. The sim stack (or the
    # tunnel) can restart independently of this node -- any failure tears
    # the socket down and retries the connect on a fixed interval.
    buf = b''
    while not nepi_sdk.is_shutdown():
      sock = None
      try:
        sock = socket.create_connection((self.sim_host, int(self.bridge_port)),
                                        timeout = self.SOCKET_TIMEOUT_SEC)
        sock.settimeout(self.SOCKET_TIMEOUT_SEC)
      except Exception as e:
        self.msg_if.pub_warn("Bridge connect to " + self.sim_host + ":" +
                             str(self.bridge_port) + " failed: " + str(e))
        time.sleep(self.RECONNECT_INTERVAL_SEC)
        continue
      with self.sock_lock:
        self.sock = sock
      self.msg_if.pub_info("Connected to sim bridge at " + self.sim_host +
                           ":" + str(self.bridge_port))
      # Sync the VM side to this node's actual current camera settings on
      # every (re)connect -- a bare restart of this node resets settings_dict
      # to factory, but the VM's camera_rig_controller.py keeps whatever ROS
      # params it last had, so an explicit push avoids relying on both sides
      # coincidentally matching factory defaults.
      self.sendCameraSettings()
      # Same reasoning, same fix, for the environment state -- found live
      # (2026-08-20): a fresh driver launch defaults its own
      # settings_dict['environment'] to FLAT_GROUND but never told the
      # bridge, so a Gazebo/bridge session left over from an earlier test
      # (some model already spawned, sim_bridge_node.py's own spawner state
      # still tracking it) stayed in that mode indefinitely -- "flat ground"
      # only took effect once an operator happened to manually re-toggle the
      # Environment dropdown, which is the only other code path that calls
      # setEnvironmentAction. On the very first connect (this thread hasn't
      # reached the receive loop yet, so ENVIRONMENT_VALUE_TO_MODEL is still
      # whatever __init__ left it as) this is always FLAT_GROUND -> None,
      # which is correct regardless -- settings_dict only ever holds a
      # scanned-model value once a user has explicitly selected one, and
      # that can't happen until after __init__'s bounded wait has already
      # completed.
      self.setEnvironmentAction(self.settings_dict['environment']['value'])
      buf = b''
      while not nepi_sdk.is_shutdown():
        try:
          data = sock.recv(4096)
        except socket.timeout:
          # Server pushes at 10 Hz -- a quiet-but-open socket past the
          # timeout means the far side is gone (e.g. tunnel half-open)
          data = b''
        except Exception:
          data = b''
        if not data:
          break
        buf += data
        while b'\n' in buf:
          line, buf = buf.split(b'\n', 1)
          if line.strip():
            self.processBridgeLine(line)
      with self.sock_lock:
        self.sock = None
      try:
        sock.close()
      except Exception:
        pass
      self.msg_if.pub_warn("Sim bridge connection lost -- retrying in " +
                           str(self.RECONNECT_INTERVAL_SEC) + "s")
      time.sleep(self.RECONNECT_INTERVAL_SEC)

  def processBridgeLine(self, line):
    # Single entry point for every line off the bridge socket: parse once,
    # then dispatch by key presence (no mandatory "type" tag on the
    # already-verified telemetry shape, which predates this dispatch and
    # carries none) -- image frames carry "type":"image", environment-model
    # announcements carry "type":"environment_options"; everything else is
    # telemetry, the only other shape the bridge server ever sends.
    try:
      msg = json.loads(line)
    except Exception as e:
      self.msg_if.pub_warn("Bad line from bridge: " + str(e))
      return
    if msg.get('type') == 'image':
      self.processImageLine(msg)
    elif msg.get('type') == 'environment_options':
      self.processEnvironmentOptionsLine(msg)
    else:
      self.processTelemetryLine(msg)

  def processEnvironmentOptionsLine(self, msg):
    # Sent by sim_bridge_node.py on every (re)connect, listing whatever
    # environment_models.py finds on the VM (obstacle_course plus any
    # scan_to_environment.py output) -- see
    # docs/SCAN_TO_SIM_ENVIRONMENT_PLAN.md section 5.6. Rebuilds both
    # CAP_SETTINGS['environment']['options'] (what the Discrete setting
    # reports as selectable) and ENVIRONMENT_VALUE_TO_MODEL (what a selected
    # option string actually spawns) from the same list, wholesale --
    # FLAT_GROUND always means "nothing spawned" and is not itself a model
    # directory. self.environment_options_received_event.set() unblocks the
    # bounded startup wait in __init__ as soon as the first announcement
    # arrives, rather than waiting out the full timeout.
    model_names = [str(n) for n in msg.get('options', [])]
    self.CAP_SETTINGS['environment']['options'] = ["FLAT_GROUND"] + [n.upper() for n in model_names]
    self.ENVIRONMENT_VALUE_TO_MODEL = {"FLAT_GROUND": None}
    self.ENVIRONMENT_VALUE_TO_MODEL.update({n.upper(): n for n in model_names})
    self.environment_options_received_event.set()

  # Which publisher each bridge "camera" tag routes to. Built once as a class
  # dict of attribute names (not direct publisher references) since the
  # actual publishers don't exist yet at class-definition time -- resolved
  # against self in processImageLine below.
  CAMERA_PUB_ATTR = {
    "robot_color": "image_pub_robot_color",
    "scene_color": "image_pub_scene_color",
    "robot_depth": "image_pub_robot_depth",
    "scene_depth": "image_pub_scene_depth",
    "robot_depth_map": "image_pub_robot_depth_map",
    "scene_depth_map": "image_pub_scene_depth_map",
  }
  # Raw depth maps travel as 16-bit millimeter PNGs (see
  # camera_rig_controller.py's depthToMillimeterPng) -- matches the encoding
  # this file's own DEPTH_MAP_MAX_MM-equivalent uses on the sending side.
  DEPTH_MAP_CAMERAS = ("robot_depth_map", "scene_depth_map")

  def processImageLine(self, msg):
    # Bridge image frame -> decode the relayed frame and republish as a raw
    # sensor_msgs/Image on this instance's own namespaced image topic (see
    # the image_pub_robot_color / setImageTopicCb comments in __init__ for
    # why the topic name is device_name-qualified and RBXRobotIF is pointed
    # at one of them via set_image_topic). "camera" picks which of the six
    # publishers this frame goes to; an older sender with no "camera" field
    # defaults to robot_color, matching the original single-topic behavior.
    #
    # The two depth_map cameras carry 16-bit millimeter PNGs, not JPEG color
    # -- decoded back into a genuine 32FC1-meters NumPy array (float32,
    # meters) before republishing, since that raw array -- not the PNG
    # bytes -- is what every NEPI depth-map consumer (and cv2/numpy in
    # general) expects an Image message's data to already be.
    try:
      camera = msg.get('camera', self.ROBOT_COLOR_TOPIC_SUFFIX)
      pub_attr = self.CAMERA_PUB_ATTR.get(camera, "image_pub_robot_color")
      image_pub = getattr(self, pub_attr)
      encoded_bytes = base64.b64decode(msg['data'])
      arr = np.frombuffer(encoded_bytes, dtype = np.uint8)
      if camera in self.DEPTH_MAP_CAMERAS:
        depth_mm = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
        if depth_mm is None or depth_mm.dtype != np.uint16:
          raise ValueError("cv2.imdecode did not return a 16-bit depth map")
        # NumPy array in actual meters, float32 -- the format every
        # depth-map consumer downstream expects, not raw millimeter PNG bytes.
        depth_m = depth_mm.astype(np.float32) / 1000.0
        ros_img = nepi_img.cv2img_to_rosimg(depth_m, encoding = "32FC1")
      else:
        cv2_img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if cv2_img is None:
          raise ValueError("cv2.imdecode returned None")
        ros_img = nepi_img.cv2img_to_rosimg(cv2_img, encoding = "bgr8")
      image_pub.publish(ros_img)
    except Exception as e:
      self.msg_if.pub_warn("Failed to process camera image frame: " + str(e), throttle_s = 5.0)

  def processTelemetryLine(self, telem):
    # Bridge telemetry -> navpose dict consumed by getNavPoseCb (published
    # as the standard NEPI navpose by RBXRobotIF's NPXDeviceIF and bridged
    # to the current_* attributes its goto convergence checks use)
    now = nepi_utils.get_time()
    x_m = float(telem.get('x', 0.0))
    y_m = float(telem.get('y', 0.0))
    yaw_rad = float(telem.get('yaw', 0.0))
    lin_mps = float(telem.get('linear_x', 0.0))
    ang_radps = float(telem.get('angular_z', 0.0))

    # Position: sim world frame is ENU, ground vehicle so z stays 0
    self.navpose_dict['has_position'] = True
    self.navpose_dict['time_position'] = now
    self.navpose_dict['x_m'] = x_m
    self.navpose_dict['y_m'] = y_m
    self.navpose_dict['z_m'] = 0.0
    # No WGS84 fix on this rover (has_location stays unset/False), but
    # RBXRobotIF's set_home_current path unconditionally mirrors
    # navpose_dict's latitude/longitude/altitude_m into its own
    # current_location_wgs84_geo bookkeeping regardless of has_location --
    # mirroring the local x/y/z here too is what makes "use current
    # position as home" (setHome above) capture the rover's real position
    # instead of always reading back (0, 0, 0).
    self.navpose_dict['latitude'] = x_m
    self.navpose_dict['longitude'] = y_m
    self.navpose_dict['altitude_m'] = 0.0
    # Body-forward speed decomposed into the nav frame
    self.navpose_dict['x_m_per_sec'] = lin_mps * math.cos(yaw_rad)
    self.navpose_dict['y_m_per_sec'] = lin_mps * math.sin(yaw_rad)
    self.navpose_dict['z_m_per_sec'] = 0.0

    # Orientation: flat-ground rover, only yaw is meaningful
    self.navpose_dict['has_orientation'] = True
    self.navpose_dict['time_orientation'] = now
    self.navpose_dict['roll_deg'] = 0.0
    self.navpose_dict['pitch_deg'] = 0.0
    self.navpose_dict['yaw_deg'] = math.degrees(yaw_rad)
    self.navpose_dict['yaw_deg_per_sec'] = math.degrees(ang_radps)

    self.last_telemetry_time = now

  def sendVelocityCmd(self, linear_x, angular_z):
    cmd = {'linear_x': linear_x, 'angular_z': angular_z}
    self.sendLineToBridge(cmd, "Velocity command")

  def sendCameraSettings(self):
    # No view_mode -- both views are always-live, separate ROS topics now
    # (see CAMERA_SETTING_NAMES's own comment), nothing left to switch.
    # offset_yaw/offset_tilt/scene_offset_yaw/scene_offset_tilt are in
    # degrees, same convention as every other angle this app sends over the
    # bridge -- sim_bridge_node.py converts to radians itself.
    cmd = {
      'type': 'camera_settings',
      'offset_x': float(self.settings_dict['camera_offset_x']['value']),
      'offset_y': float(self.settings_dict['camera_offset_y']['value']),
      'offset_z': float(self.settings_dict['camera_offset_z']['value']),
      'offset_yaw': float(self.settings_dict['camera_offset_yaw']['value']),
      'offset_tilt': float(self.settings_dict['camera_offset_tilt']['value']),
      'scene_offset_x': float(self.settings_dict['scene_offset_x']['value']),
      'scene_offset_y': float(self.settings_dict['scene_offset_y']['value']),
      'scene_offset_z': float(self.settings_dict['scene_offset_z']['value']),
      'scene_offset_yaw': float(self.settings_dict['scene_offset_yaw']['value']),
      'scene_offset_tilt': float(self.settings_dict['scene_offset_tilt']['value']),
    }
    self.sendLineToBridge(cmd, "Camera settings")

  def sendLineToBridge(self, line_dict, description):
    with self.sock_lock:
      sock = self.sock
    if sock is None:
      self.msg_if.pub_warn(description + " dropped -- sim bridge not connected", throttle_s = 5.0)
      return
    try:
      sock.sendall((json.dumps(line_dict) + '\n').encode())
    except Exception as e:
      # bridgeLoop's recv will fail on the same dead socket and reconnect
      self.msg_if.pub_warn("Failed to send " + description.lower() + " to bridge: " + str(e))

  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    """Stops the rover on node shutdown by sending a zero velocity command."""
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    self.sendVelocityCmd(0.0, 0.0)


#########################################
# Main
#########################################
if __name__ == '__main__':
  SimNode()
