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

# The one generic app node that hosts a SimDeviceIF instance plus the single,
# well-known TCP/JSON listener any simulator's own bridge script dials into.
# Connecting a new simulator means writing a small bridge script on the
# simulator side that speaks this protocol -- not writing new NEPI code. This
# node is simulator-agnostic: it will be pointed at other simulators with no
# code change here.
#
# This node is the ONLY place that understands the wire protocol.
# api/device_if_sim.py stays protocol-agnostic, exactly as device_if_rbx.py never
# knows what transport its drivers speak or that a bridge socket exists.
#
# Wire protocol -- newline-delimited JSON both ways on one persistent
# connection, dispatched by "type" key presence:
#
#   in  -- bare line (no "type" key): NavPose telemetry, covering the full
#          NavPose contract rather than any one vehicle's shape. Every field
#          optional, gated exactly the way NavPose.msg itself is: x_m/y_m/z_m
#          => has_position, roll_deg/pitch_deg/yaw_deg => has_orientation,
#          latitude/longitude => has_location, altitude_m => has_altitude. One
#          shape fits a wheeled ground robot (position only) and a flying robot
#          (position + orientation + location + altitude) with no per-vehicle
#          special-casing.
#   in  -- {"type":"sensor_topics","topics":[{"topic_name":...,"msg_type":...},...]}
#          The bridge announces its current live topic list; fed straight into
#          getAvailableSensorTopicsFunction's return value.
#   in  -- {"type":"environment_options","options":[...]}
#          Same idea for available_environment_options.
#   in  -- {"type":"image","topic_name":...,"data":"<base64 jpeg>","stamp":...}
#          topic_name distinguishes multiple announced cameras; omitted means
#          the currently active image topic. Only frames matching the active
#          topic are decoded and republished -- nothing subscribes to the others.
#   in  -- {"type":"goto_result","success":bool}
#          Sent by a bridge once its own goto controller reports the current
#          target reached (or definitively failed), asynchronously -- the
#          fire-and-forget goto*Cb methods in device_if_sim.py return before
#          any convergence is known, so this is the only path that ever sets
#          cmd_success for a goto command. Applies to "whichever goto is most
#          recently pending" (no per-command ID) since no bridge tracks more
#          than one goto target at a time. Optional -- a bridge that never
#          sends this simply leaves cmd_success untouched for goto commands,
#          which is the same behavior as before this line existed.
#
#   out -- {"type":"motor_control","motor_ind":N,"speed_ratio":R}
#   out -- {"type":"goto_position","x_meters":...,"y_meters":...,"z_meters":...,"yaw_deg":...}
#   out -- {"type":"goto_pose","roll_deg":...,"pitch_deg":...,"yaw_deg":...}
#   out -- {"type":"goto_location","lat":...,"long":...,"altitude_meters":...,"yaw_deg":...}
#          Field names match GotoPosition.msg / GotoPose.msg / GotoLocation.msg
#          1:1 -- no reason to invent different names on the wire.
#   out -- {"type":"go_home"} / {"type":"go_stop"}
#   out -- {"type":"setup_action","action":"<string>"} / {"type":"go_action","action":"<string>"}
#   out -- {"type":"camera_settings","view_mode":...}
#   out -- {"type":"set_active_image_topic","topic_name":...}
#   out -- {"type":"environment_option","option":...,"enabled":bool}
#   out -- {"type":"robot_config","config":"<name>"}
#          Tells the simulator which kind of robot is wanted. Its own type key,
#          following the existing type-keyed convention rather than inventing a
#          second framing.
#
# Capability timing. The contract decides capabilities once at construction and
# caches them, which is what lets a client render controls purely from the
# flags. A generic connector cannot know a specific simulator's wheel or motor
# counts, or which goto functions make sense, until after a bridge connects --
# by which point this process and its SimDeviceIF already exist. Resolution:
# the two fields the contract calls genuinely dynamic
# (available_sensor_topics, available_environment_options) are live-refreshed
# from the bridge; everything else is a per-deployment config decision, read
# from a named SIM_VEHICLE_DICT robot-config entry. Switching robot config is
# the one case that legitimately changes a robot's kind, and it re-derives the
# cached report in place through SimDeviceIF.apply_capability_profile -- see
# that method's docstring for why that is wire-safe.

import base64
import copy
import json
import os
import socket
import threading

import yaml

# nepi_api.device_if_sim (imported below) pulls in nepi_api.device_if_npx ->
# nepi_api.system_if -> open3d transitively. On this aarch64 target, open3d's
# native libs (via libgomp) grab static TLS at import time, and if cv2 (also
# native, also TLS-hungry) has already been imported first, open3d's later
# import fails outright: "ImportError: .../libgomp.so.1: cannot allocate
# memory in static TLS block" -- confirmed by reproducing the exact failure
# standalone and confirming the fix is import order, not a code bug. Importing
# device_if_sim (and therefore open3d) before cv2 reserves its TLS slots
# first and avoids the exhaustion. Do not reorder cv2 above this without
# re-verifying against that failure.
from nepi_api.device_if_sim import SimDeviceIF

import numpy as np
import cv2

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_img
from nepi_sdk import nepi_system

from std_msgs.msg import Bool, Empty, String, Float32
from sensor_msgs.msg import Image
from geographic_msgs.msg import GeoPoint

from nepi_interfaces.msg import AxisControls, DeviceRBXStatus

from nepi_api.messages_if import MsgIF

# Additive simulator auto-launch capability (see
# docs/SIMULATOR_AUTO_LAUNCH_PLAN.md in nepi_drones) -- both imports are
# optional at runtime. simulator_launcher.py is installed into nepi_api
# alongside device_if_sim.py by this package's own CMakeLists (api/ ->
# nepi_api dist-packages), so it is present on any device that has this
# app installed; SimLauncherStatus is generated from this same package's
# msg/. Neither existing import above changes.
from nepi_api.simulator_launcher import (SimulatorLauncher, LauncherError,
                                         find_config_path)
from nepi_app_sim_connector.msg import SimLauncherStatus

PKG_NAME = 'SIM_CONNECTOR'

# This app's own ROS package name, used to find its installed params file by
# matching APP_DICT.pkg_name -- the same key getAppsDict itself keys apps on,
# so a params-file rename can't silently break the lookup.
APP_PKG_NAME = 'nepi_app_sim_connector'
SIM_VEHICLE_DICT_KEY = 'SIM_VEHICLE_DICT'
# system_folders is published by system_mgr, which starts well before any app.
# A short timeout is enough, and timing out is non-fatal (falls back to the
# capability-empty factory profile, exactly as before this fallback existed).
SYSTEM_FOLDERS_TIMEOUT_MSEC = 5000

# Listen port a simulator's bridge script dials into. Configurable per
# deployment via the params yaml.
FACTORY_LISTEN_PORT = 9030

# Factory robot-config profile: capability-empty, which renders no controls at
# all. That is the intended safe default, not a broken state -- an operator
# picks a real robot config from the selector, or edits SIM_VEHICLE_DICT.
FACTORY_ROBOT_CONFIG_NAME = 'default'

# Reserved robot_configs key for an operator-uploaded config (see
# uploadRobotConfigCb) -- a single slot, not a growing list: uploading again
# replaces whatever was there, matching "I'm iterating on my own robot's
# config and want to try the latest version" rather than accumulating a
# history of uploads. Chosen to be unlikely to collide with any real,
# checked-in config key.
UPLOADED_ROBOT_CONFIG_NAME = 'custom_uploaded'

# Physical-dimension editing (robot chassis/wheel geometry, environment
# corridor/ramp geometry) -- distinct from robot_configs' driver-capability
# profiles above, which never touch Gazebo geometry at all. Two roles for
# v1, each backed by one in-repo Gazebo model: 'robot' -> generic_rover (the
# quadcopter's airframe geometry is a vendored third-party Gazebo/ArduPilot
# model living outside this repo, nothing here to point a role at), and
# 'environment' -> obstacle_course. See generate_model_sdf.py for how
# curated dimensions.yaml fields become model.sdf, and
# SimulatorLauncher.push_dimensions for how they reach the VM.
ROBOT_DIMENSIONS_MODEL = 'generic_rover'
ENVIRONMENT_DIMENSIONS_MODEL = 'obstacle_course'
DIMENSION_ROLES = ('robot', 'environment')
DIMENSION_ROLE_MODEL = {'robot': ROBOT_DIMENSIONS_MODEL, 'environment': ENVIRONMENT_DIMENSIONS_MODEL}

# Device-side authoritative store for the above -- survives a Docker
# container restart (unlike the container's own writable layer), the same
# persistence category as nepi_app_onvif_mgr's own /mnt/nepi_storage/
# user_cfg/ usage. The VM's copy under sim_container/models/<model>/ is a
# synced DEPLOYMENT TARGET, not the source of truth: pushed here via
# pushDirtyDimensions whenever a value changes or this app restarts.
DIMENSIONS_STORAGE_DIR = '/mnt/nepi_storage/databases/nepi_app_sim_connector/dimensions'

# Default target used to push dimensions on app startup, before any
# simulator has been launched/selected this session -- gazebo_rover is the
# primary, always-available Gazebo target on this VM, matching the same
# fallback several other code paths already use.
DEFAULT_DIMENSIONS_PUSH_TARGET = 'gazebo_rover'

# Phone-scan (Stray Scanner) uploads land here, device-side -- same
# persistent-storage category as DIMENSIONS_STORAGE_DIR above, so an upload
# survives a container restart. Populating this directory (the actual
# browser -> device upload) is not implemented yet -- see
# docs/SCAN_TO_SIM_ENVIRONMENT_PLAN.md phase 1; convertPhoneScanCb below only
# handles the already-uploaded-here -> VM-converted half of the pipeline.
SCAN_UPLOADS_STORAGE_DIR = '/mnt/nepi_storage/databases/nepi_app_sim_connector/phone_scans'

FACTORY_ROBOT_CONFIG = dict(
    description = 'No capabilities. Safe default until a robot config is selected.',
    # Internal fallback only -- never a real choice an operator should be
    # offered (there is no robot it describes). getAvailableRobotConfigs
    # already has a mechanism for exactly this (skip from the offered list,
    # stay fully valid for setSelectedRobotConfig/buildProfileFromConfig) --
    # see that method's own comment.
    hidden_from_selector = True,
    wheel_count = 0,
    motor_count = 0,
    has_goto_position = False,
    has_goto_pose = False,
    has_goto_location = False,
    has_go_home = False,
    has_set_home = False,
    has_go_stop = False,
    setup_actions = [],
    go_actions = [],
    has_camera_view_control = False,
    available_camera_view_modes = [],
    has_environment_controls = False,
)

# Simulator discovery. Generic and simulator-agnostic on both axes:
#   - device type: a device is a candidate only if it publishes a status topic
#     of this message type, which is what "is an addressable NEPI robot device"
#     means on the wire.
#   - capability: a candidate is a simulator only if it declares itself one
#     through its own data_source_description, a first-class constructor
#     argument every device interface already publishes. A driver opts in by
#     setting one string; nothing here matches on any simulator's product name,
#     world name, or model name.
# Any future simulator driver appears in the selector by declaring the same
# thing, with no change to this app.
SIM_DEVICE_STATUS_MSG_TYPES = ['DeviceRBXStatus']
SIM_SOURCE_DESCRIPTION = 'simulator'
SIM_DISCOVERY_RATE_HZ = 1.0

# A discovered device drops off the list if its status goes quiet this long,
# so a driver node that dies stops being offered.
SIM_DEVICE_STALE_SEC = 10.0

BRIDGE_ACCEPT_BACKLOG = 1
TELEMETRY_AGE_IF_NEVER_CONNECTED = -1.0
BRIDGE_RECV_BYTES = 4096

STATUS_PUBLISH_RATE_HZ = 1.0

# Text that identifies launch_command's own "a gzserver is already running"
# refuse-to-launch guard (both gazebo_rover and gazebo_quadcopter's
# launch_command raise this exact wording -- see simulator_launch_targets.yaml)
# so runLaunch can offer the operator a real choice (attach to what's
# already there, or force past it) instead of just reporting a generic
# failure. Matched on the wording those two guards actually share, not on
# any structured error code -- LauncherError carries plain text by design
# (see its own docstring), and inventing a second, parallel signal just for
# this one case isn't worth it while only these two targets can ever raise it.
GAZEBO_ALREADY_RUNNING_ERROR_SIGNATURE = "a gzserver is already running"


def isGazeboConflictError(error_message):
  return GAZEBO_ALREADY_RUNNING_ERROR_SIGNATURE in str(error_message).lower()


#########################################
# Node Class
#########################################

class NepiSimConnectorApp:

  DEFAULT_NODE_NAME = "app_sim_connector"

  sim_if = None

  def __init__(self):
    ####  APP NODE INIT SETUP ####
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
    # Per-deployment robot configs, read from the params namespace first.
    #
    # CORRECTION to an earlier assumption documented here: apps_mgr does NOT
    # load every top-level key of an app's params yaml onto its param
    # namespace. nepi_sdk/nepi_apps.py's getAppsDict extracts APP_DICT (plus
    # RUI_DICT, nested inside it) and discards every other top-level key, and
    # apps_mgr then set_params only that one app_dict -- so SIM_VEHICLE_DICT
    # never arrived at all, and this app silently ran with nothing but the
    # capability-empty factory profile (confirmed on a real device:
    # `rosparam list` under this node showed only app_dict/npx/sim, and
    # available_robot_configs reported just ['default']). This app is the only
    # one in the repo that ships a third top-level params key, which is why
    # nothing else surfaced the gap.
    #
    # Rather than add a new generic behavior to apps_mgr -- core, shared by
    # every app, and a stop-and-write-up change per this repo's own rules --
    # the app reads its own installed params file directly when the param is
    # absent. The param still wins when set, so this stays backward compatible
    # with a future apps_mgr that does propagate these keys, and with anything
    # that sets the param itself.
    vehicle_dict_ns = nepi_sdk.create_namespace(self.node_namespace, 'SIM_VEHICLE_DICT')
    self.vehicle_dict = nepi_sdk.get_param(vehicle_dict_ns, dict())
    if not isinstance(self.vehicle_dict, dict):
      self.vehicle_dict = dict()
    if not self.vehicle_dict:
      self.vehicle_dict = self.loadVehicleDictFromParamsFile()

    self.listen_port = int(self.vehicle_dict.get('listen_port', FACTORY_LISTEN_PORT))

    robot_configs = self.vehicle_dict.get('robot_configs', dict())
    if not isinstance(robot_configs, dict):
      robot_configs = dict()
    self.robot_configs = copy.deepcopy(robot_configs)
    # The capability-empty factory profile is always present as the fallback,
    # whatever the yaml says, so there is never a state with no valid selection.
    if FACTORY_ROBOT_CONFIG_NAME not in self.robot_configs:
      self.robot_configs[FACTORY_ROBOT_CONFIG_NAME] = copy.deepcopy(FACTORY_ROBOT_CONFIG)
    # Enforced here, not just on FACTORY_ROBOT_CONFIG's own definition above:
    # found live (2026-08-19) that sim_connector_app_params.yaml ALSO checks
    # in a "default" entry of its own (predating hidden_from_selector, and
    # missing the flag), which is what deployments actually load -- the
    # FACTORY_ROBOT_CONFIG fallback above is only ever reached when the yaml
    # doesn't define one at all. Setting it unconditionally here means "the
    # capability-empty placeholder is never a real offered choice" holds
    # regardless of which of the two sources actually provided this entry, or
    # whether an older/hand-edited params file forgets the flag.
    if isinstance(self.robot_configs.get(FACTORY_ROBOT_CONFIG_NAME), dict):
      self.robot_configs[FACTORY_ROBOT_CONFIG_NAME]['hidden_from_selector'] = True

    default_config = str(self.vehicle_dict.get('default_robot_config', FACTORY_ROBOT_CONFIG_NAME))
    if default_config not in self.robot_configs:
      default_config = FACTORY_ROBOT_CONFIG_NAME
    self.selected_robot_config = default_config
    self.profile = self.buildProfileFromConfig(default_config)

    ##############################
    # Bridge connection and live announced state -- the genuinely dynamic
    # fields, refreshed from whatever the connected bridge last sent.
    self.client_conn = None
    self.client_lock = threading.Lock()
    self.last_telemetry_time = 0.0
    self.connected_since = None
    self.available_sensor_topics = []          # [(topic_name, msg_type), ...]
    self.available_environment_options = []
    self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    self.active_image_topic = ""
    self.motor_ratios = [0.0] * int(self.profile['motor_count'])

    ##############################
    # Simulator discovery state. Populated by a timer scan of the live ROS
    # graph; never by a blocking wait.
    self.sim_scan_lock = threading.Lock()
    self.sim_device_subs = dict()      # status topic -> subscriber handle
    self.sim_device_info = dict()      # status topic -> dict(name, source, time)
    self.selected_simulator = ""

    ##############################
    # Common six-topic mirror -- a stable, simulator-agnostic viewing point
    # for whichever robot is currently selected_simulator, regardless of
    # which underlying driver/bridge type it is. Distinct from
    # image_pub/color_2d_image below (the generic-connector bridge protocol's
    # own relay, fed over the TCP bridge for a simulator with no RBX driver
    # of its own): every simulator actually launchable today (Gazebo rover/
    # quadcopter, Webots rover/quadcopter) instead goes through an RBX
    # driver that already publishes real ROS Image topics on this same ROS
    # master -- see rbx_sim_node.py/rbx_ardupilot_node.py's own
    # color_2d_image/robot_color + .../scene_color + depth/depth_map topics
    # -- so this mirror is a plain re-subscribe, not a protocol decode.
    # Reported live (2026-08-18): the operator wants one topic to look at
    # "no matter the simulator" rather than needing to know the current
    # instance's own device-name-qualified topic. Expanded 2026-08-20 from
    # two topics (robot_view/scene_view) to six (color, colorized-depth-view,
    # raw-depth-map x robot/scene), matching the driver-side removal of the
    # depth_map_enabled toggle in favor of always-simultaneous publishing.
    # Any of the six can have no publisher at all for a robot that honestly
    # has no scene camera or no depth sensor (the Webots drivers) -- an
    # absent feed there is accurate, not a bug.
    self.MIRROR_VIEWS = ["robot_color", "scene_color", "robot_depth", "scene_depth",
                         "robot_depth_map", "scene_depth_map"]
    self.mirror_pubs = dict()
    self.mirror_subs = dict()
    self.mirror_source_topics = dict()
    for view in self.MIRROR_VIEWS:
      self.mirror_pubs[view] = nepi_sdk.create_publisher(self.node_name + "/" + view, Image, queue_size = 1)
      self.mirror_subs[view] = None
      self.mirror_source_topics[view] = None

    ##############################
    # FOV data -- published as plain, latched Float32 topics rather than
    # extending any .msg (avoids a catkin message-regeneration rebuild for
    # this). horizontal_fov is the SDF's static camera sensor value, shared
    # identically by every camera_rig/camera_rig_chase model in this sim;
    # vertical_fov is derived from it via the standard pinhole relation for
    # the sensor's fixed 640x480 resolution. Read once here (mount time)
    # since neither ever changes at runtime.
    self.CAMERA_HORIZONTAL_FOV_DEG = 80.0
    self.CAMERA_VERTICAL_FOV_DEG = 2.0 * np.degrees(np.arctan(
        np.tan(np.radians(self.CAMERA_HORIZONTAL_FOV_DEG) / 2.0) * (480.0 / 640.0)))
    self.camera_horizontal_fov_pub = nepi_sdk.create_publisher(
        self.node_name + "/sim/camera_horizontal_fov_deg", Float32, queue_size = 1, latch = True)
    self.camera_vertical_fov_pub = nepi_sdk.create_publisher(
        self.node_name + "/sim/camera_vertical_fov_deg", Float32, queue_size = 1, latch = True)
    self.camera_horizontal_fov_pub.publish(Float32(data = self.CAMERA_HORIZONTAL_FOV_DEG))
    self.camera_vertical_fov_pub.publish(Float32(data = self.CAMERA_VERTICAL_FOV_DEG))

    ##############################
    # Home position state -- reused GeoPoint plumbing with its three floats
    # reinterpreted as local ENU x/y/z meters, the same reinterpretation an RBX
    # driver for a vehicle with no independent WGS84 reference already uses. A
    # deployment whose simulator does have a real WGS84 reference can keep the
    # telemetry-supplied latitude/longitude as home instead; this app does not
    # need to know which case it is.
    self.home_x_m = 0.0
    self.home_y_m = 0.0
    self.home_z_m = 0.0

    ##############################
    # Goto target state for the thin goto delegators (see device_if_sim.py's own
    # documented scope note). Stored only so a future convergence controller has
    # somewhere to read from; this app runs no convergence controller itself --
    # forwarding the setpoint and letting the simulator-side bridge or vehicle
    # model reach it is the contract's intent.
    self.goto_target_lock = threading.Lock()
    self.goto_target = None

    ##############################
    # Image republish -- decode and republish only the currently active image
    # topic, the one thing a client actually displays, on a name qualified by
    # this node's own name so a second instance on the same device cannot
    # collide with it.
    self.image_topic_name = self.node_name + "/color_2d_image"
    self.image_pub = nepi_sdk.create_publisher(self.image_topic_name, Image, queue_size = 1)

    ##############################
    # Launch the generic SimDeviceIF. Every callback below is a thin
    # wire-protocol sender or getter; device_if_sim.py never knows there is a
    # TCP bridge underneath.
    device_info = dict(device_name = self.node_name, path = "",
                       serial_number = "", hw_version = "", sw_version = "")

    self.sim_if = SimDeviceIF(
        device_info = device_info,
        getNavPoseCb = self.getNavPoseCb,
        getAvailableSensorTopicsFunction = self.getAvailableSensorTopics,
        getAvailableEnvironmentOptionsFunction = self.getAvailableEnvironmentOptions,
        setActiveImageTopicFunction = self.setActiveImageTopic,
        getBridgeConnectedFunction = self.isBridgeConnected,
        getTelemetryAgeFunction = self.getTelemetryAge,
        getAvailableSimulatorsFunction = self.getAvailableSimulators,
        getSelectedSimulatorFunction = self.getSelectedSimulator,
        setSelectedSimulatorFunction = self.setSelectedSimulator,
        getAvailableRobotConfigsFunction = self.getAvailableRobotConfigs,
        getSelectedRobotConfigFunction = self.getSelectedRobotConfig,
        setSelectedRobotConfigFunction = self.setSelectedRobotConfig,
        msg_if = self.msg_if,
        **self.buildCapabilityKwargs(self.profile)
    )

    ##############################
    # Bridge server thread. This app OWNS the listen socket -- it is the one
    # stable, well-known connection surface a simulator's bridge dials into,
    # not a per-simulator client reaching out.
    self.server_thread = threading.Thread(target = self.bridgeServerLoop)
    self.server_thread.daemon = True
    self.server_thread.start()

    ##############################
    # Simulator auto-launch (additive convenience trigger over the launcher
    # helper -- see docs/SIMULATOR_AUTO_LAUNCH_PLAN.md). self.launcher stays
    # None, and the topics below become permanent no-ops, on any deployment
    # with no launch-targets config present (see find_config_path) -- this
    # never becomes a required part of the app's own contract.
    self.launcher = None
    self.launcher_lock = threading.Lock()
    # Launch, stop, AND install all share this one thread/lock -- simplest
    # correct answer to "can these run concurrently": no, all three are
    # heavy SSH operations against the same launch target, and running two
    # at once (e.g. installing Gazebo while also trying to launch it) has no
    # sensible outcome worth supporting.
    self.launcher_thread = None
    self.launcher_state = 'idle'
    self.launcher_last_error = ''
    # Set alongside launcher_last_error (via setLauncherError below) only
    # when the LauncherError that produced it carries its own
    # manual_fallback_commands -- currently just the reverse-tunnel
    # connectivity diagnosis in simulator_launcher.py's
    # _classify_connection_failure. Takes priority over the per-target
    # install fallback in publishLauncherStatus, since a dead tunnel isn't
    # fixed by that target's own install_command.
    self.launcher_tunnel_fallback_commands = ''
    # Which target a dependency-related failure was actually about -- NOT
    # always active_launch_target (runInstall can fail for a target that
    # isn't the one currently running/attempted), so publishLauncherStatus's
    # manual_fallback_commands derivation keys off this instead. Cleared at
    # the top of every runLaunch/runInstall attempt so a stale fallback from
    # an earlier, unrelated failure never lingers into a new one.
    self.launcher_failed_target = ''
    self.selected_launch_target = ''
    # The target actually running, once resolve_launch_target has done its
    # work -- may differ from selected_launch_target (the operator's own
    # pick, e.g. "gazebo_rover"/"Gazebo") when the current robot config
    # redirects it to a different target entirely (e.g. gazebo_quadcopter
    # for a flight profile). stop_command/ready_check_command always target
    # THIS, since it's the one with real SSH-launched processes.
    self.active_launch_target = ''
    # target_key -> bool / 'unknown'|'checking'|'installed'|'not_installed'.
    # Independent of launcher_state: every target's dependencies get checked
    # in the background regardless of which one (if any) is selected.
    self.launch_target_installed = {}
    self.launch_target_installed_check_state = {}
    launcher_config_path = find_config_path()
    if launcher_config_path:
      try:
        self.launcher = SimulatorLauncher(launcher_config_path)
        self.msg_if.pub_info("Simulator auto-launch enabled from " + launcher_config_path)
      except LauncherError as e:
        self.msg_if.pub_warn("Simulator auto-launch disabled (config at " +
                             launcher_config_path + " unusable): " + str(e))

    self.launcher_status_pub = nepi_sdk.create_publisher(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/launcher_status'),
        SimLauncherStatus, queue_size = 1, latch = True)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/launch_simulator'),
        String, self.launchSimulatorCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/stop_simulator'),
        Empty, self.stopSimulatorCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/install_simulator'),
        String, self.installSimulatorCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/redeploy_simulator'),
        String, self.redeploySimulatorCb, queue_size = 1)
    # "Use Existing" -- explicit operator override of launch_command's own
    # "a gzserver is already running" refuse-to-launch guard, offered
    # specifically when that guard fires (launcher_state 'gazebo_conflict').
    # See runLaunch's attach handling and SimulatorLauncher.launch's own
    # attach_launch_command docstring.
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/attach_simulator'),
        String, self.attachSimulatorCb, queue_size = 1)
    # "Launch New" -- the other choice offered alongside Use Existing:
    # force past the conflict by killing whatever gazebo is in the way
    # first, then launching fresh. Distinct from redeploy_simulator, which
    # assumes THIS app is what's currently running (stops via its own
    # tracked pgid) -- here the blocking gzserver is by definition
    # something this app never started, so there is nothing of its own to
    # stop; kill_all_gazebo is the only mechanism that reaches it.
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/force_launch_simulator'),
        String, self.forceLaunchSimulatorCb, queue_size = 1)
    # Standalone escape hatch, not tied to any one target -- see
    # SimulatorLauncher.kill_all_gazebo's own docstring for why this is
    # deliberately separate from (and much blunter than) the ordinary,
    # pgid-scoped stop_command path.
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/kill_all_gazebo'),
        Empty, self.killAllGazeboCb, queue_size = 1)
    # Lets an operator try their own robot without editing and redeploying
    # sim_connector_app_params.yaml -- independent of self.launcher (that's
    # only auto-launch), so this stays available on every deployment.
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/upload_robot_config'),
        String, self.uploadRobotConfigCb, queue_size = 1)
    # Backend counterpart of the RUI's per-config "View" button
    # (Nepi_IF_Sim.js's onViewConfigClicked/renderRobotConfigViewer) -- that
    # side was fully built already (request publisher, latched-reply
    # listener, a YAML text box, a download button) but nothing here ever
    # subscribed to answer it, so every click silently went nowhere (found
    # live 2026-08-19). Latched so a client that (re)subscribes after the
    # request already went out -- e.g. RUI reload mid-view -- still gets the
    # last-requested config's text rather than nothing.
    self.robot_config_yaml_pub = nepi_sdk.create_publisher(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/robot_config_yaml'),
        String, queue_size = 1, latch = True)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/get_robot_config'),
        String, self.getRobotConfigCb, queue_size = 1)

    ##############################
    # Physical-dimension editing (robot chassis/wheel + environment
    # corridor/ramp geometry) -- see DIMENSION_ROLES's own comment above for
    # the robot/environment -> model mapping. Curated-fields set/get mirrors
    # the robot_configs pattern just above; upload_*_model_sdf is the raw-
    # SDF escape hatch. dimensions_dirty seeds from whatever's ALREADY in
    # the device-side store at startup (not just False) -- otherwise a
    # container restart would silently forget that the VM's own copy still
    # needs a re-push, since the VM is a separate machine whose own state
    # this app has no other way to know is stale.
    self.dimensions_dirty = dict()
    self.dimensions_dirty_pubs = dict()
    self.dimensions_yaml_pubs = dict()
    for role in DIMENSION_ROLES:
      has_stored = bool(self.readStoredDimensionsYaml(role)) or bool(self.readStoredSdfOverride(role))
      self.dimensions_dirty[role] = has_stored
      self.dimensions_dirty_pubs[role] = nepi_sdk.create_publisher(
          nepi_sdk.create_namespace(self.node_namespace, 'sim/' + role + '_dimensions_dirty'),
          Bool, queue_size = 1, latch = True)
      self.dimensions_dirty_pubs[role].publish(Bool(data = has_stored))
      self.dimensions_yaml_pubs[role] = nepi_sdk.create_publisher(
          nepi_sdk.create_namespace(self.node_namespace, 'sim/' + role + '_dimensions_yaml'),
          String, queue_size = 1, latch = True)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/set_robot_dimensions'),
        String, self.setRobotDimensionsCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/set_environment_dimensions'),
        String, self.setEnvironmentDimensionsCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/upload_robot_model_sdf'),
        String, self.uploadRobotModelSdfCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/upload_environment_model_sdf'),
        String, self.uploadEnvironmentModelSdfCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/get_robot_dimensions'),
        Empty, self.getRobotDimensionsCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/get_environment_dimensions'),
        Empty, self.getEnvironmentDimensionsCb, queue_size = 1)

    ##############################
    # Phone-scan -> Gazebo environment conversion (see
    # docs/SCAN_TO_SIM_ENVIRONMENT_PLAN.md). Separate from the dimensions/
    # SDF-override system above -- a scanned mesh environment isn't a
    # dimensions.yaml field set, and running scan_to_environment.py takes
    # minutes (TSDF fusion + convex decomposition over ~1400 frames), so it
    # runs in its own background thread rather than blocking like
    # pushDirtyDimensions does for the small text pushes.
    self.scan_conversion_status_pub = nepi_sdk.create_publisher(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/phone_scan_conversion_status'),
        String, queue_size = 1, latch = True)
    self.scan_conversion_status_pub.publish(String(data = 'idle'))
    self.scan_conversion_thread = None
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/convert_phone_scan'),
        String, self.convertPhoneScanCb, queue_size = 1)
    # Best-effort: the VM may not be reachable yet at startup (tunnel not up,
    # device just booted) -- pushDirtyDimensions logs and moves on rather
    # than blocking init, and the next real Launch will retry via the same
    # dirty flags this left set on failure.
    if any(self.dimensions_dirty.values()):
      self.pushDirtyDimensions(DEFAULT_DIMENSIONS_PUSH_TARGET)

    # Latched, so a client that subscribes after startup still gets a real
    # report (available targets, "idle") instead of waiting for the first
    # launch/stop -- the same reasoning SimStatus's own latch already uses.
    self.publishLauncherStatus()
    if self.launcher is not None:
      self.startInstalledCheckAll()

    ##############################
    # Simulator discovery scan, and the app's own status cadence
    nepi_sdk.start_timer_process(float(1) / SIM_DISCOVERY_RATE_HZ, self.simDiscoveryCb)
    nepi_sdk.start_timer_process(float(1) / STATUS_PUBLISH_RATE_HZ, self.statusPublishCb)

    self.msg_if.pub_info("Sim connector listening on 0.0.0.0:" + str(self.listen_port))
    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.on_shutdown(self.cleanup_actions)
    nepi_sdk.spin()

  #**********************
  # Robot-config profiles

  def loadVehicleDictFromParamsFile(self):
    # Fallback source for SIM_VEHICLE_DICT when it is not on the param server
    # -- see the correction note in __init__ for why that is the normal case.
    # Reads this app's own installed params yaml, found via system_folders'
    # apps_param entry (the same folder apps_mgr scans) rather than a
    # hardcoded path, and identified by APP_DICT.pkg_name rather than a
    # hardcoded filename.
    #
    # Every failure path returns an empty dict rather than raising: the caller
    # then proceeds with the capability-empty factory profile, which is the
    # documented safe default and exactly the behavior that existed before
    # this fallback. A missing or malformed params file must never stop the
    # node from starting.
    try:
      folders = nepi_system.get_system_folders(timeout = SYSTEM_FOLDERS_TIMEOUT_MSEC)
      if not isinstance(folders, dict):
        self.msg_if.pub_warn("Could not read system_folders; " + SIM_VEHICLE_DICT_KEY +
                             " unavailable, using factory robot config only")
        return dict()
      params_folder = folders.get('apps_param', '')
      if not params_folder or not os.path.isdir(params_folder):
        self.msg_if.pub_warn("Apps params folder not found (" + str(params_folder) + "); " +
                             SIM_VEHICLE_DICT_KEY + " unavailable, using factory robot config only")
        return dict()

      # Same filename convention getAppsDict matches on ("*params*.yaml").
      for filename in sorted(os.listdir(params_folder)):
        if not filename.endswith('.yaml') or 'params' not in filename:
          continue
        file_path = os.path.join(params_folder, filename)
        file_dict = nepi_utils.read_yaml_2_dict(file_path)
        if not isinstance(file_dict, dict):
          continue
        app_dict = file_dict.get('APP_DICT', dict())
        if not isinstance(app_dict, dict) or app_dict.get('pkg_name', '') != APP_PKG_NAME:
          continue
        vehicle_dict = file_dict.get(SIM_VEHICLE_DICT_KEY, dict())
        if not isinstance(vehicle_dict, dict) or not vehicle_dict:
          self.msg_if.pub_warn("Found " + file_path + " but it has no usable " +
                               SIM_VEHICLE_DICT_KEY + ", using factory robot config only")
          return dict()
        self.msg_if.pub_info("Loaded " + SIM_VEHICLE_DICT_KEY + " from " + file_path)
        return vehicle_dict

      self.msg_if.pub_warn("No params file for " + APP_PKG_NAME + " found in " + params_folder +
                           ", using factory robot config only")
    except Exception as e:
      self.msg_if.pub_warn("Failed to load " + SIM_VEHICLE_DICT_KEY + " from params file: " + str(e) +
                           ", using factory robot config only")
    return dict()

  def buildProfileFromConfig(self, config_name):
    # A robot config is a capability profile read whole from the params yaml. A
    # missing key means the capability is off, so a partially-written config
    # entry degrades to fewer controls rather than to a crash.
    entry = self.robot_configs.get(config_name, dict())
    if not isinstance(entry, dict):
      entry = dict()
    return self.buildProfileFromEntry(entry)

  def buildProfileFromEntry(self, entry):
    # The field-by-field coercion buildProfileFromConfig above applies to
    # whatever entry it looks up -- split out so uploadRobotConfigCb can run
    # the exact same coercion (and hit the exact same int()/bool() failures)
    # against an uploaded entry BEFORE accepting it into self.robot_configs,
    # instead of only finding out a field is bad the next time the config
    # happens to get selected.
    return dict(
        wheel_count = int(entry.get('wheel_count', 0)),
        motor_count = int(entry.get('motor_count', 0)),
        has_goto_position = bool(entry.get('has_goto_position', False)),
        has_goto_pose = bool(entry.get('has_goto_pose', False)),
        has_goto_location = bool(entry.get('has_goto_location', False)),
        has_go_home = bool(entry.get('has_go_home', False)),
        has_set_home = bool(entry.get('has_set_home', False)),
        has_go_stop = bool(entry.get('has_go_stop', False)),
        setup_actions = list(entry.get('setup_actions', [])),
        go_actions = list(entry.get('go_actions', [])),
        has_camera_view_control = bool(entry.get('has_camera_view_control', False)),
        available_camera_view_modes = list(entry.get('available_camera_view_modes', [])),
        has_environment_controls = bool(entry.get('has_environment_controls', False)),
    )

  def buildCapabilityKwargs(self, profile):
    # Turns a capability profile into the exact keyword set SimDeviceIF derives
    # its has_* flags from: a real bound method where the profile says the robot
    # has that capability, None where it does not. This is the only place the
    # config-to-callback mapping lives, so construction and a later
    # apply_capability_profile can never disagree.
    has_motors = profile['motor_count'] > 0
    has_any_goto = (profile['has_goto_position'] or profile['has_goto_pose']
                    or profile['has_goto_location'])

    axis_controls = AxisControls()
    axis_controls.x = profile['has_goto_position']
    axis_controls.y = profile['has_goto_position']
    # A wheeled robot moves in the ground plane, so z is only meaningful once
    # the profile declares no wheels (a flight or subsea vehicle).
    axis_controls.z = profile['has_goto_position'] and profile['wheel_count'] == 0
    axis_controls.roll = profile['has_goto_pose']
    axis_controls.pitch = profile['has_goto_pose']
    axis_controls.yaw = profile['has_goto_pose'] or profile['has_goto_position']

    return dict(
        axisControls = axis_controls,
        wheel_count = profile['wheel_count'],
        motor_count = profile['motor_count'],
        setMotorControlRatio = self.setMotorControlRatio if has_motors else None,
        getMotorControlRatios = self.getMotorControlRatios if has_motors else None,
        manualControlsReadyFunction = self.isBridgeConnected if has_motors else None,
        autonomousControlsReadyFunction = self.autonomousControlsReady if has_any_goto else None,
        setup_actions = profile['setup_actions'],
        setSetupActionIndFunction = self.setSetupActionInd if profile['setup_actions'] else None,
        go_actions = profile['go_actions'],
        setGoActionIndFunction = self.setGoActionInd if profile['go_actions'] else None,
        getHomeFunction = self.getHome if profile['has_set_home'] else None,
        setHomeFunction = self.setHome if profile['has_set_home'] else None,
        goHomeFunction = self.goHome if profile['has_go_home'] else None,
        goStopFunction = self.goStop if profile['has_go_stop'] else None,
        gotoPoseFunction = self.gotoPose if profile['has_goto_pose'] else None,
        gotoPositionFunction = self.gotoPosition if profile['has_goto_position'] else None,
        gotoLocationFunction = self.gotoLocation if profile['has_goto_location'] else None,
        setCameraViewModeFunction = (self.setCameraViewMode
                                     if profile['has_camera_view_control'] else None),
        available_camera_view_modes = profile['available_camera_view_modes'],
        setEnvironmentOptionFunction = (self.setEnvironmentOption
                                        if profile['has_environment_controls'] else None),
        available_environment_options = list(self.available_environment_options),
    )

  #**********************
  # Robot config selector

  def getAvailableRobotConfigs(self):
    # Returns (keys, names) -- keys are what select_robot_config actually
    # takes and what a bridge script matches against on the wire, so they
    # never change; names are read from each config entry's own
    # display_name (falling back to the key itself for any entry that
    # doesn't have one yet, e.g. an older or hand-written params file), and
    # exist purely for a UI to show something readable.
    #
    # A config with hidden_from_selector skips this list -- it exists to be
    # applied automatically (a launch target's own robot_config_overrides
    # resolves a plain, offered choice like "2-Wheel Rover" to it for that
    # specific simulator), not to be picked directly. It stays fully valid
    # for setSelectedRobotConfig to accept -- only the offered list changes,
    # not what selection is legal.
    keys = []
    names = []
    for key in sorted(self.robot_configs.keys()):
      entry = self.robot_configs.get(key, dict())
      if isinstance(entry, dict) and entry.get('hidden_from_selector', False):
        continue
      display_name = entry.get('display_name', '') if isinstance(entry, dict) else ''
      keys.append(key)
      names.append(display_name if display_name else key)
    return keys, names

  def getSelectedRobotConfig(self):
    return self.selected_robot_config

  def setSelectedRobotConfig(self, config_name):
    config_name = str(config_name)
    if config_name not in self.robot_configs:
      self.msg_if.pub_warn("Robot config '" + config_name + "' is not one of " +
                           str(sorted(self.robot_configs.keys())) + ", ignoring")
      return
    self.selected_robot_config = config_name
    self.profile = self.buildProfileFromConfig(config_name)
    self.motor_ratios = [0.0] * int(self.profile['motor_count'])
    # Selecting a config is selecting a kind of robot, and a robot's kind IS its
    # capability set -- so the cached report is re-derived in place from the new
    # profile. SimDeviceIF.apply_capability_profile republishes info and status
    # itself, so the new flags reach a client within one status interval.
    #
    # NOTE: self.sim_if is None until the SimDeviceIF constructor call below
    # (in __init__) fully returns -- and that constructor's own sub-IF setup
    # (NavPoseIF/SaveDataIF/SettingsIF/Transform3DIF) can take several seconds.
    # Its select_robot_config SUBSCRIBER is live before that finishes (it's
    # registered early in NodeClassIF's own setup), so a config-select message
    # arriving during that startup window silently no-ops here rather than
    # queuing -- confirmed via live testing, not a defect worth chasing further
    # given it only matters in the first few seconds after node start.
    if self.sim_if is not None:
      self.sim_if.apply_capability_profile(**self.buildCapabilityKwargs(self.profile))
    # Tell the simulator which kind of robot is wanted.
    self.sendLineToBridge({'type': 'robot_config', 'config': config_name}, "Robot config")
    self.msg_if.pub_info("Selected robot config: " + config_name)

  def uploadRobotConfigCb(self, msg):
    # Lets an operator try their own robot against the sim without editing
    # and redeploying sim_connector_app_params.yaml -- the uploaded text is
    # expected to be one robot_configs entry (the same field shape as e.g.
    # ground_robot_2_wheel in that file, minus the wrapping key -- see the
    # RUI's downloadable sample for the exact shape). Rejects clearly
    # (pub_warn, same convention as an unrecognized config key in
    # setSelectedRobotConfig above) rather than accepting something that
    # would only fail later when the config gets selected or a field gets
    # read.
    try:
      entry = yaml.safe_load(str(msg.data))
    except yaml.YAMLError as e:
      self.msg_if.pub_warn("Uploaded robot config is not valid YAML: " + str(e))
      return
    if not isinstance(entry, dict):
      self.msg_if.pub_warn("Uploaded robot config must be a YAML mapping of fields "
                           "(got " + type(entry).__name__ + ") -- see the downloadable "
                           "sample config for the expected shape")
      return
    entry = copy.deepcopy(entry)
    # An upload is always meant to be picked directly -- hidden_from_selector
    # is a mechanism for checked-in configs reached only through a launch
    # target's robot_config_overrides, not something an uploaded file should
    # be able to set on itself.
    entry.pop('hidden_from_selector', None)
    try:
      self.buildProfileFromEntry(entry)
    except (TypeError, ValueError) as e:
      self.msg_if.pub_warn("Uploaded robot config has an invalid field value: " + str(e))
      return
    display_name = str(entry.get('display_name') or 'Custom Robot')
    entry['display_name'] = display_name
    self.robot_configs[UPLOADED_ROBOT_CONFIG_NAME] = entry
    self.msg_if.pub_info("Uploaded robot config '" + display_name + "', applying it now")

  def getRobotConfigCb(self, msg):
    # Answers with whatever this app's OWN self.robot_configs dict currently
    # holds for that key -- real, currently-loaded configs, same source
    # setSelectedRobotConfig reads from -- see onViewConfigClicked's own
    # comment in Nepi_IF_Sim.js for why the sample/uploaded configs aren't
    # reachable this way (they're not real entries a device round-trip
    # through get_robot_config would find).
    config_name = str(msg.data)
    entry = self.robot_configs.get(config_name)
    if entry is None:
      self.msg_if.pub_warn("Robot config '" + config_name + "' not found, cannot show it")
      self.robot_config_yaml_pub.publish(String(data =
          "# Robot config '" + config_name + "' not found"))
      return
    try:
      yaml_text = yaml.safe_dump(entry, default_flow_style = False, sort_keys = False)
    except yaml.YAMLError as e:
      self.msg_if.pub_warn("Failed to serialize robot config '" + config_name + "': " + str(e))
      return
    self.robot_config_yaml_pub.publish(String(data = yaml_text))
    self.setSelectedRobotConfig(UPLOADED_ROBOT_CONFIG_NAME)

  #**********************
  # Physical-dimension editing (robot chassis/wheel + environment
  # corridor/ramp geometry) -- see DIMENSION_ROLES's own comment for the
  # role -> model mapping and the device-store-is-authoritative design.

  def dimensionsYamlStoragePath(self, role):
    return os.path.join(DIMENSIONS_STORAGE_DIR, role + '.yaml')

  def sdfOverrideStoragePath(self, role):
    return os.path.join(DIMENSIONS_STORAGE_DIR, role + '.sdf')

  def readStoredDimensionsYaml(self, role):
    path = self.dimensionsYamlStoragePath(role)
    if not os.path.exists(path):
      return ''
    try:
      with open(path, 'r') as f:
        return f.read()
    except Exception as e:
      self.msg_if.pub_warn("Failed to read stored " + role + " dimensions: " + str(e))
      return ''

  def writeStoredDimensionsYaml(self, role, yaml_text):
    path = self.dimensionsYamlStoragePath(role)
    try:
      os.makedirs(os.path.dirname(path), exist_ok = True)
      with open(path, 'w') as f:
        f.write(yaml_text)
    except Exception as e:
      self.msg_if.pub_warn("Failed to write stored " + role + " dimensions: " + str(e))

  def readStoredSdfOverride(self, role):
    path = self.sdfOverrideStoragePath(role)
    if not os.path.exists(path):
      return ''
    try:
      with open(path, 'r') as f:
        return f.read()
    except Exception as e:
      self.msg_if.pub_warn("Failed to read stored " + role + " SDF override: " + str(e))
      return ''

  def writeStoredSdfOverride(self, role, sdf_text):
    path = self.sdfOverrideStoragePath(role)
    try:
      os.makedirs(os.path.dirname(path), exist_ok = True)
      with open(path, 'w') as f:
        f.write(sdf_text)
    except Exception as e:
      self.msg_if.pub_warn("Failed to write stored " + role + " SDF override: " + str(e))

  def clearStoredSdfOverride(self, role):
    path = self.sdfOverrideStoragePath(role)
    try:
      if os.path.exists(path):
        os.remove(path)
    except Exception as e:
      self.msg_if.pub_warn("Failed to clear stored " + role + " SDF override: " + str(e))

  def markDimensionsDirty(self, role):
    self.dimensions_dirty[role] = True
    self.publishDimensionsDirty(role)

  def publishDimensionsDirty(self, role):
    pub = self.dimensions_dirty_pubs.get(role)
    if pub is not None:
      pub.publish(Bool(data = self.dimensions_dirty.get(role, False)))

  def pushDirtyDimensions(self, target_key):
    # Called right before a real Launch (see runLaunch) and once at startup
    # for whatever the device-side store already had persisted -- best
    # effort either way: a failed push leaves the dirty flag set so the
    # next attempt retries, rather than silently giving up.
    if self.launcher is None:
      return
    try:
      target = self.launcher.get_target(target_key)
    except Exception as e:
      self.msg_if.pub_warn("Could not resolve launch target '" + target_key +
                           "' to push dimensions: " + str(e))
      return
    for role in DIMENSION_ROLES:
      if not self.dimensions_dirty.get(role, False):
        continue
      model_name = DIMENSION_ROLE_MODEL[role]
      try:
        self.launcher.push_dimensions(target, model_name,
            self.readStoredDimensionsYaml(role), self.readStoredSdfOverride(role))
        self.dimensions_dirty[role] = False
        self.publishDimensionsDirty(role)
        self.msg_if.pub_info("Pushed " + role + " dimensions (" + model_name + ") to the sim VM")
      except LauncherError as e:
        self.msg_if.pub_warn("Failed to push " + role + " dimensions to the sim VM: " + str(e))

  def setRobotDimensionsCb(self, msg):
    self.setDimensionsCb('robot', msg)

  def setEnvironmentDimensionsCb(self, msg):
    self.setDimensionsCb('environment', msg)

  def setDimensionsCb(self, role, msg):
    # Curated-fields path -- validate the uploaded text really is a YAML
    # mapping of dimension fields before persisting it, same "reject
    # clearly rather than fail later" convention as uploadRobotConfigCb.
    # generate_model_sdf.py itself fills in any field this doesn't set with
    # its own default, so a partial edit (only the field that changed)
    # doesn't need to carry every other field along.
    try:
      fields = yaml.safe_load(str(msg.data))
    except yaml.YAMLError as e:
      self.msg_if.pub_warn(role + " dimensions is not valid YAML: " + str(e))
      return
    if not isinstance(fields, dict):
      self.msg_if.pub_warn(role + " dimensions must be a YAML mapping of fields (got " +
                           type(fields).__name__ + ")")
      return
    try:
      yaml_text = yaml.safe_dump(fields, default_flow_style = False, sort_keys = False)
    except yaml.YAMLError as e:
      self.msg_if.pub_warn("Failed to serialize " + role + " dimensions: " + str(e))
      return
    self.writeStoredDimensionsYaml(role, yaml_text)
    # A curated-fields edit supersedes any earlier raw-SDF-upload override --
    # otherwise pushDirtyDimensions would keep re-applying the stale raw SDF
    # (push_dimensions gives it precedence) and the new curated values would
    # silently never take effect.
    self.clearStoredSdfOverride(role)
    self.markDimensionsDirty(role)
    self.msg_if.pub_info("Updated " + role + " dimensions: " + yaml_text.replace(chr(10), ' '))

  def uploadRobotModelSdfCb(self, msg):
    self.uploadModelSdfCb('robot', msg)

  def uploadEnvironmentModelSdfCb(self, msg):
    self.uploadModelSdfCb('environment', msg)

  def uploadModelSdfCb(self, role, msg):
    # Raw-SDF-upload escape hatch -- bypasses generate_model_sdf.py entirely,
    # for geometry the curated fields don't cover. No XML validation here
    # (this app has no SDF parser); a bad upload surfaces the same way a bad
    # hand-edit would -- Gazebo refusing/misbehaving on the next launch --
    # rather than this node trying to second-guess Gazebo's own validation.
    sdf_text = str(msg.data)
    if not sdf_text.strip():
      self.msg_if.pub_warn(role + " SDF upload was empty, ignoring")
      return
    self.writeStoredSdfOverride(role, sdf_text)
    self.markDimensionsDirty(role)
    self.msg_if.pub_info("Uploaded raw model.sdf override for " + role)

  def convertPhoneScanCb(self, msg):
    # msg.data is a scan name -- the raw scan folder must already exist at
    # SCAN_UPLOADS_STORAGE_DIR/<name>/ (see that constant's own comment: the
    # browser -> device upload step that would populate it isn't built yet).
    # One conversion at a time -- a second request while one is already
    # running is rejected rather than queued, same "reject clearly" instinct
    # as uploadModelSdfCb's empty-upload check.
    scan_name = str(msg.data).strip()
    if self.scan_conversion_thread is not None and self.scan_conversion_thread.is_alive():
      self.msg_if.pub_warn("Ignoring convert_phone_scan for '" + scan_name +
                           "' -- a conversion is already running")
      return
    self.scan_conversion_thread = threading.Thread(
        target = self.runPhoneScanConversion, args = (scan_name,))
    self.scan_conversion_thread.start()

  def runPhoneScanConversion(self, scan_name):
    # Runs in its own thread (see convertPhoneScanCb) -- push_scan_directory
    # (scp of ~100MB+ of video/PNG frames) and convert_scan_to_environment
    # (TSDF fusion + convex decomposition, minutes) are both far too slow for
    # a subscriber callback. Uses DEFAULT_DIMENSIONS_PUSH_TARGET, the same
    # fixed VM target the dimensions system already pushes to -- there is
    # only the one dev VM in this setup today.
    self.scan_conversion_status_pub.publish(String(data = 'running: ' + scan_name))
    local_scan_dir = os.path.join(SCAN_UPLOADS_STORAGE_DIR, scan_name)
    if not os.path.isdir(local_scan_dir):
      msg = ("failed: scan '" + scan_name + "' not found at " + local_scan_dir +
             " (upload it first)")
      self.msg_if.pub_warn(msg)
      self.scan_conversion_status_pub.publish(String(data = msg))
      return
    if self.launcher is None:
      msg = "failed: simulator launcher not configured"
      self.msg_if.pub_warn(msg)
      self.scan_conversion_status_pub.publish(String(data = msg))
      return
    try:
      target = self.launcher.get_target(DEFAULT_DIMENSIONS_PUSH_TARGET)
      self.launcher.push_scan_directory(target, local_scan_dir, scan_name)
      self.launcher.convert_scan_to_environment(target, scan_name, scan_name)
    except LauncherError as e:
      msg = "failed: " + str(e)
      self.msg_if.pub_warn("Phone scan conversion failed for '" + scan_name + "': " + str(e))
      self.scan_conversion_status_pub.publish(String(data = msg))
      return
    # The new model now exists at sim_container/models/<scan_name>/ on the
    # VM, alongside obstacle_course -- but rbx_sim_node.py's `environment`
    # Setting option list is read once at driver construction (see
    # docs/SCAN_TO_SIM_ENVIRONMENT_PLAN.md section 2.1), so it won't appear
    # as a selectable option until the rover driver (re)starts or the sim is
    # relaunched. Said plainly here rather than implying this is immediately
    # selectable.
    msg = ("done: '" + scan_name + "' ready on the VM -- restart the rover driver or "
           "relaunch the sim to select it as an environment")
    self.msg_if.pub_info(msg)
    self.scan_conversion_status_pub.publish(String(data = msg))

  def getRobotDimensionsCb(self, msg):
    self.getDimensionsCb('robot')

  def getEnvironmentDimensionsCb(self, msg):
    self.getDimensionsCb('environment')

  def getDimensionsCb(self, role):
    yaml_text = self.readStoredDimensionsYaml(role)
    pub = self.dimensions_yaml_pubs.get(role)
    if pub is None:
      return
    if not yaml_text:
      yaml_text = "# No stored " + role + " dimensions yet -- using factory defaults"
    pub.publish(String(data = yaml_text))

  #**********************
  # Simulator selector. Discovery is a timer scan of the live ROS graph, matched
  # on device type plus the device's own declared source description. Never a
  # blocking wait, and never a match on any simulator product name.

  def simDiscoveryCb(self, timer):
    try:
      topics, msg_types = nepi_sdk.find_topics_by_msgs(SIM_DEVICE_STATUS_MSG_TYPES)
    except Exception as e:
      self.msg_if.pub_warn("Simulator scan failed: " + str(e), throttle_s = 10.0)
      return

    # find_topics_by_msgs returns two PARALLEL lists, not a list of tuples --
    # zip them rather than iterating one of them as pairs.
    found = dict(zip(topics, msg_types))

    with self.sim_scan_lock:
      # Subscribe to any newly seen candidate device status topic. The declared
      # source description only rides on the status message, so a subscription
      # is how a candidate is qualified.
      for topic in found.keys():
        if topic in self.sim_device_subs:
          continue
        try:
          sub = nepi_sdk.create_subscriber(topic, DeviceRBXStatus,
                                          self.simDeviceStatusCb,
                                          queue_size = 1, callback_args = (topic,))
        except Exception as e:
          self.msg_if.pub_warn("Failed to subscribe candidate " + topic + ": " + str(e),
                               throttle_s = 10.0)
          continue
        if sub is None:
          # create_subscriber logs and returns None on failure rather than
          # raising, so this is the real failure path -- retried next scan.
          continue
        self.sim_device_subs[topic] = sub

      # Drop candidates whose status topic has gone away entirely, OR whose
      # status messages have gone stale.
      #
      # The staleness half is NOT redundant with the vanished half -- without
      # it this loop can never actually unregister anything. find_topics_by_msgs
      # (used to build `found` above) counts a topic as present if it has
      # EITHER a publisher or a subscriber, and the subscription THIS loop
      # would need to drop is itself one of those subscribers -- so as long as
      # we still hold it, `topic in found` stays true, so this loop never
      # unregisters, so the topic never goes away: our own subscription is the
      # only thing keeping a dead device's status topic visible in the ROS
      # graph, which is exactly what the RUI's Devices page reads (it scans
      # raw topic names via rosapi, not this app's own state) -- confirmed
      # live: after killing a rover sim, both the RBX node and its discovery
      # were gone, yet .../rbx/status still listed with "Publishers: None" and
      # this app as the sole subscriber, and the RUI kept showing the device.
      #
      # SIM_DEVICE_STALE_SEC is the same threshold getAvailableSimulators
      # already uses to hide a stale device from the selector; this makes the
      # subscription itself follow the same rule instead of outliving it. A
      # candidate with no entry yet (subscribed this scan, no message received)
      # is NOT stale -- it has no 'time' to judge and gets the next scan to
      # report in.
      now = nepi_utils.get_time()
      for topic in list(self.sim_device_subs.keys()):
        entry = self.sim_device_info.get(topic)
        stale = (entry is not None and (now - entry['time']) > SIM_DEVICE_STALE_SEC)
        if topic in found and not stale:
          continue
        sub = self.sim_device_subs.pop(topic)
        self.sim_device_info.pop(topic, None)
        try:
          sub.unregister()
        except Exception:
          pass

    # A selection that is no longer available falls back to no selection rather
    # than silently pointing at a device that is gone.
    if self.selected_simulator != "":
      available, _names = self.getAvailableSimulators()
      if self.selected_simulator not in available:
        self.msg_if.pub_warn("Selected simulator " + self.selected_simulator +
                             " is no longer available, clearing selection")
        self.selected_simulator = ""

    # Auto-select when exactly one real candidate is discovered and nothing
    # is currently selected. Found live (2026-08-18) as the actual root cause
    # of every Sim Connector configuration control (capability toggles,
    # camera/movement Settings -- see Nepi_IF_Sim-Controls.js's rbx_namespace,
    # which is driven entirely by selected_simulator) staying permanently
    # unreachable: available_simulators correctly listed the running RBX
    # driver the whole time (confirmed via direct inspection), but nothing
    # ever called select_simulator to actually choose it, because the manual
    # "Simulator" selector control that would have was removed earlier under
    # the belief that this list is always empty in practice -- it is not,
    # only the SELECTION step was missing. This project only ever deploys one
    # simulator at a time, so auto-selecting the sole candidate is a correct
    # default, not a guess; more than one simultaneous candidate (not a real
    # scenario today) still requires an explicit choice rather than picking
    # one arbitrarily.
    if self.selected_simulator == "":
      available, _names = self.getAvailableSimulators()
      if len(available) == 1:
        self.setSelectedSimulator(available[0])

    self.updateCommonViewSubscriptions()

  def updateCommonViewSubscriptions(self):
    # Re-points each of the six mirror_pubs at whichever real Image topics
    # the currently selected_simulator's own RBX driver publishes -- see
    # those publishers' own comment in __init__ for the full reasoning.
    # selected_simulator is the DeviceRBXStatus publisher's namespace, e.g.
    # ".../sim_rover1/rbx"; its own image topics are siblings under the
    # plain node namespace (one level up), the same relationship
    # NepiDeviceRBX.js's createImageOptions relies on for the same reason.
    node_namespace = self.selected_simulator.split('/rbx')[0] if self.selected_simulator else ""

    sources = dict()
    if node_namespace == "":
      for view in self.MIRROR_VIEWS:
        sources[view] = ""
    else:
      base = node_namespace + "/color_2d_image"
      for view in self.MIRROR_VIEWS:
        sources[view] = nepi_sdk.find_topic(base + "/" + view)
      # Fall back to the bare topic for a driver honestly reporting one
      # single camera (the Webots drivers) -- only robot_color has a
      # single-camera fallback; there is no depth/scene equivalent to fall
      # back to.
      if sources["robot_color"] == "":
        sources["robot_color"] = nepi_sdk.find_topic(base, exact = True)

    for view in self.MIRROR_VIEWS:
      self.repointCommonViewSub(view, sources[view])

  def repointCommonViewSub(self, which, source_topic):
    pub = self.mirror_pubs[which]
    old_source = self.mirror_source_topics[which]
    if old_source == source_topic:
      return  # Already pointed at the right thing (including both empty).
    old_sub = self.mirror_subs[which]
    if old_sub is not None:
      try:
        old_sub.unregister()
      except Exception:
        pass
      self.mirror_subs[which] = None
    self.mirror_source_topics[which] = source_topic
    if source_topic == "":
      # The underlying RBX driver's own image topic just disappeared (SITL/
      # the sim killed, driver node gone) -- without this, the mirror_pubs
      # simply stop receiving new frames and every viewer (this app's own
      # preview, the generic Robot Viewer, image_viewer) freezes on the last
      # real frame forever, since ROS/web_video_server have no "the source
      # went away" signal of their own. Publishing one blank frame here
      # makes that state visibly obvious instead of looking like a
      # stuck-but-still-live feed. Only when there WAS a real source before
      # (old_source not empty) -- otherwise this fires once at startup for
      # every never-yet-connected view, which is just noise.
      if old_source != "":
        self.publishBlankCommonViewFrame(pub, which)
      return
    new_sub = nepi_sdk.create_subscriber(source_topic, Image, self.commonViewImageCb,
                                        queue_size = 1, callback_args = (pub,))
    self.mirror_subs[which] = new_sub

  def publishBlankCommonViewFrame(self, pub, which):
    try:
      if which in ("robot_depth_map", "scene_depth_map"):
        blank = np.zeros((480, 640), dtype = np.float32)
        pub.publish(nepi_img.cv2img_to_rosimg(blank, encoding = "32FC1"))
      else:
        blank = np.zeros((480, 640, 3), dtype = np.uint8)
        pub.publish(nepi_img.cv2img_to_rosimg(blank, encoding = "bgr8"))
    except Exception as e:
      self.msg_if.pub_warn("Failed to publish blank common-view frame: " + str(e))

  def commonViewImageCb(self, msg, args):
    pub = args[0] if isinstance(args, tuple) else args
    pub.publish(msg)

  def simDeviceStatusCb(self, msg, args):
    topic = args[0] if isinstance(args, tuple) else args
    with self.sim_scan_lock:
      self.sim_device_info[topic] = dict(
          device_name = msg.device_name,
          data_source_description = msg.data_source_description,
          time = nepi_utils.get_time())

  def getAvailableSimulators(self):
    # Returns two parallel lists (namespaces, display names) -- the same
    # reported-list shape the RUI's other selectors consume. Empty is valid.
    namespaces = []
    names = []
    now = nepi_utils.get_time()
    with self.sim_scan_lock:
      for topic in sorted(self.sim_device_info.keys()):
        entry = self.sim_device_info[topic]
        if entry['data_source_description'] != SIM_SOURCE_DESCRIPTION:
          continue
        if (now - entry['time']) > SIM_DEVICE_STALE_SEC:
          continue
        # The device namespace is the status topic minus its trailing '/status'.
        namespace = topic
        if namespace.endswith('/status'):
          namespace = namespace[:-len('/status')]
        namespaces.append(namespace)
        names.append(entry['device_name'] if entry['device_name'] else namespace)
    return namespaces, names

  def getSelectedSimulator(self):
    return self.selected_simulator

  def setSelectedSimulator(self, namespace):
    namespace = str(namespace)
    if namespace in ("", "None"):
      self.selected_simulator = ""
      self.msg_if.pub_info("Cleared simulator selection")
      return
    available, _names = self.getAvailableSimulators()
    if namespace not in available:
      self.msg_if.pub_warn("Simulator '" + namespace + "' is not currently available, ignoring")
      return
    self.selected_simulator = namespace
    self.msg_if.pub_info("Selected simulator: " + namespace)

  #**********************
  # Simulator auto-launch. A convenience trigger over the existing passive
  # flow, not a parallel path: on success this calls the same
  # setSelectedRobotConfig already used by the robot-config selector above,
  # so the rest of the app behaves exactly as if an operator had started the
  # simulator by hand and picked that config themselves. Deliberately does
  # NOT touch setSelectedSimulator/selected_simulator -- that selector picks
  # among other simulator-*capable NEPI devices* discovered on the ROS graph
  # (simDiscoveryCb above), a different axis entirely from which simulator
  # *software* this launch target starts on a dev VM.
  #
  # launcher.launch()/wait_until_ready()/stop() all block for real seconds
  # (an ssh round trip, sleeps while the simulator comes up) -- run on a
  # background thread so the bridge server thread and ROS callback dispatch
  # are never held up by one. launcher_lock only guards against two
  # launch/stop requests racing each other, not against the sim's own I/O.

  def setLauncherError(self, error):
    """Single place that sets both launcher_last_error and
    launcher_tunnel_fallback_commands, so every call site (there are over a
    dozen) doesn't need its own copy of "does this LauncherError carry a
    manual_fallback_commands override" logic. Accepts either a plain string
    (the '' clear case, or a hand-built message that was never a
    LauncherError) or an exception -- str(error) becomes the message either
    way, and manual_fallback_commands is read off the exception if present
    (see simulator_launcher.LauncherError/_classify_connection_failure),
    else cleared, so a fallback from a PREVIOUS failure never lingers onto
    an unrelated new one."""
    self.launcher_last_error = str(error)
    self.launcher_tunnel_fallback_commands = getattr(error, 'manual_fallback_commands', None) or ''

  def parseLaunchPayload(self, msg):
    """Parses a sim/launch_simulator-family String message into
    (target_key, robot_config). The RUI encodes both as one JSON object,
    {"target_key": ..., "robot_config": ...}, so the operator's current
    robot-config selection travels atomically with the launch request
    itself. Reading self.selected_robot_config here instead -- set by
    select_robot_config's own subscriber callback, on an independent
    thread with no ordering guarantee relative to this one -- raced it:
    confirmed live (2026-08-28) that a fresh page load, picking
    Quadcopter, then an immediate Deploy could still resolve and launch
    the rover, because resendRobotConfigIfKnown sending select_robot_config
    first on the same websocket connection only narrows that race, it
    doesn't close it (each topic gets its own TCPROS connection into this
    process, and rospy dispatches each on its own thread).
    Falls back to treating the whole payload as a bare target_key with no
    robot_config (the topic's original plain-string format) if it isn't a
    JSON object with a target_key field, for any caller still publishing
    that way -- runLaunch/runRedeploy/runForceLaunch then fall back to
    self.selected_robot_config themselves, same as before this fix."""
    raw = str(msg.data).strip()
    try:
      payload = json.loads(raw)
      if isinstance(payload, dict) and 'target_key' in payload:
        robot_config = payload.get('robot_config')
        robot_config = str(robot_config).strip() if robot_config else None
        return str(payload['target_key']).strip(), robot_config
    except ValueError:
      pass
    return raw, None

  def launchSimulatorCb(self, msg):
    if self.launcher is None:
      self.msg_if.pub_warn("Simulator auto-launch is not configured on this deployment "
                           "(no launch-targets config found), ignoring launch request")
      return
    target_key, robot_config = self.parseLaunchPayload(msg)
    with self.launcher_lock:
      if self.launcher_thread is not None and self.launcher_thread.is_alive():
        self.msg_if.pub_warn("A launch/stop is already in progress, ignoring")
        return
      self.launcher_thread = threading.Thread(target = self.runLaunch,
                                              args = (target_key,),
                                              kwargs = {'robot_config': robot_config})
      self.launcher_thread.daemon = True
      self.launcher_thread.start()

  def stopSimulatorCb(self, msg):
    # Checks BOTH, not just selected_launch_target -- an app restart (e.g.
    # nepicommit restarting the whole container) resets this node's
    # in-memory selected_launch_target/active_launch_target to '' even
    # while a simulator launched by a PREVIOUS process instance is still
    # genuinely running on the VM (nothing there depends on this app's own
    # process lifetime). Guarding on selected_launch_target alone made this
    # a silent no-op in exactly that case -- launcher_state was already
    # sitting at its fresh-boot 'idle' default, so it read as "stop
    # succeeded" without runStop (and therefore the actual remote
    # stop_command) ever being attempted, leaving the orphaned sim running
    # and blocking the next launch attempt's own "already running" guard.
    # Confirmed live as the actual mechanism behind repeated "stop reports
    # idle but the process is still there" reports this session -- matches
    # the same active-or-selected fallback stop_target already uses below.
    if self.launcher is None or not (self.active_launch_target or self.selected_launch_target):
      return
    with self.launcher_lock:
      if self.launcher_thread is not None and self.launcher_thread.is_alive():
        self.msg_if.pub_warn("A launch/stop is already in progress, ignoring")
        return
      # active_launch_target -- the target with real SSH-launched processes
      # -- may differ from selected_launch_target when the current robot
      # config redirected the launch elsewhere (see runLaunch); falls back
      # to selected_launch_target for the ordinary, unredirected case.
      stop_target = self.active_launch_target or self.selected_launch_target
      self.launcher_thread = threading.Thread(target = self.runStop, args = (stop_target,))
      self.launcher_thread.daemon = True
      self.launcher_thread.start()

  def redeploySimulatorCb(self, msg):
    # The explicit "start fresh" action offered alongside the reuse path (see
    # runLaunch's own short-circuit below) when a client already knows a sim
    # is up and wants a clean restart rather than reusing it -- e.g. to reset
    # world state, or because it wants a DIFFERENT target than what's
    # currently running. Shares the same busy-guard and thread as
    # launch/stop, since it is just those two run back-to-back.
    if self.launcher is None:
      self.msg_if.pub_warn("Simulator auto-launch is not configured on this deployment "
                           "(no launch-targets config found), ignoring redeploy request")
      return
    target_key, robot_config = self.parseLaunchPayload(msg)
    with self.launcher_lock:
      if self.launcher_thread is not None and self.launcher_thread.is_alive():
        self.msg_if.pub_warn("A launch/stop/install is already in progress, ignoring")
        return
      self.launcher_thread = threading.Thread(target = self.runRedeploy,
                                              args = (target_key,),
                                              kwargs = {'robot_config': robot_config})
      self.launcher_thread.daemon = True
      self.launcher_thread.start()

  def runRedeploy(self, target_key, robot_config=None):
    # Stops whatever is currently tracked as running (if anything) then
    # launches target_key fresh. Reuses runStop/runLaunch directly rather
    # than duplicating their logic -- this method IS just those two run
    # back-to-back on the one background thread redeploySimulatorCb already
    # started.
    #
    # Checks active_launch_target too, not just launcher_state == 'running'
    # -- same reasoning as stopSimulatorCb's own guard: an app restart
    # resets launcher_state to its fresh-boot 'idle' even while a simulator
    # launched by a previous process instance is genuinely still running on
    # the VM. Skipping runStop here because launcher_state said 'idle' left
    # the real orphan alive, so runLaunch below hit the launch script's own
    # "already running" refuse-guard instead of actually redeploying.
    if (self.launcher_state == 'running' or self.active_launch_target) and (self.active_launch_target or self.selected_launch_target):
      self.runStop(self.active_launch_target or self.selected_launch_target)
      if self.launcher_state == 'failed':
        return  # runStop already published the failure; nothing more to do
    self.runLaunch(target_key, robot_config=robot_config)

  def attachSimulatorCb(self, msg):
    # "Use Existing" -- see runLaunch's attach handling.
    if self.launcher is None:
      self.msg_if.pub_warn("Simulator auto-launch is not configured on this deployment "
                           "(no launch-targets config found), ignoring attach request")
      return
    target_key, robot_config = self.parseLaunchPayload(msg)
    with self.launcher_lock:
      if self.launcher_thread is not None and self.launcher_thread.is_alive():
        self.msg_if.pub_warn("A launch/stop/install is already in progress, ignoring")
        return
      self.launcher_thread = threading.Thread(target = self.runLaunch, args = (target_key,),
                                              kwargs = {'attach': True, 'robot_config': robot_config})
      self.launcher_thread.daemon = True
      self.launcher_thread.start()

  def forceLaunchSimulatorCb(self, msg):
    # "Launch New" -- see runForceLaunch.
    if self.launcher is None:
      self.msg_if.pub_warn("Simulator auto-launch is not configured on this deployment "
                           "(no launch-targets config found), ignoring launch request")
      return
    target_key, robot_config = self.parseLaunchPayload(msg)
    with self.launcher_lock:
      if self.launcher_thread is not None and self.launcher_thread.is_alive():
        self.msg_if.pub_warn("A launch/stop/install is already in progress, ignoring")
        return
      self.launcher_thread = threading.Thread(target = self.runForceLaunch,
                                              args = (target_key,),
                                              kwargs = {'robot_config': robot_config})
      self.launcher_thread.daemon = True
      self.launcher_thread.start()

  def runForceLaunch(self, target_key, robot_config=None):
    # Clears whatever gazebo is in the way first (see
    # SimulatorLauncher.kill_all_gazebo's own docstring for why this, and
    # not stop(), is the right tool here -- the blocking gzserver isn't
    # tracked as any target's own launch), then launches target_key exactly
    # as a normal Deploy click would. A kill_all_gazebo failure (host
    # unreachable) is reported the same way a launch failure already is --
    # there is nothing target-specific to fall back to.
    try:
      self.launcher.kill_all_gazebo()
    except LauncherError as e:
      self.launcher_state = 'failed'
      self.launcher_last_error = "Could not clear the existing gazebo: " + str(e)
      self.publishLauncherStatus()
      return
    self.runLaunch(target_key, robot_config=robot_config)

  def killAllGazeboCb(self, msg):
    if self.launcher is None:
      self.msg_if.pub_warn("Simulator auto-launch is not configured on this deployment "
                           "(no launch-targets config found), ignoring kill-all request")
      return
    with self.launcher_lock:
      if self.launcher_thread is not None and self.launcher_thread.is_alive():
        self.msg_if.pub_warn("A launch/stop/install is already in progress, ignoring")
        return
      self.launcher_thread = threading.Thread(target = self.runKillAllGazebo)
      self.launcher_thread.daemon = True
      self.launcher_thread.start()

  def runKillAllGazebo(self):
    # If this app has something tracked as running, stop it properly FIRST
    # (its own stop_command cleans up SITL/the bridge/camera_rig, not just
    # gazebo) -- confirmed the hard way: kill_all_gazebo alone pulls
    # gzserver out from under a running SITL+bridge session without
    # clearing them, and since it also resets selected_launch_target below,
    # the ordinary Kill button has nothing left to stop afterward either
    # (stopSimulatorCb no-ops with no target tracked). kill_all_gazebo
    # itself still runs unconditionally after, for whatever stray instance
    # this app was never tracking in the first place.
    if self.launcher_state == 'running' and self.active_launch_target:
      self.runStop(self.active_launch_target)
      if self.launcher_state == 'failed':
        return
    try:
      self.launcher.kill_all_gazebo()
    except LauncherError as e:
      self.launcher_state = 'failed'
      self.setLauncherError(e)
      self.publishLauncherStatus()
      return
    # Not tied to any one target's launch/stop bookkeeping -- back to idle
    # unconditionally, since kill_all_gazebo just cleared everything.
    self.launcher_state = 'idle'
    self.setLauncherError('')
    self.selected_launch_target = ''
    self.active_launch_target = ''
    self.publishLauncherStatus()

  def runLaunch(self, target_key, attach=False, robot_config=None):
    # robot_config, when given, is the value the caller's own message
    # carried (see parseLaunchPayload) and takes priority over
    # self.selected_robot_config below -- reading self.selected_robot_config
    # for the launch-time decision raced select_robot_config's own
    # subscriber callback (independent thread, no cross-topic ordering
    # guarantee), so a value passed in here explicitly is what actually
    # fixes that race; self.selected_robot_config remains the fallback for
    # any caller that truly doesn't know (e.g. a bare legacy string message,
    # or "Use Open Sim" re-clicked with nothing new selected).
    explicit_robot_config = robot_config

    # attach=True skips the reuse-check below (nothing is tracked as
    # running yet -- the gzserver in the way isn't this app's own, so there
    # is no in-place config update to make), but target resolution still
    # matters just as much as the normal path: the operator still only
    # ever picks "Gazebo" (gazebo_rover), and which target's own
    # attach_launch_command actually needs to run still depends on the
    # currently selected robot config -- confirmed the hard way, an
    # earlier version of this skipped resolution entirely and attached
    # gazebo_rover's OWN rover bridge to an already-running
    # iris_arducopter_cmac.world, which can never satisfy that bridge's own
    # ready_check (no rover model in that world) regardless of how long it
    # waits.
    if attach:
      robot_config = explicit_robot_config if explicit_robot_config else self.selected_robot_config
      if not robot_config or robot_config == FACTORY_ROBOT_CONFIG_NAME:
        robot_config = self.launcher.get_default_robot_config(target_key)
      actual_target = (self.launcher.resolve_launch_target(target_key, robot_config)
                       if robot_config else target_key)

      self.selected_launch_target = target_key
      self.active_launch_target = actual_target
      self.launcher_state = 'launching'
      self.setLauncherError('')
      self.publishLauncherStatus()
      try:
        self.launcher.launch(actual_target, attach=True)
        ready = self.launcher.wait_until_ready(actual_target)
      except LauncherError as e:
        self.launcher_state = 'failed'
        self.setLauncherError(e)
        self.publishLauncherStatus()
        return
      if not ready:
        # Same orphan-prevention reasoning as the non-attach path below:
        # launch(attach=True) still starts this target's OWN bridge/camera
        # scripts against the pre-existing gzserver, and those are exactly
        # what's left running (and blocking every retry) if the ready-check
        # times out without a stop() call here.
        try:
          self.launcher.stop(actual_target)
        except LauncherError as stop_err:
          self.msg_if.pub_warn("Best-effort stop after failed ready-check also failed: " + str(stop_err))
        self.launcher_state = 'failed'
        self.launcher_last_error = ('Timed out waiting for the simulator to become ready -- '
                                    'the gazebo that was already running may not have had the '
                                    'right world loaded for this target')
        self.publishLauncherStatus()
        return
      self.launcher_state = 'running'
      self.publishLauncherStatus()
      # explicit_robot_config still wins if the launch message carried one;
      # otherwise recomputed fresh from self.selected_robot_config (not the
      # pre-launch snapshot above) in case a select_robot_config arrived
      # during the launch's several seconds of blocking I/O -- same
      # race-safety reasoning as the non-attach path below.
      robot_config = explicit_robot_config if explicit_robot_config else self.selected_robot_config
      if not robot_config or robot_config == FACTORY_ROBOT_CONFIG_NAME:
        robot_config = self.launcher.get_default_robot_config(actual_target)
      if robot_config:
        robot_config = self.launcher.resolve_robot_config(actual_target, robot_config)
        self.setSelectedRobotConfig(robot_config)
      return

    # Resolved BEFORE the reuse check below, using whatever robot config is
    # selected right now: a target whose own launch mechanics fundamentally
    # can't serve that robot config (a 4-motor flight profile against the
    # rover-only Gazebo world/bridge) needs a DIFFERENT target's
    # launch_command entirely, not just a different config applied on top
    # of the same one -- see resolve_launch_target's docstring. Most
    # target/config combinations resolve to target_key unchanged.
    pre_launch_robot_config = explicit_robot_config if explicit_robot_config else self.selected_robot_config
    if not pre_launch_robot_config or pre_launch_robot_config == FACTORY_ROBOT_CONFIG_NAME:
      pre_launch_robot_config = self.launcher.get_default_robot_config(target_key)
    actual_target = (self.launcher.resolve_launch_target(target_key, pre_launch_robot_config)
                     if pre_launch_robot_config else target_key)

    # Reuse path: the operator's own pick (target_key) still matches what's
    # tracked as selected, AND the real target this combination resolves to
    # still matches what's actually running (active_launch_target) -- e.g.
    # the operator picked a different robot model that resolves to the SAME
    # real target. Nothing on the VM needs touching; only the
    # already-connected bridge needs the new config, exactly as if
    # select_robot_config had been published directly, resolved through the
    # ACTIVE target's own robot_config_overrides first (see
    # resolve_robot_config's docstring). If the resolved target differs
    # instead (switching to a robot config that needs a different
    # world/bridge while "Gazebo" stays picked), this is NOT a reuse --
    # falls through to a real (re)launch below.
    if (self.launcher_state == 'running' and self.selected_launch_target == target_key
        and self.active_launch_target == actual_target):
      if pre_launch_robot_config:
        resolved_config = self.launcher.resolve_robot_config(actual_target, pre_launch_robot_config)
        self.setSelectedRobotConfig(resolved_config)
      return

    self.selected_launch_target = target_key
    self.active_launch_target = actual_target
    self.launcher_state = 'launching'
    self.setLauncherError('')
    self.launcher_failed_target = ''
    self.publishLauncherStatus()

    # Auto-install before attempting to launch, so a bare VM missing a
    # target's dependencies still comes up from a single Deploy click --
    # previously Install and Deploy were two fully separate actions (see
    # installSimulatorCb/runInstall below), and an operator on a fresh VM had
    # to notice the not_installed state and click Install first before Deploy
    # even appeared (Nepi_IF_SimLauncher.js's renderDeployControls). Skipped
    # on the attach path above (attach=True returns before reaching here) --
    # attaching only ever targets an ALREADY-running foreign gzserver, which
    # implies the dependency is already present. A failed installed-check
    # (e.g. a dead tunnel) falls through to the real launch attempt below
    # rather than blocking here on something this app can't confirm is
    # actually needed -- that attempt fails with its own clear error if the
    # dependency truly is missing.
    try:
      needs_install = not self.launcher.is_installed(actual_target)
    except LauncherError:
      needs_install = False
    if needs_install:
      self.launcher_state = 'installing'
      self.launch_target_installed_check_state[actual_target] = 'checking'
      self.publishLauncherStatus()
      try:
        self.launcher.install(actual_target)
      except LauncherError as e:
        self.launcher_state = 'failed'
        self.setLauncherError(e)
        self.launcher_failed_target = actual_target
        self.checkInstalledOne(actual_target)
        self.publishLauncherStatus()
        return
      self.checkInstalledOne(actual_target)
      self.launcher_state = 'launching'
      self.publishLauncherStatus()

    # Push any pending dimension edits before starting a FRESH gzserver --
    # unlike the attach path above, this one actually loads the world/models
    # from disk, so this is the one moment a pushed model.sdf can take
    # effect. Best-effort (pushDirtyDimensions never raises); a push failure
    # here should not block the launch it's ahead of.
    self.pushDirtyDimensions(actual_target)
    try:
      self.launcher.launch(actual_target)
      ready = self.launcher.wait_until_ready(actual_target)
    except LauncherError as e:
      # A real choice, not just a failure: launch_command's own refuse
      # guard means a gazebo is already up but not one this app started --
      # offer to reuse it (Use Existing / sim/attach_simulator) or force
      # past it (Launch New / sim/force_launch_simulator) rather than just
      # reporting a dead end. Every other LauncherError stays plain 'failed'.
      self.launcher_state = 'gazebo_conflict' if isGazeboConflictError(str(e)) else 'failed'
      self.setLauncherError(e)
      self.publishLauncherStatus()
      return
    if not ready:
      # Best-effort cleanup before reporting failed -- active_launch_target
      # was already set above (needed so a mid-launch status query shows
      # something), and it stayed set on every prior version of this
      # timeout path even though the VM-side session the launch_command
      # started (gzserver, SITL, bridge scripts) is still genuinely running
      # there. That left a live, untracked orphan blocking every recovery
      # path: a plain retry hits launch_command's own "already running"
      # guard, "Use Existing" hits the same ArduCopter-port guard, and
      # "Launch New" (kill_all_gazebo) only ever pkills gzserver/gzclient,
      # never SITL. Stopping here, before reporting failed, means a timeout
      # actually leaves nothing behind for the operator to manually clean up.
      try:
        self.launcher.stop(actual_target)
      except LauncherError as stop_err:
        self.msg_if.pub_warn("Best-effort stop after failed ready-check also failed: " + str(stop_err))
      self.launcher_state = 'failed'
      self.launcher_last_error = 'Timed out waiting for the simulator to become ready'
      self.publishLauncherStatus()
      return
    self.launcher_state = 'running'
    self.publishLauncherStatus()
    # Prefers whatever robot config the operator already picked from the
    # existing selector over the launch target's own canned default -- this
    # is what makes "choose a simulator, choose a model, Deploy" apply BOTH
    # in one action without a new message: the model choice was already
    # made through setSelectedRobotConfig (the existing, unmodified flow),
    # Deploy just needs to not immediately stomp on it with a one-size
    # default meant only for an operator who hasn't picked anything yet.
    # FACTORY_ROBOT_CONFIG_NAME ('default', capability-empty) counts as
    # "hasn't picked anything" rather than a real choice worth preserving.
    # explicit_robot_config still wins if the launch message carried one;
    # otherwise recomputed fresh from self.selected_robot_config (not
    # pre_launch_robot_config) since launch()/wait_until_ready() block for
    # real seconds, during which another callback could legitimately have
    # changed the selection.
    robot_config = explicit_robot_config if explicit_robot_config else self.selected_robot_config
    if not robot_config or robot_config == FACTORY_ROBOT_CONFIG_NAME:
      robot_config = self.launcher.get_default_robot_config(actual_target)
    # Resolves the plain, selector-offered choice (e.g. "2-Wheel Rover") to
    # whatever profile the ACTUAL target needs -- most targets need no
    # mapping (Gazebo's rover configs already match the generic keys), but
    # WPILib-style targets redirect to their own hidden_from_selector
    # profile here, so there is no separate "2-Wheel Rover (WPILib)" entry
    # to pick.
    if robot_config:
      robot_config = self.launcher.resolve_robot_config(actual_target, robot_config)
      self.setSelectedRobotConfig(robot_config)

  def runStop(self, target_key):
    self.launcher_state = 'stopping'
    self.publishLauncherStatus()
    try:
      self.launcher.stop(target_key)
    except LauncherError as e:
      self.launcher_state = 'failed'
      self.launcher_last_error = str(e)
      self.publishLauncherStatus()
      return
    self.launcher_state = 'idle'
    self.launcher_last_error = ''
    self.selected_launch_target = ''
    self.active_launch_target = ''
    self.publishLauncherStatus()

  def installSimulatorCb(self, msg):
    if self.launcher is None:
      self.msg_if.pub_warn("Simulator auto-launch is not configured on this deployment "
                           "(no launch-targets config found), ignoring install request")
      return
    target_key = str(msg.data).strip()
    with self.launcher_lock:
      if self.launcher_thread is not None and self.launcher_thread.is_alive():
        self.msg_if.pub_warn("A launch/stop/install is already in progress, ignoring")
        return
      self.launcher_thread = threading.Thread(target = self.runInstall, args = (target_key,))
      self.launcher_thread.daemon = True
      self.launcher_thread.start()

  def runInstall(self, target_key):
    self.launcher_state = 'installing'
    self.setLauncherError('')
    self.launcher_failed_target = ''
    self.launch_target_installed_check_state[target_key] = 'checking'
    self.publishLauncherStatus()
    try:
      self.launcher.install(target_key)
    except LauncherError as e:
      self.launcher_state = 'failed'
      self.setLauncherError(e)
      self.launcher_failed_target = target_key
      # Not marked 'not_installed' here -- the install command failing
      # doesn't necessarily mean the dependency is confirmed absent (could
      # be a transient network/package-mirror failure), so re-check for real
      # rather than assume either outcome.
      self.checkInstalledOne(target_key)
      self.publishLauncherStatus()
      return
    self.launcher_state = 'idle'
    self.checkInstalledOne(target_key)
    self.publishLauncherStatus()

  def checkInstalledOne(self, target_key):
    # Shared by the install-all background sweep and by runInstall's
    # post-install re-check -- always leaves a definite state (never
    # 'checking') so a caller doesn't have to remember to do that itself.
    try:
      installed = self.launcher.is_installed(target_key)
      self.launch_target_installed[target_key] = installed
      self.launch_target_installed_check_state[target_key] = 'installed' if installed else 'not_installed'
    except LauncherError as e:
      self.msg_if.pub_warn("Could not check install state for '" + target_key + "': " + str(e),
                           throttle_s = 30.0)
      self.launch_target_installed_check_state[target_key] = 'unknown'

  def startInstalledCheckAll(self):
    # Runs once at startup and once per config reload (see
    # refreshLauncherConfigCb) -- NOT on the launcher_thread/launcher_lock
    # launch/stop/install share, since checking is a read-only, low-stakes
    # operation against every target and shouldn't be blocked by (or block)
    # an in-flight launch/stop/install of one specific target.
    thread = threading.Thread(target = self.checkInstalledAllCb)
    thread.daemon = True
    thread.start()

  def checkInstalledAllCb(self):
    if self.launcher is None:
      return
    # ALL targets, hidden_from_selector or not -- a hidden target (e.g.
    # gazebo_quadcopter, reached only through gazebo_rover's own
    # launch_target_overrides) still needs its own dependency state tracked
    # in the background, exactly like a hidden robot_configs entry stays
    # fully valid/checked without being offered directly.
    keys = self.launcher.list_all_target_keys()
    for key in keys:
      self.launch_target_installed_check_state[key] = 'checking'
    self.publishLauncherStatus()
    for key in keys:
      self.checkInstalledOne(key)
      self.publishLauncherStatus()

  def publishLauncherStatus(self):
    status = SimLauncherStatus()
    if self.launcher is not None:
      keys, names = self.launcher.get_available_targets()
      status.available_launch_targets = keys
      status.available_launch_target_names = names
      status.available_launch_target_installed = [
          bool(self.launch_target_installed.get(k, False)) for k in keys]
      status.available_launch_target_installed_check_state = [
          self.launch_target_installed_check_state.get(k, 'unknown') for k in keys]
    status.selected_launch_target = self.selected_launch_target
    status.active_launch_target = self.active_launch_target
    # active_launch_target can be a hidden_from_selector target (e.g.
    # gazebo_quadcopter), which get_available_targets() above deliberately
    # excludes -- look its display name up directly rather than searching
    # the (possibly not-containing-it) keys/names lists just built.
    if self.active_launch_target and self.launcher is not None:
      try:
        status.active_launch_target_name = (
            self.launcher.get_target(self.active_launch_target).get(
                'display_name', self.active_launch_target))
      except LauncherError:
        status.active_launch_target_name = self.active_launch_target
    status.launcher_state = self.launcher_state
    status.last_error = self.launcher_last_error
    # launcher_tunnel_fallback_commands (set by setLauncherError whenever the
    # LauncherError behind the current failure carries its own
    # manual_fallback_commands -- currently just the reverse-tunnel
    # connectivity diagnosis, see simulator_launcher's
    # _classify_connection_failure) takes priority over the per-target
    # install fallback below: a dead tunnel isn't fixed by that target's own
    # install_command, and showing both would bury the one that's actually
    # relevant under a wall of unrelated apt/pip commands.
    #
    # The install fallback itself is derived, not stored -- computed fresh
    # from state this app already tracks (launcher_failed_target,
    # launch_target_installed) rather than threaded through every individual
    # failure site that sets launcher_last_error. Keyed off
    # launcher_failed_target, NOT active_launch_target -- runInstall can
    # fail for a target that was never the one actively running/attempted.
    # Only shown once a failure is confirmed dependency-related (the target
    # is known NOT installed), so a timeout/conflict/network failure with a
    # perfectly good install doesn't get an irrelevant wall of install
    # commands attached to it.
    if self.launcher_state == 'failed' and self.launcher_tunnel_fallback_commands:
      status.manual_fallback_commands = self.launcher_tunnel_fallback_commands
    elif (self.launcher_state == 'failed' and self.launcher_failed_target
        and self.launch_target_installed.get(self.launcher_failed_target, True) is False):
      status.manual_fallback_commands = self.launcher.get_manual_fallback_commands(
          self.launcher_failed_target)
    else:
      status.manual_fallback_commands = ''
    self.launcher_status_pub.publish(status)

  #**********************
  # Connection health

  def isBridgeConnected(self):
    with self.client_lock:
      return self.client_conn is not None

  def getTelemetryAge(self):
    if self.connected_since is None:
      return TELEMETRY_AGE_IF_NEVER_CONNECTED
    return nepi_utils.get_time() - self.last_telemetry_time

  def autonomousControlsReady(self):
    # Goto commands need a live bridge AND fresh telemetry, since a setpoint is
    # computed against the current pose. A direct motor command does not, which
    # is why manual control gates on connection alone.
    if not self.isBridgeConnected():
      return False
    age = self.getTelemetryAge()
    return age >= 0.0 and age < SIM_DEVICE_STALE_SEC

  #**********************
  # SimDeviceIF callbacks -- thin wire-protocol senders and getters, gated at
  # construction (and on a config change) by the selected robot config

  def setMotorControlRatio(self, motor_ind, speed_ratio):
    if motor_ind < 0 or motor_ind >= len(self.motor_ratios):
      self.msg_if.pub_warn("Motor control ignored: motor index " + str(motor_ind) + " out of range")
      return
    self.motor_ratios[motor_ind] = max(0.0, min(1.0, speed_ratio))
    self.sendLineToBridge({'type': 'motor_control', 'motor_ind': motor_ind,
                          'speed_ratio': self.motor_ratios[motor_ind]}, "Motor control")

  def getMotorControlRatios(self):
    return self.motor_ratios

  def gotoPosition(self, msg):
    with self.goto_target_lock:
      self.goto_target = {'x_meters': msg.x_meters, 'y_meters': msg.y_meters,
                          'z_meters': msg.z_meters, 'yaw_deg': msg.yaw_deg}
    self.sendLineToBridge({'type': 'goto_position', 'x_meters': msg.x_meters,
                           'y_meters': msg.y_meters, 'z_meters': msg.z_meters,
                           'yaw_deg': msg.yaw_deg}, "Goto position")

  def gotoPose(self, attitude_enu_degs):
    self.sendLineToBridge({'type': 'goto_pose', 'roll_deg': attitude_enu_degs[0],
                           'pitch_deg': attitude_enu_degs[1],
                           'yaw_deg': attitude_enu_degs[2]}, "Goto pose")

  def gotoLocation(self, msg):
    self.sendLineToBridge({'type': 'goto_location', 'lat': msg.lat, 'long': msg.long,
                           'altitude_meters': msg.altitude_meters,
                           'yaw_deg': msg.yaw_deg}, "Goto location")

  def getHome(self):
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

  def goHome(self):
    self.sendLineToBridge({'type': 'go_home'}, "Go home")
    return self.isBridgeConnected()

  def goStop(self):
    with self.goto_target_lock:
      self.goto_target = None
    self.sendLineToBridge({'type': 'go_stop'}, "Go stop")
    return self.isBridgeConnected()

  def setSetupActionInd(self, action_ind):
    actions = self.profile['setup_actions']
    if action_ind < 0 or action_ind >= len(actions):
      return False
    self.sendLineToBridge({'type': 'setup_action', 'action': actions[action_ind]},
                          "Setup action")
    return self.isBridgeConnected()

  def setGoActionInd(self, action_ind):
    actions = self.profile['go_actions']
    if action_ind < 0 or action_ind >= len(actions):
      return False
    self.sendLineToBridge({'type': 'go_action', 'action': actions[action_ind]}, "Go action")
    return self.isBridgeConnected()

  def setCameraViewMode(self, view_mode):
    self.sendLineToBridge({'type': 'camera_settings', 'view_mode': view_mode},
                          "Camera settings")

  def setEnvironmentOption(self, option, enabled = True):
    self.sendLineToBridge({'type': 'environment_option', 'option': option,
                           'enabled': bool(enabled)}, "Environment option")

  def setActiveImageTopic(self, topic_name):
    self.active_image_topic = topic_name
    self.sendLineToBridge({'type': 'set_active_image_topic', 'topic_name': topic_name},
                          "Set active image topic")

  def getAvailableSensorTopics(self):
    return list(self.available_sensor_topics)

  def getAvailableEnvironmentOptions(self):
    return list(self.available_environment_options)

  def getNavPoseCb(self):
    return self.navpose_dict

  #**********************
  # Bridge server. Single active client at a time.

  def bridgeServerLoop(self):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # rospy installs a process-global socket.setdefaulttimeout(60) during
    # init_node, which accept() would otherwise apply to every accepted
    # connection. A command stream is legitimately idle for long stretches, so a
    # recv timeout must not be read as client death: clear the timeout and block
    # instead. A real disconnect still unblocks recv with EOF. This is the
    # socket-handling care documented in src/nepi_drivers/CLAUDE.md, and it must
    # happen after init_node, which it does -- this thread starts at the end of
    # __init__.
    srv.settimeout(None)
    try:
      srv.bind(('0.0.0.0', self.listen_port))
      srv.listen(BRIDGE_ACCEPT_BACKLOG)
    except Exception as e:
      self.msg_if.pub_warn("Could not listen on port " + str(self.listen_port) + ": " + str(e) +
                           ". No simulator bridge can connect until this is resolved.")
      return

    while not nepi_sdk.is_shutdown():
      try:
        conn, addr = srv.accept()
        conn.settimeout(None)
      except Exception:
        continue
      self.msg_if.pub_info("Bridge client connected from " + str(addr))
      with self.client_lock:
        self.client_conn = conn
        self.connected_since = nepi_utils.get_time()
        self.last_telemetry_time = self.connected_since
      self.serveClient(conn)
      with self.client_lock:
        if self.client_conn is conn:
          self.client_conn = None
      try:
        conn.close()
      except Exception:
        pass
      self.msg_if.pub_info("Bridge client disconnected")

  def serveClient(self, conn):
    buf = b''
    while not nepi_sdk.is_shutdown():
      try:
        data = conn.recv(BRIDGE_RECV_BYTES)
      except Exception as e:
        self.msg_if.pub_warn("Bridge client recv error: " + repr(e))
        return
      if not data:
        self.msg_if.pub_info("Bridge client closed connection (EOF)")
        return
      buf += data
      while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        if line.strip():
          self.processBridgeLine(line)

  def processBridgeLine(self, line):
    try:
      msg = json.loads(line)
    except Exception as e:
      self.msg_if.pub_warn("Bad line from bridge: " + str(e), throttle_s = 5.0)
      return
    if not isinstance(msg, dict):
      self.msg_if.pub_warn("Ignoring non-object line from bridge", throttle_s = 5.0)
      return
    msg_type = msg.get('type')
    if msg_type == 'sensor_topics':
      self.processSensorTopicsLine(msg)
    elif msg_type == 'environment_options':
      self.processEnvironmentOptionsLine(msg)
    elif msg_type == 'image':
      self.processImageLine(msg)
    elif msg_type == 'goto_result':
      # Async ack a bridge sends once its own goto controller reports the
      # target reached (or failed) -- see device_if_sim.py's report_goto_result
      # for why this is the only place cmd_success gets set for goto commands.
      self.sim_if.report_goto_result(bool(msg.get('success', False)))
    else:
      # A line with no "type" key is telemetry -- the dispatch-by-key-presence
      # convention the existing bridge scripts already use. An unrecognized type
      # lands here too and is harmless: nothing in a NavPose parse is required.
      self.processTelemetryLine(msg)

  def processSensorTopicsLine(self, msg):
    topics = msg.get('topics', [])
    parsed = []
    for entry in topics:
      if not isinstance(entry, dict):
        continue
      topic_name = entry.get('topic_name', '')
      msg_type = entry.get('msg_type', '')
      if topic_name and msg_type:
        parsed.append((topic_name, msg_type))
    self.available_sensor_topics = parsed

  def processEnvironmentOptionsLine(self, msg):
    options = msg.get('options', [])
    if not isinstance(options, list):
      return
    self.available_environment_options = [str(o) for o in options]

  def processImageLine(self, msg):
    topic_name = msg.get('topic_name', self.active_image_topic)
    if self.active_image_topic and topic_name != self.active_image_topic:
      return  # Not the selected camera -- nothing subscribes to it
    try:
      jpeg_bytes = base64.b64decode(msg['data'])
      arr = np.frombuffer(jpeg_bytes, dtype = np.uint8)
      cv2_img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
      if cv2_img is None:
        raise ValueError("cv2.imdecode returned None")
      ros_img = nepi_img.cv2img_to_rosimg(cv2_img, encoding = "bgr8")
      self.image_pub.publish(ros_img)
    except Exception as e:
      self.msg_if.pub_warn("Failed to process camera image frame: " + str(e), throttle_s = 5.0)

  def processTelemetryLine(self, telem):
    # Generalized past any one vehicle's shape to the full NavPose contract:
    # every field optional, each block gated by presence exactly the way
    # NavPose.msg's own has_* flags are.
    now = nepi_utils.get_time()
    nd = self.navpose_dict

    if any(k in telem for k in ('x_m', 'y_m', 'z_m')):
      nd['has_position'] = True
      nd['time_position'] = now
      nd['x_m'] = float(telem.get('x_m', nd['x_m']))
      nd['y_m'] = float(telem.get('y_m', nd['y_m']))
      nd['z_m'] = float(telem.get('z_m', nd['z_m']))
      nd['x_m_per_sec'] = float(telem.get('x_m_per_sec', nd['x_m_per_sec']))
      nd['y_m_per_sec'] = float(telem.get('y_m_per_sec', nd['y_m_per_sec']))
      nd['z_m_per_sec'] = float(telem.get('z_m_per_sec', nd['z_m_per_sec']))

    if any(k in telem for k in ('roll_deg', 'pitch_deg', 'yaw_deg')):
      nd['has_orientation'] = True
      nd['time_orientation'] = now
      nd['roll_deg'] = float(telem.get('roll_deg', nd['roll_deg']))
      nd['pitch_deg'] = float(telem.get('pitch_deg', nd['pitch_deg']))
      nd['yaw_deg'] = float(telem.get('yaw_deg', nd['yaw_deg']))
      nd['roll_deg_per_sec'] = float(telem.get('roll_deg_per_sec', nd['roll_deg_per_sec']))
      nd['pitch_deg_per_sec'] = float(telem.get('pitch_deg_per_sec', nd['pitch_deg_per_sec']))
      nd['yaw_deg_per_sec'] = float(telem.get('yaw_deg_per_sec', nd['yaw_deg_per_sec']))

    if any(k in telem for k in ('latitude', 'longitude')):
      nd['has_location'] = True
      nd['time_location'] = now
      nd['latitude'] = float(telem.get('latitude', nd['latitude']))
      nd['longitude'] = float(telem.get('longitude', nd['longitude']))

    if 'heading_deg' in telem:
      nd['has_heading'] = True
      nd['time_heading'] = now
      nd['heading_deg'] = float(telem.get('heading_deg', nd['heading_deg']))

    if 'altitude_m' in telem:
      nd['has_altitude'] = True
      nd['time_altitude'] = now
      nd['altitude_m'] = float(telem.get('altitude_m', nd['altitude_m']))

    if 'depth_m' in telem:
      nd['has_depth'] = True
      nd['time_depth'] = now
      nd['depth_m'] = float(telem.get('depth_m', nd['depth_m']))

    self.last_telemetry_time = now

  def sendLineToBridge(self, line_dict, description):
    with self.client_lock:
      conn = self.client_conn
      if conn is None:
        self.msg_if.pub_warn(description + " dropped, no bridge client connected",
                             throttle_s = 5.0)
        return
      # The send happens under the same lock as the client_conn read: this is a
      # single TCP stream, and two unsynchronized sendall calls can interleave
      # their bytes and corrupt the newline-delimited JSON the far side parses.
      try:
        conn.sendall((json.dumps(line_dict) + '\n').encode())
      except Exception as e:
        self.msg_if.pub_warn("Failed to send " + description.lower() + " to bridge: " + str(e))
        if self.client_conn is conn:
          self.client_conn = None
        # shutdown() before close(): bridgeServerLoop's thread is almost
        # certainly blocked in serveClient's timeout=None recv() on this
        # exact socket. Closing a fd out from under a thread blocked in
        # recv() on it does not reliably unblock that recv() on Linux --
        # without the shutdown(), that thread can stay wedged inside
        # serveClient forever, never returning to accept() the bridge's next
        # reconnect attempt. Same fix as sim_bridge_node.py/camera_rig_
        # controller_ardupilot.py's sendLineToClient (found while chasing a
        # simulator camera flicker-in-and-out bug with the identical
        # send-thread/recv-thread split).
        try:
          conn.shutdown(socket.SHUT_RDWR)
        except Exception:
          pass
        try:
          conn.close()
        except Exception:
          pass

  #**********************
  # Status

  def statusPublishCb(self, timer):
    # SimDeviceIF owns the 2 Hz device status. This slower tick exists so the
    # two selector reports refresh promptly after a discovery scan even if the
    # device status cadence is ever retuned.
    if self.sim_if is not None:
      self.sim_if.publish_status()
    self.refreshLauncherConfigCb()

  def refreshLauncherConfigCb(self):
    # Picks up launch-target config changes (an edited file, or a file that
    # appears after this node started) without an app restart, so adding a
    # simulator target is just an edit. Both paths are a cheap stat/isfile
    # check per tick that only does real work on an actual change.
    #
    # Skipped entirely while a launch or stop is in flight -- swapping the
    # config out from under a running launch would leave its stop_command and
    # ready_check_command referring to a target definition that no longer
    # matches what was actually started.
    if self.launcher_thread is not None and self.launcher_thread.is_alive():
      return
    try:
      if self.launcher is None:
        config_path = find_config_path()
        if not config_path:
          return
        self.launcher = SimulatorLauncher(config_path)
        self.msg_if.pub_info("Simulator auto-launch enabled from " + config_path)
        self.publishLauncherStatus()
        self.startInstalledCheckAll()
      elif self.launcher.reload_if_changed():
        self.msg_if.pub_info("Reloaded launch targets from " + self.launcher.config_path)
        # A target that vanished from the config must not stay selected.
        if self.selected_launch_target:
          available, _names = self.launcher.get_available_targets()
          if self.selected_launch_target not in available:
            self.selected_launch_target = ''
        self.publishLauncherStatus()
        # Targets may have been added/edited -- re-check all of them rather
        # than trying to diff what changed.
        self.startInstalledCheckAll()
    except LauncherError as e:
      # throttle_s alone is correct here: pub_warn derives the throttle uid
      # itself from the caller's file/function/line, and takes no uid argument
      # (passing one is a TypeError) -- unlike the lower-level pub_msg, whose
      # throttling is a no-op without an explicit uid.
      self.msg_if.pub_warn("Could not load launch targets: " + str(e), throttle_s = 30.0)

  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    """Closes the bridge connection on node shutdown.

    Closing the accepted connection here unblocks the server thread's recv, so a
    connected simulator bridge sees a clean disconnect rather than a half-open
    socket it has to time out on.
    """
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    with self.client_lock:
      conn = self.client_conn
      self.client_conn = None
    if conn is not None:
      try:
        conn.close()
      except Exception:
        pass


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiSimConnectorApp()
