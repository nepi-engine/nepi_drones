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

from std_msgs.msg import Bool, Empty, String
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

FACTORY_ROBOT_CONFIG = dict(
    description = 'No capabilities. Safe default until a robot config is selected.',
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

# How long to wait between re-running the per-target dependency sweep while
# any target is still 'unknown' (i.e. ssh could not reach the sim host) --
# see refreshLauncherConfigCb. Deliberately slow: each sweep costs one ssh
# per configured target, and the condition it recovers from (the VM or the
# reverse tunnel not up yet when this app started) resolves on human
# timescales, not sub-second ones. 60 s means a VM that comes up is usable
# without touching the app, while an offline VM is probed once a minute
# rather than once a second.
UNREACHABLE_RECHECK_SEC = 60.0

# How often to ask the sim host "is a gzserver already running?" while this
# app has nothing of its own tracked as running -- see
# detectExternalSimCb. One ssh per host per poll, and what it detects
# (someone else's simulator) changes on human timescales, so this is
# deliberately slow.
EXTERNAL_SIM_DETECT_SEC = 20.0

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
    # Timestamp of the last dependency/reachability sweep, for the
    # self-healing retry in refreshLauncherConfigCb. 0 = never swept.
    self.last_installed_check_time = 0
    # Timestamp of the last external-simulator probe (see detectExternalSimCb).
    self.last_external_detect_time = 0
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
    self.setSelectedRobotConfig(UPLOADED_ROBOT_CONFIG_NAME)

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
      # The staleness half is not redundant with the vanished half -- without
      # it the two form a self-sustaining loop that never releases a dead
      # robot. A ROS topic exists in the master for as long as it has EITHER a
      # publisher or a subscriber, so when a robot's node dies its status topic
      # stays listed purely because THIS app is still subscribed to it. The
      # scan above then keeps "finding" it, so `topic in found` stays true, so
      # this loop never unregisters, so the topic never goes away: the app's
      # own subscription is the only thing keeping it alive.
      #
      # That leaked out into the RUI, which builds its Devices -> Robots list
      # from topic NAMES: a killed simulator's robot stayed listed forever,
      # unselectable and uncontrollable, until the whole app restarted.
      # Confirmed live 2026-08-12 -- after killing the sim, both the rbx node
      # AND its discovery were gone, yet .../sim_rover1/rbx/status still
      # listed with "Publishers: None" and this app as the sole subscriber
      # (reported as "even after killing the quadcopter and gazebo,
      # ardupilot_sitl still shows up in the devices -> robot section").
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
        if stale:
          self.msg_if.pub_info("Dropping stale simulator device " + topic +
                               " (no status for " + str(SIM_DEVICE_STALE_SEC) + "s)",
                               throttle_s = 60.0)

    # A selection that is no longer available falls back to no selection rather
    # than silently pointing at a device that is gone.
    if self.selected_simulator != "":
      available, _names = self.getAvailableSimulators()
      if self.selected_simulator not in available:
        self.msg_if.pub_warn("Selected simulator " + self.selected_simulator +
                             " is no longer available, clearing selection")
        self.selected_simulator = ""

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

  def launchSimulatorCb(self, msg):
    if self.launcher is None:
      self.msg_if.pub_warn("Simulator auto-launch is not configured on this deployment "
                           "(no launch-targets config found), ignoring launch request")
      return
    target_key = str(msg.data).strip()
    with self.launcher_lock:
      if self.launcher_thread is not None and self.launcher_thread.is_alive():
        self.msg_if.pub_warn("A launch/stop is already in progress, ignoring")
        return
      self.launcher_thread = threading.Thread(target = self.runLaunch, args = (target_key,))
      self.launcher_thread.daemon = True
      self.launcher_thread.start()

  def stopSimulatorCb(self, msg):
    if self.launcher is None or not self.selected_launch_target:
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
    target_key = str(msg.data).strip()
    with self.launcher_lock:
      if self.launcher_thread is not None and self.launcher_thread.is_alive():
        self.msg_if.pub_warn("A launch/stop/install is already in progress, ignoring")
        return
      self.launcher_thread = threading.Thread(target = self.runRedeploy, args = (target_key,))
      self.launcher_thread.daemon = True
      self.launcher_thread.start()

  def runRedeploy(self, target_key):
    # Stops whatever is currently tracked as running (if anything) then
    # launches target_key fresh. Reuses runStop/runLaunch directly rather
    # than duplicating their logic -- this method IS just those two run
    # back-to-back on the one background thread redeploySimulatorCb already
    # started.
    if self.launcher_state == 'running' and self.selected_launch_target:
      self.runStop(self.active_launch_target or self.selected_launch_target)
      if self.launcher_state == 'failed':
        return  # runStop already published the failure; nothing more to do
    self.runLaunch(target_key)

  def attachSimulatorCb(self, msg):
    # "Use Existing" -- see runLaunch's attach handling.
    if self.launcher is None:
      self.msg_if.pub_warn("Simulator auto-launch is not configured on this deployment "
                           "(no launch-targets config found), ignoring attach request")
      return
    target_key = str(msg.data).strip()
    with self.launcher_lock:
      if self.launcher_thread is not None and self.launcher_thread.is_alive():
        self.msg_if.pub_warn("A launch/stop/install is already in progress, ignoring")
        return
      self.launcher_thread = threading.Thread(target = self.runLaunch, args = (target_key,),
                                              kwargs = {'attach': True})
      self.launcher_thread.daemon = True
      self.launcher_thread.start()

  def forceLaunchSimulatorCb(self, msg):
    # "Launch New" -- see runForceLaunch.
    if self.launcher is None:
      self.msg_if.pub_warn("Simulator auto-launch is not configured on this deployment "
                           "(no launch-targets config found), ignoring launch request")
      return
    target_key = str(msg.data).strip()
    with self.launcher_lock:
      if self.launcher_thread is not None and self.launcher_thread.is_alive():
        self.msg_if.pub_warn("A launch/stop/install is already in progress, ignoring")
        return
      self.launcher_thread = threading.Thread(target = self.runForceLaunch, args = (target_key,))
      self.launcher_thread.daemon = True
      self.launcher_thread.start()

  def runForceLaunch(self, target_key):
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
    self.runLaunch(target_key)

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
      self.launcher_last_error = str(e)
      self.publishLauncherStatus()
      return
    # Not tied to any one target's launch/stop bookkeeping -- back to idle
    # unconditionally, since kill_all_gazebo just cleared everything.
    self.launcher_state = 'idle'
    self.launcher_last_error = ''
    self.selected_launch_target = ''
    self.active_launch_target = ''
    self.publishLauncherStatus()

  def runLaunch(self, target_key, attach=False):
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
      robot_config = self.selected_robot_config
      if not robot_config or robot_config == FACTORY_ROBOT_CONFIG_NAME:
        robot_config = self.launcher.get_default_robot_config(target_key)
      actual_target = (self.launcher.resolve_launch_target(target_key, robot_config)
                       if robot_config else target_key)

      self.selected_launch_target = target_key
      self.active_launch_target = actual_target
      self.launcher_state = 'launching'
      self.launcher_last_error = ''
      self.publishLauncherStatus()
      try:
        self.launcher.launch(actual_target, attach=True)
        ready = self.launcher.wait_until_ready(actual_target)
      except LauncherError as e:
        self.launcher_state = 'failed'
        self.launcher_last_error = str(e)
        self.publishLauncherStatus()
        return
      if not ready:
        self.launcher_state = 'failed'
        self.launcher_last_error = ('Timed out waiting for the simulator to become ready -- '
                                    'the gazebo that was already running may not have had the '
                                    'right world loaded for this target')
        self.publishLauncherStatus()
        return
      self.launcher_state = 'running'
      self.publishLauncherStatus()
      # Recomputed fresh (not reusing the pre-launch snapshot above), same
      # race-safety reasoning as the non-attach path below.
      robot_config = self.selected_robot_config
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
    pre_launch_robot_config = self.selected_robot_config
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
    self.launcher_last_error = ''
    self.publishLauncherStatus()
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
      self.launcher_last_error = str(e)
      self.publishLauncherStatus()
      return
    if not ready:
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
    # Recomputed fresh here (not reusing pre_launch_robot_config) since
    # launch()/wait_until_ready() block for real seconds, during which
    # another callback could legitimately have changed the selection.
    robot_config = self.selected_robot_config
    if not robot_config or robot_config == FACTORY_ROBOT_CONFIG_NAME:
      robot_config = self.launcher.get_default_robot_config(actual_target)
    # Resolves the plain, selector-offered choice (e.g. "2-Wheel Rover") to
    # whatever profile the ACTUAL target needs -- most targets need no
    # mapping (Gazebo's rover configs already match the generic keys), but
    # Stage/WPILib-style targets redirect to their own hidden_from_selector
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
    self.launcher_last_error = ''
    self.launch_target_installed_check_state[target_key] = 'checking'
    self.publishLauncherStatus()
    try:
      self.launcher.install(target_key)
    except LauncherError as e:
      self.launcher_state = 'failed'
      self.launcher_last_error = str(e)
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
    # Runs at startup, on config reload, and from the unreachable-retry in
    # refreshLauncherConfigCb -- NOT on the launcher_thread/launcher_lock
    # launch/stop/install share, since checking is a read-only, low-stakes
    # operation against every target and shouldn't be blocked by (or block)
    # an in-flight launch/stop/install of one specific target.
    self.last_installed_check_time = nepi_utils.get_time()
    thread = threading.Thread(target = self.checkInstalledAllCb)
    thread.daemon = True
    thread.start()

  def anyTargetUnreachable(self):
    # True when at least one target's dependency state is 'unknown', which
    # is_installed()/checkInstalledOne() use specifically to mean "ssh never
    # reached the host" (as opposed to 'not_installed', a confirmed answer) --
    # see SimulatorLauncher.is_installed's own docstring. An empty dict counts
    # too: it means no sweep has ever produced a result.
    if not self.launch_target_installed_check_state:
      return True
    return any(state == 'unknown'
               for state in self.launch_target_installed_check_state.values())

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
    status.launcher_state = self.launcher_state
    status.last_error = self.launcher_last_error
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

  def setEnvironmentOption(self, option):
    # The contract's environment control is a single option string today, not an
    # (option, enabled) pair. Forwarded as an enable until that control's own
    # on/off semantics are designed; the only real precedent (an obstacle-course
    # toggle) is a one-way "turn this on".
    self.sendLineToBridge({'type': 'environment_option', 'option': option,
                           'enabled': True}, "Environment option")

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
    self.detectExternalSimCb()

  def detectExternalSimCb(self):
    """Notices a simulator running on the sim host that this app is not
    tracking, and reports it as a gazebo_conflict so the RUI offers the
    Launch New / Use Existing / Kill All Gazebo choice instead of a bare
    Deploy button that could only fail.

    The gap this closes: launch bookkeeping (launcher_state,
    active_launch_target) lives only in this node's memory, so it resets
    whenever the app node restarts -- an app restart, or a container restart
    from nepicommit -- while the VM-side simulator keeps running untouched.
    The app then reported 'idle' against a fully-running sim, and the only
    way to find out was to click Deploy and watch it refuse. Reported live
    2026-08-12: a Gazebo/SITL was up on the VM and the RUI showed no sign of
    it and no way to kill it.

    Reuses the existing 'gazebo_conflict' state deliberately rather than
    adding a new one: that state already means "something is running that we
    do not own, here are your options", and the RUI already renders exactly
    the three buttons for it (including the Kill All Gazebo escape hatch),
    so no SimLauncherStatus schema change or RUI rebuild is needed.

    Only runs while genuinely idle -- never during launching/stopping/
    installing (the launcher_lock holder is mid-operation and its own state
    is authoritative), never while this app already tracks something running
    (then the sim IS ours and Kill already works), and never once already in
    gazebo_conflict (nothing to re-decide). That also keeps it from fighting
    runStop/runLaunch over launcher_state."""
    if self.launcher is None:
      return
    if self.launcher_state != 'idle' or self.active_launch_target:
      return
    if self.launcher_thread is not None and self.launcher_thread.is_alive():
      return
    if (nepi_utils.get_time() - self.last_external_detect_time) < EXTERNAL_SIM_DETECT_SEC:
      return
    self.last_external_detect_time = nepi_utils.get_time()
    # Probed on a worker thread: this runs from the 1 Hz status tick, and an
    # ssh to an unreachable host blocks for the connect timeout, which would
    # stall status publication for every other consumer.
    thread = threading.Thread(target = self.runExternalSimDetect)
    thread.daemon = True
    thread.start()

  def runExternalSimDetect(self):
    try:
      detected = self.launcher.detect_running_gazebo()
    except Exception as e:
      self.msg_if.pub_warn("External simulator detection failed: " + str(e), throttle_s = 300.0)
      return
    if not detected:
      return
    # Re-check the guards: this ran on a worker thread, so a launch/stop may
    # have started (and taken ownership of launcher_state) while the ssh was
    # in flight.
    if self.launcher_state != 'idle' or self.active_launch_target:
      return
    if self.launcher_thread is not None and self.launcher_thread.is_alive():
      return
    self.msg_if.pub_info("Detected a simulator already running on the sim host "
                         "that this app did not launch", throttle_s = 300.0)
    self.launcher_state = 'gazebo_conflict'
    self.launcher_last_error = ("A Gazebo simulator is already running on the sim host, and this app "
                                "is not tracking it (it was started outside this app, or before this "
                                "app last restarted). Use Kill All Gazebo to clear it, Launch New to "
                                "replace it, or Use Existing to connect to it as-is.")
    self.publishLauncherStatus()

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
      elif self.anyTargetUnreachable():
        # Self-heal the "app came up before the VM/tunnel did" case. The
        # startup and config-reload sweeps above were the ONLY things that
        # ever refreshed reachability, so a target that was unreachable at
        # app start stayed 'unknown' indefinitely -- the app sat there
        # believing the VM was unreachable even after it came back, and the
        # operator had to restart the app (or touch the config) before a
        # Deploy could work. Since 'unknown' specifically means "couldn't
        # reach the host", retrying it on a slow cadence is exactly the
        # recovery that was missing. Rate-limited because each sweep is one
        # ssh per target: at 1 Hz (this callback's own rate, via
        # statusPublishCb) an unthrottled retry would hammer the VM.
        # Confirmed states settle to 'installed'/'not_installed' once the
        # host answers, so this stops retrying on its own.
        if (nepi_utils.get_time() - self.last_installed_check_time) >= UNREACHABLE_RECHECK_SEC:
          self.msg_if.pub_info("Re-checking simulator host reachability", throttle_s = 300.0)
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
