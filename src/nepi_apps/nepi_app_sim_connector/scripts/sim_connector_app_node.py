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

# The Sim Connector app node. It hosts a SimDeviceIF instance (the Sim
# Connector panel's controls and status), drives the NEPI simulator services
# on the sim machine -- Gazebo and MuJoCo -- through their config files in
# NEPI storage (Deploy/Kill, robot and environment config -- see
# nepi_sim_service.py), starts and stops the robot's RBX driver to follow what
# those services report (syncRoverDriver), and
# forwards the panel's commands to the selected simulator's RBX device
# (forwardToRbx).
#
# Capability timing. The contract decides capabilities once at construction and
# caches them, which is what lets a client render controls purely from the
# flags. Capabilities are a per-deployment config decision, read from the
# selected robot config. Switching robot config is the one case that
# legitimately changes a robot's kind, and it re-derives the cached report in
# place through SimDeviceIF.apply_capability_profile -- see that method's
# docstring for why that is safe.

import collections
import copy
import json
import os
import re
import threading
import time

import yaml

# nepi_api.device_if_sim (imported below) pulls in nepi_api.device_if_npx ->
# nepi_api.system_if -> open3d transitively. On this aarch64 target, open3d's
# native libs (via libgomp) grab static TLS at import time, and if cv2 (also
# native, also TLS-hungry) has already been imported first, open3d's later
# import fails outright: "ImportError: .../libgomp.so.1: cannot allocate
# memory in static TLS block" -- confirmed by reproducing the exact failure
# standalone and confirming the fix is import order, not a code bug. Importing
# device_if_sim (and therefore open3d) before cv2 reserves its TLS slots
# first and avoids the exhaustion. Do not reorder cv2 (nepi_img imports it)
# above this without re-verifying against that failure.
from nepi_api.device_if_sim import SimDeviceIF
from nepi_api.robot_link_relay import RobotLinkRelay

import numpy as np

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_img
from nepi_sdk import nepi_system
from nepi_sdk import nepi_drvs

from std_msgs.msg import Bool, Empty, String, Float32, Int32
from sensor_msgs.msg import Image
from geographic_msgs.msg import GeoPoint

from nepi_interfaces.msg import AxisControls, DeviceRBXStatus, GotoPose, MotorControl, NavPose
from geographic_msgs.msg import GeoPoint

from nepi_api.messages_if import MsgIF

# Simulator launch (Deploy/Kill) goes through the NEPI simulator services on
# the sim machine -- see nepi_sim_service.py's own module comment. It is
# installed into nepi_api alongside device_if_sim.py by this package's own
# CMakeLists (api/ -> nepi_api dist-packages); SimLauncherStatus is generated
# from this same package's msg/.
from nepi_api import nepi_sim_service
from nepi_app_sim_connector.msg import SimLauncherStatus

PKG_NAME = 'SIM_CONNECTOR'

# This app's own ROS package name.
APP_PKG_NAME = 'nepi_app_sim_connector'
SIM_VEHICLE_DICT_KEY = 'SIM_VEHICLE_DICT'
# SIM_VEHICLE_DICT's own file, installed next to this app's params yaml (see
# that file's header for why it can't live inside the params yaml itself).
SIM_VEHICLE_DICT_FILENAME = 'sim_connector_vehicle_configs.yaml'
# system_folders is published by system_mgr, which starts well before any app.
# A short timeout is enough, and timing out is non-fatal (falls back to the
# capability-empty factory profile, exactly as before this fallback existed).
SYSTEM_FOLDERS_TIMEOUT_MSEC = 5000

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

# The three checked-in configs (sim_connector_app_params.yaml's own
# robot_configs section) -- never deletable, and never what
# persistRobotConfigEntry writes over even if an operator names a save the same
# as one of their display names (the KEY is what's checked here, not the
# display name, matching how selection/deletion already work on keys
# everywhere else in this file).
PROTECTED_ROBOT_CONFIG_KEYS = {FACTORY_ROBOT_CONFIG_NAME, 'ground_robot_4_wheel', 'flight_robot_4_motor'}

# Device-side persistent store for operator-saved robot configs -- survives
# a container restart, same persistence category as DIMENSIONS_STORAGE_DIR
# below (loaded back into self.robot_configs at startup, see
# loadPersistedRobotConfigs). Distinct from UPLOADED_ROBOT_CONFIG_NAME's
# single transient slot above: this is a real, named, growing collection.
ROBOT_CONFIGS_STORAGE_DIR = '/mnt/nepi_storage/databases/nepi_app_sim_connector/robot_configs'

# Physical-dimension editing (robot chassis/wheel geometry, environment
# corridor/ramp geometry) -- distinct from robot_configs' driver-capability
# profiles above, which never touch Gazebo geometry at all. Two roles for
# v1: 'robot' -> generic_rover, always (the quadcopter's airframe geometry
# is a vendored third-party Gazebo/ArduPilot model living outside this repo,
# nothing here to point a role at), and 'environment' -> whichever model the
# CURRENTLY SELECTED environment config names via its own reserved
# '_environment_model' field (see ENVIRONMENT_BUILTIN_DIMENSION_CONFIGS)
# rather than one fixed model, since an operator switches between Flat/
# Obstacle Course/Aerial Obstacle Course by switching configs. See
# generate_model_sdf.py for how curated dimensions.yaml fields become
# model.sdf. Getting them onto the sim machine is not wired up yet for the
# Gazebo service -- see pushDirtyDimensions.
ROBOT_DIMENSIONS_MODEL = 'generic_rover'
DIMENSION_ROLES = ('robot', 'environment')
DIMENSION_ROLE_MODEL = {'robot': ROBOT_DIMENSIONS_MODEL}

# Reserved dimensions.yaml key naming which Gazebo model an environment
# config's fields apply to -- generate_model_sdf.py's own loadDimensions
# tolerates unknown keys harmlessly, so pushing this key along with the rest
# of a config's fields is safe. ENVIRONMENT_MODEL_NONE ('none') is a real,
# valid value (the Flat built-in): it means "no model to generate or push,"
# not a missing/unset field.
ENVIRONMENT_MODEL_FIELD_KEY = '_environment_model'
ENVIRONMENT_MODEL_NONE = 'none'
# Used only to resolve a config saved before this key existed (an upgrading
# deployment's migrated legacy config, see ensureBuiltinDimensionConfigs) --
# matches the one fixed model 'environment' always meant previously.
ENVIRONMENT_LEGACY_MODEL_FALLBACK = 'obstacle_course'

# Built-in (undeletable) named dimensions configs per role -- see the
# constructor's own "Named dimensions configs" comment for the full design,
# and ensureBuiltinDimensionConfigs for how these get seeded on disk. No
# config is ever named "Default": each role's built-ins are named for what
# they actually are, and are exactly the configs an operator can never
# delete (see PROTECTED_DIMENSION_CONFIG_NAMES). Robot's one built-in name
# ('4-Wheel Rover') is deliberately the same display name
# ground_robot_4_wheel already uses in robot_configs above -- the RUI merges
# the two axes (capability profile + chassis/wheel geometry) into one button
# list per robot kind, and a matching name is what makes that merge line up.
ROBOT_BUILTIN_DIMENSION_CONFIG_NAME = '4-Wheel Rover'
ENVIRONMENT_BUILTIN_DIMENSION_CONFIGS = {
    # Genuinely no model spawned -- EnvironmentModelSpawner's own "nothing
    # active" state (see sim_container/scripts/environment_models.py) is
    # already a first-class option; this just gives it dimensions-config
    # standing alongside the real models below.
    'Flat': {ENVIRONMENT_MODEL_FIELD_KEY: ENVIRONMENT_MODEL_NONE},
    'Obstacle Course': {ENVIRONMENT_MODEL_FIELD_KEY: 'obstacle_course'},
    'Aerial Obstacle Course': {ENVIRONMENT_MODEL_FIELD_KEY: 'aerial_obstacle_course'},
    # No fixed curated fields of its own -- its 'obstacles' list (an
    # operator-built sequence of walls/circles/triangles, see
    # generate_model_sdf.py's buildCustomObstaclesSdf) starts empty and is
    # built up entirely through the RUI's add/drag/resize obstacle editor.
    # Requires no other backend support: a dimensions config is already
    # "any YAML mapping of fields" (setDimensionsCb never restricted this to
    # scalars), so a nested list travels the exact same save/select/push
    # path every other config already uses.
    'Custom Obstacles': {ENVIRONMENT_MODEL_FIELD_KEY: 'custom_obstacles', 'obstacles': []},
}
PROTECTED_DIMENSION_CONFIG_NAMES = {
    'robot': {ROBOT_BUILTIN_DIMENSION_CONFIG_NAME},
    'environment': set(ENVIRONMENT_BUILTIN_DIMENSION_CONFIGS.keys()),
}
# Built-ins always list first, in this order; everything else (custom saved
# configs) sorts alphabetically after them -- see listDimensionConfigs.
DIMENSION_CONFIG_BUILTIN_ORDER = {
    'robot': [ROBOT_BUILTIN_DIMENSION_CONFIG_NAME],
    'environment': ['Flat', 'Obstacle Course', 'Aerial Obstacle Course', 'Custom Obstacles'],
}
# Landed on after a delete removes whatever config was active -- always one
# of the protected/undeletable names above, so there is always something
# valid to fall back to. Environment's is 'Obstacle Course', not 'Flat',
# matching what the old single-file store always held before this
# per-role-builtins design existed.
FALLBACK_DIMENSION_CONFIG_NAME = {
    'robot': ROBOT_BUILTIN_DIMENSION_CONFIG_NAME,
    'environment': 'Obstacle Course',
}
# Pre-2026-08-31 seed name -- ensureBuiltinDimensionConfigs migrates any
# leftover Default.yaml on disk into FALLBACK_DIMENSION_CONFIG_NAME[role]
# once, then removes it, so "no such thing as Default anywhere" holds for a
# deployment upgrading from that earlier design too.
LEGACY_DEFAULT_DIMENSION_CONFIG_NAME = 'Default'

# Device-side authoritative store for the above -- survives a Docker
# container restart (unlike the container's own writable layer), the same
# persistence category as nepi_app_onvif_mgr's own /mnt/nepi_storage/
# user_cfg/ usage. The VM's copy under sim_container/models/<model>/ is a
# synced DEPLOYMENT TARGET, not the source of truth: pushed here via
# pushDirtyDimensions whenever a value changes or this app restarts.
DIMENSIONS_STORAGE_DIR = '/mnt/nepi_storage/databases/nepi_app_sim_connector/dimensions'

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

STATUS_PUBLISH_RATE_HZ = 1.0

# The simulators this app launches, each through its NEPI simulator service on
# the sim machine (nepi_api/nepi_sim_service.py). Keys are the launch-target
# keys the RUI's Simulator selector sends; 'gazebo_rover' keeps the RUI's own
# gazebo_ prefix check (Nepi_IF_Sim.js selectedTargetUsesModelSdf) working.
# Webots was taken out of the launcher 2026-09-28; MuJoCo came back on this
# service pattern 2026-10-05.
#
# default_environment / default_robot: what a Deploy asks for when nothing
# else is picked, by name within the service's ENVIRONMENT_CONFIGS/
# ROBOT_CONFIGS folders -- the rover on flat ground. Always sent explicitly,
# because the service otherwise keeps whatever it last ran.
GAZEBO_TARGET_KEY = 'gazebo_rover'
MUJOCO_TARGET_KEY = 'mujoco_rover'
SIM_TARGETS = collections.OrderedDict([
    (GAZEBO_TARGET_KEY, dict(
        service = nepi_sim_service.GAZEBO,
        default_environment = 'flat.world',
        default_robot = 'generic_rover',
        # Shown in the RUI's "Run These Commands Manually" box whenever a
        # request fails because the service isn't responding.
        fallback_commands = """On the sim machine, start the NEPI Gazebo service:

  nepi_gazebo_start <nepi_storage password>

It mounts NEPI storage, starts Gazebo, and then keeps watching
databases/sims/gazebo/config/nepi_gazebo_config.yaml for Deploy/Kill requests
from this device. First-time setup: nepi_simulations/nepi_gazebo/gazebo_setup.""")),
    (MUJOCO_TARGET_KEY, dict(
        service = nepi_sim_service.MUJOCO,
        default_environment = 'flat.xml',
        default_robot = 'generic_rover',
        fallback_commands = """On the sim machine, start the NEPI MuJoCo service:

  nepi_mujoco_start <nepi_storage password>

It mounts NEPI storage and then keeps watching
databases/sims/mujoco/config/nepi_mujoco_config.yaml for Deploy/Kill requests
from this device. First-time setup: nepi_simulations/nepi_mujoco/mujoco_setup.""")),
])
DEFAULT_SIM_TARGET = GAZEBO_TARGET_KEY
# A robot config profile's pointer at the service robot folder it runs as.
# The value keeps its original name so robot configs saved on a device before
# MuJoCo existed still load; it applies to whichever simulator is selected.
SHARE_ROBOT_FOLDER_FIELD = 'gazebo_robot_config'
# How often the selected simulator's ROBOT_CONFIGS/ENVIRONMENT_CONFIGS are
# re-read, so a robot folder or world dropped on the share shows up without an
# app restart.
SIM_CONFIGS_REFRESH_SEC = 5.0
# Operator-saved robot configs (persisted on the device)
# are no longer offered: robots come only from the share's ROBOT_CONFIGS
# (2026-09-29). The save/upload/delete code and the RUI controls for it are
# kept, just switched off (the RUI side is Nepi_IF_Sim.js's
# SHOW_ROBOT_CONFIG_EDITING) -- saving may come back later as "write a new
# folder into ROBOT_CONFIGS".
OFFER_DEVICE_SAVED_ROBOT_CONFIGS = False

# How long a Deploy/Kill waits for the service to finish. Start covers the
# launch plus Gazebo's robot spawn, which its service retries for up to ~20s;
# the 120s is the old launcher's ready-check budget, sized for a cold VM.
SIM_START_TIMEOUT_SEC = 120
SIM_STOP_TIMEOUT_SEC = 60

# A service's STATE -> launcher_state (SimLauncherStatus's own vocabulary,
# which the RUI renders).
SERVICE_STATE_TO_LAUNCHER_STATE = {
    'idle': 'idle',
    'starting': 'launching',
    'running': 'running',
    'stopping': 'stopping',
    'failed': 'failed',
}

# The simulated rover's RBX driver (nepi_drivers' rbx_sim_node.py). This app
# runs it while a simulator service reports running with a robot config whose
# nepi_robot_config.yaml names nepi_driver: rbx_sim, and stops it whenever the
# service reports anything else -- Kill, a crash, a stop on the sim machine --
# so the rover is in Devices -> Robots exactly while it exists. Replaces
# rbx_sim_discovery.py's heartbeat discovery (port 9022), which is no longer
# used. See syncRoverDriver.
ROVER_DRIVER = 'rbx_sim'
ROVER_DRIVER_FILE = 'rbx_sim_node.py'
ROVER_DRIVER_DEVICE_NAME = 'sim_rover'
# Where rbx_sim_node.py listens for the sim machine's nepi_gazebo_bridge.py,
# UDP and TCP -- the same number as nepi_gazebo.sh's GAZEBO_BRIDGE_PORT.
ROVER_BRIDGE_PORT = 9023
# How long a driver that crashed or failed to launch waits before the next
# launch, so one that dies at once is not relaunched on every status tick.
ROVER_DRIVER_RETRY_SEC = 10.0
# Carried over from rbx_sim_discovery.py: on aarch64 the RBX driver's
# open3d import can fail with "cannot allocate memory in static TLS block"
# unless libgomp is preloaded.
ROVER_DRIVER_LD_PRELOAD = '/lib/aarch64-linux-gnu/libgomp.so.1'


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
    # the app reads its own installed SIM_VEHICLE_DICT_FILENAME directly when
    # the param is absent. (That file split out of the params yaml 2026-09-29,
    # after the third top-level key turned out to also break this app's RUI
    # page -- see that file's header.) The param still wins when set, so this
    # stays backward compatible with anything that sets the param itself.
    vehicle_dict_ns = nepi_sdk.create_namespace(self.node_namespace, 'SIM_VEHICLE_DICT')
    self.vehicle_dict = nepi_sdk.get_param(vehicle_dict_ns, dict())
    if not isinstance(self.vehicle_dict, dict):
      self.vehicle_dict = dict()
    if not self.vehicle_dict:
      self.vehicle_dict = self.loadVehicleDictFromParamsFile()

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
    # Operator-saved configs (see persistRobotConfigEntry) used to merge in on top
    # of the checked-in ones above -- persisted separately so a save survives
    # independently of that file ever being redeployed/reset. Off while robots
    # come only from the share; see OFFER_DEVICE_SAVED_ROBOT_CONFIGS.
    if OFFER_DEVICE_SAVED_ROBOT_CONFIGS:
      self.loadPersistedRobotConfigs()
    # The simulator the RUI's Simulator selector currently shows. Its share's
    # robots and worlds are what the Robot Config and Environment Config
    # selectors offer -- see selectLaunchTargetCb.
    self.selected_sim_target = DEFAULT_SIM_TARGET
    # Robots in the selected simulator's share join these, and claim or extend
    # the profiles above -- see refreshShareRobotConfigs. Done before the
    # default is picked so the default profile already carries whatever
    # capabilities its share folder adds.
    self.share_robot_config_keys = set()
    self.claimed_profile_bases = dict()
    self.refreshShareRobotConfigs()

    default_config = str(self.vehicle_dict.get('default_robot_config', FACTORY_ROBOT_CONFIG_NAME))
    if default_config not in self.robot_configs:
      default_config = FACTORY_ROBOT_CONFIG_NAME
    self.selected_robot_config = default_config
    self.profile = self.buildProfileFromConfig(default_config)

    ##############################
    # Live state for the SimDeviceIF. No simulator reports environment
    # options, so that list stays empty.
    self.available_environment_options = []
    self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    self.motor_ratios = [0.0] * int(self.profile['motor_count'])

    ##############################
    # Simulator discovery state. Populated by a timer scan of the live ROS
    # graph; never by a blocking wait.
    self.sim_scan_lock = threading.Lock()
    self.sim_device_subs = dict()      # status topic -> subscriber handle
    self.sim_device_info = dict()      # status topic -> dict(name, source, time)
    self.selected_simulator = ""
    self.rbx_forward_pubs = dict()      # (namespace, topic_name) -> publisher, see forwardToRbx

    ##############################
    # Common four-topic mirror -- a stable, simulator-agnostic viewing point
    # for whichever robot is currently selected_simulator, regardless of
    # which underlying driver/bridge type it is. Every simulator goes through
    # an RBX driver that already publishes real ROS Image topics on this same ROS
    # master -- see rbx_sim_node.py/rbx_ardupilot_node.py's own
    # color_2d_image/robot_color + .../scene_color topics -- so this mirror
    # is a plain re-subscribe, not a protocol decode. Reported live
    # (2026-08-18): the operator wants one topic to look at "no matter the
    # simulator" rather than needing to know the current instance's own
    # device-name-qualified topic. The raw robot_depth_map/scene_depth_map
    # pair (float32-meters, for downstream processing) removed entirely
    # (2026-09-14, requested live: "unness and dont do anything") -- never a
    # viewing feed and had no processing consumer either. Either remaining
    # topic can have no publisher at all for a robot that honestly has no
    # scene camera or no depth sensor (the Webots drivers) -- an absent
    # feed there is accurate, not a bug.
    self.MIRROR_VIEWS = ["robot_color", "scene_color", "robot_depth", "scene_depth"]
    self.mirror_pubs = dict()
    self.mirror_subs = dict()
    self.mirror_source_topics = dict()
    for view in self.MIRROR_VIEWS:
      self.mirror_pubs[view] = nepi_sdk.create_publisher(self.node_name + "/" + view, Image, queue_size = 1)
      self.mirror_subs[view] = None
      self.mirror_source_topics[view] = None

    # NavPose mirror -- same "one stable thing to look at no matter the
    # simulator" reasoning as the six-topic image mirror above, for
    # self.navpose_dict (which already feeds this app's own NavPoseIF via
    # getNavPoseCb below). Reported live (2026-09-03): "navpose data in
    # devices -> apps also doesn't seem to detect any of the values" --
    # root cause: self.navpose_dict was only ever populated by the
    # generic-connector bridge protocol (webots_rover, since removed) --
    # Gazebo rover/quadcopter
    # go through an RBX driver instead (rbx_sim_node.py/rbx_ardupilot_node.py),
    # exactly like the image mirror's own comment already documents, and
    # that driver's already-correct NavPose (its own NPXDeviceIF, publishing
    # <device>/npx/navpose) was never connected to this app's copy at all --
    # so this app's own NavPose consumer showed nothing whenever a Gazebo/
    # Webots target was actually selected. Mirrors that topic the same way
    # updateCommonViewSubscriptions already mirrors images, from whichever
    # node_namespace selected_simulator currently resolves to.
    self.navpose_mirror_sub = None
    self.navpose_mirror_source_topic = None

    ##############################
    # FOV data -- published as plain, latched Float32 topics rather than
    # extending any .msg (avoids a catkin message-regeneration rebuild for
    # this). horizontal_fov is now an editable ROBOT dimension field
    # (camera_horizontal_fov_deg, see generate_model_sdf.py's
    # ROVER_DEFAULT_DIMENSIONS and buildRoverSdf) rather than a fixed
    # constant -- requested live (2026-09-01). Editing it through the
    # existing robot-dimensions curated-fields path (same save/select flow
    # as wheel_radius_m etc.) changes what Gazebo actually renders AND
    # updates these two reported values together via
    # updateCameraFovFromRobotDimensions, called here once at startup and
    # again from setDimensionsCb/applyDimensionConfigByName whenever the
    # robot role's fields change, so the two can never drift out of sync.
    # vertical_fov is derived from horizontal via the standard pinhole
    # relation for the sensor's fixed 640x480 resolution.
    self.camera_horizontal_fov_pub = nepi_sdk.create_publisher(
        self.node_name + "/sim/camera_horizontal_fov_deg", Float32, queue_size = 1, latch = True)
    self.camera_vertical_fov_pub = nepi_sdk.create_publisher(
        self.node_name + "/sim/camera_vertical_fov_deg", Float32, queue_size = 1, latch = True)
    self.updateCameraFovFromRobotDimensions()

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
    # Launch the generic SimDeviceIF. Its commands are forwarded to the
    # selected simulator's RBX device (see forwardToRbx).
    device_info = dict(device_name = self.node_name, path = "",
                       serial_number = "", hw_version = "", sw_version = "")

    self.sim_if = SimDeviceIF(
        device_info = device_info,
        getNavPoseCb = self.getNavPoseCb,
        getAvailableEnvironmentOptionsFunction = self.getAvailableEnvironmentOptions,
        getBridgeConnectedFunction = self.isBridgeConnected,
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
    # Simulator launch -- the RUI launcher panel's Deploy/Kill, carried out by
    # the NEPI simulator service (Gazebo or MuJoCo) on the sim machine through
    # its config file in NEPI storage (see nepi_sim_service.py). Same division
    # of labor as software_mgr and nepi_docker.sh: this node sets a request
    # flag, then reports what the service writes back. A simulator is offered
    # once its service's config file exists (the service's first start creates
    # it); with none, the panel shows "Not configured on this deployment" and
    # the topics below are no-ops.
    self.launcher_lock = threading.Lock()
    # Launch and stop share this one thread/lock -- the service carries out
    # one request at a time, and a stop racing a start has no sensible
    # outcome worth supporting.
    self.launcher_thread = None
    self.launcher_state = 'idle'
    self.launcher_last_error = ''
    # Copy-paste commands shown alongside the current failure (the RUI's "Run
    # These Commands Manually" box) -- set and cleared with
    # launcher_last_error by setLauncherError.
    self.launcher_fallback_commands = ''
    # The simulator whose state the launcher panel shows: the one running,
    # or the last one deployed/stopped. Reported to the RUI as
    # selected_launch_target / active_launch_target (see publishLauncherStatus).
    self.selected_launch_target = ''
    self.active_launch_target = ''
    # What the status timer last saw from each service -- see
    # refreshLauncherStateCb.
    self.service_config_found = {t: SIM_TARGETS[t]['service'].config_exists() for t in SIM_TARGETS}
    self.service_alive = {t: False for t in SIM_TARGETS}
    self.service_state = {t: None for t in SIM_TARGETS}

    # The rover's RBX driver process -- see syncRoverDriver.
    self.rover_driver_lock = threading.Lock()
    self.rover_driver_process = None
    self.rover_driver_launched = None             # (target_key, robot folder) it was launched for
    self.rover_driver_node_name = ''
    self.rover_driver_busy = False
    self.rover_driver_retry_after = 0.0
    self.rover_robot_capabilities = dict()        # target_key -> {robot folder: capabilities}
    self.rover_robot_capabilities_time = dict()   # target_key -> last read time

    # "New Sim" secondary instances. Each service runs a single simulation,
    # so this only ever records a refusal for the RUI's "Other Running
    # Instances" list (see launchNewSimulatorCb).
    self.secondary_launch_states = {}   # target_key -> 'failed: ...'

    self.launcher_status_pub = nepi_sdk.create_publisher(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/launcher_status'),
        SimLauncherStatus, queue_size = 1, latch = True)
    # Environment Config: a world from the selected simulator's
    # ENVIRONMENT_CONFIGS, picked for the next Deploy. The list is a JSON
    # array of world file names; both are latched so a client that subscribes
    # late still gets them.
    self.environment_configs = []
    self.selected_environment_config = SIM_TARGETS[self.selected_sim_target]['default_environment']
    self.sim_configs_refreshed_at = time.monotonic()
    self.environment_config_names_pub = nepi_sdk.create_publisher(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/environment_config_names'),
        String, queue_size = 1, latch = True)
    self.selected_environment_config_pub = nepi_sdk.create_publisher(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/selected_environment_config'),
        String, queue_size = 1, latch = True)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/select_environment_config'),
        String, self.selectEnvironmentConfigCb, queue_size = 1)
    self.refreshEnvironmentConfigs(force_publish = True)
    # Simulator selector: switches the Robot Config and Environment Config
    # lists to that simulator's share. Deploy carries its own target too.
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/select_launch_target'),
        String, self.selectLaunchTargetCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/stop_simulator'),
        Empty, self.stopSimulatorCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/redeploy_simulator'),
        String, self.redeploySimulatorCb, queue_size = 1)
    # "New Sim" -- a second, independent instance alongside the primary one.
    # Same payload shape as redeploy_simulator (JSON {target_key, robot_config}
    # or a bare target_key string), parsed with the same parseLaunchPayload.
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/launch_new_simulator'),
        String, self.launchNewSimulatorCb, queue_size = 1)
    # Clears one entry from secondary_launcher_status by its target key --
    # NOT the primary slot's own sim/stop_simulator.
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/stop_new_simulator'),
        String, self.stopNewSimulatorCb, queue_size = 1)
    # JSON-in-String rather than a new .msg field on SimLauncherStatus --
    # avoids a catkin message-regen for what is, for now, a secondary/
    # additive status concern (mirrors this app's own established
    # convention for late-added fields: Robot Link status, save/select
    # dimension-config payloads). {"active_targets": [...], "states": {...}}
    self.secondary_launcher_status_pub = nepi_sdk.create_publisher(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/secondary_launcher_status'),
        String, queue_size = 1, latch = True)
    # Lets an operator try their own robot without editing and redeploying
    # sim_connector_app_params.yaml -- independent of the simulator launch
    # above, so this stays available on every deployment.
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
    # Persisted delete -- see PROTECTED_ROBOT_CONFIG_KEYS/
    # ROBOT_CONFIGS_STORAGE_DIR's own comments and deleteRobotConfigCb below.
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/delete_robot_config'),
        String, self.deleteRobotConfigCb, queue_size = 1)

    ##############################
    # Robot Link -- mirrors RBX flight/motion commands from the currently
    # attached sim onto a selected physical robot. See robot_link_relay.py's
    # own module docstring for the full design and what is deliberately not
    # mirrored. self.robot_link's sim-side target is kept in sync with
    # self.selected_simulator from simDiscoveryCb, not set here.
    self.robot_link = RobotLinkRelay(self.msg_if)
    self.robot_link_status_pub = nepi_sdk.create_publisher(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/robot_link/status'),
        String, queue_size = 1, latch = True)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/robot_link/select_physical_robot'),
        String, self.selectPhysicalRobotCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/robot_link/set_props_off_acknowledged'),
        Bool, self.setPropsOffAcknowledgedCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/robot_link/enable'),
        Bool, self.setRobotLinkEnabledCb, queue_size = 1)

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
    # Named dimensions configs (robot + environment) -- lets an operator save
    # a curated field set under a name and switch back to it later, the same
    # convenience robot_configs already has, but with real persistence and a
    # real delete: reported live (2026-08-31) that dimensions had no way to
    # keep more than the one currently-active set around at all. Layered on
    # top of the existing single-file dimensionsYamlStoragePath/
    # writeStoredDimensionsYaml mechanism above rather than replacing it --
    # that file stays "whatever is currently active" exactly as before (what
    # pushDirtyDimensions/generate_model_sdf.py read), and a named config is
    # just a separate saved snapshot a client can select to become the new
    # "currently active" one. Each role's built-in names (see
    # ENSURE_BUILTIN... / ensureBuiltinDimensionConfigs) are seeded once from
    # whatever the single file already had (for the one matching
    # FALLBACK_DIMENSION_CONFIG_NAME[role]) so upgrading a deployment never
    # means "the one that was already working" silently resets to a blank
    # slate -- and none of them can ever be deleted, so there is always at
    # least one config to fall back to. environment_dimensions_model_pub
    # lets the RUI know which model (and therefore which field set) the
    # currently-selected ENVIRONMENT config actually targets, since that
    # varies per config for this role only -- see dimension_config_model.
    self.selected_dimension_config = dict()
    self.dimension_config_model = dict()
    # Which environment dimensions config the TOP "Environment Config"
    # deploy selector last actually deployed -- deliberately its OWN,
    # separate field from selected_dimension_config['environment'] (the
    # EDIT dropdown's own selection; see this file's own long-standing
    # "these are separate" decoupling). Requested live (2026-09-18): "if
    # trying to delete the config of something thats currently selected,
    # it should give a popup saying that environment is currently selected
    # and cant be deleted." Checked in deleteDimensionConfigCb -- server-side
    # enforcement, mirroring how PROTECTED_DIMENSION_CONFIG_NAMES is already
    # checked both client-side (an immediate alert) and here (authoritative
    # regardless of client). Its only setter was the old deploy_environment
    # topic, which nothing published, so it stays '' for now.
    self.deployed_environment_config_name = ''
    self.deployed_environment_config_name_pub = nepi_sdk.create_publisher(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/deployed_environment_config_name'),
        String, queue_size = 1, latch = True)
    self.dimension_config_names_pubs = dict()
    self.dimension_config_selected_pubs = dict()
    # Not latched -- a transient "this just happened" notice for the RUI to
    # show as a fading indicator (see setDimensionsCb's own comment), not
    # persistent state a late subscriber should replay on mount the way
    # dimension_config_selected_pubs's own latched value should.
    self.dimension_config_autosaved_pubs = dict()
    self.environment_dimensions_model_pub = nepi_sdk.create_publisher(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/environment_dimensions_selected_model'),
        String, queue_size = 1, latch = True)
    for role in DIMENSION_ROLES:
      self.dimension_config_names_pubs[role] = nepi_sdk.create_publisher(
          nepi_sdk.create_namespace(self.node_namespace, 'sim/' + role + '_dimensions_config_names'),
          String, queue_size = 1, latch = True)
      self.dimension_config_selected_pubs[role] = nepi_sdk.create_publisher(
          nepi_sdk.create_namespace(self.node_namespace, 'sim/' + role + '_dimensions_selected_config'),
          String, queue_size = 1, latch = True)
      self.dimension_config_autosaved_pubs[role] = nepi_sdk.create_publisher(
          nepi_sdk.create_namespace(self.node_namespace, 'sim/' + role + '_dimensions_autosaved'),
          String, queue_size = 1, latch = False)
      # Resolved from the active file's ACTUAL content, not just assumed to
      # be the fallback -- see resolveMatchingDimensionConfigName's own
      # comment for why (reported live, 2026-09-01: after some unsaved
      # editing, a restart kept showing "4-Wheel Rover"/"Obstacle Course" as
      # selected even though the active values no longer matched what
      # either file actually holds, reading as "editing changed the
      # default" even though the named files themselves were never
      # touched).
      self.selected_dimension_config[role] = (
          self.resolveMatchingDimensionConfigName(role) or FALLBACK_DIMENSION_CONFIG_NAME[role])
      if role == 'environment':
        self.dimension_config_model[role] = self.resolveEnvironmentModelFromYaml(
            self.readStoredDimensionsYaml(role))
      self.publishAvailableDimensionConfigs(role)
      self.publishSelectedDimensionConfig(role)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/select_robot_dimensions_config'),
        String, self.selectRobotDimensionsConfigCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/select_environment_dimensions_config'),
        String, self.selectEnvironmentDimensionsConfigCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/save_robot_dimensions_config'),
        String, self.saveRobotDimensionsConfigCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/save_environment_dimensions_config'),
        String, self.saveEnvironmentDimensionsConfigCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/delete_robot_dimensions_config'),
        String, self.deleteRobotDimensionsConfigCb, queue_size = 1)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/delete_environment_dimensions_config'),
        String, self.deleteEnvironmentDimensionsConfigCb, queue_size = 1)
    # Read-only YAML preview -- requested live (2026-09-04): "in the
    # environment configs section in the bottom to view the yaml files,
    # clicking between those shouldnt change the actual env config in the
    # dropdown - they should be separate." Unlike
    # select_environment_dimensions_config (which makes a config the active
    # one being edited -- see applyDimensionConfigByName), this is a pure
    # file read with no side effects at all: no selected_dimension_config
    # change, no dimensions_dirty, no re-derived model, no
    # dimensions_yaml_pubs echo. Mirrors get_robot_config/robot_config_yaml's
    # own request/response shape for the capability axis.
    self.environment_dimensions_config_yaml_preview_pub = nepi_sdk.create_publisher(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/environment_dimensions_config_yaml_preview'),
        String, queue_size = 1, latch = True)
    nepi_sdk.create_subscriber(
        nepi_sdk.create_namespace(self.node_namespace, 'sim/get_environment_dimensions_config'),
        String, self.getEnvironmentDimensionsConfigCb, queue_size = 1)

    # Latched, so a client that subscribes after startup still gets a real
    # report (available targets, "idle") instead of waiting for the first
    # launch/stop -- the same reasoning SimStatus's own latch already uses.
    # The status timer below replaces it with whatever the simulator services
    # report within a second.
    self.publishLauncherStatus()

    ##############################
    # Simulator discovery scan, and the app's own status cadence
    nepi_sdk.start_timer_process(float(1) / SIM_DISCOVERY_RATE_HZ, self.simDiscoveryCb)
    nepi_sdk.start_timer_process(float(1) / STATUS_PUBLISH_RATE_HZ, self.statusPublishCb)

    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.on_shutdown(self.cleanup_actions)
    nepi_sdk.spin()

  #**********************
  # Robot-config profiles

  def loadVehicleDictFromParamsFile(self):
    # Fallback source for SIM_VEHICLE_DICT when it is not on the param server
    # -- see the correction note in __init__ for why that is the normal case.
    # Reads SIM_VEHICLE_DICT_FILENAME from system_folders' apps_param entry
    # (the folder apps_mgr scans, where CMakeLists.txt installs it next to
    # this app's params yaml) rather than a hardcoded path.
    #
    # Every failure path returns an empty dict rather than raising: the caller
    # then proceeds with the capability-empty factory profile, which is the
    # documented safe default and exactly the behavior that existed before
    # this fallback. A missing or malformed file must never stop the node
    # from starting.
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

      file_path = os.path.join(params_folder, SIM_VEHICLE_DICT_FILENAME)
      if not os.path.isfile(file_path):
        self.msg_if.pub_warn("No " + SIM_VEHICLE_DICT_FILENAME + " for " + APP_PKG_NAME + " found in " +
                             params_folder + ", using factory robot config only")
        return dict()
      file_dict = nepi_utils.read_yaml_2_dict(file_path)
      vehicle_dict = file_dict.get(SIM_VEHICLE_DICT_KEY, dict()) if isinstance(file_dict, dict) else None
      if not isinstance(vehicle_dict, dict) or not vehicle_dict:
        self.msg_if.pub_warn("Found " + file_path + " but it has no usable " +
                             SIM_VEHICLE_DICT_KEY + ", using factory robot config only")
        return dict()
      self.msg_if.pub_info("Loaded " + SIM_VEHICLE_DICT_KEY + " from " + file_path)
      return vehicle_dict
    except Exception as e:
      self.msg_if.pub_warn("Failed to load " + SIM_VEHICLE_DICT_KEY + " from " + SIM_VEHICLE_DICT_FILENAME +
                           ": " + str(e) + ", using factory robot config only")
    return dict()

  def buildProfileFromConfig(self, config_name):
    # A robot config is a capability profile read whole from SIM_VEHICLE_DICT. A
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
    # Also re-apply the matching "robot" dimensions config (by display_name,
    # not key -- the two axes are keyed independently, see
    # sanitizeRobotConfigKey's own comment) if one exists. Reported live
    # 2026-09-14: saving a new custom robot ("diyrover1") left it selected
    # on the DIMENSIONS axis, and picking "4-Wheel Rover" back on the
    # CAPABILITY axis (this method) did nothing to the dimensions axis --
    # so the shared generic_rover Gazebo model kept diyrover1's geometry
    # even while "4-Wheel Rover" showed as selected, and deploying it
    # summoned diyrover1's dimensions under the 4-Wheel-Rover name. A config
    # with no dimensions counterpart (Quadcopter -- a vendored third-party
    # airframe, see ROBOT_BUILTIN_DIMENSION_CONFIG_NAME's own comment) is
    # silently skipped, not a warning-worthy failure.
    entry = self.robot_configs.get(config_name, dict())
    display_name = entry.get('display_name', config_name) if isinstance(entry, dict) else config_name
    if display_name in self.listDimensionConfigs('robot'):
      self.applyDimensionConfigByName('robot', display_name)
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

  def sanitizeRobotConfigKey(self, name):
    # A robot_configs key is a plain dict key everywhere else in this file
    # (never exposed to the filesystem the way a dimensions config name is,
    # see sanitizeDimensionConfigName), but a SAVED one still becomes a
    # literal filename here (robotConfigPath) -- same safe character set for
    # the same reason, plus a 'custom_' prefix so an operator's own name can
    # never collide with a checked-in key by accident (e.g. someone naming
    # their save "default").
    name = str(name).strip()
    safe = re.sub(r'[^A-Za-z0-9 _-]', '_', name)[:64].strip() or 'Unnamed'
    return 'custom_' + safe.lower().replace(' ', '_')

  def robotConfigPath(self, key):
    return os.path.join(ROBOT_CONFIGS_STORAGE_DIR, key + '.yaml')

  def loadPersistedRobotConfigs(self):
    # Called once at startup (see __init__, right after the three checked-in
    # configs are loaded) -- merges every previously-saved config back into
    # self.robot_configs so a save survives a container restart. A file that
    # fails to parse or validate is skipped with a warning rather than
    # aborting the whole load, same "one bad entry doesn't take down
    # everything else" instinct as uploadRobotConfigCb's own validation.
    try:
      filenames = os.listdir(ROBOT_CONFIGS_STORAGE_DIR)
    except OSError:
      return
    for filename in filenames:
      if not filename.endswith('.yaml'):
        continue
      key = filename[:-len('.yaml')]
      path = os.path.join(ROBOT_CONFIGS_STORAGE_DIR, filename)
      try:
        with open(path, 'r') as f:
          entry = yaml.safe_load(f)
      except (OSError, yaml.YAMLError) as e:
        self.msg_if.pub_warn("Failed to load persisted robot config '" + key + "': " + str(e))
        continue
      if not isinstance(entry, dict):
        self.msg_if.pub_warn("Persisted robot config '" + key + "' is not a YAML mapping, skipping")
        continue
      try:
        self.buildProfileFromEntry(entry)
      except (TypeError, ValueError) as e:
        self.msg_if.pub_warn("Persisted robot config '" + key + "' has an invalid field value, skipping: " + str(e))
        continue
      entry.pop('hidden_from_selector', None)
      self.robot_configs[key] = entry

  def persistRobotConfigEntry(self, name, entry):
    # Tail of saveDimensionConfigCb's own auto-link (see that method's own
    # comment) -- "this display name is now a real, selectable capability
    # profile." Returns the new key, or None if the write failed (already
    # warned).
    entry = copy.deepcopy(entry)
    entry['display_name'] = name
    key = self.sanitizeRobotConfigKey(name)
    try:
      os.makedirs(ROBOT_CONFIGS_STORAGE_DIR, exist_ok = True)
      with open(self.robotConfigPath(key), 'w') as f:
        yaml.safe_dump(entry, f, default_flow_style = False, sort_keys = False)
    except Exception as e:
      self.msg_if.pub_warn("Failed to save robot config '" + name + "': " + str(e))
      return None
    self.robot_configs[key] = entry
    return key

  def findRobotConfigKeyByDisplayName(self, name):
    for key, entry in self.robot_configs.items():
      if isinstance(entry, dict) and entry.get('display_name') == name:
        return key
    return None

  def deleteRobotConfigByKey(self, key):
    # Core delete, shared by deleteRobotConfigCb (an operator deleting by
    # key directly) and deleteDimensionConfigCb's own cascade below (finding
    # a linked capability entry by display_name) -- returns whether it
    # actually deleted anything so a cascade caller can stay silent on a
    # no-op rather than warn about something that was never there.
    if key in PROTECTED_ROBOT_CONFIG_KEYS or key == UPLOADED_ROBOT_CONFIG_NAME:
      return False
    if key not in self.robot_configs:
      return False
    path = self.robotConfigPath(key)
    try:
      if os.path.exists(path):
        os.remove(path)
    except Exception as e:
      self.msg_if.pub_warn("Failed to delete robot config '" + key + "': " + str(e))
      return False
    del self.robot_configs[key]
    # Falls back to the 4-Wheel Rover, not FACTORY_ROBOT_CONFIG_NAME -- an
    # operator who just deleted a config they were actively using almost
    # certainly wants a REAL, capable robot to land on, not the
    # capability-empty placeholder (the same reasoning setSelectedRobotConfig
    # itself never applies automatically, since that placeholder is only
    # ever a fallback for "nothing valid was ever selected").
    if self.selected_robot_config == key:
      self.setSelectedRobotConfig(SIM_TARGETS[self.selected_sim_target]['default_robot'])
    return True

  def deleteRobotConfigCb(self, msg):
    key = str(msg.data).strip()
    if not key:
      return
    if key in PROTECTED_ROBOT_CONFIG_KEYS or key == UPLOADED_ROBOT_CONFIG_NAME:
      self.msg_if.pub_warn("Cannot delete the built-in robot config '" + key + "'")
      return
    if key not in self.robot_configs:
      self.msg_if.pub_warn("Robot config '" + key + "' not found, cannot delete")
      return
    # Captured before deleteRobotConfigByKey removes the entry, so the
    # dimensions-axis cascade below still knows which display_name to look
    # for (see linkRobotConfigToDimensions -- a custom robot's capability
    # and dimensions entries are created together under the same
    # display_name, so they are removed together here too, never leaving
    # one to orphan as an undeletable "ghost" entry -- reported live
    # 2026-09-17: a config whose dimensions entry had already been deleted
    # kept showing up in the Robot Config dropdown and the merged button
    # row with no way to remove it, since the only delete path on offer was
    # dimensions-only).
    entry = self.robot_configs.get(key, dict())
    display_name = entry.get('display_name') if isinstance(entry, dict) else None
    if not self.deleteRobotConfigByKey(key):
      self.msg_if.pub_warn("Failed to delete robot config '" + key + "'")
      return
    self.msg_if.pub_info("Deleted robot config '" + key + "'")
    if display_name and self.sanitizeDimensionConfigName(display_name) not in PROTECTED_DIMENSION_CONFIG_NAMES.get('robot', set()):
      dim_path = self.dimensionConfigPath('robot', display_name)
      if os.path.exists(dim_path):
        try:
          os.remove(dim_path)
        except Exception as e:
          self.msg_if.pub_warn("Failed to delete linked robot dimensions config '" + display_name + "': " + str(e))
        else:
          self.publishAvailableDimensionConfigs('robot')
          if self.selected_dimension_config.get('robot') == self.sanitizeDimensionConfigName(display_name):
            self.applyDimensionConfigByName('robot', FALLBACK_DIMENSION_CONFIG_NAME['robot'])
          self.msg_if.pub_info("Deleted linked robot dimensions config: " + display_name)

  #**********************
  # Physical-dimension editing (robot chassis/wheel + environment
  # corridor/ramp geometry) -- see DIMENSION_ROLES's own comment for the
  # role -> model mapping and the device-store-is-authoritative design.

  # Default matches generate_model_sdf.py's own ROVER_DEFAULT_DIMENSIONS
  # entry -- used here only as the fallback when the robot dimensions store
  # doesn't (yet) have the field, e.g. a fresh install or a config saved
  # before this field existed.
  CAMERA_HORIZONTAL_FOV_DEFAULT_DEG = 80.0

  # Same fallback defaults as the RUI's own ROBOT_DIMENSION_FIELDS, used
  # only when a field is missing from a partial edit (setDimensionsCb lets
  # a partial edit through -- generate_model_sdf.py fills in its own
  # defaults for anything unset, so this check does too, rather than
  # treating "field absent" as "field is zero" and flagging a false
  # positive against a robot that's actually fine).
  ROBOT_DIMENSIONS_VIABILITY_DEFAULTS = {
      'track_width_m': 0.34, 'wheel_width_m': 0.05, 'wheel_radius_m': 0.1,
      'wheelbase_m': 0.3, 'chassis_width_m': 0.3, 'chassis_length_m': 0.4,
  }

  def checkRobotDimensionsViable(self, fields):
    # Authoritative gate, mirrored from the RUI's own client-side
    # checkRobotDimensionsViable (Nepi_IF_Sim.js) -- requested live
    # (2026-09-17) after the RUI-only check turned out not to be enough:
    # "the wheels touching each other still doesnt give a warning. its
    # deployable on the sim." A client-side check only guards the one path
    # that calls it; this is the single choke point every robot-dimensions
    # write actually passes through (setDimensionsCb/saveDimensionConfigCb),
    # so nothing -- RUI button, a raw rostopic pub, a future caller -- can
    # persist or push unviable geometry. Same AABB touch test as the RUI:
    # detached the instant there's a gap on EITHER axis (front-to-back,
    # wheelbase_m vs chassis_length_m, or side-to-side, track_width_m vs
    # chassis_width_m); touching/overlapping when the two wheels on one
    # axle are closer together than their own width. Returns
    # (True, '') or (False, reason).
    def get(name):
      try:
        return float(fields.get(name, self.ROBOT_DIMENSIONS_VIABILITY_DEFAULTS[name]))
      except (TypeError, ValueError):
        return self.ROBOT_DIMENSIONS_VIABILITY_DEFAULTS[name]
    track_width = get('track_width_m')
    wheel_width = get('wheel_width_m')
    wheel_radius = get('wheel_radius_m')
    wheelbase = get('wheelbase_m')
    chassis_width = get('chassis_width_m')
    chassis_length = get('chassis_length_m')
    if track_width <= wheel_width:
      return False, "the wheels are touching or overlapping each other"
    # Front-to-back overlap (same side): the front and rear wheels are two
    # circles of radius wheel_radius, wheelbase apart center-to-center --
    # they touch/overlap once that's less than the sum of their radii, i.e.
    # less than 2x the (shared) radius. Missing entirely before this pass
    # (2026-09-18, reported live: "overlapping wheels still doesnt get
    # detected as not viable") -- the only overlap check here was the
    # side-to-side (track_width vs wheel_width) one above; a short
    # wheelbase with normal-sized wheels passed straight through.
    if wheelbase <= 2.0 * wheel_radius:
      return False, "the front and rear wheels are touching or overlapping each other"
    gap_along_length = (wheelbase / 2.0 - wheel_radius) - (chassis_length / 2.0)
    gap_along_width = (track_width / 2.0 - wheel_width / 2.0) - (chassis_width / 2.0)
    if gap_along_length > 0 or gap_along_width > 0:
      return False, "the wheels are detached from the chassis"
    return True, ''

  def updateCameraFovFromRobotDimensions(self):
    # Reads camera_horizontal_fov_deg out of the robot dimensions store
    # (an editable curated field, same as wheel_radius_m etc. -- see
    # generate_model_sdf.py's ROVER_DEFAULT_DIMENSIONS/buildRoverSdf) and
    # republishes both reported FOV topics from it. Called once at startup
    # and again from setDimensionsCb/applyDimensionConfigByName whenever
    # the robot role's fields change, so a live edit's effect on the
    # rendered camera and on what this app REPORTS about that camera never
    # drift apart. vertical_fov is derived from horizontal via the
    # standard pinhole relation for the sensor's fixed 640x480 resolution.
    try:
      fields = yaml.safe_load(self.readStoredDimensionsYaml('robot') or '')
    except yaml.YAMLError:
      fields = None
    horizontal_deg = self.CAMERA_HORIZONTAL_FOV_DEFAULT_DEG
    if isinstance(fields, dict) and 'camera_horizontal_fov_deg' in fields:
      try:
        horizontal_deg = float(fields['camera_horizontal_fov_deg'])
      except (TypeError, ValueError):
        pass
    self.CAMERA_HORIZONTAL_FOV_DEG = horizontal_deg
    self.CAMERA_VERTICAL_FOV_DEG = 2.0 * np.degrees(np.arctan(
        np.tan(np.radians(horizontal_deg) / 2.0) * (480.0 / 640.0)))
    self.camera_horizontal_fov_pub.publish(Float32(data = self.CAMERA_HORIZONTAL_FOV_DEG))
    self.camera_vertical_fov_pub.publish(Float32(data = self.CAMERA_VERTICAL_FOV_DEG))

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

  def pushDirtyDimensions(self):
    # Called right before a Deploy (see runLaunch). Pushing edited geometry
    # to the sim machine went through the old launcher's VM mailbox, which
    # the Gazebo service replaces; its equivalent (writing into the service's
    # ROBOT_CONFIGS/ENVIRONMENT_CONFIGS folders in NEPI storage) is a later
    # pass. Until then, edits stay on the device, flagged dirty, so nothing
    # is lost when that lands.
    dirty_roles = [role for role in DIMENSION_ROLES if self.dimensions_dirty.get(role, False)]
    if dirty_roles:
      self.msg_if.pub_warn("Edited " + " and ".join(dirty_roles) + " dimensions were not sent "
                           "to the sim machine -- the Gazebo service doesn't support pushing "
                           "dimensions yet", throttle_s = 30.0)

  #**********************
  # Named dimensions configs -- see the constructor's own "Named dimensions
  # configs" comment for the design. Storage is one small YAML file per
  # named config, under a per-role subdirectory of DIMENSIONS_STORAGE_DIR --
  # separate from (and never touched by) dimensionsYamlStoragePath's own
  # single "currently active" file above, which these actions read from/
  # write to as a side effect of "select"/"save" meaning "make this the
  # active one", exactly as clicking Save Dimensions already did.

  def dimensionConfigsDir(self, role):
    return os.path.join(DIMENSIONS_STORAGE_DIR, role + '_configs')

  def sanitizeDimensionConfigName(self, name):
    # Filesystem-safe and short enough to be a sane filename -- letters,
    # digits, spaces, underscore, hyphen only; everything else collapses to
    # '_'. Matters because this name becomes a literal file on disk (see
    # dimensionConfigPath), unlike a robot_configs key, which is just a
    # dict key with no filesystem exposure.
    name = str(name).strip()
    safe = re.sub(r'[^A-Za-z0-9 _-]', '_', name)[:64]
    return safe if safe else 'Unnamed'

  def dimensionConfigPath(self, role, name):
    return os.path.join(self.dimensionConfigsDir(role), self.sanitizeDimensionConfigName(name) + '.yaml')

  def resolveEnvironmentModelFromYaml(self, yaml_text):
    # Reads ENVIRONMENT_MODEL_FIELD_KEY out of a config's own fields --
    # falls back to the one fixed model 'environment' always meant before
    # per-config model routing existed, for a config saved before that key
    # existed at all (a migrated legacy config, or a hand-edited file).
    try:
      fields = yaml.safe_load(yaml_text) if yaml_text else None
    except yaml.YAMLError:
      fields = None
    if isinstance(fields, dict) and ENVIRONMENT_MODEL_FIELD_KEY in fields:
      return str(fields[ENVIRONMENT_MODEL_FIELD_KEY])
    return ENVIRONMENT_LEGACY_MODEL_FALLBACK

  def resolveMatchingDimensionConfigName(self, role):
    # Finds which named config (if any) currently has EXACTLY the same
    # fields as the "active" single-file store -- used at startup (and
    # after an edit invalidates whatever was selected before it) so
    # self.selected_dimension_config never just ASSUMES the fallback name,
    # which would be wrong wherever the active file was left mid-edit, or
    # was last set by selecting some OTHER config. Returns '' when nothing
    # matches (an unsaved edit) -- callers decide their own fallback.
    try:
      active_fields = yaml.safe_load(self.readStoredDimensionsYaml(role) or '')
    except yaml.YAMLError:
      active_fields = None
    if not isinstance(active_fields, dict):
      return ''
    for name in self.listDimensionConfigs(role):
      try:
        with open(self.dimensionConfigPath(role, name), 'r') as f:
          named_fields = yaml.safe_load(f.read())
      except Exception:
        continue
      if named_fields == active_fields:
        return name
    return ''

  def ensureBuiltinDimensionConfigs(self, role):
    # Seeds every built-in name in PROTECTED_DIMENSION_CONFIG_NAMES[role]
    # that doesn't already exist on disk. The one matching
    # FALLBACK_DIMENSION_CONFIG_NAME[role] seeds from whatever the existing
    # single-file store (or a legacy Default.yaml, migrated and removed
    # here) already has, so upgrading an existing deployment never silently
    # resets an operator's current setup to a blank slate; every other
    # built-in seeds fresh -- a genuinely valid, if unfilled, config
    # (generate_model_sdf.py fills in its own defaults for anything a
    # config doesn't set).
    os.makedirs(self.dimensionConfigsDir(role), exist_ok = True)
    legacy_path = self.dimensionConfigPath(role, LEGACY_DEFAULT_DIMENSION_CONFIG_NAME)
    legacy_fields = None
    if os.path.exists(legacy_path):
      try:
        with open(legacy_path, 'r') as f:
          legacy_fields = yaml.safe_load(f.read())
      except Exception as e:
        self.msg_if.pub_warn("Failed to read legacy Default " + role + " dimensions config: " + str(e))
      try:
        os.remove(legacy_path)
      except Exception as e:
        self.msg_if.pub_warn("Failed to remove legacy Default " + role + " dimensions config: " + str(e))

    if role == 'robot':
      builtins = {ROBOT_BUILTIN_DIMENSION_CONFIG_NAME: {}}
    else:
      builtins = ENVIRONMENT_BUILTIN_DIMENSION_CONFIGS
    fallback_name = FALLBACK_DIMENSION_CONFIG_NAME[role]
    for name, extra_fields in builtins.items():
      path = self.dimensionConfigPath(role, name)
      if os.path.exists(path):
        continue
      if name == fallback_name:
        if isinstance(legacy_fields, dict):
          base = dict(legacy_fields)
        else:
          try:
            base = yaml.safe_load(self.readStoredDimensionsYaml(role) or '')
          except yaml.YAMLError:
            base = None
          base = dict(base) if isinstance(base, dict) else {}
      else:
        base = {}
      base.update(extra_fields)
      try:
        with open(path, 'w') as f:
          yaml.safe_dump(base, f, default_flow_style = False, sort_keys = False)
      except Exception as e:
        self.msg_if.pub_warn("Failed to seed " + role + " dimensions config '" + name + "': " + str(e))

  def listDimensionConfigs(self, role):
    self.ensureBuiltinDimensionConfigs(role)
    names = []
    try:
      for filename in os.listdir(self.dimensionConfigsDir(role)):
        if filename.endswith('.yaml'):
          names.append(filename[:-len('.yaml')])
    except Exception as e:
      self.msg_if.pub_warn("Failed to list " + role + " dimensions configs: " + str(e))
    # Built-ins always first, in DIMENSION_CONFIG_BUILTIN_ORDER's order;
    # everything else (custom saved configs) sorts alphabetically after.
    builtin_order = DIMENSION_CONFIG_BUILTIN_ORDER.get(role, [])
    def sortKey(n):
      return (0, builtin_order.index(n)) if n in builtin_order else (1, n.lower())
    names.sort(key = sortKey)
    return names

  def publishAvailableDimensionConfigs(self, role):
    pub = self.dimension_config_names_pubs.get(role)
    if pub is not None:
      pub.publish(String(data = json.dumps(self.listDimensionConfigs(role))))

  def publishSelectedDimensionConfig(self, role):
    pub = self.dimension_config_selected_pubs.get(role)
    if pub is not None:
      pub.publish(String(data = self.selected_dimension_config.get(role, FALLBACK_DIMENSION_CONFIG_NAME.get(role, ''))))
    if role == 'environment' and self.environment_dimensions_model_pub is not None:
      self.environment_dimensions_model_pub.publish(String(data =
          self.dimension_config_model.get(role, ENVIRONMENT_LEGACY_MODEL_FALLBACK)))

  def applyDimensionConfigByName(self, role, name):
    # Shared by selectDimensionConfigCb (an explicit operator pick) and
    # deleteDimensionConfigCb (falling back to FALLBACK_DIMENSION_CONFIG_NAME
    # after removing whatever was selected) -- both mean "make this saved
    # config the active one," which is exactly writeStoredDimensionsYaml's
    # own job, the same file pushDirtyDimensions/generate_model_sdf.py read.
    path = self.dimensionConfigPath(role, name)
    if not os.path.exists(path):
      self.msg_if.pub_warn(role + " dimensions config '" + name + "' not found, ignoring")
      return False
    try:
      with open(path, 'r') as f:
        yaml_text = f.read()
    except Exception as e:
      self.msg_if.pub_warn("Failed to read " + role + " dimensions config '" + name + "': " + str(e))
      return False
    if role == 'environment':
      self.dimension_config_model[role] = self.resolveEnvironmentModelFromYaml(yaml_text)
    self.writeStoredDimensionsYaml(role, yaml_text)
    self.clearStoredSdfOverride(role)
    self.markDimensionsDirty(role)
    if role == 'robot':
      self.updateCameraFovFromRobotDimensions()
    self.selected_dimension_config[role] = self.sanitizeDimensionConfigName(name)
    self.publishSelectedDimensionConfig(role)
    # Echoes to the same latched topic applyDimensionsYaml already listens
    # to on the RUI side, so the editable fields (and the preview diagram)
    # refresh to match the newly-selected config without any new RUI-side
    # listener needed for this half of the flow.
    pub = self.dimensions_yaml_pubs.get(role)
    if pub is not None:
      pub.publish(String(data = yaml_text))
    return True

  def getEnvironmentDimensionsConfigCb(self, msg):
    # Read-only counterpart of applyDimensionConfigByName -- same file
    # lookup (dimensionConfigPath), but returns the text without touching
    # selected_dimension_config, dimension_config_model, dimensions_dirty,
    # or the stored-active-yaml file at all. See this subscriber's own
    # registration comment for why this needed to exist as its own
    # request/response pair rather than reusing select_environment_
    # dimensions_config.
    name = str(msg.data).strip()
    if not name:
      return
    path = self.dimensionConfigPath('environment', name)
    if not os.path.exists(path):
      self.msg_if.pub_warn("Environment dimensions config '" + name + "' not found, cannot preview it")
      self.environment_dimensions_config_yaml_preview_pub.publish(String(data =
          "# Environment dimensions config '" + name + "' not found"))
      return
    try:
      with open(path, 'r') as f:
        yaml_text = f.read()
    except Exception as e:
      self.msg_if.pub_warn("Failed to read environment dimensions config '" + name + "' for preview: " + str(e))
      return
    self.environment_dimensions_config_yaml_preview_pub.publish(String(data = yaml_text))

  def selectRobotDimensionsConfigCb(self, msg):
    self.selectDimensionConfigCb('robot', msg)

  def selectEnvironmentDimensionsConfigCb(self, msg):
    self.selectDimensionConfigCb('environment', msg)

  def selectDimensionConfigCb(self, role, msg):
    name = str(msg.data).strip()
    if not name:
      return
    if self.applyDimensionConfigByName(role, name):
      self.msg_if.pub_info("Selected " + role + " dimensions config: " + name)

  def saveRobotDimensionsConfigCb(self, msg):
    self.saveDimensionConfigCb('robot', msg)

  def saveEnvironmentDimensionsConfigCb(self, msg):
    self.saveDimensionConfigCb('environment', msg)

  def saveDimensionConfigCb(self, role, msg):
    # Payload is JSON {"name": ..., "yaml": ...} rather than a bare string --
    # this is the one dimensions action that needs two pieces of data in one
    # message (matching the same shape sim_connector's own launch_simulator
    # payload uses for target_key+robot_config, for the same reason: sending
    # them as two separate messages would race). "Save" always also makes
    # the new config the active one -- there is no separate "save without
    # using" action, matching how Save Dimensions already behaves today.
    try:
      payload = json.loads(str(msg.data))
    except (ValueError, TypeError) as e:
      self.msg_if.pub_warn("Save " + role + " dimensions config payload is not valid JSON: " + str(e))
      return
    if not isinstance(payload, dict):
      self.msg_if.pub_warn("Save " + role + " dimensions config payload must be a JSON object")
      return
    name = str(payload.get('name', '')).strip()
    if not name:
      self.msg_if.pub_warn("Cannot save a " + role + " dimensions config with an empty name")
      return
    if self.sanitizeDimensionConfigName(name) in PROTECTED_DIMENSION_CONFIG_NAMES.get(role, set()):
      self.msg_if.pub_warn("Cannot save over the built-in " + role + " dimensions config '" + name + "'")
      return
    try:
      fields = yaml.safe_load(str(payload.get('yaml', '')))
    except yaml.YAMLError as e:
      self.msg_if.pub_warn("Cannot save " + role + " dimensions config '" + name + "': not valid YAML: " + str(e))
      return
    if not isinstance(fields, dict):
      self.msg_if.pub_warn("Cannot save " + role + " dimensions config '" + name + "': must be a YAML mapping")
      return
    if role == 'robot':
      viable, reason = self.checkRobotDimensionsViable(fields)
      if not viable:
        self.msg_if.pub_warn("Cannot save robot dimensions config '" + name + "': not viable -- " + reason)
        return
    if role == 'environment':
      # A save always carries forward whichever model was active for the
      # config being edited -- the editable fields never include this
      # reserved key themselves (see ENVIRONMENT_MODEL_FIELD_KEY's own
      # comment), so without this a "Save As" derived from Obstacle Course
      # would silently save a config with no model to push at all.
      fields[ENVIRONMENT_MODEL_FIELD_KEY] = self.dimension_config_model.get(
          role, ENVIRONMENT_LEGACY_MODEL_FALLBACK)
    try:
      clean_yaml = yaml.safe_dump(fields, default_flow_style = False, sort_keys = False)
      os.makedirs(self.dimensionConfigsDir(role), exist_ok = True)
      with open(self.dimensionConfigPath(role, name), 'w') as f:
        f.write(clean_yaml)
    except Exception as e:
      self.msg_if.pub_warn("Failed to save " + role + " dimensions config '" + name + "': " + str(e))
      return
    if role == 'robot':
      self.linkRobotConfigToDimensions(name)
    self.publishAvailableDimensionConfigs(role)
    self.applyDimensionConfigByName(role, name)
    self.msg_if.pub_info("Saved " + role + " dimensions config: " + name)

  def linkRobotConfigToDimensions(self, name):
    # A "robot" dimensions config (chassis/wheel geometry) and a
    # robot_configs entry (capability profile -- wheel_count, goto flags,
    # setup_actions, etc.) are two independent axes with two independent
    # key spaces (sanitizeRobotConfigKey prefixes "custom_" and lowercases;
    # sanitizeDimensionConfigName does neither -- see each one's own
    # comment) -- merged in the RUI purely by matching DISPLAY NAME
    # (renderRobotConfigAndDimensionsButtons). Reported live 2026-09-14:
    # saving a new custom robot ("diyrover1") only ever touched the
    # dimensions axis, so it never appeared in available_robot_configs (the
    # plain Robot Config dropdown, which only lists capability entries),
    # its capability YAML view kept showing whichever capability profile
    # was last actually selected (stale display_name), and the merged
    # button row's selected-highlight logic -- which keys off the
    # capability axis matching too -- had nothing of its own to match.
    #
    # Only creates an entry if this exact display_name has no capability
    # counterpart yet -- re-saving an already-linked custom config (further
    # edits under the same name) must not reset its capability profile back
    # to whatever the CURRENTLY selected one happens to be right now.
    for entry in self.robot_configs.values():
      if isinstance(entry, dict) and entry.get('display_name') == name:
        return
    # Clones the CURRENTLY selected capability profile as the new entry's
    # starting point -- matches the natural workflow (pick "4-Wheel Rover",
    # customize its dimensions, Save As a new name): the capability side
    # (wheel count, which goto modes, setup actions) carries over unchanged,
    # only display_name and the physical dimensions differ.
    source_key = self.selected_robot_config
    source_entry = self.robot_configs.get(source_key, dict())
    entry = copy.deepcopy(source_entry) if isinstance(source_entry, dict) else dict()
    entry.pop('hidden_from_selector', None)
    key = self.persistRobotConfigEntry(name, entry)
    if key is not None:
      # Also makes it the selected capability config (setSelectedRobotConfig
      # re-applies this same dimensions config, harmlessly redundant with
      # saveDimensionConfigCb's own applyDimensionConfigByName call right
      # after this returns) -- without this, the capability axis kept
      # showing whichever profile was selected before the save, so the
      # merged button row's selected-highlight never matched the config
      # that was actually just created and applied.
      self.setSelectedRobotConfig(key)
      self.msg_if.pub_info("Linked new capability config '" + name + "' (from '" +
                           source_key + "') to its dimensions config")

  def deleteRobotDimensionsConfigCb(self, msg):
    self.deleteDimensionConfigCb('robot', msg)

  def deleteEnvironmentDimensionsConfigCb(self, msg):
    self.deleteDimensionConfigCb('environment', msg)

  def deleteDimensionConfigCb(self, role, msg):
    name = str(msg.data).strip()
    if not name:
      return
    if self.sanitizeDimensionConfigName(name) in PROTECTED_DIMENSION_CONFIG_NAMES.get(role, set()):
      self.msg_if.pub_warn("Cannot delete the built-in " + role + " dimensions config '" + name + "'")
      return
    # Environment only -- robot has no comparable "currently deployed by
    # name" concept (see deployed_environment_config_name's own comment).
    # Server-side enforcement of the same rule the RUI's own delete button
    # checks first, so a deploy started elsewhere (another browser tab, a
    # raw publish) can't out-race a client that hasn't heard about it yet.
    if (role == 'environment' and self.deployed_environment_config_name != ''
        and self.sanitizeDimensionConfigName(name)
            == self.sanitizeDimensionConfigName(self.deployed_environment_config_name)):
      self.msg_if.pub_warn(role + " dimensions config '" + name + "' is currently deployed "
                           "and can't be deleted while it's live -- deploy something else first")
      return
    path = self.dimensionConfigPath(role, name)
    if not os.path.exists(path):
      self.msg_if.pub_warn(role + " dimensions config '" + name + "' not found, cannot delete")
      return
    try:
      os.remove(path)
    except Exception as e:
      self.msg_if.pub_warn("Failed to delete " + role + " dimensions config '" + name + "': " + str(e))
      return
    self.publishAvailableDimensionConfigs(role)
    if self.selected_dimension_config.get(role) == self.sanitizeDimensionConfigName(name):
      self.applyDimensionConfigByName(role, FALLBACK_DIMENSION_CONFIG_NAME[role])
    self.msg_if.pub_info("Deleted " + role + " dimensions config: " + name)
    # Cascade (robot only): a linked capability config sharing this exact
    # display_name -- created together by linkRobotConfigToDimensions -- is
    # removed too, so neither axis can orphan the other into an undeletable
    # ghost entry (see deleteRobotConfigCb's own cascade for the reverse
    # direction, and its comment for the reported symptom this fixes).
    if role == 'robot':
      linked_key = self.findRobotConfigKeyByDisplayName(name)
      if linked_key is not None:
        if self.deleteRobotConfigByKey(linked_key):
          self.msg_if.pub_info("Deleted linked robot config: " + linked_key)

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
    if role == 'robot':
      # Checked against the RESULTING geometry (currently-active fields
      # merged with this edit), not fields alone -- this callback allows a
      # partial edit (see this method's own docstring), and validating just
      # the partial dict against bare component defaults could pass or fail
      # against a hypothetical robot that was never actually requested.
      try:
        current_fields = yaml.safe_load(self.readStoredDimensionsYaml('robot') or '')
      except yaml.YAMLError:
        current_fields = None
      merged_fields = dict(current_fields) if isinstance(current_fields, dict) else {}
      merged_fields.update(fields)
      viable, reason = self.checkRobotDimensionsViable(merged_fields)
      if not viable:
        self.msg_if.pub_warn("Rejected robot dimensions update: not viable -- " + reason)
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
    # Auto-save: if a CUSTOM (non-built-in) config was selected before this
    # edit landed, write the edited fields straight back to that same named
    # file too, so incremental tweaks to a robot you created don't each
    # need a separate "Save As <same name>" click. Reported live
    # 2026-09-14: "if a custom robot is selected like diyrover1 and changes
    # are made to it, there should be an indicator every time a change is
    # made which says auto-saved to config."
    #
    # A BUILT-IN (4-Wheel Rover/Obstacle Course) must never be silently
    # overwritten this way -- same protection saveDimensionConfigCb already
    # enforces on an explicit save, just checked here too since this path
    # bypasses that method entirely. For a built-in (or nothing selected --
    # an edit made with no saved config active yet), behavior is UNCHANGED
    # from before: re-resolve from content, dropping to '' if the edit no
    # longer matches anything on disk (see this method's own long-standing
    # comment below, kept verbatim for that case).
    prev_selected = self.selected_dimension_config.get(role, '')
    is_custom = (prev_selected != ''
                 and self.sanitizeDimensionConfigName(prev_selected)
                     not in PROTECTED_DIMENSION_CONFIG_NAMES.get(role, set()))
    if is_custom:
      try:
        with open(self.dimensionConfigPath(role, prev_selected), 'w') as f:
          f.write(yaml_text)
        self.selected_dimension_config[role] = prev_selected
        pub = self.dimension_config_autosaved_pubs.get(role)
        if pub is not None:
          pub.publish(String(data = prev_selected))
        self.msg_if.pub_info("Auto-saved " + role + " dimensions edit to '" + prev_selected + "'")
      except Exception as e:
        self.msg_if.pub_warn("Failed to auto-save " + role + " dimensions config '" +
                             prev_selected + "': " + str(e))
        is_custom = False
    if not is_custom:
      # Re-resolves rather than just clearing to "unsaved" outright -- an edit
      # that happens to reproduce a saved config's fields exactly (e.g.
      # editing a value and back) should still show that config as selected.
      # This is the fix for a real report (2026-09-01): editing while a
      # BUILT-IN like "4-Wheel Rover"/"Obstacle Course" was selected kept
      # showing that name as selected even after the values no longer
      # matched what its own file holds, reading as "editing changed the
      # default" -- it never did (setDimensionsCb only ever writes the
      # ACTIVE single-file store, see this method's own docstring, never a
      # named config's own file); only the SELECTED-NAME INDICATOR was wrong.
      # Now it correctly drops to '' (nothing selected -- shown in the RUI as
      # unsaved edits) the moment the active content no longer matches
      # anything on disk, exactly the "only applies if saved and selected"
      # behavior reported missing.
      self.selected_dimension_config[role] = self.resolveMatchingDimensionConfigName(role)
    self.publishSelectedDimensionConfig(role)
    if role == 'robot':
      self.updateCameraFovFromRobotDimensions()
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

    # Robot Link tracks self.selected_simulator automatically (no separate
    # "select sim" control of its own -- there is already exactly one sim
    # selector in this app). Also drop a selected physical robot that has
    # gone away, same reasoning as the simulator-selection check above --
    # an active link whose physical target vanished is disabled outright
    # rather than left running against a dead namespace.
    self.robot_link.select_sim_robot(self.selected_simulator)
    if self.robot_link.physical_namespace != "":
      available_physical, _names = self.getAvailablePhysicalRobots()
      if self.robot_link.physical_namespace not in available_physical:
        self.msg_if.pub_warn("Linked physical robot " + self.robot_link.physical_namespace +
                             " is no longer available, clearing link")
        self.robot_link.select_physical_robot("")
        self.publishRobotLinkStatus()

    self.updateCommonViewSubscriptions()
    self.updateNavPoseMirrorSubscription()

  def updateNavPoseMirrorSubscription(self):
    # Re-points the NavPose mirror at whichever real <device>/npx/navpose
    # topic the currently selected_simulator's own RBX driver publishes --
    # see self.navpose_mirror_sub's own comment in __init__ for the full
    # reasoning. node_namespace derivation matches
    # updateCommonViewSubscriptions exactly (same selected_simulator value,
    # same "strip /rbx" relationship to the driver's own node namespace).
    node_namespace = self.selected_simulator.split('/rbx')[0] if self.selected_simulator else ""
    source_topic = ""
    if node_namespace != "":
      source_topic = nepi_sdk.find_topic(node_namespace + "/npx/navpose", exact = True)
    if source_topic == self.navpose_mirror_source_topic:
      return  # Already pointed at the right thing (including both empty).
    if self.navpose_mirror_sub is not None:
      try:
        self.navpose_mirror_sub.unregister()
      except Exception:
        pass
      self.navpose_mirror_sub = None
    self.navpose_mirror_source_topic = source_topic
    if source_topic == "":
      # The underlying RBX driver's own NavPose topic just disappeared
      # (SITL/Gazebo stopped, or nothing selected) -- leave navpose_dict at
      # whatever it last reported rather than snapping it back to blank.
      return
    self.navpose_mirror_sub = nepi_sdk.create_subscriber(
        source_topic, NavPose, self.navPoseMirrorCb, queue_size = 1)
    self.msg_if.pub_info("NavPose mirror now following: " + source_topic)

  def navPoseMirrorCb(self, msg):
    # Copies every has_*/value field this message carries into
    # self.navpose_dict -- see updateNavPoseMirrorSubscription's own comment.
    nd = self.navpose_dict
    if msg.has_location:
      nd['has_location'] = True
      nd['time_location'] = msg.time_location
      nd['latitude'] = msg.latitude
      nd['longitude'] = msg.longitude
      nd['location_m_per_sec'] = msg.location_m_per_sec
    if msg.has_heading:
      nd['has_heading'] = True
      nd['time_heading'] = msg.time_heading
      nd['heading_deg'] = msg.heading_deg
      nd['heading_m_per_sec'] = msg.heading_m_per_sec
    if msg.has_position:
      nd['has_position'] = True
      nd['time_position'] = msg.time_position
      nd['x_m'] = msg.x_m
      nd['y_m'] = msg.y_m
      nd['z_m'] = msg.z_m
    if msg.has_velocity:
      nd['has_velocity'] = True
      nd['time_velocity'] = msg.time_velocity
      nd['x_m_per_sec'] = msg.x_m_per_sec
      nd['y_m_per_sec'] = msg.y_m_per_sec
      nd['z_m_per_sec'] = msg.z_m_per_sec
    if msg.has_orientation:
      nd['has_orientation'] = True
      nd['time_orientation'] = msg.time_orientation
      nd['roll_deg'] = msg.roll_deg
      nd['pitch_deg'] = msg.pitch_deg
      nd['yaw_deg'] = msg.yaw_deg
      nd['roll_deg_per_sec'] = msg.roll_deg_per_sec
      nd['pitch_deg_per_sec'] = msg.pitch_deg_per_sec
      nd['yaw_deg_per_sec'] = msg.yaw_deg_per_sec
    if msg.has_altitude:
      nd['has_altitude'] = True
      nd['time_altitude'] = msg.time_altitude
      nd['altitude_m'] = msg.altitude_m
      nd['geoid_height_meters'] = msg.geoid_height_meters
      nd['altitude_m_per_sec'] = msg.altitude_m_per_sec
    if msg.has_depth:
      nd['has_depth'] = True
      nd['time_depth'] = msg.time_depth
      nd['depth_m'] = msg.depth_m
      nd['depth_m_per_sec'] = msg.depth_m_per_sec

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
        self.publishBlankCommonViewFrame(pub)
      return
    new_sub = nepi_sdk.create_subscriber(source_topic, Image, self.commonViewImageCb,
                                        queue_size = 1, callback_args = (pub,))
    self.mirror_subs[which] = new_sub

  def publishBlankCommonViewFrame(self, pub):
    try:
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
          time = nepi_utils.get_time(),
          # Cached so forwardToSelectedSimulator's own readiness checks (see
          # isBridgeConnected/autonomousControlsReady) can answer for the
          # selected RBX device without a fresh round-trip -- this status
          # message is the only thing this app ever receives from it.
          ready = msg.ready,
          manual_control_mode_ready = msg.manual_control_mode_ready,
          autonomous_control_mode_ready = msg.autonomous_control_mode_ready)

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

  def getAvailablePhysicalRobots(self):
    # Mirror image of getAvailableSimulators() over the exact same
    # self.sim_device_info scan (simDiscoveryCb subscribes to every
    # DeviceRBXStatus-publishing device in the ROS graph, sim or not) --
    # everything that ISN'T a simulator is a candidate physical robot to
    # link a sim to.
    namespaces = []
    names = []
    now = nepi_utils.get_time()
    with self.sim_scan_lock:
      for topic in sorted(self.sim_device_info.keys()):
        entry = self.sim_device_info[topic]
        if entry['data_source_description'] == SIM_SOURCE_DESCRIPTION:
          continue
        if (now - entry['time']) > SIM_DEVICE_STALE_SEC:
          continue
        namespace = topic
        if namespace.endswith('/status'):
          namespace = namespace[:-len('/status')]
        namespaces.append(namespace)
        names.append(entry['device_name'] if entry['device_name'] else namespace)
    return namespaces, names

  def selectPhysicalRobotCb(self, msg):
    namespace = str(msg.data)
    if namespace in ("", "None"):
      self.robot_link.select_physical_robot("")
      self.msg_if.pub_info("Cleared physical robot selection")
      self.publishRobotLinkStatus()
      return
    available, _names = self.getAvailablePhysicalRobots()
    if namespace not in available:
      self.msg_if.pub_warn("Physical robot '" + namespace + "' is not currently available, ignoring")
      return
    self.robot_link.select_physical_robot(namespace)
    self.msg_if.pub_info("Selected physical robot for linking: " + namespace)
    self.publishRobotLinkStatus()

  def setPropsOffAcknowledgedCb(self, msg):
    self.robot_link.acknowledge_props_off(bool(msg.data))
    self.publishRobotLinkStatus()

  def setRobotLinkEnabledCb(self, msg):
    self.robot_link.set_enabled(bool(msg.data))
    self.publishRobotLinkStatus()

  def publishRobotLinkStatus(self):
    if self.robot_link_status_pub is None:
      return
    status = self.robot_link.get_status_dict()
    available_physical, available_physical_names = self.getAvailablePhysicalRobots()
    status['available_physical_robots'] = available_physical
    status['available_physical_robot_names'] = available_physical_names
    status['selected_sim'] = self.selected_simulator
    try:
      self.robot_link_status_pub.publish(String(data = json.dumps(status)))
    except Exception as e:
      self.msg_if.pub_warn("Failed to publish robot link status: " + str(e), throttle_s = 5.0)

  #**********************
  # Simulator launch. A convenience trigger over the existing passive flow,
  # not a parallel path: on success this calls the same
  # setSelectedRobotConfig already used by the robot-config selector above,
  # so the rest of the app behaves exactly as if an operator had started the
  # simulator by hand and picked that config themselves. Deliberately does
  # NOT touch setSelectedSimulator/selected_simulator -- that selector picks
  # among other simulator-*capable NEPI devices* discovered on the ROS graph
  # (simDiscoveryCb above), a different axis entirely from which simulator
  # *software* a simulator service starts on the sim machine.
  #
  # A start or stop blocks for real seconds while the service carries it out
  # (see nepi_sim_service.SimService.wait_for_request) -- run on a background thread
  # so the bridge server thread and ROS callback dispatch are never held up
  # by one. launcher_lock only guards against two launch/stop requests
  # racing each other.

  def setLauncherError(self, error, fallback_commands = ''):
    """Single place that sets launcher_last_error together with the
    copy-paste commands shown alongside it, so a fallback from a PREVIOUS
    failure never lingers onto an unrelated new one."""
    self.launcher_last_error = str(error)
    self.launcher_fallback_commands = fallback_commands

  def failLauncher(self, error, fallback_commands = ''):
    self.launcher_state = 'failed'
    self.setLauncherError(error, fallback_commands)
    self.publishLauncherStatus()

  def parseLaunchPayload(self, msg):
    """Parses a sim/redeploy_simulator-family String message into
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
    that way -- resolveLaunchRobotConfig then falls back to
    self.selected_robot_config itself, same as before this fix."""
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

  def startLauncherThread(self, run_function, *args):
    if not any(self.service_config_found.values()):
      self.msg_if.pub_warn("No simulator service has been set up on the sim machine yet (no " +
                           " or ".join(SIM_TARGETS[t]['service'].config_file for t in SIM_TARGETS) +
                           "), ignoring request")
      return
    with self.launcher_lock:
      if self.launcher_thread is not None and self.launcher_thread.is_alive():
        self.msg_if.pub_warn("A launch/stop is already in progress, ignoring")
        return
      self.launcher_thread = threading.Thread(target = run_function, args = args)
      self.launcher_thread.daemon = True
      self.launcher_thread.start()

  def stopSimulatorCb(self, msg):
    # Stops whichever simulator its service reports running -- the services,
    # not this node's memory, know what's running, so this works the same
    # after an app restart as it does for a simulation this app just started.
    self.startLauncherThread(self.runStop)

  def redeploySimulatorCb(self, msg):
    # Deploy and "Use Open Sim" both publish this: a clean restart (stop,
    # then start fresh) rather than reusing what's up.
    target_key, robot_config = self.parseLaunchPayload(msg)
    self.startLauncherThread(self.runRedeploy, target_key, robot_config)

  def readServiceState(self, target_key):
    """The STATE the target's service reports right now, '' when unreadable."""
    service = SIM_TARGETS[target_key]['service']
    config_dict = service.load_config()
    return nepi_sim_service.get_config_str(config_dict, service.key('STATE')) if config_dict else ''

  def runningSimTarget(self):
    """The simulator a service reports starting or running, '' when none is."""
    for target_key in SIM_TARGETS:
      if self.readServiceState(target_key) in ('running', 'starting'):
        return target_key
    return ''

  def runRedeploy(self, target_key, robot_config=None):
    if target_key not in SIM_TARGETS:
      self.failLauncher("Unknown simulator '" + str(target_key) + "'")
      return
    # Deploy carries its own target; make the lists match it before the
    # robot is resolved against them.
    if target_key != self.selected_sim_target:
      self.setSelectedSimTarget(target_key)
    # Checked BEFORE stopping anything, so a Deploy that can't go ahead
    # leaves whatever is running alone. robot_config itself is passed through
    # unresolved, so runLaunch still re-reads the selection once it is up.
    _resolved, _robot_folder, error = self.resolveLaunchRobotConfig(target_key, robot_config)
    if error:
      self.failLauncher(error)
      return
    # One simulator at a time: both drive the same rover driver over the same
    # bridge port, so any OTHER simulator that is up is stopped first.
    for other_key in SIM_TARGETS:
      if other_key == target_key or self.readServiceState(other_key) not in ('running', 'starting'):
        continue
      if not self.service_alive[other_key]:
        self.msg_if.pub_warn(SIM_TARGETS[other_key]['service'].name + " reports running but its "
                             "service isn't responding, so it can't be stopped from here")
        continue
      self.msg_if.pub_info("Stopping " + SIM_TARGETS[other_key]['service'].name + " before deploying " +
                           SIM_TARGETS[target_key]['service'].name)
      self.runStop(other_key)
      if self.launcher_state == 'failed':
        return  # runStop already published the failure; nothing more to do
    if self.readServiceState(target_key) in ('running', 'starting'):
      self.runStop(target_key)
      if self.launcher_state == 'failed':
        return
    self.runLaunch(target_key, robot_config)

  def publishSecondaryLauncherStatus(self):
    self.secondary_launcher_status_pub.publish(String(data = json.dumps({
        'active_targets': [t for t, s in self.secondary_launch_states.items() if s == 'running'],
        'states': dict(self.secondary_launch_states),
    })))

  def launchNewSimulatorCb(self, msg):
    # "New Sim" -- a second, independent instance alongside the primary one.
    # Each simulator service runs exactly one simulation (one config file,
    # one tracked process), so there is nowhere for a second one to go;
    # reported through the RUI's own "Other Running Instances" list (whose
    # Stop clears it) rather than silently dropped.
    target_key, _robot_config = self.parseLaunchPayload(msg)
    if not target_key or target_key == 'None':
      return
    self.secondary_launch_states[target_key] = 'failed: each simulator service runs a single instance'
    self.publishSecondaryLauncherStatus()

  def stopNewSimulatorCb(self, msg):
    target_key = str(msg.data).strip()
    if target_key in self.secondary_launch_states:
      del self.secondary_launch_states[target_key]
      self.publishSecondaryLauncherStatus()

  #**********************
  # Simulator, Robot and Environment Config -- what a Deploy asks a simulator
  # service to run. Robots and worlds come from the SELECTED simulator's share
  # (NEPI storage databases/sims/<simulator>), the contract described in its
  # ROBOT_CONFIGS/README.md: a robot is a ROBOT_CONFIGS folder, an environment
  # is a world file in ENVIRONMENT_CONFIGS, and dropping either on the share is
  # all it takes to offer it. Selections are only applied on the next Deploy.

  def selectLaunchTargetCb(self, msg):
    target_key = str(msg.data).strip()
    if target_key not in SIM_TARGETS:
      if target_key not in ('', 'None'):
        self.msg_if.pub_warn("Simulator '" + target_key + "' is not one of " +
                             str(list(SIM_TARGETS.keys())) + ", ignoring")
      return
    if target_key != self.selected_sim_target:
      self.setSelectedSimTarget(target_key)

  def setSelectedSimTarget(self, target_key):
    """Switches the Robot Config and Environment Config lists to target_key's
    share. The environment goes back to that simulator's default world (world
    files are per simulator), and a robot config that only the old share had
    falls back to the new one's default robot."""
    self.selected_sim_target = target_key
    info = SIM_TARGETS[target_key]
    self.selected_environment_config = info['default_environment']
    self.refreshShareRobotConfigs()
    self.refreshEnvironmentConfigs(force_publish = True)
    if self.selected_robot_config not in self.robot_configs:
      default_robot = info['default_robot']
      self.setSelectedRobotConfig(default_robot if default_robot in self.robot_configs
                                  else FACTORY_ROBOT_CONFIG_NAME)
    self.publishLauncherStatus()
    self.msg_if.pub_info("Selected simulator: " + info['service'].name)

  def refreshShareRobotConfigs(self):
    """Folds the selected simulator's ROBOT_CONFIGS folders into
    self.robot_configs:

    - A folder a device profile claims through SHARE_ROBOT_FOLDER_FIELD
      (4-Wheel Rover -> generic_rover) is listed once, under the profile's key
      and name, with the folder's capability file merged over the profile's
      own fields -- rebuilt from the untouched profile each time, so a field
      removed from the folder's file stops applying.
    - Any other folder becomes a robot config of its own, keyed by folder
      name, named by its display_name (else the folder name).
    - A folder gone from the share takes its robot config with it, and so
      does every folder of the previous simulator when the selection changes.

    Returns True when the offered set changed."""
    share = SIM_TARGETS[self.selected_sim_target]['service'].list_robot_configs()
    changed = False
    for key, base in self.claimed_profile_bases.items():
      if key in self.robot_configs:
        self.robot_configs[key] = copy.deepcopy(base)
    claims = dict()
    for key, entry in self.robot_configs.items():
      if key in self.share_robot_config_keys or not isinstance(entry, dict):
        continue
      folder = entry.get(SHARE_ROBOT_FOLDER_FIELD, '')
      if folder:
        claims[folder] = key
    claimed_bases = dict()
    share_keys = set()
    for folder, capabilities in share.items():
      if folder in claims:
        key = claims[folder]
        base = copy.deepcopy(self.robot_configs[key])
        merged = copy.deepcopy(base)
        merged.update({k: v for k, v in capabilities.items() if k != 'display_name'})
        self.robot_configs[key] = merged
        claimed_bases[key] = base
        continue
      if folder in self.robot_configs and folder not in self.share_robot_config_keys:
        self.msg_if.pub_warn("Sim share robot '" + folder + "' has the same name as a device robot "
                             "config -- not offering it", throttle_s = 60.0)
        continue
      entry = copy.deepcopy(capabilities)
      entry['display_name'] = str(capabilities.get('display_name', '') or folder)
      entry[SHARE_ROBOT_FOLDER_FIELD] = folder
      if self.robot_configs.get(folder) != entry:
        changed = True
      self.robot_configs[folder] = entry
      share_keys.add(folder)
    for key in self.share_robot_config_keys - share_keys:
      self.robot_configs.pop(key, None)
      changed = True
    self.share_robot_config_keys = share_keys
    self.claimed_profile_bases = claimed_bases
    return changed

  def refreshEnvironmentConfigs(self, force_publish = False):
    names = SIM_TARGETS[self.selected_sim_target]['service'].list_environment_configs()
    if names != self.environment_configs or force_publish:
      self.environment_configs = names
      self.environment_config_names_pub.publish(String(data = json.dumps(names)))
      self.selected_environment_config_pub.publish(String(data = self.selected_environment_config))

  def refreshSimConfigsIfDue(self):
    # Cheap (a couple of directory listings), but no need to do it every
    # status tick -- this is only how a newly dropped folder or world shows up.
    now = time.monotonic()
    if now - self.sim_configs_refreshed_at < SIM_CONFIGS_REFRESH_SEC:
      return
    self.sim_configs_refreshed_at = now
    self.refreshShareRobotConfigs()
    self.refreshEnvironmentConfigs()

  def selectEnvironmentConfigCb(self, msg):
    name = str(msg.data).strip()
    if name not in self.environment_configs:
      self.msg_if.pub_warn("Environment config '" + name + "' is not in the " +
                           SIM_TARGETS[self.selected_sim_target]['service'].name +
                           " share's ENVIRONMENT_CONFIGS " + str(self.environment_configs) + ", ignoring")
      return
    self.selected_environment_config = name
    self.selected_environment_config_pub.publish(String(data = name))
    self.msg_if.pub_info("Selected environment config for the next Deploy: " + name)

  def resolveLaunchRobotConfig(self, target_key, robot_config=None):
    """Returns (robot_config, robot_folder, error): the robot config a Deploy
    applies once the simulation is up and the service robot folder it runs
    as, or (None, None, message) when this launch can't go ahead.

    robot_config, when given, is the value the caller's own message carried
    (see parseLaunchPayload) and takes priority over
    self.selected_robot_config -- reading only self.selected_robot_config
    raced select_robot_config's own subscriber callback (independent thread,
    no cross-topic ordering guarantee). FACTORY_ROBOT_CONFIG_NAME ('default',
    capability-empty) counts as "hasn't picked anything" rather than a real
    choice worth preserving."""
    if target_key not in SIM_TARGETS:
      return None, None, ("Unknown simulator '" + str(target_key) + "' -- can deploy " +
                          ", ".join(SIM_TARGETS[t]['service'].name for t in SIM_TARGETS))
    default_robot = SIM_TARGETS[target_key]['default_robot']
    if not robot_config:
      robot_config = self.selected_robot_config
    if not robot_config or robot_config == FACTORY_ROBOT_CONFIG_NAME:
      robot_config = default_robot
    entry = self.robot_configs.get(robot_config, dict())
    if not isinstance(entry, dict):
      entry = dict()
    # Every share robot carries its own folder name here (see
    # refreshShareRobotConfigs); only the hidden capability-empty placeholder
    # falls through to the default robot.
    robot_folder = entry.get(SHARE_ROBOT_FOLDER_FIELD, '') or default_robot
    return robot_config, robot_folder, ''

  def checkService(self, target_key):
    """Returns (error, config_dict): error is '' when target_key's service
    looks reachable -- checked before sending a request, so a dead service
    fails right away instead of after the request's full wait. config_dict
    is the config as read just now, which wait_for_request compares the
    outcome against."""
    service = SIM_TARGETS[target_key]['service']
    config_dict = service.load_config()
    if config_dict is None:
      return "Could not read " + service.config_file, None
    service.note_service_poll(config_dict)
    if not service.is_service_alive():
      return ("The " + service.name + " service isn't running on the sim machine (" +
              service.config_file + " has not been updated in over " +
              str(nepi_sim_service.SERVICE_STALE_AFTER_SEC) + "s)"), config_dict
    return '', config_dict

  def runLaunch(self, target_key, robot_config=None):
    info = SIM_TARGETS[target_key]
    service = info['service']
    explicit_robot_config = robot_config
    _resolved, robot_folder, error = self.resolveLaunchRobotConfig(target_key, explicit_robot_config)
    if error:
      self.failLauncher(error)
      return
    # Checked against the share up front, since the service answers an
    # unknown name by quietly keeping what it had -- see the post-start
    # check below.
    if robot_folder not in service.list_robot_configs():
      self.failLauncher("Robot '" + robot_folder + "' is not in the " + service.name +
                        " share's ROBOT_CONFIGS (" + service.sim_folder + ")")
      return
    environment_config = self.selected_environment_config
    if environment_config not in service.list_environment_configs():
      self.failLauncher("Environment '" + environment_config + "' is not in the " + service.name +
                        " share's ENVIRONMENT_CONFIGS (" + service.sim_folder + ")")
      return
    error, before_dict = self.checkService(target_key)
    if error:
      self.failLauncher(error, info['fallback_commands'])
      return

    self.selected_launch_target = target_key
    self.active_launch_target = target_key
    self.launcher_state = 'launching'
    self.setLauncherError('')
    self.publishLauncherStatus()

    self.pushDirtyDimensions()
    staged_values = {'UPDATE_ENVIRONMENT_CONFIG': environment_config,
                     'UPDATE_ROBOT_CONFIG': robot_folder}
    if not service.start(staged_values):
      self.failLauncher("Could not write the start request to " + service.config_file)
      return
    # The service clears START only once the start is done (for Gazebo,
    # after the robot spawn that follows a launch), so a 'running' result
    # here already has the robot in the world, or has reported why not
    # through LAST_ERROR.
    done, result = service.wait_for_request(
        'START', nepi_sim_service.START_DONE_STATES, nepi_sim_service.START_FAILED_STATES,
        SIM_START_TIMEOUT_SEC, before_dict, staged_values)
    if not done:
      service.clear_request('START')
      self.failLauncher(result, info['fallback_commands'] if not service.is_service_alive() else '')
      return
    self.service_state[target_key] = nepi_sim_service.get_config_str(result, service.key('STATE'))
    if self.service_state[target_key] == 'failed':
      self.failLauncher(nepi_sim_service.get_config_str(result, service.key('LAST_ERROR'))
                        or service.name + " failed to start")
      return
    self.launcher_state = 'running'
    # The service validates a staged name against its own local copy of the
    # share (synced when the service is started), and answers one it doesn't
    # have yet by keeping what it had and clearing its own error when the
    # start is handled -- so this comparison is the only place that mismatch
    # ever shows up. The simulation is running either way, so it is reported
    # alongside 'running', not as a failure.
    running_environment = nepi_sim_service.get_config_str(result, service.key('CURRENT_ENVIRONMENT_CONFIG'))
    running_robot = nepi_sim_service.get_config_str(result, service.key('CURRENT_ROBOT_CONFIG'))
    if (running_environment, running_robot) != (environment_config, robot_folder):
      self.setLauncherError(service.name + " started with " + running_environment + " / " +
                            running_robot + " instead of " + environment_config + " / " +
                            robot_folder + " -- the sim machine doesn't have them yet. Restart its " +
                            service.name + " service so it syncs from the share.")
    self.publishLauncherStatus()
    # Applies whatever robot config the operator already picked from the
    # existing selector (or the default when they haven't) -- this is what
    # makes "choose a model, Deploy" apply BOTH in one action without a new
    # message. Resolved again rather than reusing the pre-launch value: the
    # wait above blocks for real seconds, during which another callback
    # could legitimately have changed the selection.
    robot_config, _robot_folder, error = self.resolveLaunchRobotConfig(target_key, explicit_robot_config)
    if not error:
      self.setSelectedRobotConfig(robot_config)

  def runStop(self, target_key=None):
    # Kill passes no target: stop whichever simulator is up.
    if target_key is None:
      target_key = self.runningSimTarget() or self.selected_launch_target or self.selected_sim_target
    if target_key not in SIM_TARGETS:
      return
    info = SIM_TARGETS[target_key]
    service = info['service']
    error, before_dict = self.checkService(target_key)
    if error:
      self.failLauncher(error, info['fallback_commands'])
      return
    self.selected_launch_target = target_key
    self.launcher_state = 'stopping'
    self.setLauncherError('')
    self.publishLauncherStatus()
    if not service.stop():
      self.failLauncher("Could not write the stop request to " + service.config_file)
      return
    done, result = service.wait_for_request(
        'STOP', nepi_sim_service.STOP_DONE_STATES, nepi_sim_service.STOP_FAILED_STATES,
        SIM_STOP_TIMEOUT_SEC, before_dict)
    if not done:
      service.clear_request('STOP')
      self.failLauncher(result, info['fallback_commands'] if not service.is_service_alive() else '')
      return
    self.service_state[target_key] = nepi_sim_service.get_config_str(result, service.key('STATE'))
    self.launcher_state = 'idle'
    self.selected_launch_target = ''
    self.active_launch_target = ''
    self.publishLauncherStatus()

  def publishLauncherStatus(self):
    status = SimLauncherStatus()
    # Only simulators whose service has been set up (its config file exists)
    # are offered.
    targets = [t for t in SIM_TARGETS if self.service_config_found[t]]
    status.available_launch_targets = targets
    status.available_launch_target_names = [SIM_TARGETS[t]['service'].name for t in targets]
    # The services have no install step, so "installed" reports the one
    # thing that can be checked: the service is up and answering. 'unknown'
    # rather than 'not_installed' when it isn't -- the RUI would otherwise
    # offer an Install button with nothing behind it.
    status.available_launch_target_installed = [self.service_alive[t] for t in targets]
    status.available_launch_target_installed_check_state = [
        'installed' if self.service_alive[t] else 'unknown' for t in targets]
    # The simulator being run or acted on, else the Simulator selector's own
    # choice -- so a client that has picked nothing yet shows that one.
    status.selected_launch_target = self.selected_launch_target or self.selected_sim_target
    status.active_launch_target = self.active_launch_target
    status.active_launch_target_name = (SIM_TARGETS[self.active_launch_target]['service'].name
                                        if self.active_launch_target in SIM_TARGETS else '')
    status.launcher_state = self.launcher_state
    status.last_error = self.launcher_last_error
    status.manual_fallback_commands = (self.launcher_fallback_commands
                                       if self.launcher_state == 'failed' else '')
    self.launcher_status_pub.publish(status)

  #**********************
  # Connection health

  # RBX forwarding -- the app's own Sim Connector control surface
  # (setMotorControlRatio/gotoPosition/gotoPose/gotoLocation/goHome/goStop/
  # setHome/setSetupActionInd below) was built against a generic TCP bridge
  # protocol (port 9030, since removed), which only a dedicated bridge
  # script (e.g. the ArduPilot quadcopter one) ever dialed into.
  # Confirmed live: every simulator this app discovers (rover or
  # quadcopter -- see SIM_DEVICE_STATUS_MSG_TYPES, DeviceRBXStatus only) is
  # itself an RBX device with its own working goto/motor/home topics
  # (that's what Devices -> Robots already talks to), so a selected
  # simulator is forwarded to directly instead: no consumer ever depended
  # on the bridge path once a real RBX device was selected, and this makes
  # the Sim Connector's own panel work for the first time for a rover sim.
  # With nothing selected, a command goes nowhere and this returns False.
  def forwardToRbx(self, topic_name, msg, description):
    namespace = self.selected_simulator
    if not namespace:
      return False
    pub = self.rbx_forward_pubs.get((namespace, topic_name))
    if pub is None:
      try:
        pub = nepi_sdk.create_publisher(namespace + "/" + topic_name, type(msg), queue_size = 1)
      except Exception as e:
        self.msg_if.pub_warn("Could not create RBX forwarding publisher for " +
                             description + ": " + str(e), throttle_s = 5.0)
        return False
      self.rbx_forward_pubs[(namespace, topic_name)] = pub
      # A fresh publisher needs a moment to connect before publish() reaches
      # a subscriber -- same one-time settle this app's own node_if-based
      # publishers get implicitly from being created well before first use.
      nepi_sdk.sleep(0.3)
    try:
      pub.publish(msg)
    except Exception as e:
      self.msg_if.pub_warn("Failed to forward " + description.lower() + " to " +
                           namespace + ": " + str(e))
      return False
    return True

  def getSelectedSimDeviceInfo(self):
    topic = self.selected_simulator
    if not topic:
      return None
    if not topic.endswith("/status"):
      topic = topic + "/status"
    with self.sim_scan_lock:
      return self.sim_device_info.get(topic)

  def isBridgeConnected(self):
    # Reported as SimStatus.bridge_connected: whether the selected simulator's
    # RBX device is ready for manual control.
    info = self.getSelectedSimDeviceInfo()
    if info is not None:
      return bool(info.get("manual_control_mode_ready", False))
    return False

  def autonomousControlsReady(self):
    info = self.getSelectedSimDeviceInfo()
    if info is not None:
      return bool(info.get("autonomous_control_mode_ready", False))
    return False

  #**********************
  # SimDeviceIF callbacks -- forwarded to the selected simulator's RBX device,
  # gated at construction (and on a config change) by the selected robot config

  def setMotorControlRatio(self, motor_ind, speed_ratio):
    if motor_ind < 0 or motor_ind >= len(self.motor_ratios):
      self.msg_if.pub_warn("Motor control ignored: motor index " + str(motor_ind) + " out of range")
      return
    self.motor_ratios[motor_ind] = max(0.0, min(1.0, speed_ratio))
    self.forwardToRbx("set_motor_control",
                      MotorControl(motor_ind = motor_ind, speed_ratio = self.motor_ratios[motor_ind]),
                      "Motor control")

  def getMotorControlRatios(self):
    return self.motor_ratios

  def gotoPosition(self, msg):
    with self.goto_target_lock:
      self.goto_target = {'x_meters': msg.x_meters, 'y_meters': msg.y_meters,
                          'z_meters': msg.z_meters, 'yaw_deg': msg.yaw_deg}
    self.forwardToRbx("goto_position", msg, "Goto position")

  def gotoPose(self, attitude_enu_degs):
    # This app's own SimDeviceIF already converted whatever the caller sent
    # into ENU (see device_if_sim.py's gotoPoseCb); the target RBX device's
    # own /goto_pose does the identical NED->ENU conversion internally
    # (device_if_rbx.py's setpoint_attitude_ned), so it has to be handed
    # back NED here, or forwarding would double-apply the 90-yaw flip.
    if self.selected_simulator:
      yaw_ned_deg = nepi_nav.convert_yaw_enu2ned(attitude_enu_degs[2])
      msg = GotoPose(roll_deg = attitude_enu_degs[0], pitch_deg = attitude_enu_degs[1],
                     yaw_deg = yaw_ned_deg)
      self.forwardToRbx("goto_pose", msg, "Goto pose")

  def gotoLocation(self, msg):
    self.forwardToRbx("goto_location", msg, "Goto location")

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
    self.forwardToRbx("set_home", geo_point, "Set home")
    return True

  def goHome(self):
    return self.forwardToRbx("go_home", Empty(), "Go home")

  def goStop(self):
    with self.goto_target_lock:
      self.goto_target = None
    return self.forwardToRbx("go_stop", Empty(), "Go stop")

  def setSetupActionInd(self, action_ind):
    actions = self.profile['setup_actions']
    if action_ind < 0 or action_ind >= len(actions):
      return False
    # Forwarded by index, not name -- the profile's own action label
    # (e.g. crabrover's "RESET") is not always spelled the same as the
    # target device's own action list (e.g. a driver's "RESET_SIM"),
    # matching this codebase's own established index-based convention
    # rather than a name lookup this app has no reliable way to do (it
    # never sees the target device's own setup_action_options).
    return self.forwardToRbx("setup_action", Int32(data = action_ind), "Setup action")

  def setGoActionInd(self, action_ind):
    actions = self.profile['go_actions']
    if action_ind < 0 or action_ind >= len(actions):
      return False
    return self.forwardToRbx("go_action", Int32(data = action_ind), "Go action")

  # No simulator takes a camera view mode or an environment option yet. Both
  # stay so the capability-gated controls a robot config turns on still show.
  def setCameraViewMode(self, view_mode):
    pass

  def setEnvironmentOption(self, option, enabled = True):
    pass

  def getAvailableEnvironmentOptions(self):
    return list(self.available_environment_options)

  def getNavPoseCb(self):
    return self.navpose_dict

  #**********************
  # Status

  def statusPublishCb(self, timer):
    # SimDeviceIF owns the 2 Hz device status. This slower tick exists so the
    # two selector reports refresh promptly after a discovery scan even if the
    # device status cadence is ever retuned.
    self.refreshSimConfigsIfDue()
    if self.sim_if is not None:
      self.sim_if.publish_status()
    self.refreshLauncherStateCb()
    self.publishRobotLinkStatus()

  def refreshLauncherStateCb(self):
    # Follows each simulator service's own report, so the launcher panel
    # matches what is actually running on the sim machine: a simulation that
    # crashed or was closed there shows as failed with the service's own
    # error, one started there directly shows as running, and an app restart
    # picks up a simulation that is still up instead of forgetting it. Also
    # picks up a config file appearing after this node started (a service's
    # first run creates it). A cheap stat plus one small YAML read per
    # service per tick.
    changed = False
    configs = dict()
    for target_key, info in SIM_TARGETS.items():
      service = info['service']
      config_found = service.config_exists()
      config_dict = service.load_config() if config_found else None
      if config_dict is not None:
        service.note_service_poll(config_dict)
      service_alive = config_found and service.is_service_alive()
      if (config_found != self.service_config_found[target_key] or
          service_alive != self.service_alive[target_key]):
        changed = True
      self.service_config_found[target_key] = config_found
      self.service_alive[target_key] = service_alive
      configs[target_key] = (config_found, config_dict)

    # A launch/stop in flight owns launcher_state until it finishes, and
    # records the service state it finished on (service_state), so its own
    # result isn't applied a second time here. Only a CHANGE in what a
    # service reports is applied, so an app-side failure (e.g. a refused
    # Deploy) keeps showing its own error rather than being overwritten by
    # the service's unchanged 'idle'.
    busy = self.launcher_thread is not None and self.launcher_thread.is_alive()
    if not busy:
      for target_key, (_config_found, config_dict) in configs.items():
        if config_dict is None:
          continue
        service = SIM_TARGETS[target_key]['service']
        state = nepi_sim_service.get_config_str(config_dict, service.key('STATE'))
        if state == self.service_state[target_key] or state not in SERVICE_STATE_TO_LAUNCHER_STATE:
          continue
        self.service_state[target_key] = state
        if state in ('starting', 'running'):
          # A simulation coming up becomes the panel's simulator, and the
          # Robot/Environment lists follow it.
          self.selected_launch_target = target_key
          self.active_launch_target = target_key
          if target_key != self.selected_sim_target:
            self.setSelectedSimTarget(target_key)
        elif self.selected_launch_target and self.selected_launch_target != target_key:
          # The other simulator went idle or failed while the panel shows
          # the one that is up -- nothing to show for it.
          continue
        elif state == 'idle':
          self.selected_launch_target = ''
          self.active_launch_target = ''
        self.launcher_state = SERVICE_STATE_TO_LAUNCHER_STATE[state]
        if state == 'failed':
          self.setLauncherError(nepi_sim_service.get_config_str(config_dict, service.key('LAST_ERROR'))
                                or service.name + " stopped unexpectedly")
        else:
          self.setLauncherError('')
        changed = True
    if changed:
      self.publishLauncherStatus()
    # A file that exists but did not parse was caught mid-write: that says
    # nothing about the simulation, so the driver is left as it is until a
    # good read. Treating it as "not running" killed the driver every minute
    # or two while the service rewrote the file.
    if all(config_dict is not None or not config_found
           for config_found, config_dict in configs.values()):
      self.syncRoverDriver(configs)

  #**********************
  # Rover RBX driver

  def syncRoverDriver(self, configs):
    # Runs on every status tick, so every way a simulation can change --
    # Deploy, Kill, a robot swap, a switch of simulator, a crash, a start or
    # stop on the sim machine, this app restarting -- is covered by the one
    # comparison below instead of each path having to remember the driver.
    # The launch/kill itself runs on its own thread, since stopping a node can
    # take a few seconds. wanted is (target_key, robot folder): the same robot
    # name in a different simulator is a different driver launch.
    wanted = None
    capabilities = None
    for target_key, (_config_found, config_dict) in configs.items():
      service = SIM_TARGETS[target_key]['service']
      if (not self.service_alive[target_key] or config_dict is None or
          nepi_sim_service.get_config_str(config_dict, service.key('STATE')) != 'running'):
        continue
      robot = nepi_sim_service.get_config_str(config_dict, service.key('CURRENT_ROBOT_CONFIG'))
      if robot:
        robot_capabilities = self.getShareRobotCapabilities(target_key, robot)
        if robot_capabilities is not None and robot_capabilities.get('nepi_driver') == ROVER_DRIVER:
          wanted = (target_key, robot)
          capabilities = robot_capabilities
          break

    with self.rover_driver_lock:
      if self.rover_driver_busy:
        return
      process = self.rover_driver_process
      running = process is not None and process.poll() is None
      if process is not None and not running:
        self.msg_if.pub_warn("Rover driver " + self.rover_driver_node_name + " exited (rc " +
                             str(process.returncode) + ")")
        self.rover_driver_process = None
        self.rover_driver_launched = None
        self.rover_driver_retry_after = nepi_utils.get_time() + ROVER_DRIVER_RETRY_SEC
      if running and self.rover_driver_launched == wanted:
        return
      if not running and wanted is None:
        return
      # Only a crash or a failed launch waits -- a driver this app stopped
      # itself (Kill, a robot swap) comes back at once.
      if not running and nepi_utils.get_time() < self.rover_driver_retry_after:
        return
      self.rover_driver_busy = True
    thread = threading.Thread(target = self.applyRoverDriver, args = (wanted, capabilities))
    thread.daemon = True
    thread.start()

  def applyRoverDriver(self, wanted, capabilities):
    try:
      self.killRoverDriver()
      if wanted is not None:
        self.launchRoverDriver(wanted, capabilities)
    finally:
      with self.rover_driver_lock:
        self.rover_driver_busy = False

  def getShareRobotCapabilities(self, target_key, robot):
    # The folder's nepi_robot_config.yaml as target_key's share has it,
    # re-read at most every SIM_CONFIGS_REFRESH_SEC.
    now = nepi_utils.get_time()
    if (now - self.rover_robot_capabilities_time.get(target_key, 0.0)) > SIM_CONFIGS_REFRESH_SEC:
      try:
        self.rover_robot_capabilities[target_key] = SIM_TARGETS[target_key]['service'].list_robot_configs()
      except Exception as e:
        self.msg_if.pub_warn("Could not read the " + SIM_TARGETS[target_key]['service'].name +
                             " share's ROBOT_CONFIGS: " + str(e), throttle_s = 30.0)
      self.rover_robot_capabilities_time[target_key] = now
    return self.rover_robot_capabilities.get(target_key, dict()).get(robot)

  def launchRoverDriver(self, wanted, capabilities):
    target_key, robot = wanted
    service = SIM_TARGETS[target_key]['service']
    node_name = nepi_system.get_device_alias(ROVER_DRIVER_DEVICE_NAME)
    # The same drv_dict rbx_sim_discovery.py used to write, at the path
    # rbx_sim_node.py reads it from, plus the robot folder's capabilities,
    # which decide which controls the driver offers. device_path names the
    # simulator and robot, e.g. "mujoco/generic_rover".
    drv_dict = dict(
      DEVICE_DICT = dict(device_name = ROVER_DRIVER_DEVICE_NAME,
                         device_path = os.path.basename(service.sim_folder) + '/' + robot,
                         bridge_port = ROVER_BRIDGE_PORT),
      ROBOT_CONFIG = copy.deepcopy(capabilities))
    nepi_sdk.set_param(nepi_sdk.create_namespace(self.base_namespace, node_name + '/drv_dict'), drv_dict)

    self.msg_if.pub_info("Starting rover driver " + node_name + " for " + service.name +
                         " robot config '" + robot + "'")
    prev_ld_preload = os.environ.get('LD_PRELOAD')
    if os.path.exists(ROVER_DRIVER_LD_PRELOAD):
      os.environ['LD_PRELOAD'] = ROVER_DRIVER_LD_PRELOAD
    try:
      [success, msg, process] = nepi_drvs.launchDriverNode(ROVER_DRIVER_FILE, node_name)
    except Exception as e:
      [success, msg, process] = [False, str(e), None]
    finally:
      if prev_ld_preload is None:
        os.environ.pop('LD_PRELOAD', None)
      else:
        os.environ['LD_PRELOAD'] = prev_ld_preload

    with self.rover_driver_lock:
      if success:
        self.rover_driver_process = process
        self.rover_driver_launched = wanted
        self.rover_driver_node_name = node_name
      else:
        self.rover_driver_retry_after = nepi_utils.get_time() + ROVER_DRIVER_RETRY_SEC
    if not success:
      self.msg_if.pub_warn("Failed to start rover driver " + node_name + ": " + str(msg))

  def killRoverDriver(self):
    with self.rover_driver_lock:
      process = self.rover_driver_process
      node_name = self.rover_driver_node_name
      self.rover_driver_process = None
      self.rover_driver_launched = None
    if process is None:
      return
    self.msg_if.pub_info("Stopping rover driver " + node_name)
    try:
      nepi_drvs.killDriverNode(node_name, process)
    except Exception as e:
      self.msg_if.pub_warn("Failed to stop rover driver " + node_name + ": " + str(e))

  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    """Stops the rover driver, which is this app's child, on node shutdown."""
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")
    self.killRoverDriver()


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiSimConnectorApp()
