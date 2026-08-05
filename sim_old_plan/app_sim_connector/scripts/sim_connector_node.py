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

# Phase 2 of docs/SIMULATION_INTERFACE_SPEC.md: the one generic app node that
# hosts a SimDeviceIF instance and a generalized TCP/JSON bridge listener --
# any simulator's own bridge script dials INTO this app (the reverse
# direction from rbx_sim_node.py, which dials OUT to sim_bridge_node.py's
# server), per the spec's Packaging section: this app is the single, stable,
# well-known connection surface, not a per-simulator driver.
#
# Wire protocol (newline-delimited JSON both ways on one persistent
# connection, dispatch by "type" key presence -- same convention
# sim_bridge_node.py already proved out):
#
#   in  -- bare (no "type" key): NavPose telemetry, generalized past the
#          rover-only {"x","y","yaw",...} shape to the full NavPose contract.
#          Every field optional; presence of x_m/y_m/z_m sets has_position,
#          roll_deg/pitch_deg/yaw_deg sets has_orientation, latitude/longitude
#          sets has_location, altitude_m sets has_altitude -- same has_*
#          gating NavPose.msg itself uses, so a rover (position only) and a
#          drone (position + orientation + location + altitude) both fit one
#          shape with no per-vehicle special-casing.
#   in  -- {"type":"sensor_topics","topics":[{"topic_name":...,"msg_type":...},...]}
#          2026-08-04 typed sensor-topics decision (Step 2.1, new): the
#          bridge announces its current live topic list; fed straight into
#          getAvailableSensorTopicsFunction's cached return value.
#   in  -- {"type":"environment_options","options":[...]}
#          Step 2.1, new: same idea, generalizing the one hardcoded
#          obstacle_course toggle into a reported list.
#   in  -- {"type":"image","topic_name":...,"data":"<base64 jpeg>","stamp":...}
#          Existing shape (sim_bridge_node.py's imageCompressedCb), extended
#          with topic_name so multiple announced cameras are distinguishable
#          -- omitted topic_name is treated as the current active_image_topic
#          for single-camera backward compatibility. Only frames matching the
#          active topic are actually decoded/republished; see
#          setActiveImageTopicFunction below for why only the *selected*
#          camera needs a live ROS topic, not every announced one.
#
#   out -- {"type":"motor_control","motor_ind":N,"speed_ratio":R}
#   out -- {"type":"goto_position","x_meters":...,"y_meters":...,"z_meters":...,"yaw_deg":...}
#   out -- {"type":"goto_pose","roll_deg":...,"pitch_deg":...,"yaw_deg":...}
#   out -- {"type":"goto_location","lat":...,"long":...,"altitude_meters":...,"yaw_deg":...}
#          Field names match GotoPosition.msg/GotoPose.msg/GotoLocation.msg
#          1:1 -- no reason to invent different names on the wire.
#   out -- {"type":"go_home"} / {"type":"go_stop"}
#   out -- {"type":"setup_action","action":"<string>"} / {"type":"go_action","action":"<string>"}
#          Generalizes RESET_SIM-style named actions -- which action strings
#          are valid is a per-deployment config decision (SIM_VEHICLE_DICT
#          below), not hardcoded here.
#   out -- {"type":"camera_settings","view_mode":...}
#          Existing shape, reused as-is (sim_bridge_node.py already handles
#          this exact line for the rover).
#   out -- {"type":"set_active_image_topic","topic_name":...}
#   out -- {"type":"environment_option","option":...,"enabled":bool}
#          Generalizes the old hardcoded {"type":"obstacle_course","enabled":bool}
#          into a named option -- "obstacle_course" becomes a value of
#          "option" rather than its own "type".
#
# Capability-timing resolution (a real design decision, not an oversight):
# device_if_sim.py's contract -- like device_if_rbx.py's -- decides
# capabilities ONCE at construction time, cached, never recomputed. But a
# genuinely generic connector app cannot know a specific simulator's
# wheel/motor counts or which goto functions make sense until AFTER a bridge
# actually connects and this process is already constructed. Rather than
# fight that "decided once" principle (which is what makes the RUI's
# capability-flag-driven rendering work at all -- see the spec's Capability ->
# UI mapping table), the fields the spec's own contract table lists as
# genuinely dynamic (available_sensor_topics, available_environment_options)
# ARE live-refreshed from the bridge as designed; everything else
# (wheel_count, motor_count, which goto/setup functions exist,
# available_camera_view_modes) is a per-deployment CONFIG decision, read once
# at startup from this app's own SIM_VEHICLE_DICT param (see
# params/app_sim_connector_params.yaml) -- exactly the same shape as
# rbx_sim_node.py/rbx_ardupilot_node.py each hardcoding their own vehicle's
# capabilities as class constants, except configurable per instance since the
# code itself has to stay vehicle-agnostic. Factory defaults are the same
# capability-empty profile Phase 1's registration test already verified safe.

import base64
import copy
import json
import socket
import threading
import time

import numpy as np
import cv2

from nepi_sdk import nepi_sdk
from nepi_sdk import nepi_utils
from nepi_sdk import nepi_nav
from nepi_sdk import nepi_img

from sensor_msgs.msg import Image
from geographic_msgs.msg import GeoPoint

from nepi_api.messages_if import MsgIF
from nepi_api.device_if_sim import SimDeviceIF

PKG_NAME = 'SIM_CONNECTOR'

# Next free port clear of sim_container's existing allocations (9021 gz
# reset, 9022 heartbeat, 9023 sim_bridge BRIDGE_PORT) and the 576x MAVLink
# range -- see sim_bridge_node.py's own BRIDGE_PORT comment for that block.
FACTORY_LISTEN_PORT = 9030

FACTORY_VEHICLE_DICT = dict(
    listen_port = FACTORY_LISTEN_PORT,
    wheel_count = 0,
    motor_count = 0,
    has_goto_position = False,
    has_goto_pose = False,
    has_goto_location = False,
    has_go_home = False,
    has_set_home = False,
    setup_actions = [],
    go_actions = [],
    has_camera_view_control = False,
    available_camera_view_modes = [],
    has_environment_controls = False,
)

# Same rationale as sim_bridge_node.py's srv.settimeout(None): rospy sets a
# process-global socket.setdefaulttimeout(60) that a plain accept()/recv()
# would otherwise inherit, misreading a merely-quiet (not dead) connection as
# a failure. See src/nepi_drivers/CLAUDE.md's documented rospy socket
# default-timeout gotcha.
BRIDGE_ACCEPT_BACKLOG = 1
TELEMETRY_STALE_SEC_IF_NEVER_CONNECTED = -1.0


#########################################
# Node Class
#########################################

class SimConnectorAppNode:

  DEFAULT_NODE_NAME = "app_sim_connector"

  def __init__(self):
    nepi_sdk.init_node(name = self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.node_name = nepi_sdk.get_node_name()
    self.node_namespace = nepi_sdk.get_node_namespace()

    self.msg_if = MsgIF(log_name = self.class_name)
    self.msg_if.pub_info("Starting Node Initialization Processes")

    ##############################
    # Per-deployment vehicle profile -- see module docstring above for why
    # this is config, not a live bridge announcement, for these fields.
    vehicle_dict_ns = nepi_sdk.create_namespace(self.node_namespace, 'SIM_VEHICLE_DICT')
    self.vehicle_dict = nepi_sdk.get_param(vehicle_dict_ns, copy.deepcopy(FACTORY_VEHICLE_DICT))
    self.listen_port = int(self.vehicle_dict.get('listen_port', FACTORY_LISTEN_PORT))
    self.wheel_count = int(self.vehicle_dict.get('wheel_count', 0))
    self.motor_count = int(self.vehicle_dict.get('motor_count', 0))
    self.has_goto_position = bool(self.vehicle_dict.get('has_goto_position', False))
    self.has_goto_pose = bool(self.vehicle_dict.get('has_goto_pose', False))
    self.has_goto_location = bool(self.vehicle_dict.get('has_goto_location', False))
    self.has_go_home = bool(self.vehicle_dict.get('has_go_home', False))
    self.has_set_home = bool(self.vehicle_dict.get('has_set_home', False))
    self.setup_actions = list(self.vehicle_dict.get('setup_actions', []))
    self.go_actions = list(self.vehicle_dict.get('go_actions', []))
    self.has_camera_view_control = bool(self.vehicle_dict.get('has_camera_view_control', False))
    self.available_camera_view_modes = list(self.vehicle_dict.get('available_camera_view_modes', []))
    self.has_environment_controls = bool(self.vehicle_dict.get('has_environment_controls', False))

    ##############################
    # Bridge connection + live announced state (the genuinely dynamic
    # fields -- refreshed from whatever the connected bridge last sent, not
    # this app's own config)
    self.client_conn = None
    self.client_lock = threading.Lock()
    self.bridge_connected = False
    self.last_telemetry_time = 0.0
    self.connected_since = None
    self.available_sensor_topics = []          # [(topic_name, msg_type), ...]
    self.navpose_dict = copy.deepcopy(nepi_nav.BLANK_NAVPOSE_DICT)
    self.active_image_topic = ""
    self.motor_ratios = [0.0] * self.motor_count

    ##############################
    # Image republish -- decode/republish only the currently-active image
    # topic (the one thing the RUI actually displays), on a bare relative
    # name matching rbx_sim_node.py's own convention -- resolves against the
    # shared per-device namespace, qualified by this node's own name so a
    # second sim-connector instance on the same device wouldn't collide.
    self.image_topic_name = self.node_name + "/color_2d_image"
    self.image_pub = nepi_sdk.create_publisher(self.image_topic_name, Image, queue_size = 1)

    ##############################
    # Launch the generic SimDeviceIF -- every callback below is a thin
    # wire-protocol sender/getter; device_if_sim.py itself never knows this
    # is a TCP bridge underneath, exactly as it never knows about Gazebo.
    device_info = dict(device_name = self.node_name, path = "",
                        serial_number = "", hw_version = "", sw_version = "")

    self.sim_if = SimDeviceIF(
        device_info = device_info,
        setMotorControlRatio = self.setMotorControlRatio if self.motor_count > 0 else None,
        getMotorControlRatios = self.getMotorControlRatios if self.motor_count > 0 else None,
        manualControlsReadyFunction = self.isBridgeConnected if self.motor_count > 0 else None,
        autonomousControlsReadyFunction = (
            self.isBridgeConnected if (self.has_goto_position or self.has_goto_pose
                                        or self.has_goto_location) else None),
        wheel_count = self.wheel_count,
        motor_count = self.motor_count,
        setup_actions = self.setup_actions,
        setSetupActionIndFunction = self.setSetupActionInd if self.setup_actions else None,
        go_actions = self.go_actions,
        setGoActionIndFunction = self.setGoActionInd if self.go_actions else None,
        getHomeFunction = self.getHome if self.has_set_home else None,
        setHomeFunction = self.setHome if self.has_set_home else None,
        goHomeFunction = self.goHome if self.has_go_home else None,
        goStopFunction = self.goStop,
        gotoPoseFunction = self.gotoPose if self.has_goto_pose else None,
        gotoPositionFunction = self.gotoPosition if self.has_goto_position else None,
        gotoLocationFunction = self.gotoLocation if self.has_goto_location else None,
        getNavPoseCb = self.getNavPoseCb,
        getAvailableSensorTopicsFunction = self.getAvailableSensorTopics,
        setActiveImageTopicFunction = self.setActiveImageTopic,
        setCameraViewModeFunction = self.setCameraViewMode if self.has_camera_view_control else None,
        available_camera_view_modes = self.available_camera_view_modes,
        setEnvironmentOptionFunction = self.setEnvironmentOption if self.has_environment_controls else None,
        available_environment_options = [],
        getBridgeConnectedFunction = self.isBridgeConnected,
        getTelemetryAgeFunction = self.getTelemetryAge,
        msg_if = self.msg_if,
    )

    ##############################
    # Home position state -- reused GeoPoint plumbing, same reinterpretation
    # rbx_sim_node.py already uses for a vehicle with no independent WGS84
    # reference; a drone-shaped deployment that actually has one can instead
    # keep the telemetry-supplied navpose_dict latitude/longitude as home,
    # this app doesn't need to know which case it is.
    self.home_x_m = 0.0
    self.home_y_m = 0.0
    self.home_z_m = 0.0

    ##############################
    # Goto target state for the (not-yet-blocking, see device_if_sim.py's own
    # documented Phase 1 gap) thin goto delegators -- stored only so a real
    # future controller loop has somewhere to read from; this app does not
    # run a convergence controller itself, that is entirely the connected
    # bridge's job (mirrors gotoPositionFunction's "Reuse as-is" contract
    # intent: forward the setpoint, let the simulator-side bridge/vehicle
    # model handle reaching it).
    self.goto_target_lock = threading.Lock()
    self.goto_target = None

    ##############################
    # Bridge server thread -- this app OWNS the listen socket (the reverse
    # direction from rbx_sim_node.py/sim_bridge_node.py's client/server
    # relationship); any simulator's bridge script dials in.
    self.server_thread = threading.Thread(target = self.bridgeServerLoop)
    self.server_thread.daemon = True
    self.server_thread.start()

    self.msg_if.pub_info("Sim Connector listening on 0.0.0.0:" + str(self.listen_port))
    self.msg_if.pub_info("Initialization Complete")
    nepi_sdk.on_shutdown(self.cleanup_actions)
    nepi_sdk.spin()

  #**********************
  # Connection health

  def isBridgeConnected(self):
    with self.client_lock:
      return self.client_conn is not None

  def getTelemetryAge(self):
    if self.connected_since is None:
      return TELEMETRY_STALE_SEC_IF_NEVER_CONNECTED
    return nepi_utils.get_time() - self.last_telemetry_time

  #**********************
  # SimDeviceIF callbacks -- motor/goto/home/actions -- all thin wire-protocol
  # senders, gated at construction by this deployment's SIM_VEHICLE_DICT

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
    roll_deg, pitch_deg, yaw_deg = attitude_enu_degs[0], attitude_enu_degs[1], attitude_enu_degs[2]
    self.sendLineToBridge({'type': 'goto_pose', 'roll_deg': roll_deg,
                           'pitch_deg': pitch_deg, 'yaw_deg': yaw_deg}, "Goto pose")

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
    self.sendLineToBridge({'type': 'go_stop'}, "Go stop")
    return self.isBridgeConnected()

  def setSetupActionInd(self, action_ind):
    if action_ind < 0 or action_ind >= len(self.setup_actions):
      return False
    self.sendLineToBridge({'type': 'setup_action', 'action': self.setup_actions[action_ind]},
                          "Setup action")
    return self.isBridgeConnected()

  def setGoActionInd(self, action_ind):
    if action_ind < 0 or action_ind >= len(self.go_actions):
      return False
    self.sendLineToBridge({'type': 'go_action', 'action': self.go_actions[action_ind]},
                          "Go action")
    return self.isBridgeConnected()

  def setCameraViewMode(self, view_mode):
    self.sendLineToBridge({'type': 'camera_settings', 'view_mode': view_mode}, "Camera settings")

  def setEnvironmentOption(self, option):
    # SimDeviceIF's setEnvironmentOptionCb (device_if_sim.py) currently
    # passes a single string through (the RUI's environment-toggle control),
    # not an (option, enabled) pair yet -- forwarded as an always-True enable
    # until that control's own on/off semantics are designed; matches the
    # only real precedent (obstacle_course) being a one-way "turn this on".
    self.sendLineToBridge({'type': 'environment_option', 'option': option,
                           'enabled': True}, "Environment option")

  def setActiveImageTopic(self, topic_name):
    self.active_image_topic = topic_name
    self.sendLineToBridge({'type': 'set_active_image_topic', 'topic_name': topic_name},
                          "Set active image topic")

  def getAvailableSensorTopics(self):
    return self.available_sensor_topics

  def getNavPoseCb(self):
    return self.navpose_dict

  #**********************
  # Bridge server -- this app owns the listen socket; any simulator's bridge
  # script dials in. Single active client at a time, same model
  # sim_bridge_node.py already proves works for this project's sim assets.

  def bridgeServerLoop(self):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # See src/nepi_drivers/CLAUDE.md's documented rospy socket default-timeout
    # gotcha: rospy's process-global socket.setdefaulttimeout(60) would
    # otherwise apply to accept()/recv() here too.
    srv.settimeout(None)
    srv.bind(('0.0.0.0', self.listen_port))
    srv.listen(BRIDGE_ACCEPT_BACKLOG)
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
        data = conn.recv(4096)
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
      self.msg_if.pub_warn("Bad line from bridge: " + str(e))
      return
    msg_type = msg.get('type')
    if msg_type == 'sensor_topics':
      self.processSensorTopicsLine(msg)
    elif msg_type == 'environment_options':
      self.processEnvironmentOptionsLine(msg)
    elif msg_type == 'image':
      self.processImageLine(msg)
    else:
      # Bare shape (no "type" key), or an unrecognized type -- dispatch by
      # key presence like sim_bridge_node.py already does: anything with no
      # "type" key is treated as NavPose telemetry.
      self.processTelemetryLine(msg)

  def processSensorTopicsLine(self, msg):
    topics = msg.get('topics', [])
    parsed = []
    for entry in topics:
      topic_name = entry.get('topic_name', '')
      msg_type = entry.get('msg_type', '')
      if topic_name and msg_type:
        parsed.append((topic_name, msg_type))
    self.available_sensor_topics = parsed

  def processEnvironmentOptionsLine(self, msg):
    # available_environment_options isn't behind a live-refresh callback in
    # device_if_sim.py's Phase 1 contract (unlike available_sensor_topics) --
    # mutated directly on the cached caps_report instead, which
    # capabilities_query_callback returns unmodified otherwise. Minimal,
    # doesn't require changing device_if_sim.py's public contract.
    options = list(msg.get('options', []))
    if self.sim_if is not None:
      self.sim_if.caps_report.available_environment_options = options

  def processImageLine(self, msg):
    topic_name = msg.get('topic_name', self.active_image_topic)
    if self.active_image_topic and topic_name != self.active_image_topic:
      return  # Not the selected camera -- nothing subscribes to it yet
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
      nd['yaw_deg_per_sec'] = float(telem.get('yaw_deg_per_sec', nd['yaw_deg_per_sec']))

    if any(k in telem for k in ('latitude', 'longitude')):
      nd['has_location'] = True
      nd['time_location'] = now
      nd['latitude'] = float(telem.get('latitude', nd['latitude']))
      nd['longitude'] = float(telem.get('longitude', nd['longitude']))

    if 'altitude_m' in telem:
      nd['has_altitude'] = True
      nd['time_altitude'] = now
      nd['altitude_m'] = float(telem.get('altitude_m', nd['altitude_m']))

    self.last_telemetry_time = now

  def sendLineToBridge(self, line_dict, description):
    with self.client_lock:
      conn = self.client_conn
    if conn is None:
      self.msg_if.pub_warn(description + " dropped -- no bridge client connected", throttle_s = 5.0)
      return
    try:
      conn.sendall((json.dumps(line_dict) + '\n').encode())
    except Exception as e:
      self.msg_if.pub_warn("Failed to send " + description.lower() + " to bridge: " + str(e))

  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################

if __name__ == '__main__':
  SimConnectorAppNode()
