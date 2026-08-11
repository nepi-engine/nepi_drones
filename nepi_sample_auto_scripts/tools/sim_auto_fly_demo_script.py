#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# SITL/Gazebo-only demo scaffolding -- NOT a real mission script. Exists so
# drone_follow_object_mission_script.py's RUI "Auto-Start Sim Requirements"
# button does more than just bring the sim processes up: it also puts the
# vehicle in the air, so there's something visibly happening in Gazebo the
# instant the requirements checklist goes green, instead of a vehicle sitting
# disarmed on the ground. Deploy and launch this exactly like a normal NEPI
# automation script (get_scripts/launch_script) -- it's meant to run
# alongside sim_ai_targeting_bridge_script.py, not replace it: that script
# produces the target_localizations feed, this one flies the vehicle.
#
# What this does:
#   - One-shot fire-and-forget trigger of sitl_gazebo_full on the dev VM
#     (127.0.0.1:9028 over the existing reverse tunnel), same pattern and
#     same reasoning as sim_ai_targeting_bridge_script.py's own
#     trigger_remote_sim_launch -- harmless no-op if the stack is already up.
#   - Connects to the RBX ArduPilot driver using the same inlined
#     rbx_initialize()/set_rbx_state()/set_rbx_mode()/setup_rbx_action()/
#     goto_rbx_position() helpers as drone_follow_object_mission_script.py
#     (nepi_sdk.nepi_rbx is still broken against current message names --
#     see that script's own header comment -- so there is no shared helper
#     module to import from; this is a deliberate, small duplication of a
#     ~150-line block, not an oversight).
#   - Enables Fake GPS and sets a home location, exactly like
#     drone_follow_object_mission_script.py does, since SITL/Gazebo has no
#     real GPS of its own.
#   - Issues the LAUNCH setup action (GUIDED -> ARM -> takeoff chained
#     server-side in one call -- see set_stream_rate fix note below for why
#     this now actually reaches altitude instead of timing out).
#   - Repeatedly flies a small square pattern using body-frame relative
#     goto_position moves (forward/right/back/left), pausing briefly at each
#     corner, looping forever until the script is stopped -- so there's
#     continuous, visible motion in Gazebo for as long as this script runs,
#     not a one-shot flight that ends and leaves the vehicle hovering.
#
# 2026-08-11: rbx_ardupilot_node.py was missing a mavros set_stream_rate
# request (STREAM_ALL) after connecting -- mavros reporting "connected" only
# means heartbeat/timesync are flowing, not GPS/global_position, so every
# altitude/position-based completion check (takeoff climb, goto_position)
# saw permanently stale telemetry and always timed out even when the vehicle
# was actually moving. That's now fixed in the driver itself; this script
# doesn't work around it.

import rospy
import socket
import time
import math
from nepi_sdk import nepi_ros
from nepi_api.messages_if import MsgIF

from std_msgs.msg import Empty, Bool, String, UInt32, Int32
from geographic_msgs.msg import GeoPoint
from nepi_interfaces.msg import DeviceRBXInfo, DeviceRBXStatus, Setting, \
     GotoPose, GotoPosition, GotoLocation
from nepi_interfaces.srv import RBXCapabilitiesQuery

#########################################
# USER SETTINGS - Edit as Necessary
#########################################
RBX_ROBOT_NAME = "ardupilot"

TAKEOFF_HEIGHT_M = 10.0

# Fake GPS / home, same defaults as drone_follow_object_mission_script.py so
# both scripts agree on where "home" is when run together.
ENABLE_FAKE_GPS = True
SET_HOME = True
HOME_LOCATION = [47.6540828, -122.3187578, 0.0]

# Square pattern flown via body-frame relative moves: [x_meters, y_meters,
# z_meters, yaw_deg] per leg, x=forward, y=right, z=up, executed in order and
# looped. -999 yaw = ignore/keep-current, same sentinel convention as
# drone_follow_object_mission_script.py.
SQUARE_LEG_M = 8.0
SQUARE_PATTERN = [
  [SQUARE_LEG_M, 0.0, 0.0, -999],
  [0.0, SQUARE_LEG_M, 0.0, -999],
  [-SQUARE_LEG_M, 0.0, 0.0, -999],
  [0.0, -SQUARE_LEG_M, 0.0, -999],
]
CORNER_PAUSE_SEC = 3

# CMD Timeout Values
CMD_STATE_TIMEOUT_SEC = 5
CMD_MODE_TIMEOUT_SEC = 5
# Gazebo's simulation speed varies with VM CPU load, so the real-world time
# for a takeoff climb to converge is not fixed -- confirmed live taking
# anywhere from ~10s to a bit over 30s under load. 60s gives real headroom
# without making a genuinely stuck command wait too long.
CMD_ACTION_TIMEOUT_SEC = 60
CMD_GOTO_TIMEOUT_SEC = 30

# sim_launch_listener.py on the dev VM (see nepi_sitl_dev_env.sh) -- one-shot
# trigger for sitl_gazebo_full, same as sim_ai_targeting_bridge_script.py.
LAUNCH_TRIGGER_HOST = "127.0.0.1"
LAUNCH_TRIGGER_PORT = 9028
LAUNCH_TRIGGER_TIMEOUT_SEC = 3.0


def trigger_remote_sim_launch(msg_if):
  # Fire-and-forget: connect, read one reply line (or time out), close. See
  # sim_ai_targeting_bridge_script.py's identical helper for the full
  # reasoning -- kept as a small duplicate here rather than a shared import
  # since these tools scripts are each meant to be standalone/deployable on
  # their own.
  try:
    sock = socket.create_connection((LAUNCH_TRIGGER_HOST, LAUNCH_TRIGGER_PORT),
                                     timeout = LAUNCH_TRIGGER_TIMEOUT_SEC)
    sock.settimeout(LAUNCH_TRIGGER_TIMEOUT_SEC)
    reply = sock.recv(200)
    sock.close()
    msg_if.pub_info("Triggered remote sim launch: " + reply.decode(errors = "replace").strip())
  except Exception as e:
    msg_if.pub_warn("Could not reach sim_launch_listener at " + LAUNCH_TRIGGER_HOST + ":" +
                     str(LAUNCH_TRIGGER_PORT) + " (" + str(e) + ") -- if the dev VM has never " +
                     "run sitl_gazebo/sitl_gazebo_full this session, start one of those there " +
                     "manually first")


class sim_auto_fly_demo(object):

  rbx_info = None
  rbx_status = None

  DEFAULT_NODE_NAME = "sim_auto_fly_demo"

  def __init__(self):
    nepi_ros.init_node(name = self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    self.msg_if = MsgIF(log_name = self.node_name)
    self.msg_if.pub_info("Starting Initialization Processes")

    self.msg_if.pub_info("Attempting to trigger the full sim stack on the dev VM (harmless no-op if already running)")
    trigger_remote_sim_launch(self.msg_if)

    self.msg_if.pub_info("Waiting for namespace containing: " + RBX_ROBOT_NAME)
    robot_namespace = nepi_ros.wait_for_node(RBX_ROBOT_NAME)
    robot_namespace = robot_namespace + "/"
    self.msg_if.pub_info("Found namespace: " + robot_namespace)
    rbx_namespace = (robot_namespace + "rbx/")
    self.rbx_initialize(rbx_namespace)
    time.sleep(1)
    while self.rbx_status is None and not nepi_ros.is_shutdown():
      time.sleep(1)

    if ENABLE_FAKE_GPS:
      self.msg_if.pub_info("Enabling Fake GPS")
      self.fake_gps_enable_pub.publish(True)
      time.sleep(2)
    if SET_HOME:
      self.msg_if.pub_info("Updating RBX Home Location")
      new_home_geo = GeoPoint()
      new_home_geo.latitude = HOME_LOCATION[0]
      new_home_geo.longitude = HOME_LOCATION[1]
      new_home_geo.altitude = HOME_LOCATION[2]
      self.rbx_set_home_pub.publish(new_home_geo)
      nepi_ros.sleep(2)
      if ENABLE_FAKE_GPS:
        nepi_ros.sleep(15, 100)

    self.msg_if.pub_info("Initialization Complete")

    ###########################
    ## Launch once, then fly the pattern forever
    ###########################
    # LAUNCH (GUIDED -> ARM -> takeoff) only makes sense from the ground --
    # calling it again on every loop iteration while already airborne is not
    # a real "takeoff" and reports a spurious failure (the vehicle has
    # nowhere further to climb). Launch once here; the loop below only
    # repeats the flight pattern.
    if not self.launch():
      self.msg_if.pub_warn("LAUNCH did not succeed -- flying the pattern anyway in case it partially completed")
    while not nepi_ros.is_shutdown():
      self.fly_square()
      self.msg_if.pub_info("Pattern loop complete -- restarting")
      nepi_ros.sleep(2, 10)

  #######################
  ### RBX Initialize and Control Helpers
  # Inlined from drone_follow_object_mission_script.py's own copy (itself
  # inlined because nepi_sdk.nepi_rbx is currently broken against the
  # current message names -- see that script's header comment). Kept
  # identical rather than factored out so this script stays a standalone
  # drop-in tool, matching this directory's existing convention.

  def rbx_initialize(self, rbx_namespace):
    self.rbx_cap_states = [""]
    self.rbx_cap_modes = [""]
    self.rbx_cap_setup_actions = [""]
    self.rbx_info = None
    self.rbx_status = None

    rbx_topic = nepi_ros.wait_for_topic(rbx_namespace)
    NEPI_ROBOT_NAMESPACE = rbx_topic.rpartition("rbx")[0]
    NEPI_RBX_NAMESPACE = (NEPI_ROBOT_NAMESPACE + "rbx/")
    self.msg_if.pub_info("Found rbx namespace: " + NEPI_RBX_NAMESPACE)

    NEPI_RBX_CAPABILITIES_TOPIC = NEPI_RBX_NAMESPACE + "capabilities_query"
    nepi_ros.wait_for_service(NEPI_RBX_CAPABILITIES_TOPIC)
    rbx_caps_service = nepi_ros.connect_service(NEPI_RBX_CAPABILITIES_TOPIC, RBXCapabilitiesQuery)
    time.sleep(1)
    rbx_caps = rbx_caps_service()
    self.rbx_cap_states = rbx_caps.state_options
    self.rbx_cap_modes = rbx_caps.mode_options
    self.rbx_cap_setup_actions = rbx_caps.setup_action_options

    self.NEPI_RBX_INFO_TOPIC = NEPI_RBX_NAMESPACE + "info"
    nepi_ros.wait_for_topic(self.NEPI_RBX_INFO_TOPIC)
    rbx_info_pub = nepi_ros.create_publisher(NEPI_RBX_NAMESPACE + 'publish_info', Empty, queue_size=1)
    nepi_ros.create_subscriber(self.NEPI_RBX_INFO_TOPIC, DeviceRBXInfo, self.rbx_info_callback, queue_size=None)
    while self.rbx_info is None and not nepi_ros.is_shutdown():
      self.msg_if.pub_info("Waiting for current rbx info to publish")
      time.sleep(1)
      rbx_info_pub.publish(Empty())

    self.NEPI_RBX_STATUS_TOPIC = NEPI_RBX_NAMESPACE + "status"
    nepi_ros.wait_for_topic(self.NEPI_RBX_STATUS_TOPIC)
    rbx_status_pub = nepi_ros.create_publisher(NEPI_RBX_NAMESPACE + 'publish_status', Empty, queue_size=1)
    nepi_ros.create_subscriber(self.NEPI_RBX_STATUS_TOPIC, DeviceRBXStatus, self.rbx_status_callback, queue_size=None)
    while self.rbx_status is None and not nepi_ros.is_shutdown():
      self.msg_if.pub_info("Waiting for current rbx status to publish")
      time.sleep(0.1)
      rbx_status_pub.publish(Empty())

    NEPI_RBX_SETTINGS_UPDATE_TOPIC = NEPI_RBX_NAMESPACE + "settings/update_setting"
    self.rbx_setting_update_pub = nepi_ros.create_publisher(NEPI_RBX_SETTINGS_UPDATE_TOPIC, Setting, queue_size=1)

    self.rbx_set_state_pub = nepi_ros.create_publisher(NEPI_RBX_NAMESPACE + "set_state", Int32, queue_size=1)
    self.rbx_set_mode_pub = nepi_ros.create_publisher(NEPI_RBX_NAMESPACE + "set_mode", Int32, queue_size=1)
    self.rbx_setup_action_pub = nepi_ros.create_publisher(NEPI_RBX_NAMESPACE + "setup_action", Int32, queue_size=1)
    self.rbx_set_cmd_timeout_pub = nepi_ros.create_publisher(NEPI_RBX_NAMESPACE + "set_goto_timeout", UInt32, queue_size=1)
    self.rbx_set_home_pub = nepi_ros.create_publisher(NEPI_RBX_NAMESPACE + "set_home", GeoPoint, queue_size=1)

    self.rbx_goto_position_pub = nepi_ros.create_publisher(NEPI_RBX_NAMESPACE + "goto_position", GotoPosition, queue_size=1)

    # Fake GPS is a standalone app (nepi_app_fake_gps), not a per-robot rbx/
    # topic -- same as drone_follow_object_mission_script.py.
    FAKE_GPS_NAMESPACE = self.base_namespace + "app_fake_gps/"
    self.fake_gps_enable_pub = nepi_ros.create_publisher(FAKE_GPS_NAMESPACE + "enable", Bool, queue_size=1)

    self.msg_if.pub_info("RBX initialize process complete")

  def rbx_info_callback(self, msg):
    self.rbx_info = msg

  def rbx_status_callback(self, msg):
    self.rbx_status = msg

  def wait_for_rbx_status_ready(self, timeout_sec=10):
    self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
    time.sleep(0.1)
    count_goal = 3
    counter = 0
    timeout_timer = 0
    sleep_time_sec = 0.1
    while (counter < count_goal) and timeout_timer < timeout_sec and not nepi_ros.is_shutdown():
      if self.rbx_status.ready is True:
        counter += 1
      else:
        counter = 0
      time.sleep(sleep_time_sec)
      timeout_timer += sleep_time_sec
    return self.rbx_status.ready

  def wait_for_rbx_status_busy(self, timeout_sec=10):
    self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
    time.sleep(0.1)
    count_goal = 3
    counter = 0
    timeout_timer = 0
    sleep_time_sec = 0.1
    while (counter < count_goal) and timeout_timer < timeout_sec and not nepi_ros.is_shutdown():
      if self.rbx_status.ready is False:
        counter += 1
      else:
        counter = 0
      time.sleep(sleep_time_sec)
      timeout_timer += sleep_time_sec
    return self.rbx_status.ready == False

  def setup_rbx_action(self, action_str, timeout_sec=10):
    self.msg_if.pub_info("Setup Action Request Recieved: " + action_str)
    success = False
    self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
    time.sleep(0.1)
    action_ind = -1
    for ind, action in enumerate(self.rbx_cap_setup_actions):
      if action == action_str:
        action_ind = ind
    if action_ind == -1:
      self.msg_if.pub_warn("No matching action found: " + action_str)
    else:
      ready = self.wait_for_rbx_status_ready(timeout_sec)
      if ready:
        self.rbx_setup_action_pub.publish(action_ind)
        busy = self.wait_for_rbx_status_busy(timeout_sec)
        if busy:
          self.wait_for_rbx_status_ready(timeout_sec)
      time.sleep(1)
      success = self.rbx_status.cmd_success
    return success

  def set_rbx_mode(self, mode_str, timeout_sec=5):
    self.msg_if.pub_info("Set Mode Request Recieved: " + mode_str)
    success = False
    self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
    new_mode_ind = -1
    for ind, mode in enumerate(self.rbx_cap_modes):
      if mode == mode_str:
        new_mode_ind = ind
    if new_mode_ind == -1:
      self.msg_if.pub_warn("No matching mode found: " + mode_str)
    else:
      self.rbx_set_mode_pub.publish(new_mode_ind)
      timeout_timer = 0
      sleep_time_sec = 1
      while self.rbx_info.mode != new_mode_ind and timeout_timer < timeout_sec and not nepi_ros.is_shutdown():
        time.sleep(sleep_time_sec)
        timeout_timer += sleep_time_sec
      if self.rbx_info.mode == new_mode_ind:
        success = True
    time.sleep(1)
    return success

  def goto_rbx_position(self, goto_data, timeout_sec=10):
    self.rbx_set_cmd_timeout_pub.publish(timeout_sec)
    time.sleep(0.1)
    ready = self.wait_for_rbx_status_ready(timeout_sec)
    if ready:
      self.msg_if.pub_info("Starting goto Position Body Process")
      goto_msg = GotoPosition()
      goto_msg.x_meters = goto_data[0]
      goto_msg.y_meters = goto_data[1]
      goto_msg.z_meters = goto_data[2]
      goto_msg.yaw_deg = goto_data[3]
      self.rbx_goto_position_pub.publish(goto_msg)
      busy = self.wait_for_rbx_status_busy(timeout_sec)
      if busy:
        self.wait_for_rbx_status_ready(timeout_sec)
    time.sleep(1)
    return self.rbx_status.cmd_success

  #######################
  ### Demo Flight

  def launch(self):
    self.msg_if.pub_info("Sending LAUNCH (GUIDED -> ARM -> takeoff)")
    success = self.setup_rbx_action("LAUNCH", timeout_sec = CMD_ACTION_TIMEOUT_SEC)
    error_str = str(self.rbx_status.errors_current)
    if not success:
      self.msg_if.pub_warn("LAUNCH failed with errors: " + error_str)
      return False
    self.msg_if.pub_info("LAUNCH completed with errors: " + error_str)
    nepi_ros.sleep(2, 10)
    return True

  def fly_square(self):
    for leg in SQUARE_PATTERN:
      if nepi_ros.is_shutdown():
        return
      self.msg_if.pub_info("Flying leg: " + str(leg))
      success = self.goto_rbx_position(leg, timeout_sec = CMD_GOTO_TIMEOUT_SEC)
      error_str = str(self.rbx_status.errors_current)
      if success:
        self.msg_if.pub_info("Leg completed with errors: " + error_str)
      else:
        self.msg_if.pub_info("Leg failed with errors: " + error_str)
      nepi_ros.sleep(CORNER_PAUSE_SEC, 100)

  #######################
  # Node Cleanup Function

  def cleanup_actions(self):
    self.msg_if.pub_info("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  sim_auto_fly_demo()
