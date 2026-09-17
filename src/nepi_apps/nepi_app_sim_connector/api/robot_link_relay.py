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

# Mirrors RBX flight/motion commands from a currently-attached simulator onto
# a selected physical robot, so whatever an operator does to the sim (arm,
# change mode, motor test, teleop, goto) happens on the real vehicle too.
# Requested live (2026-09-15): "if a drone and ardupilot sitl are both up,
# the user should be able to link both, and whatever motor commands and
# flying happens in the sim should happen with the physical drone too."
#
# Pure topic relay, no new nepi_interfaces message types -- every mirrored
# topic already uses a stock or existing nepi_interfaces message on both the
# sim's and the physical robot's own RBX interface (device_if_rbx.py's own
# SUBS_DICT), so the same message can be re-published as-is with no field
# mapping. This also sidesteps a catkin-rebuild-and-restart risk for what is
# otherwise a pure application-layer feature (same reasoning
# device_if_rbx.py's own set_teleop_velocity comment gives for reusing
# geometry_msgs/Twist rather than inventing a new message).
#
# Deliberately NOT mirrored, and why:
#   - set_image_topic, enable_image_overlay, publish_status, publish_info,
#     set_process_name, set_navpose_frame: viewer/bookkeeping controls, not
#     "what happens to the robot" in the sense this feature is for.
#
# Safety: enabling the relay requires an explicit, standing
# acknowledge_props_off(True) call in addition to selecting both robots --
# see _start_locked. Changing either robot selection, or withdrawing the
# acknowledgment, immediately disables an active link rather than leaving it
# running against a stale target.

import copy
import threading

from std_msgs.msg import Int32, UInt32, Empty
from geometry_msgs.msg import Twist, PoseStamped
from geographic_msgs.msg import GeoPoint, GeoPoseStamped
from mavros_msgs.msg import AttitudeTarget
from nepi_interfaces.msg import MotorControl, GotoLocation, GotoPosition, GotoPose, ErrorBounds
from nepi_interfaces.srv import RBXCapabilitiesQuery, RBXCapabilitiesQueryRequest

from nepi_sdk import nepi_sdk

# A setup_action name that is NEVER forwarded to the physical robot, however
# the two devices' own RBX_SETUP_ACTIONS lists line up -- teleporting/
# resetting a real vehicle's position has no meaning and no safe physical
# analogue, unlike TAKEOFF/LAUNCH which are meaningful arm+mode+motor
# sequences on real hardware too.
NEVER_MIRRORED_ACTIONS = ('RESET_SIM',)

# mavros setpoint topic (relative to each device's own mavlink_<id> node
# namespace, a SIBLING of the RBX device's own namespace, not nested under
# it) -> message type. Requested live 2026-09-16: "launching and the drone
# flying in gazebo doesnt affect the physical drone at all... maybe it
# might be better then to constantly send the commands... and translate it
# over". Once GUIDED-mode autonomous flight is underway (from LAUNCH or any
# goto_*), rbx_ardupilot_node.py's own sendGotoCommandLoop streams
# position/attitude SETPOINTS directly to its own mavros connection --
# entirely inside the MAVLink layer, never touching the RBX ROS command
# topics MIRRORED_TOPICS relays. Mirroring only those RBX-level commands
# (arm/mode/goto/motor-test) is why the physical drone armed and launched
# correctly but then went no further: nothing carries the ONGOING setpoint
# stream that is actually flying the sim. Two other approaches were
# considered and rejected: MAV_CMD_DO_MOTOR_TEST (this driver's own
# set_motor_control) is explicitly a ground-test command with no relation
# to flight once airborne; RC_CHANNELS_OVERRIDE represents pilot STICK
# axes, which GUIDED mode ignores entirely for its own internal control
# loop, so it has nothing to feed the sim's autonomous flight into anyway.
# Relaying the actual setpoint stream instead lets the physical drone's OWN
# flight controller compute its own motor response to the SAME target the
# sim is tracking -- each vehicle keeps its own control/safety loop, only
# the target is shared.
#
# Only wired when BOTH devices' own device_node_name starts with
# "ardupilot_" (see _maybe_wire_mavlink_setpoint_relay) -- this scheme is
# ArduPilot/mavros-specific, not a generic RBX contract, and both ends
# happening to share a driver class (ArduPilot serial + ArduPilot SITL) is
# what makes the mavlink_<id> sibling-namespace derivation below safe to
# assume.
MAVLINK_SETPOINT_TOPICS = {
  'setpoint_position/local': PoseStamped,
  'setpoint_position/global': GeoPoseStamped,
  'setpoint_raw/attitude': AttitudeTarget,
}

# Reported live 2026-09-16: mirroring worked "up to an instant" (while the sim
# was actively mid-goto), then the physical drone "just sat in armed and
# guided mode, but not really flying or hovering anymore -- the motors just
# doing their default arm thing". Root cause: sendGotoCommandLoop only
# republishes a setpoint WHILE a goto is actively converging (its own loop
# checks status_msg.ready == False); once the target is reached (or the
# attempt times out) it stops publishing entirely and the sim's OWN flight
# controller holds position from then on using nothing but its internal EKF
# -- no further ROS traffic to relay. The physical drone's mavros then has no
# guided-mode target fresher than that one last message, and ArduPilot's own
# "no recent guided target" handling is exactly the idle-pulsing behavior
# reported. Fixed by caching the latest message per setpoint topic and
# re-publishing it continuously on a timer (with a freshened header stamp)
# instead of relaying only on receipt -- the physical drone's own flight
# controller then always has a live, recent target to track, whether the sim
# is actively moving or holding still.
MAVLINK_SETPOINT_STREAM_RATE_HZ = 10.0


# topic name (relative to a device's own ".../rbx" namespace) -> message type,
# for every RBX command this relay mirrors from sim to physical robot.
MIRRORED_TOPICS = {
  'set_state': Int32,
  'set_mode': Int32,
  'set_motor_control': MotorControl,
  'set_teleop_velocity': Twist,
  'go_action': Int32,
  'set_goto_timeout': UInt32,
  'go_home': Empty,
  'set_home': GeoPoint,
  'set_home_current': GotoLocation,
  'goto_location': GotoLocation,
  'goto_position': GotoPosition,
  'goto_pose': GotoPose,
  'go_stop': Empty,
  'set_goto_error_bounds': ErrorBounds,
}


class RobotLinkRelay:
  def __init__(self, msg_if, log_name_list = None):
    self.msg_if = msg_if
    self.log_name_list = log_name_list if log_name_list is not None else []
    self.lock = threading.Lock()

    self.sim_namespace = ""
    self.physical_namespace = ""
    self.props_off_acknowledged = False
    self.enabled = False
    self.last_error = ""

    self._subs = {}   # topic_name -> rospy Subscriber, on sim_namespace
    self._pubs = {}   # topic_name -> rospy Publisher, on physical_namespace

    # mavlink setpoint continuous-restream state (see MAVLINK_SETPOINT_
    # STREAM_RATE_HZ's own comment). _mavlink_lock is separate from self.lock:
    # the stream timer callback must never risk blocking behind self.lock,
    # which _start_locked/_stop_locked can hold for the whole (re)wiring
    # pass -- a timer tick landing mid-rewire would otherwise deadlock or
    # publish through a half-torn-down publisher.
    self._mavlink_lock = threading.Lock()
    self._mavlink_latest_msgs = {}   # 'mavlink:<topic_name>' -> last received msg

  def select_sim_robot(self, sim_namespace):
    sim_namespace = str(sim_namespace)
    with self.lock:
      if sim_namespace != self.sim_namespace and self.enabled:
        self._stop_locked("sim robot selection changed")
      self.sim_namespace = sim_namespace

  def select_physical_robot(self, physical_namespace):
    physical_namespace = str(physical_namespace)
    with self.lock:
      if physical_namespace != self.physical_namespace:
        # Withdraw any prior acknowledgment -- "props are off" is a
        # statement about a SPECIFIC physical vehicle, not a standing
        # preference that should carry over to a newly selected one.
        self.props_off_acknowledged = False
        if self.enabled:
          self._stop_locked("physical robot selection changed")
      self.physical_namespace = physical_namespace

  def acknowledge_props_off(self, acknowledged):
    with self.lock:
      self.props_off_acknowledged = bool(acknowledged)
      if not self.props_off_acknowledged and self.enabled:
        self._stop_locked("props-off acknowledgment withdrawn")

  def set_enabled(self, enabled):
    with self.lock:
      if enabled:
        self._start_locked()
      else:
        self._stop_locked("disabled by operator")

  def _start_locked(self):
    if self.enabled:
      return
    if self.sim_namespace == "" or self.physical_namespace == "":
      self.last_error = "Select both a sim and a physical robot first"
      return
    if self.sim_namespace == self.physical_namespace:
      self.last_error = "Sim and physical robot cannot be the same device"
      return
    if not self.props_off_acknowledged:
      self.last_error = "Propellers/blades must be confirmed removed before linking"
      return

    new_subs = {}
    new_pubs = {}
    for topic_name, msg_type in MIRRORED_TOPICS.items():
      pub = nepi_sdk.create_publisher(self.physical_namespace + '/' + topic_name,
                                      msg_type, queue_size = 5)
      sub = nepi_sdk.create_subscriber(self.sim_namespace + '/' + topic_name, msg_type,
                                       self._relay_cb, queue_size = 5,
                                       callback_args = (topic_name, pub))
      if pub is None or sub is None:
        self.last_error = "Failed to wire relay topic: " + topic_name
        for s in new_subs.values():
          try:
            s.unregister()
          except Exception:
            pass
        for p in new_pubs.values():
          try:
            p.unregister()
          except Exception:
            pass
        return
      new_pubs[topic_name] = pub
      new_subs[topic_name] = sub

    # setup_action (TAKEOFF/LAUNCH/RESET_SIM/...) is wired separately from
    # MIRRORED_TOPICS, translated by ACTION NAME rather than passed through
    # by raw index: each driver's own RBX_SETUP_ACTIONS list assigns these
    # names to whatever integer indices it likes, so blindly forwarding the
    # sim's index could fire a same-numbered but different-meaning action on
    # the physical robot (or, worse, a real takeoff from an index that meant
    # something harmless on the sim side). Querying both devices' own
    # capabilities_query service and mapping by name is safe regardless of
    # whether the two devices happen to share a driver class with identical
    # indices (as ArduPilot serial + ArduPilot SITL do) or not. Requested
    # live 2026-09-16: "when launched and the sim drone becomes guided and
    # armed, the same thing should happen to the physical drone too" --
    # LAUNCH/TAKEOFF call the driver's own arm/mode/takeoff methods directly
    # (see rbx_ardupilot_node.py's launch()), never publishing to set_mode/
    # set_state themselves, so mirroring only those two topics (already in
    # MIRRORED_TOPICS) never actually mirrors what LAUNCH does.
    sim_caps = self._query_capabilities(self.sim_namespace)
    phys_caps = self._query_capabilities(self.physical_namespace)
    sim_actions = list(sim_caps.setup_action_options) if sim_caps is not None else []
    phys_actions = list(phys_caps.setup_action_options) if phys_caps is not None else []
    phys_index_by_name = {name: i for i, name in enumerate(phys_actions)
                          if name not in NEVER_MIRRORED_ACTIONS}
    sim_index_to_phys_index = {i: phys_index_by_name[name]
                               for i, name in enumerate(sim_actions)
                               if name in phys_index_by_name and name not in NEVER_MIRRORED_ACTIONS}
    if sim_index_to_phys_index:
      pub = nepi_sdk.create_publisher(self.physical_namespace + '/setup_action',
                                      Int32, queue_size = 5)
      sub = nepi_sdk.create_subscriber(self.sim_namespace + '/setup_action', Int32,
                                       self._relay_setup_action_cb, queue_size = 5,
                                       callback_args = (sim_index_to_phys_index, pub))
      if pub is not None and sub is not None:
        new_pubs['setup_action'] = pub
        new_subs['setup_action'] = sub
      else:
        self.msg_if.pub_warn("Robot link: failed to wire setup_action relay, continuing without it",
                             log_name_list = self.log_name_list)
    else:
      self.msg_if.pub_info("Robot link: no common setup actions between sim and physical robot "
                          "(or capabilities_query unavailable), TAKEOFF/LAUNCH will not be mirrored",
                          log_name_list = self.log_name_list)

    self._wire_mavlink_setpoint_relay(sim_caps, phys_caps, new_subs, new_pubs)

    self._pubs = new_pubs
    self._subs = new_subs
    self.enabled = True
    self.last_error = ""
    self.msg_if.pub_info("Robot link enabled: " + self.sim_namespace + " -> " +
                        self.physical_namespace, log_name_list = self.log_name_list)

  def _query_capabilities(self, namespace):
    # Short, bounded wait -- a driver with no capabilities_query service (or
    # one that's slow to come up) should not block enabling the rest of the
    # link for 60s (nepi_sdk.wait_for_service's own default). Absence just
    # means setup_action/setpoint mirroring is skipped for this link, logged
    # by each caller.
    service_name = namespace + '/capabilities_query'
    found = nepi_sdk.wait_for_service(service_name, timeout = 3, log_name_list = self.log_name_list)
    if not found:
      return None
    service = nepi_sdk.connect_service(service_name, RBXCapabilitiesQuery,
                                       log_name_list = self.log_name_list)
    return nepi_sdk.call_service(service, RBXCapabilitiesQueryRequest(),
                                 verbose = False, log_name_list = self.log_name_list)

  def _wire_mavlink_setpoint_relay(self, sim_caps, phys_caps, new_subs, new_pubs):
    # See MAVLINK_SETPOINT_TOPICS' own module-level comment for why this
    # exists and why the other two approaches (MAV_CMD_DO_MOTOR_TEST,
    # RC_CHANNELS_OVERRIDE) don't work for mirroring in-flight behavior.
    if sim_caps is None or phys_caps is None:
      self.msg_if.pub_info("Robot link: capabilities_query unavailable for sim or physical robot, "
                          "in-flight setpoint mirroring will not be wired",
                          log_name_list = self.log_name_list)
      return
    sim_node_name = str(sim_caps.device_node_name)
    phys_node_name = str(phys_caps.device_node_name)
    if not sim_node_name.startswith('ardupilot_') or not phys_node_name.startswith('ardupilot_'):
      self.msg_if.pub_info("Robot link: in-flight setpoint mirroring only supports ArduPilot on "
                          "both ends (device_node_name " + sim_node_name + " / " + phys_node_name +
                          "), skipping",
                          log_name_list = self.log_name_list)
      return

    # rbx_ardupilot_discovery.py's own launchDeviceNode names the ardupilot
    # RBX node "ardupilot_<device_id_str>" and its mavros node
    # "mavlink_<device_id_str>" from the SAME device_id_str -- e.g.
    # "ardupilot_sitl" always has a sibling "mavlink_sitl". The mavlink node
    # is NOT nested under the RBX device's own namespace (".../ardupilot_
    # sitl/rbx"), it's a sibling directly under the shared root (".../
    # mavlink_sitl"), so this replaces the RBX namespace's device segment
    # rather than appending under it.
    def mavlink_namespace(rbx_namespace, ardupilot_node_name, mavlink_node_name):
      device_base = rbx_namespace.split('/rbx')[0]
      root = device_base[:device_base.rfind('/' + ardupilot_node_name)]
      return root + '/' + mavlink_node_name

    sim_mavlink_ns = mavlink_namespace(self.sim_namespace, sim_node_name,
                                       'mavlink_' + sim_node_name.split('_', 1)[1])
    phys_mavlink_ns = mavlink_namespace(self.physical_namespace, phys_node_name,
                                        'mavlink_' + phys_node_name.split('_', 1)[1])

    wired_any = False
    for topic_name, msg_type in MAVLINK_SETPOINT_TOPICS.items():
      topic_key = 'mavlink:' + topic_name
      pub = nepi_sdk.create_publisher(phys_mavlink_ns + '/' + topic_name, msg_type, queue_size = 5)
      sub = nepi_sdk.create_subscriber(sim_mavlink_ns + '/' + topic_name, msg_type,
                                       self._cacheAndRelayMavlinkSetpointCb, queue_size = 5,
                                       callback_args = (topic_key,))
      if pub is None or sub is None:
        self.msg_if.pub_warn("Robot link: failed to wire mavlink setpoint relay topic: " + topic_name,
                             log_name_list = self.log_name_list)
        if pub is not None:
          try:
            pub.unregister()
          except Exception:
            pass
        continue
      new_pubs[topic_key] = pub
      new_subs[topic_key] = sub
      wired_any = True

    # Kick off the continuous-restream timer (see MAVLINK_SETPOINT_STREAM_
    # RATE_HZ's own comment) -- self-rescheduling oneshot rather than a
    # repeating timer, since nepi_sdk.start_timer_process returns only a
    # success bool, not a handle this class could later shut down; the
    # callback's own `if not self.enabled: return` guard ends the chain
    # cleanly once _stop_locked flips that flag, with no leaked timer.
    if wired_any:
      with self._mavlink_lock:
        self._mavlink_latest_msgs = {}
      nepi_sdk.start_timer_process(1.0 / MAVLINK_SETPOINT_STREAM_RATE_HZ,
                                   self._mavlinkStreamCb, oneshot = True)

  def _stop_locked(self, reason):
    if not self.enabled and not self._subs:
      return
    for sub in self._subs.values():
      try:
        sub.unregister()
      except Exception:
        pass
    for pub in self._pubs.values():
      try:
        pub.unregister()
      except Exception:
        pass
    self._subs = {}
    self._pubs = {}
    with self._mavlink_lock:
      self._mavlink_latest_msgs = {}
    was_enabled = self.enabled
    self.enabled = False
    if was_enabled:
      self.msg_if.pub_info("Robot link disabled: " + reason, log_name_list = self.log_name_list)

  def _relay_cb(self, msg, args):
    topic_name, pub = args
    try:
      pub.publish(msg)
    except Exception as e:
      self.msg_if.pub_warn("Robot link relay failed on " + topic_name + ": " + str(e),
                           log_name_list = self.log_name_list, throttle_s = 5.0)

  def _cacheAndRelayMavlinkSetpointCb(self, msg, args):
    # Immediate relay (low latency while the sim is actively moving) AND
    # cache the latest value so _mavlinkStreamCb can keep re-publishing it
    # even after the sim stops sending new ones (see MAVLINK_SETPOINT_
    # STREAM_RATE_HZ's own comment for why that gap existed).
    topic_key = args[0]
    with self._mavlink_lock:
      self._mavlink_latest_msgs[topic_key] = msg
    pub = self._pubs.get(topic_key)
    if pub is not None:
      self._publishMavlinkSetpoint(topic_key, pub, msg)

  def _publishMavlinkSetpoint(self, topic_key, pub, msg):
    try:
      out = copy.deepcopy(msg)
      if hasattr(out, 'header'):
        # Freshen the stamp on every re-publish (including replays of a
        # cached message the sim itself hasn't touched in a while) -- the
        # physical FC needs to see this as a live, current target, not
        # stale data from whenever the sim last actually changed it.
        out.header.stamp = nepi_sdk.get_msg_stamp()
      pub.publish(out)
    except Exception as e:
      self.msg_if.pub_warn("Robot link relay failed on " + topic_key + ": " + str(e),
                           log_name_list = self.log_name_list, throttle_s = 5.0)

  def _mavlinkStreamCb(self, timer):
    if not self.enabled:
      # Link was disabled since this chain's last tick -- end it here
      # rather than rescheduling again. No timer handle to explicitly
      # cancel (see this chain's own start-site comment).
      return
    with self._mavlink_lock:
      items = list(self._mavlink_latest_msgs.items())
    for topic_key, msg in items:
      pub = self._pubs.get(topic_key)
      if pub is not None:
        self._publishMavlinkSetpoint(topic_key, pub, msg)
    nepi_sdk.start_timer_process(1.0 / MAVLINK_SETPOINT_STREAM_RATE_HZ,
                                 self._mavlinkStreamCb, oneshot = True)

  def _relay_setup_action_cb(self, msg, args):
    sim_index_to_phys_index, pub = args
    phys_index = sim_index_to_phys_index.get(msg.data)
    if phys_index is None:
      # Either NEVER_MIRRORED_ACTIONS (e.g. RESET_SIM) or an action this
      # particular physical robot doesn't have -- silently dropped by
      # design, not a relay failure.
      return
    try:
      pub.publish(Int32(data = phys_index))
    except Exception as e:
      self.msg_if.pub_warn("Robot link relay failed on setup_action: " + str(e),
                           log_name_list = self.log_name_list, throttle_s = 5.0)

  def get_status_dict(self):
    with self.lock:
      return dict(
        sim_namespace = self.sim_namespace,
        physical_namespace = self.physical_namespace,
        props_off_acknowledged = self.props_off_acknowledged,
        enabled = self.enabled,
        last_error = self.last_error,
      )
