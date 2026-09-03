#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# SITL/Gazebo-only test scaffolding -- NOT a real app_ai_targeting
# replacement. Stands in for the missing app_ai_targeting app documented as a
# "KNOWN GAP" in drone_follow_object_mission_script.py's own module docstring,
# so that script's follow logic can actually be exercised against ArduPilot
# SITL. Deploy and launch this exactly like a normal NEPI automation script
# (get_scripts/launch_script), alongside drone_follow_object_mission_script.py.
#
# What this does:
#   - Connects (retry/reconnect loop, mirroring rbx_sim_node.py's own
#     bridgeLoop pattern) to the dev VM's ai_targeting_controller_ardupilot.py
#     bridge at 127.0.0.1:9027 (forwarded over the existing reverse SSH
#     tunnel set up by nepi_tunnel() in nepi_sitl_dev_env.sh -- no new
#     tunnel/credentials needed on this side).
#   - Parses each newline-delimited JSON "target" line (target_name, range_m,
#     azimuth_deg, elevation_deg) and republishes it as a Targets/Target
#     message on <base_namespace>app_ai_targeting/target_localizations --
#     the exact topic drone_follow_object_mission_script.py already waits
#     for and subscribes to, unmodified.
#   - Finds the RBX ArduPilot driver's own relayed camera feed
#     ("<device_name>/color_2d_image", see rbx_ardupilot_node.py's
#     image_topic_name -- or "idx/color_image" as a real-camera fallback,
#     same convention as this repo's other camera-consuming scripts) and
#     republishes it verbatim on <base_namespace>app_ai_targeting/targeting_image
#     -- a thin passthrough, not a new image pipeline, so the mission
#     script's wait_for_topic/set_image_topic calls resolve against a real
#     live feed. Non-fatal if no camera topic is found -- the
#     target_localizations feed below is this script's actual job and starts
#     regardless.
#   - On startup, unconditionally makes one fire-and-forget connection to
#     127.0.0.1:9028 (sim_launch_listener.py on the dev VM, same
#     tunnel-forwarded-loopback pattern as everything else here) to trigger
#     sitl_gazebo_full remotely -- so launching this one script from the RUI
#     is enough to bring up the whole chain (Gazebo/SITL/camera-rig/
#     AI-targeting controllers) if it isn't already running, not just relay a
#     feed that requires all of that to already exist. Always triggered, not
#     gated on checking whether the RBX driver already looks live: that node
#     can keep running (and keep rbx/status published) even after the VM-side
#     SITL it was talking to has died, so it's not a reliable signal here --
#     sitl_gazebo_full itself is idempotent and checks the VM's real
#     processes, making a harmless no-op when everything's actually already
#     up. This is a one-shot trigger, not a dependency -- if the VM/tunnel
#     isn't reachable at all (nothing has ever been started on the VM this
#     session), this just logs a warning and the bridge/reconnect loop below
#     keeps retrying as it always would.

import rospy
import json
import socket
import threading
import time

from nepi_sdk import nepi_ros
from nepi_api.messages_if import MsgIF

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from nepi_interfaces.msg import Target, Targets, ImageTarget

#########################################
# USER SETTINGS - Edit as Necessary
#########################################
BRIDGE_HOST = "127.0.0.1"
BRIDGE_PORT = 9027
SOCKET_TIMEOUT_SEC = 5.0
RECONNECT_INTERVAL_SEC = 3.0

# sim_launch_listener.py on the dev VM (see nepi_sitl_dev_env.sh) -- one-shot
# trigger for sitl_gazebo_full, not a persistent connection like BRIDGE_PORT
# above.
LAUNCH_TRIGGER_HOST = "127.0.0.1"
LAUNCH_TRIGGER_PORT = 9028
LAUNCH_TRIGGER_TIMEOUT_SEC = 3.0


# rbx_ardupilot_node.py publishes its own relayed camera under
# "<device_name>/color_2d_image" (see that file's image_topic_name) -- not a
# bare "image" suffix under the robot's namespace, which this script
# originally searched for (a stale assumption from before the camera-rig
# relay convention existed on the deployed driver). Real IDX camera fallback
# included for the same reason the other camera-consuming scripts have one.
ROBOT_IMAGE_TOPIC_CANDIDATES = ["color_2d_image", "idx/color_image"]

PROCESS_NAME = "sim_ai_targeting_bridge"

# Requested live (2026-09-03): "put some sort of pointer on the object for
# detection" -- the marker color for the live target crosshair overlay
# (see wireUpTargetOverlay/publishTargetOverlay below). Bright red so it
# reads clearly against grass/ground colors in the sim world.
TARGET_OVERLAY_COLOR_RGB = (255, 0, 0)


def find_image_topic(candidates, timeout = 20):
  # Polls nepi_ros.find_topic() across all candidates each tick, so total
  # worst-case wait is timeout, not timeout * len(candidates).
  start_time = time.time()
  while (time.time() - start_time) < timeout and not nepi_ros.is_shutdown():
    for candidate in candidates:
      found = nepi_ros.find_topic(candidate)
      if found != "":
        return found
    time.sleep(0.1)
  return ""

# How often imageRelayThread rechecks for the camera topic once the initial
# find_image_topic() window (below) comes up empty.
IMAGE_TOPIC_RETRY_INTERVAL_SEC = 10.0


LAUNCH_TRIGGER_RETRY_ATTEMPTS = 8
LAUNCH_TRIGGER_RETRY_INTERVAL_SEC = 3.0

def trigger_remote_sim_launch(msg_if):
  # Was a genuine one-shot (single connect attempt, no retry) -- confirmed
  # live 2026-08-26 that this loses a real race: if this script starts
  # around the same time as sitl_gazebo/sitl_gazebo_full on the dev VM
  # (e.g. both kicked off right after a device/VM reboot), the reverse SSH
  # tunnel's sshd accepts the TCP connection immediately (it doesn't know
  # sim_launch_listener isn't bound yet), then closes it the instant it
  # finds nothing listening on the VM side -- this script reads that as a
  # normal (if empty) reply and never tries again, so
  # ai_targeting_controller_ardupilot/camera_rig_controller_ardupilot never
  # actually start. The drone would still arm/take off fine (RBX driver
  # doesn't depend on this), but no chair would ever spawn and
  # move_to_object_callback would wait forever for a detection that never
  # arrives. Retrying a few times over ~20s comfortably covers
  # sim_launch_listener's own startup window (a few seconds into
  # sitl_gazebo's own sequence) without meaningfully delaying this script's
  # own init on the normal case where sim_launch_listener is already up.
  for attempt in range(LAUNCH_TRIGGER_RETRY_ATTEMPTS):
    try:
      sock = socket.create_connection((LAUNCH_TRIGGER_HOST, LAUNCH_TRIGGER_PORT),
                                       timeout = LAUNCH_TRIGGER_TIMEOUT_SEC)
      sock.settimeout(LAUNCH_TRIGGER_TIMEOUT_SEC)
      reply = sock.recv(200)
      sock.close()
      reply_str = reply.decode(errors = "replace").strip()
      if reply_str:
        msg_if.pub_info("Triggered remote sim launch: " + reply_str)
        return
      # Empty reply -- the exact signature of the tunnel-accepted-but-
      # nothing-was-listening race described above. A real
      # sim_launch_listener always replies "OK triggered" or "ERR ...".
      msg_if.pub_warn("Remote sim launch trigger got an empty reply (attempt " +
                       str(attempt + 1) + "/" + str(LAUNCH_TRIGGER_RETRY_ATTEMPTS) +
                       ") -- sim_launch_listener likely isn't bound yet, retrying...")
    except Exception as e:
      msg_if.pub_warn("Could not reach sim_launch_listener at " + LAUNCH_TRIGGER_HOST + ":" +
                       str(LAUNCH_TRIGGER_PORT) + " (" + str(e) + "), attempt " +
                       str(attempt + 1) + "/" + str(LAUNCH_TRIGGER_RETRY_ATTEMPTS) + " -- retrying...")
    if attempt < LAUNCH_TRIGGER_RETRY_ATTEMPTS - 1:
      time.sleep(LAUNCH_TRIGGER_RETRY_INTERVAL_SEC)
  msg_if.pub_warn("Giving up triggering remote sim launch after " +
                   str(LAUNCH_TRIGGER_RETRY_ATTEMPTS) + " attempts -- if the dev VM has never " +
                   "run sitl_gazebo/sitl_gazebo_full this session, start one of those there " +
                   "manually")


class sim_ai_targeting_bridge(object):

  DEFAULT_NODE_NAME = "sim_ai_targeting_bridge"

  def __init__(self):
    nepi_ros.init_node(name = self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    self.msg_if = MsgIF(log_name = self.node_name)
    self.msg_if.pub_info("Starting Initialization Processes")

    TARGETS_TOPIC = self.base_namespace + "/app_ai_targeting/target_localizations"
    IMAGE_TOPIC = self.base_namespace + "/app_ai_targeting/targeting_image"
    self.targets_pub = nepi_ros.create_publisher(TARGETS_TOPIC, Targets, queue_size = 1)
    self.image_pub = nepi_ros.create_publisher(IMAGE_TOPIC, Image, queue_size = 1)

    # Live crosshair overlay on the robot's OWN image (not targeting_image,
    # which is just a passthrough relay with no overlay renderer of its
    # own) -- wired up lazily once imageRelayThread finds the robot's image
    # topic and can derive its sibling "image/" overlay namespace from it.
    # None until then; processBridgeLine no-ops the overlay call so it's
    # safe to receive target lines before this is ready.
    self.image_add_target_pub = None
    self.image_targets_enable_pub = None

    # Always trigger, unconditionally -- sitl_gazebo_full is idempotent and
    # checks real OS processes on the VM (not a ROS-level proxy), so this is
    # harmless when everything's already up. A "is the RBX driver already
    # live" pre-check was tried and removed: the RBX node process on this
    # device can stay alive and keep rbx/status published even after the
    # VM-side SITL it was talking to has died, making that check unreliable
    # for deciding whether the VM stack actually needs (re)launching.
    self.msg_if.pub_info("Attempting to trigger the full sim stack on the dev VM (harmless no-op if already running)")
    trigger_remote_sim_launch(self.msg_if)

    # Confirmed live 2026-08-28: a single 20s window at startup is not
    # enough -- this script's own process outlives many VM/device restart
    # cycles (it just keeps reconnecting bridgeLoop() below), but the RBX
    # driver's color_2d_image topic can easily still be initializing (or
    # mid-reload) at whatever moment THIS script happened to start, weeks
    # or hours ago. That one check failing once meant targeting_image never
    # got wired up for the rest of this script's lifetime, even after the
    # driver's camera topic came up fine later -- drone_follow_object_
    # mission_script.py's wait_for_topic(targeting_image) would then always
    # time out at 60s and get "" back, exactly like the same one-shot race
    # already fixed for trigger_remote_sim_launch above. Same fix here:
    # keep retrying in the background instead of giving up permanently.
    self.image_relay_thread = threading.Thread(target = self.imageRelayThread)
    self.image_relay_thread.daemon = True
    self.image_relay_thread.start()

    self.bridge_thread = threading.Thread(target = self.bridgeLoop)
    self.bridge_thread.daemon = True
    self.bridge_thread.start()

    self.msg_if.pub_info("Initialization Complete")
    rospy.spin()

  def imageRelayCb(self, msg):
    self.image_pub.publish(msg)

  def imageRelayThread(self):
    # Non-fatal either way: the image relay is a bonus (targeting_image),
    # not this script's actual job. The target_localizations feed handled
    # by bridgeLoop() is what drone_follow_object_mission_script.py and the
    # RUI's Peripheral Status check both actually depend on, and that
    # starts regardless of whether this ever finds a camera.
    self.msg_if.pub_info("Checking for image topic: " + str(ROBOT_IMAGE_TOPIC_CANDIDATES))
    while not nepi_ros.is_shutdown():
      source_image_topic = find_image_topic(ROBOT_IMAGE_TOPIC_CANDIDATES)
      if source_image_topic != "":
        self.msg_if.pub_info("Relaying image topic: " + source_image_topic)
        rospy.Subscriber(source_image_topic, Image, self.imageRelayCb, queue_size = 1)
        self.wireUpTargetOverlay(source_image_topic)
        return
      self.msg_if.pub_warn("No camera image topic found yet -- rechecking in " +
                           str(IMAGE_TOPIC_RETRY_INTERVAL_SEC) + "s")
      time.sleep(IMAGE_TOPIC_RETRY_INTERVAL_SEC)

  def wireUpTargetOverlay(self, source_image_topic):
    # source_image_topic is NOT usable here -- confirmed live 2026-09-03:
    # nepi_sdk.find_topic()'s non-exact branch (what find_image_topic above
    # calls) has a real bug, returning the SEARCH STRING it was given
    # ("color_2d_image") rather than the actual matched topic path it found
    # in the graph. rospy.Subscriber() a few lines up tolerates that by
    # accident (a bare relative name still resolves to *something*, just
    # not reliably the intended device's own topic), but there is no
    # namespace left in that string for this method to derive anything
    # from, and reusing the same rsplit trick on it (an earlier version of
    # this method did) silently produced a nonexistent overlay topic and
    # never published anything.
    #
    # Independently re-resolves the real path via rospy.get_published_topics()
    # directly instead. The generic per-device image-overlay topics
    # (add_target_degree_offsets, targets_enable, ...; same family as
    # add_target_pixel/overlay_target_pixels seen elsewhere in this app's
    # own image utils) live as siblings of "color_2d_image" under
    # "<device_ns>/image/", e.g. ".../ardupilot_sitl/image/
    # add_target_degree_offsets" next to ".../ardupilot_sitl/color_2d_image"
    # -- so splitting a REAL match on the literal "/color_2d_image"
    # component gives the right device namespace.
    device_ns = None
    try:
      for topic_name, _msg_type in rospy.get_published_topics():
        if '/color_2d_image' in topic_name:
          device_ns = topic_name.split('/color_2d_image')[0]
          break
    except Exception as e:
      self.msg_if.pub_warn("Could not resolve device namespace for target overlay: " + str(e))
    if device_ns is None:
      self.msg_if.pub_warn("No real color_2d_image topic found in the graph -- target "
                           "overlay marker will not be available this run")
      return
    self.image_add_target_pub = nepi_ros.create_publisher(
        device_ns + "/image/add_target_degree_offsets", ImageTarget, queue_size = 1)
    self.image_targets_enable_pub = nepi_ros.create_publisher(
        device_ns + "/image/targets_enable", Bool, queue_size = 1, latch = True)
    time.sleep(0.5)  # let the publishers register before the first send
    self.image_targets_enable_pub.publish(Bool(data = True))
    self.msg_if.pub_info("Target overlay wired up on " + device_ns + "/image")

  def publishTargetOverlay(self, target):
    # Requested live (2026-09-03): "put some sort of pointer on the object
    # for detection ... as it keeps updating, constantly follow it" -- a
    # live crosshair marker on the robot's own image showing exactly where
    # the currently-tracked target actually is, updated on every detection
    # (not just left showing wherever the drone last visited). Named by
    # target.name, so repeated calls here replace the same marker in place
    # (see nepi_api's add_target_degs: it keys its targets_dict by name and
    # only republishes when the entry actually changed) rather than
    # accumulating a trail of stale ones.
    #
    # y_offset_deg is negated: this sensor's own elevation_deg convention is
    # positive = up (mirrors ai_targeting_controller_ardupilot.py's Z-down
    # frame, negated -- see move_to_object_callback's own comment on this),
    # but the image overlay API's y offset follows image-row convention
    # (increasing = further down the frame, derived from its own
    # add_target_pixels: y_deg grows with y_ratio, and y_ratio 0 is the top
    # row) -- so a target above boresight (positive elevation) must map to
    # a NEGATIVE y offset to appear above center instead of below it.
    if self.image_add_target_pub is None:
      return
    msg = ImageTarget()
    msg.name = target.name
    msg.x_offset_deg = target.azimuth_deg
    msg.y_offset_deg = -target.elevation_deg
    msg.r, msg.g, msg.b = TARGET_OVERLAY_COLOR_RGB
    msg.msg_str = target.name + (" %.1fm" % target.range_m)
    self.image_add_target_pub.publish(msg)

  def bridgeLoop(self):
    buf = b''
    while not nepi_ros.is_shutdown():
      sock = None
      try:
        sock = socket.create_connection((BRIDGE_HOST, BRIDGE_PORT), timeout = SOCKET_TIMEOUT_SEC)
        sock.settimeout(SOCKET_TIMEOUT_SEC)
      except Exception as e:
        self.msg_if.pub_warn("Bridge connect to " + BRIDGE_HOST + ":" + str(BRIDGE_PORT) +
                             " failed: " + str(e))
        time.sleep(RECONNECT_INTERVAL_SEC)
        continue
      self.msg_if.pub_info("Connected to sim AI-targeting bridge at " + BRIDGE_HOST +
                           ":" + str(BRIDGE_PORT))
      buf = b''
      while not nepi_ros.is_shutdown():
        try:
          data = sock.recv(4096)
        except socket.timeout:
          # Server pushes at ~5 Hz -- a quiet-but-open socket past the
          # timeout means the far side is gone (e.g. tunnel half-open).
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
      try:
        sock.close()
      except Exception:
        pass
      self.msg_if.pub_warn("Sim AI-targeting bridge connection lost -- retrying in " +
                           str(RECONNECT_INTERVAL_SEC) + "s")
      time.sleep(RECONNECT_INTERVAL_SEC)

  def processBridgeLine(self, line):
    try:
      data = json.loads(line)
    except Exception as e:
      self.msg_if.pub_warn("Bad line from bridge: " + str(e))
      return
    if data.get('type') != 'target':
      return

    # Field names match Target.msg/Targets.msg exactly (nepi_interfaces) -- the
    # original version of this script used several fields (target_name,
    # target_uid, target_confidence, process_type, process_description,
    # source_type, has_range_data, has_bearing_data) that don't exist on
    # either message at all and crashed the bridge thread on the first real
    # line received. Confirmed by reading both .msg files directly.
    target = Target()
    target.timestamp = float(data.get('stamp', 0.0))
    target.name = str(data.get('target_name', ''))
    target.uid = target.name
    target.confidence = 1.0 if data.get('detected', False) else 0.0
    target.range_m = float(data.get('range_m', -999.0))
    target.azimuth_deg = float(data.get('azimuth_deg', -999.0))
    target.elevation_deg = float(data.get('elevation_deg', -999.0))

    targets_msg = Targets()
    targets_msg.timestamp = target.timestamp
    targets_msg.process_name = PROCESS_NAME
    targets_msg.process_namespace = self.node_name
    targets_msg.source_topic = BRIDGE_HOST + ":" + str(BRIDGE_PORT)
    targets_msg.source_timestamp = target.timestamp
    targets_msg.targets = [target]

    self.targets_pub.publish(targets_msg)
    if target.range_m != -999.0:
      self.publishTargetOverlay(target)


#########################################
# Main
#########################################
if __name__ == '__main__':
  sim_ai_targeting_bridge()
