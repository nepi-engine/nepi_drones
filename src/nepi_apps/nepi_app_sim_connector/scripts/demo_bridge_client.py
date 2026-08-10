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

# Reference/demo simulator-side bridge for sim_connector_app_node.py.
#
# NOT a real simulator integration -- there is no actual simulator behind
# this, just synthetic motion. Its only job is to speak the exact wire
# protocol documented in sim_connector_app_node.py's own header comment
# (newline-delimited JSON, dispatched by "type" key presence), so:
#   (a) you can point sim_connector_app_node.py at a live TCP client and
#       actually see bridge_connected flip to True, telemetry flow, and a
#       robot config's commands (motor/goto/etc.) get logged, without needing
#       a real simulator running yet, and
#   (b) a real simulator bridge script can be written by copying this file's
#       shape and swapping the synthetic telemetry/command-handling for real
#       calls into that simulator.
#
# Deliberately has ZERO ros/nepi_sdk dependency, matching the wire protocol's
# own design point: a simulator's bridge script can live entirely outside any
# ROS environment (see sim_connector_app_node.py's module docstring).
#
# Usage:
#   python3 demo_bridge_client.py --profile rover
#   python3 demo_bridge_client.py --profile drone --host 192.168.1.50 --port 9030

import argparse
import base64
import json
import math
import socket
import threading
import time

try:
  import numpy as np
  import cv2
  HAVE_CV2 = True
except ImportError:
  HAVE_CV2 = False

TELEMETRY_RATE_HZ = 10.0
ANNOUNCE_INTERVAL_SEC = 5.0
IMAGE_RATE_HZ = 2.0
RECONNECT_INTERVAL_SEC = 3.0

# Simple circular motion so telemetry is visibly live rather than frozen --
# same "prove it's really moving" reasoning as this session's other sim
# controllers (e.g. ai_targeting_controller_ardupilot.py's circling target).
CIRCLE_RADIUS_M = 3.0
CIRCLE_PERIOD_SEC = 20.0

PROFILES = {
  'rover': {
    'sensor_topics': [('demo_bridge/camera/image_raw', 'sensor_msgs/Image')],
    'environment_options': ['obstacle_course'],
  },
  'drone': {
    'sensor_topics': [('demo_bridge/camera/image_raw', 'sensor_msgs/Image')],
    'environment_options': [],
  },
}


def buildTelemetryLine(profile, start_time):
  t = time.time() - start_time
  theta = 2.0 * math.pi * (t / CIRCLE_PERIOD_SEC)
  x_m = CIRCLE_RADIUS_M * math.cos(theta)
  y_m = CIRCLE_RADIUS_M * math.sin(theta)
  yaw_deg = math.degrees(theta + math.pi / 2.0) % 360.0

  line = {'x_m': x_m, 'y_m': y_m, 'yaw_deg': yaw_deg}
  if profile == 'drone':
    # Slow bob in altitude so has_altitude/has_position both stay exercised.
    z_m = -5.0 + 1.0 * math.sin(theta)
    line['z_m'] = z_m
    line['roll_deg'] = 2.0 * math.sin(theta)
    line['pitch_deg'] = 2.0 * math.cos(theta)
    line['altitude_m'] = 50.0 + z_m
    line['latitude'] = 47.6540828 + 0.00001 * x_m
    line['longitude'] = -122.3187578 + 0.00001 * y_m
  return line


def buildSyntheticImageLine(profile):
  if not HAVE_CV2:
    return None
  img = np.zeros((120, 160, 3), dtype=np.uint8)
  img[:, :] = (60, 90, 60) if profile == 'rover' else (90, 60, 60)
  cv2.putText(img, profile, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
  ok, encoded = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 60])
  if not ok:
    return None
  return {
    'type': 'image',
    'data': base64.b64encode(encoded.tobytes()).decode('ascii'),
    'stamp': time.time(),
  }


def sendLine(sock, line_dict):
  sock.sendall((json.dumps(line_dict) + '\n').encode())


def senderLoop(sock, profile, stop_event):
  start_time = time.time()
  last_announce = 0.0
  last_image = 0.0
  try:
    while not stop_event.is_set():
      now = time.time()
      sendLine(sock, buildTelemetryLine(profile, start_time))

      if now - last_announce >= ANNOUNCE_INTERVAL_SEC:
        cfg = PROFILES[profile]
        sendLine(sock, {'type': 'sensor_topics', 'topics': [
          {'topic_name': t, 'msg_type': m} for t, m in cfg['sensor_topics']
        ]})
        sendLine(sock, {'type': 'environment_options', 'options': cfg['environment_options']})
        last_announce = now

      if HAVE_CV2 and now - last_image >= 1.0 / IMAGE_RATE_HZ:
        img_line = buildSyntheticImageLine(profile)
        if img_line is not None:
          sendLine(sock, img_line)
        last_image = now

      time.sleep(1.0 / TELEMETRY_RATE_HZ)
  except Exception as e:
    print("[demo_bridge_client] Sender stopped: " + repr(e))
    stop_event.set()


def receiverLoop(sock, stop_event):
  buf = b''
  try:
    while not stop_event.is_set():
      data = sock.recv(4096)
      if not data:
        print("[demo_bridge_client] Server closed the connection (EOF)")
        stop_event.set()
        return
      buf += data
      while b'\n' in buf:
        line, buf = buf.split(b'\n', 1)
        if not line.strip():
          continue
        try:
          msg = json.loads(line)
        except Exception as e:
          print("[demo_bridge_client] Bad line from server: " + str(e))
          continue
        print("[demo_bridge_client] RECEIVED: " + json.dumps(msg))
  except Exception as e:
    print("[demo_bridge_client] Receiver stopped: " + repr(e))
    stop_event.set()


def runOnce(host, port, profile):
  sock = socket.create_connection((host, port), timeout=5)
  print("[demo_bridge_client] Connected to " + host + ":" + str(port) +
        " as profile '" + profile + "'" +
        (" (no cv2/numpy -- image frames disabled)" if not HAVE_CV2 else ""))
  stop_event = threading.Event()
  recv_thread = threading.Thread(target=receiverLoop, args=(sock, stop_event))
  recv_thread.daemon = True
  recv_thread.start()
  try:
    senderLoop(sock, profile, stop_event)
  finally:
    stop_event.set()
    try:
      sock.close()
    except Exception:
      pass


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--host', default='127.0.0.1',
                      help="Host running sim_connector_app_node.py (default: 127.0.0.1)")
  parser.add_argument('--port', type=int, default=9030,
                      help="Bridge listen port (default: 9030, matches FACTORY_LISTEN_PORT)")
  parser.add_argument('--profile', choices=sorted(PROFILES.keys()), default='rover',
                      help="Synthetic vehicle shape to emit telemetry for (default: rover)")
  args = parser.parse_args()

  print("[demo_bridge_client] Reference bridge -- NOT a real simulator, synthetic motion only.")
  print("[demo_bridge_client] Select the matching robot config ('ground_robot_2_wheel' for "
        "rover, 'flight_robot_4_motor' for drone) via the app's RUI or select_robot_config "
        "topic to see full controls.")
  while True:
    try:
      runOnce(args.host, args.port, args.profile)
    except Exception as e:
      print("[demo_bridge_client] Connect to " + args.host + ":" + str(args.port) +
            " failed: " + str(e) + " -- retrying in " + str(RECONNECT_INTERVAL_SEC) + "s")
    time.sleep(RECONNECT_INTERVAL_SEC)


if __name__ == '__main__':
  main()
