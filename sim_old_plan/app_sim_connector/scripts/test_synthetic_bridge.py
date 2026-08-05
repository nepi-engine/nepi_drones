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

# Phase 2 test harness for sim_connector_node.py -- docs/SIMULATION_INTERFACE_SPEC.md's
# Implementation Plan, Test Cases 2.1-2.2. NOT production code, and NOT a real
# simulator bridge: a throwaway script that dials into sim_connector_node.py's
# TCP listen port (the reverse direction from sim_bridge_node.py, which is a
# real server) and sends hand-written lines matching the Phase 2 wire
# protocol, so the app's live state (capabilities_query / status) can be
# checked against real, running ROS state.
#
# Usage: python3 test_synthetic_bridge.py [--host HOST] [--port PORT] [--once]
#   --once sends one round of lines then exits immediately, for Test Case 2.1.
#   Without --once, stays connected and keeps pushing telemetry at 5Hz so
#   Test Case 2.2 (disconnect/reconnect) can be exercised by killing/
#   restarting this process while sim_connector_node.py's own status is
#   watched separately (rostopic echo).

import argparse
import json
import socket
import time


def send_line(sock, line_dict):
  sock.sendall((json.dumps(line_dict) + '\n').encode())


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--host', default='127.0.0.1')
  parser.add_argument('--port', type=int, default=9030)
  parser.add_argument('--once', action='store_true')
  args = parser.parse_args()

  sock = socket.create_connection((args.host, args.port), timeout=5.0)
  sock.settimeout(None)
  print("Connected to sim_connector_node.py at " + args.host + ":" + str(args.port), flush=True)

  # Test Case 2.1: sensor_topics + environment_options announcements, plus a
  # bare (no "type" key) telemetry line covering both local ENU and global
  # fields at once (proves the generalized shape handles a rover- or
  # drone-shaped simulator with the same wire format).
  send_line(sock, {
      'type': 'sensor_topics',
      'topics': [
          {'topic_name': '/synthetic/chase_cam/image_raw', 'msg_type': 'sensor_msgs/Image'},
          {'topic_name': '/synthetic/lidar', 'msg_type': 'sensor_msgs/LaserScan'},
      ],
  })
  send_line(sock, {
      'type': 'environment_options',
      'options': ['obstacle_course', 'night_mode'],
  })
  send_line(sock, {
      'x_m': 1.5, 'y_m': -2.5, 'z_m': 0.0,
      'roll_deg': 0.0, 'pitch_deg': 0.0, 'yaw_deg': 45.0,
      'latitude': 47.6, 'longitude': -122.3, 'altitude_m': 12.0,
      'x_m_per_sec': 0.1, 'y_m_per_sec': 0.0, 'z_m_per_sec': 0.0,
      'stamp': time.time(),
  })
  print("Sent sensor_topics, environment_options, and telemetry lines", flush=True)

  if args.once:
    time.sleep(1.0)  # give the server a moment to process before we close
    sock.close()
    return

  # Test Case 2.2: keep pushing telemetry so telemetry_age_sec stays low
  # while connected; the pass criteria (bridge_connected flips False, then
  # True again on reconnect) is exercised by killing/restarting this whole
  # process, not by any in-process logic here.
  try:
    while True:
      send_line(sock, {'x_m': 1.5, 'y_m': -2.5, 'yaw_deg': 45.0, 'stamp': time.time()})
      time.sleep(0.2)
  except (KeyboardInterrupt, BrokenPipeError, ConnectionResetError):
    pass
  finally:
    sock.close()


if __name__ == '__main__':
  main()
