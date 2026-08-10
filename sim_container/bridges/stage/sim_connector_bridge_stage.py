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

# ROS Stage bridge for nepi_app_sim_connector (Phase 3, MULTI_SIMULATOR_INTEGRATION_PLAN.md).
#
# Unlike the Gazebo and Webots bridges, this one is a plain ROS node with no
# apology for it -- Stage's entire interface already IS ROS topics
# (/cmd_vel in, /odom + /base_scan out), so writing a non-ROS bridge here
# would mean re-implementing a socket layer Stage doesn't need. This is the
# one bridge in the plan where "prefer no ROS on the simulator side" correctly
# does not apply.
#
# Also the plan's deliberate no-camera case: the stock willow-erratic.world
# has a LaserScan (sensor_msgs/LaserScan on /base_scan) and no camera at all.
# Announcing that scan as a sensor topic while leaving image-related fields
# empty is what exercises device_if_sim.py's SCAN_MSG_TYPES / has_camera
# derivation path honestly (has_camera stays False; a sensor topic still
# exists and is reported) -- most other bridges in this plan have a camera,
# so this is the one that proves the contract degrades correctly rather than
# assuming a camera always exists.
#
# Real, Stage-specific gotcha found and worked around here (see
# MULTI_SIMULATOR_INTEGRATION_PLAN.md's Phase 3 write-up): unlike Gazebo's
# diff-drive plugin, which LATCHES the last /cmd_vel indefinitely, Stage's
# robot model decays toward zero velocity if commands stop arriving --
# confirmed live (a one-shot `rostopic pub -1` barely moved the robot; 10Hz
# continuous publishing moved it as commanded). This bridge's control loop
# already re-publishes every tick regardless of idle/active state -- the same
# habit the Gazebo/Webots bridges already have for their own (different)
# reasons -- so no special-casing was needed here, just confirmation that the
# existing pattern is the right one.

import base64
import json
import math
import socket
import threading
import time

import rospy
import tf.transformations
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

DEFAULT_APP_HOST = "127.0.0.1"
DEFAULT_APP_PORT = 9030

ODOM_TOPIC = "/odom"
CMD_VEL_TOPIC = "/cmd_vel"
SCAN_TOPIC = "/base_scan"
SCAN_SENSOR_NAME = "stage_robot/base_scan"

RECONNECT_INTERVAL_SEC = 3.0
SOCKET_TIMEOUT_SEC = 5.0
TELEMETRY_RATE_HZ = 10.0
ANNOUNCE_INTERVAL_SEC = 5.0

# Stage's default "erratic" robot model has finite acceleration (unlike
# Gazebo's diff-drive plugin, which applies commanded velocity instantly) --
# confirmed live: 0.3 m/s commanded for 3s at 10Hz covered less distance than
# a no-accel-limit model would predict. Controller gains left modest so the
# turn/drive/turn phases don't fight that ramp.
CONTROLLER_RATE_HZ = 10.0
GOTO_KP_LIN = 0.4
GOTO_KP_ANG = 1.2
GOTO_TURN_GATE_RAD = math.radians(30.0)
GOTO_TOL_M = 0.3
GOTO_TOL_RAD = math.radians(3.0)
MAX_LINEAR_MPS = 0.3
MAX_ANGULAR_RADPS = math.radians(45.0)

MOTOR_MAX_LINEAR_MPS = 0.3
MOTOR_WHEEL_BASE_M = 0.4


class StageSimConnectorBridge:

  def __init__(self, app_host, app_port):
    self.app_host = app_host
    self.app_port = app_port

    self.pose_lock = threading.Lock()
    self.x_m = 0.0
    self.y_m = 0.0
    self.yaw_rad = 0.0
    self.lin_mps = 0.0
    self.ang_radps = 0.0

    self.goto_lock = threading.Lock()
    self.goto_target = None
    self.motor_ratios = [0.0, 0.0]

    self.home_lock = threading.Lock()
    self.home_x_m = 0.0
    self.home_y_m = 0.0

    self.scan_lock = threading.Lock()
    self.latest_scan_announced = False

    self.sock = None
    self.sock_lock = threading.Lock()

    rospy.init_node("sim_connector_bridge_stage", anonymous = False)

    self.cmd_vel_pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size = 1)
    rospy.Subscriber(ODOM_TOPIC, Odometry, self.odomCb, queue_size = 1)
    rospy.Subscriber(SCAN_TOPIC, LaserScan, self.scanCb, queue_size = 1)

    threading.Thread(target = self.bridgeLoop, daemon = True).start()
    rospy.Timer(rospy.Duration(1.0 / CONTROLLER_RATE_HZ), self.controlTickCb)

    rospy.loginfo("sim_connector_bridge_stage: connecting to %s:%d", self.app_host, self.app_port)
    rospy.spin()

  #**********************
  # Stage-side ROS callbacks

  def odomCb(self, msg):
    q = msg.pose.pose.orientation
    _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    with self.pose_lock:
      self.x_m = msg.pose.pose.position.x
      self.y_m = msg.pose.pose.position.y
      self.yaw_rad = yaw
      self.lin_mps = msg.twist.twist.linear.x
      self.ang_radps = msg.twist.twist.angular.z

  def scanCb(self, msg):
    # Only used to confirm the topic is alive for the sensor_topics
    # announcement -- no lidar data is relayed over the bridge wire protocol
    # today (out of scope for this phase; the point here is the typed
    # available_sensor_topics entry, not full LaserScan streaming).
    with self.scan_lock:
      self.latest_scan_announced = True

  #**********************
  # Closed-loop goto controller -- same shape as the Gazebo/Webots bridges.

  def normalizeAngle(self, angle_rad):
    while angle_rad > math.pi:
      angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
      angle_rad += 2.0 * math.pi
    return angle_rad

  def controlTickCb(self, event):
    with self.goto_lock:
      target = self.goto_target
      motor_ratios = list(self.motor_ratios)
    with self.pose_lock:
      cur_x, cur_y, cur_yaw = self.x_m, self.y_m, self.yaw_rad

    lin = 0.0
    ang = 0.0
    if target is not None:
      dx = target["x_m"] - cur_x
      dy = target["y_m"] - cur_y
      dist = math.hypot(dx, dy)
      if dist > GOTO_TOL_M:
        bearing_err = self.normalizeAngle(math.atan2(dy, dx) - cur_yaw)
        ang = max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, GOTO_KP_ANG * bearing_err))
        if abs(bearing_err) < GOTO_TURN_GATE_RAD:
          lin = max(0.0, min(MAX_LINEAR_MPS, GOTO_KP_LIN * dist))
      else:
        yaw_err = 0.0
        if target["yaw_deg"] is not None:
          yaw_err = self.normalizeAngle(math.radians(target["yaw_deg"]) - cur_yaw)
        if abs(yaw_err) > GOTO_TOL_RAD:
          ang = max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, GOTO_KP_ANG * yaw_err))
        else:
          with self.goto_lock:
            self.goto_target = None
          rospy.loginfo("sim_connector_bridge_stage: goto target reached")
    elif any(motor_ratios):
      left, right = motor_ratios[0], motor_ratios[1]
      lin = (left + right) / 2.0 * MOTOR_MAX_LINEAR_MPS
      ang = (right - left) / MOTOR_WHEEL_BASE_M * MOTOR_MAX_LINEAR_MPS

    # Published every tick regardless of idle/active -- required here (Stage
    # decays toward zero without fresh commands), and harmless everywhere
    # else this same pattern is used.
    twist = Twist()
    twist.linear.x = lin
    twist.angular.z = ang
    self.cmd_vel_pub.publish(twist)

  #**********************
  # sim_connector_app_node.py TCP client

  def bridgeLoop(self):
    while not rospy.is_shutdown():
      sock = None
      try:
        sock = socket.create_connection((self.app_host, self.app_port),
                                        timeout = SOCKET_TIMEOUT_SEC)
        sock.settimeout(SOCKET_TIMEOUT_SEC)
      except Exception as e:
        rospy.logwarn_throttle(10.0, "sim_connector_bridge_stage: connect failed: %s", str(e))
        time.sleep(RECONNECT_INTERVAL_SEC)
        continue

      with self.sock_lock:
        self.sock = sock
      rospy.loginfo("sim_connector_bridge_stage: connected to %s:%d", self.app_host, self.app_port)

      sender_stop = threading.Event()
      sender = threading.Thread(target = self.senderLoop, args = (sock, sender_stop), daemon = True)
      sender.start()

      buf = b""
      while not rospy.is_shutdown():
        try:
          data = sock.recv(4096)
        except socket.timeout:
          continue
        except Exception:
          data = b""
        if not data:
          break
        buf += data
        while b"\n" in buf:
          line, buf = buf.split(b"\n", 1)
          if line.strip():
            self.processLineFromApp(line)

      sender_stop.set()
      with self.sock_lock:
        self.sock = None
      try:
        sock.close()
      except Exception:
        pass
      rospy.logwarn("sim_connector_bridge_stage: connection lost, retrying in %.0fs",
                    RECONNECT_INTERVAL_SEC)
      time.sleep(RECONNECT_INTERVAL_SEC)

  def senderLoop(self, sock, stop_event):
    last_announce = 0.0
    while not stop_event.is_set() and not rospy.is_shutdown():
      now = time.time()
      self.sendLine(sock, self.buildTelemetryLine())

      if now - last_announce >= ANNOUNCE_INTERVAL_SEC:
        with self.scan_lock:
          scan_alive = self.latest_scan_announced
        topics = []
        if scan_alive:
          topics.append({"topic_name": SCAN_SENSOR_NAME, "msg_type": "sensor_msgs/LaserScan"})
        self.sendLine(sock, {"type": "sensor_topics", "topics": topics})
        self.sendLine(sock, {"type": "environment_options", "options": []})
        last_announce = now

      time.sleep(1.0 / TELEMETRY_RATE_HZ)

  def buildTelemetryLine(self):
    with self.pose_lock:
      x_m, y_m, yaw_rad = self.x_m, self.y_m, self.yaw_rad
      lin_mps, ang_radps = self.lin_mps, self.ang_radps
    return {
        "x_m": x_m, "y_m": y_m, "z_m": 0.0,
        "yaw_deg": math.degrees(yaw_rad),
        "x_m_per_sec": lin_mps * math.cos(yaw_rad),
        "y_m_per_sec": lin_mps * math.sin(yaw_rad),
        "yaw_deg_per_sec": math.degrees(ang_radps),
    }

  def sendLine(self, sock, line_dict):
    try:
      sock.sendall((json.dumps(line_dict) + "\n").encode())
    except Exception as e:
      rospy.logwarn_throttle(5.0, "sim_connector_bridge_stage: send failed: %s", str(e))

  #**********************
  # Commands from sim_connector_app_node.py

  def processLineFromApp(self, line):
    try:
      msg = json.loads(line)
    except Exception as e:
      rospy.logwarn_throttle(5.0, "sim_connector_bridge_stage: bad line from app: %s", str(e))
      return
    msg_type = msg.get("type")
    if msg_type == "motor_control":
      self.handleMotorControl(msg)
    elif msg_type == "goto_position":
      self.handleGotoPosition(msg)
    elif msg_type == "go_home":
      self.handleGoHome()
    elif msg_type == "go_stop":
      self.handleGoStop()
    elif msg_type == "robot_config":
      rospy.loginfo("sim_connector_bridge_stage: robot_config selected: %s", msg.get("config"))
    else:
      rospy.logwarn_throttle(5.0, "sim_connector_bridge_stage: unhandled command type: %s",
                             str(msg_type))

  def handleMotorControl(self, msg):
    ind = int(msg.get("motor_ind", -1))
    ratio = float(msg.get("speed_ratio", 0.0))
    if ind < 0 or ind >= len(self.motor_ratios):
      return
    with self.goto_lock:
      self.motor_ratios[ind] = max(0.0, min(1.0, ratio))

  def handleGotoPosition(self, msg):
    with self.pose_lock:
      cur_x, cur_y = self.x_m, self.y_m
    with self.goto_lock:
      self.goto_target = {
          "x_m": cur_x + float(msg.get("x_meters", 0.0)),
          "y_m": cur_y + float(msg.get("y_meters", 0.0)),
          "yaw_deg": msg.get("yaw_deg"),
      }
      self.motor_ratios = [0.0, 0.0]

  def handleGoHome(self):
    with self.home_lock:
      home_x, home_y = self.home_x_m, self.home_y_m
    with self.goto_lock:
      self.goto_target = {"x_m": home_x, "y_m": home_y, "yaw_deg": None}
      self.motor_ratios = [0.0, 0.0]

  def handleGoStop(self):
    with self.goto_lock:
      self.goto_target = None
      self.motor_ratios = [0.0, 0.0]


def main():
  import argparse
  parser = argparse.ArgumentParser(description = __doc__)
  parser.add_argument("--host", default = DEFAULT_APP_HOST)
  parser.add_argument("--port", type = int, default = DEFAULT_APP_PORT)
  args, _ = parser.parse_known_args()
  StageSimConnectorBridge(args.host, args.port)


if __name__ == "__main__":
  main()
