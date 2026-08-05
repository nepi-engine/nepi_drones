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

# Phase 1 test harness for device_if_sim.py -- docs/SIMULATION_INTERFACE_SPEC.md's
# Implementation Plan, Test Cases 1.1-1.4. NOT production code: a throwaway node
# that instantiates SimDeviceIF with one of three callback profiles (matching the
# spec's "Two worked example systems" table, plus an everything-empty case) so
# `rosservice call .../capabilities_query` and `rostopic echo .../status` can be
# checked against real, running ROS state -- run only inside the disposable
# scratch catkin workspace (~/sim_connector_test_ws), never deployed anywhere.
#
# Usage: ROS_NAMESPACE=/nepi/device1 python3 test_device_if_sim_harness.py [rover|drone|empty]
#
# ROS_NAMESPACE is required, not optional: nepi_sdk.get_base_namespace() (called
# from MsgIF.__init__, i.e. before anything else in SimDeviceIF construction)
# busy-waits with NO timeout for a currently-registered node whose full name
# contains "nepi" AND has at least 3 '/'-separated segments (e.g.
# /nepi/device1/<node>) -- confirmed by direct bisection this session (see the
# spec's Phase 1 section): without this, the harness hangs forever with no
# error, not a quick failure. This is a real nepi_sdk prerequisite, not a bug
# in this file or in device_if_sim.py.

import os
import sys
import time

import rospy

from nepi_sdk import nepi_sdk

# Real deployment convention (see nepi_app_pan_tilt_auto/CMakeLists.txt's
# install(DIRECTORY api/ DESTINATION .../nepi_api)): an app's api/*.py files
# install directly into the shared nepi_api namespace alongside
# device_if_rbx.py, so device_if_sim.py is imported as nepi_api.device_if_sim
# in production, NOT app_sim_connector.api.device_if_sim. For this scratch
# test (devel space only, no catkin_make install run), add the source api/
# directory straight to sys.path and import the bare module -- same module,
# same eventual import name, just skipping the install step this throwaway
# test doesn't need.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'api'))
from device_if_sim import SimDeviceIF

PROFILE = sys.argv[1] if len(sys.argv) > 1 else 'rover'

DEVICE_INFO = dict(device_name = "test_sim_" + PROFILE, path = "",
                    serial_number = "", hw_version = "", sw_version = "")


def getAvailableSensorTopicsReal():
  # Real scan, not a canned stub -- Test Case 1.3 proves liveness by publishing
  # a real throwaway sensor_msgs/Image topic externally and watching this
  # reflect it without restarting the node.
  topics, msg_types = nepi_sdk.find_topics_by_msgs(['Image', 'LaserScan', 'Imu'])
  return list(zip(topics, msg_types))


class DummyState:
  ind = 0


def buildRoverKwargs():
  motor_ratios = [0.0, 0.0]

  def setMotorControlRatio(ind, ratio):
    motor_ratios[ind] = ratio

  return dict(
    wheel_count = 2,
    motor_count = 2,
    setMotorControlRatio = setMotorControlRatio,
    getMotorControlRatios = lambda: motor_ratios,
    manualControlsReadyFunction = lambda: True,
    gotoPositionFunction = lambda msg: True,
    getAvailableSensorTopicsFunction = getAvailableSensorTopicsReal,
    setCameraViewModeFunction = lambda mode: None,
    available_camera_view_modes = ["FIRST_PERSON", "THIRD_PERSON"],
    setEnvironmentOptionFunction = lambda opt: None,
    available_environment_options = ["obstacle_course"],
    setHomeFunction = lambda geo: None,
    goHomeFunction = lambda: True,
    goStopFunction = lambda: True,
    getBridgeConnectedFunction = lambda: True,
    getTelemetryAgeFunction = lambda: 0.1,
  )


def buildDroneKwargs():
  motor_ratios = [0.0, 0.0, 0.0, 0.0]

  def setMotorControlRatio(ind, ratio):
    motor_ratios[ind] = ratio

  return dict(
    wheel_count = 0,
    motor_count = 4,
    setMotorControlRatio = setMotorControlRatio,
    getMotorControlRatios = lambda: motor_ratios,
    manualControlsReadyFunction = lambda: True,
    gotoPositionFunction = lambda msg: True,
    gotoPoseFunction = lambda attitude: True,
    gotoLocationFunction = lambda msg: True,
    getAvailableSensorTopicsFunction = getAvailableSensorTopicsReal,
    setHomeFunction = lambda geo: None,
    goHomeFunction = lambda: True,
    goStopFunction = lambda: True,
    getBridgeConnectedFunction = lambda: True,
    getTelemetryAgeFunction = lambda: 0.1,
  )


def buildEmptyKwargs():
  # Test Case 1.4: every optional callback left None/every list empty
  return dict()


def main():
  rospy.init_node('test_device_if_sim_harness_' + PROFILE)

  if PROFILE == 'rover':
    kwargs = buildRoverKwargs()
  elif PROFILE == 'drone':
    kwargs = buildDroneKwargs()
  elif PROFILE == 'empty':
    kwargs = buildEmptyKwargs()
  else:
    sys.exit("usage: test_device_if_sim_harness.py [rover|drone|empty]")

  sim_if = SimDeviceIF(device_info = DEVICE_INFO, **kwargs)
  print("Harness running, profile=" + PROFILE + ", namespace=" + sim_if.namespace, flush = True)
  rospy.spin()


if __name__ == '__main__':
  main()
