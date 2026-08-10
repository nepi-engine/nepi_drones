#!/usr/bin/env python3
#
# THROWAWAY TEST HARNESS -- not part of the team's deliverable, added purely to
# smoke-test the newly-pulled device_if_sim.py (nepi_app_sim_connector, pulled
# from nepi_engine_ws's nepi_apps/nepi_drivers upstream on 2026-08-06) in the
# disposable scratch catkin workspace (~/sim_connector_test_ws), mirroring the
# harness used for the 2026-08-04 device_if_sim.py sandbox pass.
#
# Usage: ROS_NAMESPACE=/nepi/device1 python3 test_device_if_sim_harness.py [empty|rover|drone]
#
# ROS_NAMESPACE is required -- nepi_sdk.get_base_namespace() busy-waits with no
# timeout for a node whose name contains "nepi" and has >=3 "/"-segments (see
# docs/SIMULATION_INTERFACE_SPEC.md's Phase 1 section for the full writeup).

import os
import sys
import time

import rospy

from nepi_sdk import nepi_sdk

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'api'))
from device_if_sim import SimDeviceIF

PROFILE = sys.argv[1] if len(sys.argv) > 1 else 'empty'

DEVICE_INFO = dict(device_name="test_sim_" + PROFILE, path="",
                    serial_number="", hw_version="", sw_version="")


def buildEmptyKwargs():
  return dict()


def buildRoverKwargs():
  motor_ratios = [0.0, 0.0]

  def setMotorControlRatio(ind, ratio):
    motor_ratios[ind] = ratio

  def getAvailableSensorTopics():
    topics, msg_types = nepi_sdk.find_topics_by_msgs(['Image', 'LaserScan', 'Imu'])
    return list(zip(topics, msg_types))

  return dict(
    wheel_count=2,
    motor_count=2,
    setMotorControlRatio=setMotorControlRatio,
    getMotorControlRatios=lambda: motor_ratios,
    manualControlsReadyFunction=lambda: True,
    gotoPositionFunction=lambda msg: True,
    getAvailableSensorTopicsFunction=getAvailableSensorTopics,
    setCameraViewModeFunction=lambda mode: None,
    available_camera_view_modes=["FIRST_PERSON", "THIRD_PERSON"],
    setEnvironmentOptionFunction=lambda opt: None,
    getAvailableEnvironmentOptionsFunction=lambda: ["obstacle_course"],
    setHomeFunction=lambda geo: None,
    goHomeFunction=lambda: True,
    goStopFunction=lambda: True,
    getBridgeConnectedFunction=lambda: True,
    getTelemetryAgeFunction=lambda: 0.1,
    getNavPoseCb=lambda: dict(has_location=True, has_heading=True, has_orientation=True,
      has_position=True, has_altitude=False, has_depth=False, has_pan_tilt=False),
  )


def buildDroneKwargs():
  motor_ratios = [0.0] * 4

  def setMotorControlRatio(ind, ratio):
    motor_ratios[ind] = ratio

  return dict(
    wheel_count=0,
    motor_count=4,
    setMotorControlRatio=setMotorControlRatio,
    getMotorControlRatios=lambda: motor_ratios,
    manualControlsReadyFunction=lambda: True,
    gotoPositionFunction=lambda msg: True,
    gotoPoseFunction=lambda attitude: True,
    gotoLocationFunction=lambda msg: True,
    getAvailableSensorTopicsFunction=lambda: [],
    setHomeFunction=lambda geo: None,
    goHomeFunction=lambda: True,
    goStopFunction=lambda: True,
    getBridgeConnectedFunction=lambda: True,
    getTelemetryAgeFunction=lambda: 0.1,
  )


def main():
  rospy.init_node('test_device_if_sim_harness_' + PROFILE)

  if PROFILE == 'empty':
    kwargs = buildEmptyKwargs()
  elif PROFILE == 'rover':
    kwargs = buildRoverKwargs()
  elif PROFILE == 'drone':
    kwargs = buildDroneKwargs()
  else:
    sys.exit("usage: test_device_if_sim_harness.py [empty|rover|drone]")

  sim_if = SimDeviceIF(device_info=DEVICE_INFO, **kwargs)
  print("Harness running, profile=" + PROFILE + ", namespace=" + sim_if.namespace, flush=True)
  rospy.spin()


if __name__ == '__main__':
  main()
