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

"""Generates rbx_rover.xml from sim_container/models/generic_rover/
dimensions.yaml -- the SAME file generate_rover_wbt.py (Webots) and
generate_model_sdf.py's buildRoverSdf (Gazebo) read. Run at mujoco_rbx_
bridge.py's own startup, before mujoco.MjModel.from_xml_path -- MuJoCo has
no live-resize mechanism for a body's own kinematic offset/joint anchor
the way Webots' Supervisor field-write does, so this is a "next launch,
not live" contract, identical to generate_rover_wbt.py's own (see that
file's module docstring).

Requested live (2026-09-21): "get mujoco to that same point too, with all
the same features i asked for with webots" -- this generator covers robot
(chassis/wheel) dimensions, matching generate_rover_wbt.py's own scope
note almost verbatim. Environment/obstacle-course geoms are a separate
generator (generate_environment_xml.py) whose output this file embeds
directly into <worldbody> -- unlike Webots (import/remove at runtime),
MuJoCo needs them compiled into the model from the start (see that file's
own docstring for why), toggled live by mujoco_rbx_bridge.py flipping
each geom's rgba alpha + contype/conaffinity instead of spawning/removing
nodes.

NOT ported: wheel_independence_enabled (Gazebo's crab-steer plugin) --
same scope decision generate_rover_wbt.py's own docstring documents;
rbx_rover.xml already exposes 4 independently-actuated wheel motors
regardless (a MuJoCo capability with no Gazebo/Webots equivalent, per
this file's own pre-existing module comment), so this field has no
effect on the body this generator produces either way.

Physical relationships mirror generate_rover_wbt.py's own as closely as
MuJoCo's box-half-extent/local-child-body convention allows:
  - Chassis body origin height (wheel axle height) = wheel_radius_m,
    matching the Robot node in both other generators.
  - Chassis box HALF-extents = chassis_length_m/2, chassis_width_m/2,
    chassis_height_m/2 (MuJoCo box "size" is half-extents, unlike Gazebo/
    Webots' full sizes).
  - Chassis mass = weight_kg (the whole rover's specified weight
    concentrated on the chassis body, matching Gazebo/Webots' own
    single-mass convention); each wheel keeps a fixed nominal 0.5 kg,
    unrelated to weight_kg, same as before this generator existed.
  - Wheel bodies (children of chassis, so LOCAL coordinates) at
    (+-wheelbase_m/2, +-track_width_m/2, 0) -- wheel axle height IS the
    chassis body's own local z=0, same convention as the other two
    generators.
  - robot_camera mounted at the chassis' own front-top edge (chassis_
    length_m/2, 0, chassis_height_m/2, LOCAL to the chassis body) --
    matches the PRE-EXISTING rbx_rover.xml's own "0.2 0 0.05" placement
    at factory defaults (0.4/2, 0.1/2) exactly, and the same front-top-
    edge convention generate_rover_wbt.py chose for Webots.
  - scene_camera/scene_camera_depth stay at the SAME fixed LOCAL
    (-2.5, 0, 1.65) chase-cam offset regardless of robot size -- LOCAL to
    the chassis body, whose own WORLD height already varies with
    wheel_radius_m, so this reproduces the same chassis-size-independent
    world-frame mount generate_rover_wbt.py's own SCENE_CAM_POS documents.
  - camera_horizontal_fov_deg maps directly to fovy (both cameras share
    one FOV), matching the pre-existing file's fovy="60" == rbx_mujoco_
    node.py's own camera_fov_deg factory value of 60.0.

Usage: generate_rover_xml.py [model_name]
  model_name defaults to "generic_rover" -- accepted as an argument anyway,
  matching the other two generators' own CLI shape.
"""

import math
import os
import sys

import yaml

from generate_environment_xml import buildObstacleCourseGeomsXml, \
    loadDimensions as loadEnvironmentDimensions, OBSTACLE_COURSE_DEFAULT_DIMENSIONS

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "..", "models"))
OUTPUT_PATH = os.path.join(SCRIPT_DIR, "models", "rbx_rover.xml")

DEFAULT_DIMENSIONS = {
    "wheel_radius_m": 0.1,
    "wheel_width_m": 0.05,
    "track_width_m": 0.34,
    "wheelbase_m": 0.3,
    "chassis_length_m": 0.4,
    "chassis_width_m": 0.3,
    "chassis_height_m": 0.1,
    "weight_kg": 5.0,
    "camera_horizontal_fov_deg": 100.0,
    "wheel_independence_enabled": 0,
}

# (name, x_sign, y_sign) -- same wheel1-4/front_left.../rear_right
# convention every other rover generator/model in this project uses.
ROVER_WHEELS = [
    ("wheel1", 1, 1),
    ("wheel2", 1, -1),
    ("wheel3", -1, 1),
    ("wheel4", -1, -1),
]

WHEEL_MASS_KG = 0.5
WHEEL_DAMPING = 0.05
SCENE_CAM_POS = (-2.5, 0.0, 1.65)


def loadDimensions(model_name, defaults):
    path = os.path.join(MODELS_DIR, model_name, "dimensions.yaml")
    dims = dict(defaults)
    if os.path.exists(path):
        with open(path, "r") as f:
            loaded = yaml.safe_load(f) or {}
        dims.update(loaded)
    for key, default_value in defaults.items():
        if isinstance(default_value, (int, float)) and key in dims:
            try:
                dims[key] = float(dims[key])
            except (TypeError, ValueError):
                pass
    return dims


def buildRoverXml(dims, obstacle_course_geoms_xml):
    wheel_radius = dims["wheel_radius_m"]
    wheel_width = dims["wheel_width_m"]
    track_width = dims["track_width_m"]
    wheelbase = dims["wheelbase_m"]
    chassis_l = dims["chassis_length_m"]
    chassis_w = dims["chassis_width_m"]
    chassis_h = dims["chassis_height_m"]
    weight_kg = dims["weight_kg"]
    fovy = dims["camera_horizontal_fov_deg"]

    x_off = wheelbase / 2.0
    y_off = track_width / 2.0

    wheel_blocks = []
    for name, x_sign, y_sign in ROVER_WHEELS:
        x = x_sign * x_off
        y = y_sign * y_off
        wheel_blocks.append(f"""      <body name="{name}" pos="{x:.6f} {y:.6f} 0">
        <joint name="{name}" type="hinge" axis="0 1 0" damping="{WHEEL_DAMPING}"/>
        <geom name="{name}_geom" type="cylinder" size="{wheel_radius:.6f} {wheel_width / 2.0:.6f}" euler="90 0 0"
              mass="{WHEEL_MASS_KG}" rgba="0.1 0.1 0.1 1" friction="1.2 0.005 0.0001"/>
      </body>""")
    wheels_xml = "\n".join(wheel_blocks)

    actuator_blocks = "\n".join(
        f'    <velocity name="{name}_vel" joint="{name}" kv="8" ctrlrange="-15 15"/>'
        for name, _, _ in ROVER_WHEELS)

    return f"""<!--
  GENERATED FILE -- do not hand-edit. Produced by
  sim_container/bridges/mujoco/generate_rover_xml.py from
  sim_container/models/generic_rover/dimensions.yaml (robot) and
  sim_container/models/obstacle_course/dimensions.yaml (environment,
  compiled in but disabled by default) at mujoco_rbx_bridge.py's own
  startup. Hand edits are overwritten on the next launch -- edit
  dimensions.yaml (or the RUI's dimensions editor) instead, or
  generate_rover_xml.py/generate_environment_xml.py's own physical-
  relationship comments if the RELATIONSHIP itself needs to change.

  4 independently-actuated wheels (wheel1=front_left, wheel2=front_right,
  wheel3=rear_left, wheel4=rear_right), unlike Webots' rbx_rover.wbt (only
  2 independently-drivable sides) -- MuJoCo has no such constraint, so
  rbx_mujoco_node.py exposes 4 motor sliders, reduced to a left/right
  average by motorControlToVelocity, same formula as the Gazebo driver.
-->
<mujoco model="rbx_rover">
  <compiler angle="degree" coordinate="local" inertiafromgeom="true"/>
  <!-- implicitfast: explicit Euler is unstable for velocity actuators at this
       kv/timestep/inertia combination (confirmed -- diverges within a few
       steps under the default integrator); implicitfast integrates actuator
       damping semi-implicitly, MuJoCo's documented fix for exactly this. -->
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast"/>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.6 0.7 0.9" rgb2="0.1 0.1 0.2" width="128" height="128"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.2 0.3 0.2" rgb2="0.25 0.35 0.25" width="256" height="256"/>
    <material name="grid" texture="grid" texrepeat="20 20" reflectance="0.1"/>
  </asset>

  <worldbody>
    <light directional="true" diffuse="0.8 0.8 0.8" pos="0 0 5" dir="0 0 -1"/>
    <geom name="ground" type="plane" size="20 20 0.1" material="grid" friction="1.0 0.005 0.0001"/>

{obstacle_course_geoms_xml}

    <!-- Chassis origin height = wheel_radius_m so wheel bottoms touch z=0. -->
    <body name="chassis" pos="0 0 {wheel_radius:.6f}">
      <freejoint name="chassis_free"/>
      <geom name="chassis_geom" type="box" size="{chassis_l / 2.0:.6f} {chassis_w / 2.0:.6f} {chassis_h / 2.0:.6f}" mass="{weight_kg:.6f}" rgba="0.2 0.5 0.8 1"/>
      <!-- Forward-facing camera at the chassis' own front-top edge. Local -Z
           (MuJoCo's camera forward convention) aligned to world +X via
           xyaxes: local X=(0,-1,0), local Y=(0,0,1) -> local Z = X x Y = (-1,0,0). -->
      <camera name="robot_camera" pos="{chassis_l / 2.0:.6f} 0 {chassis_h / 2.0:.6f}" xyaxes="0 -1 0 0 0 1" fovy="{fovy:.6f}"/>
      <!-- Scene/chase view -- fixed offset regardless of chassis size (same
           reasoning generate_rover_wbt.py's own SCENE_CAM_POS documents).
           yaxis = (sin(0.5834), 0, cos(0.5834)) tilts local Z (viewing
           direction) down so the rover stays framed from behind/above. -->
      <camera name="scene_camera" pos="{SCENE_CAM_POS[0]:.6f} {SCENE_CAM_POS[1]:.6f} {SCENE_CAM_POS[2]:.6f}" xyaxes="0 -1 0 0.5514 0 0.8342" fovy="{fovy:.6f}"/>

{wheels_xml}
    </body>
  </worldbody>

  <actuator>
{actuator_blocks}
  </actuator>
</mujoco>
"""


def regenerate(model_name):
    # Split out from main() so a caller that already has its own sys.argv
    # (mujoco_rbx_bridge.py's own heartbeat/bridge port args) can invoke
    # this directly instead of going through argv parsing meant for this
    # script's own standalone CLI use.
    dims = loadDimensions(model_name, DEFAULT_DIMENSIONS)
    env_dims = loadEnvironmentDimensions("obstacle_course", OBSTACLE_COURSE_DEFAULT_DIMENSIONS)
    obstacle_course_geoms_xml = buildObstacleCourseGeomsXml(env_dims)
    xml_text = buildRoverXml(dims, obstacle_course_geoms_xml)
    with open(OUTPUT_PATH, "w") as f:
        f.write(xml_text)
    print("generate_rover_xml: wrote %s from %s dimensions" % (OUTPUT_PATH, model_name), flush=True)


def main():
    model_name = sys.argv[1] if len(sys.argv) > 1 else "generic_rover"
    regenerate(model_name)


if __name__ == "__main__":
    main()
