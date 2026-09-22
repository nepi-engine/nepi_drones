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

"""Builds the OBSTACLE_COURSE MJCF <geom> fragment from
sim_container/models/obstacle_course/dimensions.yaml -- the SAME file
Gazebo's buildObstacleCourseSdf and Webots' buildObstacleCourseVrml
(sim_container/bridges/webots/scripts/generate_environment_wbt.py) read.
Same geometry formulas as both of those (wall centers, baffle gap, ramp
rise/run/angle) -- just MJCF <geom> in place of SDF link/VRML Solid.

UNLIKE Webots, a compiled MjModel cannot have bodies/geoms added or
removed at runtime -- there is no Supervisor-style importMFNodeFromString
equivalent. So these geoms are embedded directly in rbx_rover.xml at
generation time (see generate_rover_xml.py, which calls
buildObstacleCourseGeomsXml and splices the result into <worldbody>),
always present in the compiled model, and toggled on/off LIVE by
mujoco_rbx_bridge.py's setObstacleCourseEnabled flipping each geom's
rgba alpha (visible/invisible) and contype/conaffinity (collide/no-
collide) -- the standard MuJoCo idiom for optional geometry, no
recompile needed. Geoms start disabled (alpha 0, contype/conaffinity 0)
so a fresh launch matches FLAT_GROUND, the same default
rbx_mujoco_node.py's own ENVIRONMENT_OPTIONS[0] already assumes.

Requested live (2026-09-21): "...environment/fov can be changed on the
spot... just like they do in gazebo," for Webots first, MuJoCo second
("once that seems verified and working well with proper tests, do the
same with mujoco").
"""

import math
import os

import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "..", "models"))

OBSTACLE_COURSE_DEFAULT_DIMENSIONS = {
    "course_start_x_m": 2.0,
    "corridor_width_m": 6.0,
    "wall_length_m": 22.0,
    "wall_thickness_m": 0.2,
    "wall_height_m": 1.0,
    "baffle_a_x_m": 8.0,
    "baffle_b_x_m": 14.0,
    "baffle_gap_m": 0.4,
    "baffle_thickness_m": 0.2,
    "ramp_start_x_m": 18.0,
    "ramp_rise_m": 0.35,
    "ramp_angle_deg": 9.97,
    "ramp_plateau_length_m": 1.0,
    "left_wall_enabled": 1,
    "right_wall_enabled": 1,
    "baffle_a_enabled": 1,
    "baffle_b_enabled": 1,
}

# Geom name prefix -- mujoco_rbx_bridge.py's setObstacleCourseEnabled looks
# these exact names up via mujoco.mj_name2id, so a rename here must be
# mirrored there. Kept as one list so the bridge doesn't need its own
# hardcoded copy of "which geoms belong to the course."
OBSTACLE_COURSE_GEOM_NAMES = [
    "oc_left_wall", "oc_right_wall", "oc_baffle_a", "oc_baffle_b",
    "oc_ramp_up", "oc_ramp_plateau", "oc_ramp_down",
]

WALL_RGB = "0.917647 0.482353 0.031373"
RAMP_RGB = "0.882353 0.803922 0.070588"


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


def _boxGeom(name, x, y, z, size_x, size_y, size_z, pitch_rad, rgb):
    # MuJoCo box "size" is HALF-extents, unlike Gazebo/Webots' full sizes --
    # halved here so callers can keep passing full extents, matching the
    # other two generators' own parameters.
    euler = f'euler="0 {math.degrees(pitch_rad):.6f} 0"' if pitch_rad else ""
    # Starts disabled (invisible, no collision) -- FLAT_GROUND is the
    # default environment; setObstacleCourseEnabled flips both live.
    return (f'    <geom name="{name}" type="box" '
            f'pos="{x:.6f} {y:.6f} {z:.6f}" {euler} '
            f'size="{size_x / 2.0:.6f} {size_y / 2.0:.6f} {size_z / 2.0:.6f}" '
            f'rgba="{rgb} 0" contype="0" conaffinity="0"/>')


def buildObstacleCourseGeomsXml(dims):
    course_start_x = dims["course_start_x_m"]
    corridor_width = dims["corridor_width_m"]
    wall_length = dims["wall_length_m"]
    wall_thickness = dims["wall_thickness_m"]
    wall_height = dims["wall_height_m"]
    baffle_a_x = dims["baffle_a_x_m"]
    baffle_b_x = dims["baffle_b_x_m"]
    baffle_gap = dims["baffle_gap_m"]
    baffle_thickness = dims["baffle_thickness_m"]
    ramp_start_x = dims["ramp_start_x_m"]
    ramp_rise = dims["ramp_rise_m"]
    ramp_angle_deg = dims["ramp_angle_deg"]
    plateau_length = dims["ramp_plateau_length_m"]

    half_corridor = corridor_width / 2.0
    wall_center_x = course_start_x + wall_length / 2.0

    baffle_len = half_corridor - baffle_gap
    baffle_a_y = half_corridor - baffle_len / 2.0
    baffle_b_y = -(half_corridor - baffle_len / 2.0)

    angle_rad = math.radians(ramp_angle_deg)
    run = ramp_rise / math.tan(angle_rad)
    box_len = ramp_rise / math.sin(angle_rad)
    ramp_z = ramp_rise / 2.0
    plateau_z = ramp_rise

    ramp_up_x = ramp_start_x + run / 2.0
    plateau_x = ramp_up_x + run / 2.0 + plateau_length / 2.0
    ramp_down_x = plateau_x + plateau_length / 2.0 + run / 2.0

    geoms = []
    if dims.get("left_wall_enabled", 1):
        geoms.append(_boxGeom("oc_left_wall", wall_center_x, half_corridor, wall_height / 2.0,
                               wall_length, wall_thickness, wall_height, 0.0, WALL_RGB))
    if dims.get("right_wall_enabled", 1):
        geoms.append(_boxGeom("oc_right_wall", wall_center_x, -half_corridor, wall_height / 2.0,
                               wall_length, wall_thickness, wall_height, 0.0, WALL_RGB))
    if dims.get("baffle_a_enabled", 1):
        geoms.append(_boxGeom("oc_baffle_a", baffle_a_x, baffle_a_y, wall_height / 2.0,
                               baffle_thickness, baffle_len, wall_height, 0.0, WALL_RGB))
    if dims.get("baffle_b_enabled", 1):
        geoms.append(_boxGeom("oc_baffle_b", baffle_b_x, baffle_b_y, wall_height / 2.0,
                               baffle_thickness, baffle_len, wall_height, 0.0, WALL_RGB))

    geoms.append(_boxGeom("oc_ramp_up", ramp_up_x, 0.0, ramp_z,
                           box_len, corridor_width, 0.12, -angle_rad, RAMP_RGB))
    geoms.append(_boxGeom("oc_ramp_plateau", plateau_x, 0.0, plateau_z,
                           plateau_length, corridor_width, 0.12, 0.0, RAMP_RGB))
    geoms.append(_boxGeom("oc_ramp_down", ramp_down_x, 0.0, ramp_z,
                           box_len, corridor_width, 0.12, angle_rad, RAMP_RGB))

    return "\n".join(geoms)


def main():
    dims = loadDimensions("obstacle_course", OBSTACLE_COURSE_DEFAULT_DIMENSIONS)
    print(buildObstacleCourseGeomsXml(dims))


if __name__ == "__main__":
    main()
