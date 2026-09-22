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

"""Builds the OBSTACLE_COURSE VRML fragment from
sim_container/models/obstacle_course/dimensions.yaml -- the SAME file
buildObstacleCourseSdf (sim_container/scripts/generate_model_sdf.py) reads
for Gazebo. Unlike generate_rover_wbt.py (a fresh-launch-only, "next
launch, not live" contract), this one is imported directly by
webots_rbx_bridge.py and spawned/removed LIVE via Supervisor
importMFNodeFromString/node.remove() in response to the existing
{'type':'environment_option','option':'OBSTACLE_COURSE','enabled':bool}
wire message rbx_webots_node.py's own setEnvironmentAction already sends
-- see that method's own (now-stale) "this world has no obstacle-course
model" comment, corrected alongside this file.

Requested live (2026-09-21): "...environment/fov can be changed on the
spot." Only OBSTACLE_COURSE is built here, matching
rbx_webots_node.py's own ENVIRONMENT_OPTIONS = ["FLAT_GROUND",
"OBSTACLE_COURSE"] -- Gazebo's separate aerial_obstacle_course/
custom_obstacles models have no Webots equivalent yet (no ENVIRONMENT_
OPTIONS entry exists to trigger them either).

Geometry mirrors buildObstacleCourseSdf as closely as Webots' node model
allows -- same field names, same derived-value formulas (wall centers,
baffle gap, ramp rise/run/angle), just VRML Solid/Box in place of SDF
link/collision/visual. Extra custom obstacles (_renderExtraObstacles'
own 'obstacles' field) are NOT ported -- that's the custom_obstacles
model's own concern, not exposed as a Webots environment option either.

Everything is wrapped as children of one DEF OBSTACLE_COURSE Solid node
so the whole course spawns/despawns as a single Supervisor operation
(one importMFNodeFromString call to add, one getFromDef+remove() to take
it back out) -- see webots_rbx_bridge.py's setObstacleCourseEnabled.

Usage (standalone, for review -- prints VRML to stdout):
  generate_environment_wbt.py [model_name]
  model_name defaults to "obstacle_course".
"""

import math
import os
import sys

import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# SCRIPT_DIR is sim_container/bridges/webots/scripts, so three levels up
# (scripts -> webots -> bridges -> sim_container) then into "models" --
# same layout generate_rover_wbt.py uses (and the same mistake that file
# made once: verify with os.path.exists before trusting this, if it is
# ever restructured).
MODELS_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "..", "..", "models"))

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


def loadDimensions(model_name, defaults):
    # Mirrors generate_model_sdf.py's own loadDimensions (same defaults-
    # merge + numeric-coercion behavior) and generate_rover_wbt.py's own
    # copy of that same logic, so a dimensions.yaml written by the RUI
    # (plain YAML text, not type-preserving) behaves identically here.
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


def _boxSolid(name, x, y, z, size_x, size_y, size_z, pitch_rad, color):
    # A single static Solid/Box pair -- no physics field, matching Gazebo's
    # <static>true</static> obstacle_course model (these never move).
    rotation = f"0 1 0 {pitch_rad:.6f}" if pitch_rad else "0 0 1 0"
    return f"""      DEF {name.upper()} Solid {{
        translation {x:.6f} {y:.6f} {z:.6f}
        rotation {rotation}
        children [
          Shape {{
            appearance PBRAppearance {{
              baseColor {color}
              roughness 1
              metalness 0
            }}
            geometry Box {{
              size {size_x:.6f} {size_y:.6f} {size_z:.6f}
            }}
          }}
        ]
        name "{name}"
        boundingObject Box {{
          size {size_x:.6f} {size_y:.6f} {size_z:.6f}
        }}
      }}"""


# Webots baseColor is RGB 0-1, no named-material equivalent to Gazebo's
# "Gazebo/Orange"/"Gazebo/Yellow" -- closest visual match, same convention
# rbx_rover.wbt's own wheel/chassis colors already use (a plain RGB triple).
WALL_COLOR = "0.917647 0.482353 0.031373"
RAMP_COLOR = "0.882353 0.803922 0.070588"


def buildObstacleCourseVrml(dims):
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

    children = []
    if dims.get("left_wall_enabled", 1):
        children.append(_boxSolid("left_wall", wall_center_x, half_corridor, wall_height / 2.0,
                                   wall_length, wall_thickness, wall_height, 0.0, WALL_COLOR))
    if dims.get("right_wall_enabled", 1):
        children.append(_boxSolid("right_wall", wall_center_x, -half_corridor, wall_height / 2.0,
                                   wall_length, wall_thickness, wall_height, 0.0, WALL_COLOR))
    if dims.get("baffle_a_enabled", 1):
        children.append(_boxSolid("baffle_a", baffle_a_x, baffle_a_y, wall_height / 2.0,
                                   baffle_thickness, baffle_len, wall_height, 0.0, WALL_COLOR))
    if dims.get("baffle_b_enabled", 1):
        children.append(_boxSolid("baffle_b", baffle_b_x, baffle_b_y, wall_height / 2.0,
                                   baffle_thickness, baffle_len, wall_height, 0.0, WALL_COLOR))

    children.append(_boxSolid("ramp_up", ramp_up_x, 0.0, ramp_z,
                               box_len, corridor_width, 0.12, -angle_rad, RAMP_COLOR))
    children.append(_boxSolid("ramp_plateau", plateau_x, 0.0, plateau_z,
                               plateau_length, corridor_width, 0.12, 0.0, RAMP_COLOR))
    children.append(_boxSolid("ramp_down", ramp_down_x, 0.0, ramp_z,
                               box_len, corridor_width, 0.12, angle_rad, RAMP_COLOR))

    children_vrml = "\n".join(children)

    return f"""DEF OBSTACLE_COURSE Solid {{
  translation 0 0 0
  children [
{children_vrml}
  ]
  name "obstacle_course"
}}
"""


def main():
    model_name = sys.argv[1] if len(sys.argv) > 1 else "obstacle_course"
    dims = loadDimensions(model_name, OBSTACLE_COURSE_DEFAULT_DIMENSIONS)
    print(buildObstacleCourseVrml(dims))


if __name__ == "__main__":
    main()
