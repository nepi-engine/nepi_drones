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

"""Generates rbx_rover.wbt from sim_container/models/generic_rover/dimensions.yaml
-- the SAME file generate_model_sdf.py's buildRoverSdf() reads for Gazebo
(sim_connector_app_node.py's pushDirtyDimensions pushes it there before every
launch, regardless of which simulator target is active). Run at launch time,
same "next fresh launch, not live" contract Gazebo's own dimensions editing
already has (see docs/SIM_CONNECTOR_DEPLOY_AND_LIVE_UPDATE_REFERENCE.md's own
"dimensions/geometry pushed via pushDirtyDimensions only take effect on the
next FRESH gzserver launch" note -- identical here, just a different renderer).

Requested live (2026-09-21): "go ahead and start devving the next big part of
it for webots... robot configs work, obstacle and robot changing dimensions
work". This generator covers ONE of those three: robot (chassis/wheel)
dimensions. Environment/obstacle-course dimensions are a separate generator
(generate_environment_wbt.py) -- a different model, a different consumer
(webots_rbx_bridge.py's Supervisor-based live spawn/remove, not a fresh
launch), see that file's own docstring.

NOT ported: wheel_independence_enabled (Gazebo's optional 4-independently-
steerable-wheel crab-steer mode, via a custom Gazebo plugin). Webots'
HingeJoint motor model has no equivalent mechanism, and rbx_rover.wbt has
always used the simpler left/right-grouped 2-motor body (matching Gazebo's
DEFAULT diff_drive mode) -- this generator always produces that same
left/right-grouped body regardless of the dimensions.yaml field's value.

Physical relationships mirror buildRoverSdf's own (sim_container/scripts/
generate_model_sdf.py) as closely as Webots' different node model allows:
  - Robot origin height (wheel axle height) = wheel_radius_m
  - Chassis (BODY box) is centered wheel_radius_m + chassis_height_m/2 above
    ground, i.e. at LOCAL z = chassis_height_m/2 relative to the Robot's own
    z=wheel_radius_m origin (so its bottom face sits exactly at the wheel
    axle height, its top face chassis_height_m above that) -- NOT
    reproducing buildRoverSdf's own base_z = wheel_radius + chassis_h/2
    world-frame formula verbatim, since Webots child positions here are
    LOCAL to the Robot's own translation, not world-frame like Gazebo's
    base_link pose.
  - Wheel anchors at local (+-wheelbase_m/2, +-track_width_m/2, -chassis_height_m/2 ...
    actually 0, matching the existing file's convention of anchoring wheels
    at the Robot's own origin height (the wheel axle), not the chassis
    center) -- see WHEEL_ANCHOR_Z below.
  - robot_camera mounted at the chassis' own front-top edge (scales with
    chassis_length_m/chassis_height_m) rather than reproducing
    buildRoverSdf's own fixed (0.2, base_z+0.5) offset, which is unrelated
    to chassis size by that file's own admission ("a fixed spec value, NOT
    derived from wheelbase_m/track_width_m... just happens to be close to
    wheelbase/2 at the factory defaults, which is a coincidence").
  - scene_camera/scene_camera_depth stay at the SAME fixed (-2.5, 0, 1.65)
    chase-cam offset regardless of robot size, matching
    generic_rover/model.sdf's own fixed camera_link_chase mount (that offset
    is chosen for a good chase-cam framing distance, not derived from
    chassis size either).
  - camera_horizontal_fov_deg maps directly to fieldOfView (Webots'
    horizontal FOV, radians) for both cameras -- same "both cameras share
    one FOV" convention rbx_sim_node.py's own camera_fov_deg setting uses.
  - weight_kg -> the Robot node's own physics Physics.mass (previously
    hardcoded to 1, unrelated to the dimensions store).

Usage: generate_rover_wbt.py [model_name]
  model_name defaults to "generic_rover" (the only model this generator
  supports today) -- accepted as an argument anyway, matching
  generate_model_sdf.py's own CLI shape, so a future second wheeled model
  could reuse this file's own loadDimensions/main plumbing.
"""

import math
import os
import sys

import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# Absolute, not relative to a caller's cwd -- this script is invoked by a
# remote launch_command with an arbitrary working directory (see
# simulator_launch_targets.yaml's own webots_rover launch_command).
# SCRIPT_DIR is sim_container/bridges/webots/scripts, so three levels up
# (scripts -> webots -> bridges -> sim_container) then into "models".
MODELS_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "..", "..", "models"))
OUTPUT_PATH = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "worlds", "rbx_rover.wbt"))

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
    # Not supported here -- see module docstring. Declared so loadDimensions'
    # own numeric-coercion loop (matching generate_model_sdf.py's) doesn't
    # choke on it if a shared dimensions.yaml carries it for Gazebo's sake.
    "wheel_independence_enabled": 0,
}

# (name, x_sign, y_sign) -- matches generate_model_sdf.py's own ROVER_WHEELS
# ordering (front_left/front_right/rear_left/rear_right) and this file's own
# pre-existing wheel1-4 naming (wheel1=front_left, wheel2=front_right,
# wheel3=rear_left, wheel4=rear_right -- see rbx_mujoco_node.py's identical
# convention comment).
ROVER_WHEELS = [
    ("wheel1", 1, 1),
    ("wheel2", 1, -1),
    ("wheel3", -1, 1),
    ("wheel4", -1, -1),
]

# Fixed, chassis-size-independent scene/chase mount -- see module docstring.
SCENE_CAM_POS = (-2.5, 0.0, 1.65)
SCENE_CAM_PITCH_RAD = 0.5834
SCENE_CAM_YAXIS = (math.sin(SCENE_CAM_PITCH_RAD), 0.0, math.cos(SCENE_CAM_PITCH_RAD))

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
DEPTH_MIN_RANGE = 0.05
DEPTH_MAX_RANGE = 100.0


def loadDimensions(model_name, defaults):
    # Mirrors generate_model_sdf.py's own loadDimensions exactly (same
    # defaults-merge + numeric-coercion behavior), so a dimensions.yaml
    # written by the RUI (plain YAML text, not type-preserving) behaves
    # identically here as it does for the Gazebo generator.
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


def buildRoverWbt(dims):
    wheel_radius = dims["wheel_radius_m"]
    wheel_width = dims["wheel_width_m"]
    track_width = dims["track_width_m"]
    wheelbase = dims["wheelbase_m"]
    chassis_l = dims["chassis_length_m"]
    chassis_w = dims["chassis_width_m"]
    chassis_h = dims["chassis_height_m"]
    weight_kg = dims["weight_kg"]
    fov_rad = math.radians(dims["camera_horizontal_fov_deg"])

    x_off = wheelbase / 2.0
    y_off = track_width / 2.0
    # Robot origin sits at the wheel axle height (see module docstring) --
    # wheel anchors are therefore at LOCAL z=0, and the chassis box's own
    # center sits chassis_h/2 above that.
    chassis_center_z = chassis_h / 2.0

    # Front-top edge of the chassis -- see module docstring for why this
    # (not buildRoverSdf's own fixed, chassis-size-independent offset) is
    # used here.
    robot_cam_pos = (chassis_l / 2.0, 0.0, chassis_center_z + chassis_h / 2.0)

    wheel_blocks = []
    for name, x_sign, y_sign in ROVER_WHEELS:
        x = x_sign * x_off
        y = y_sign * y_off
        wheel_blocks.append(f"""    DEF {name.upper()} HingeJoint {{
      jointParameters HingeJointParameters {{
        axis 0 1 0
        anchor {x:.6f} {y:.6f} 0
      }}
      device [
        RotationalMotor {{
          name "{name}"
        }}
      ]
      endPoint Solid {{
        translation {x:.6f} {y:.6f} 0
        rotation 1 0 0 1.5708
        children [
          DEF {name.upper()}_SHAPE Shape {{
            appearance PBRAppearance {{
              baseColor 0.305882 0.898039 0.25098
              roughness 1
              metalness 0
            }}
            geometry Cylinder {{
              height {wheel_width:.6f}
              radius {wheel_radius:.6f}
              subdivision 24
            }}
          }}
        ]
        name "{name}_solid"
        boundingObject USE {name.upper()}_SHAPE
        physics Physics {{
        }}
      }}
    }}""")
    wheels_vrml = "\n".join(wheel_blocks)

    return f"""#VRML_SIM R2023a utf8
# GENERATED FILE -- do not hand-edit. Produced by
# sim_container/bridges/webots/scripts/generate_rover_wbt.py from
# sim_container/models/generic_rover/dimensions.yaml at launch time (see
# simulator_launch_targets.yaml's own webots_rover launch_command). Hand
# edits are overwritten on the next deploy/dimensions push -- edit
# dimensions.yaml (or the RUI's robot dimensions editor) instead, or this
# generator's own physical-relationship comments if the RELATIONSHIP itself
# (not a specific robot's numbers) needs to change.
#
# Same controller/wire-protocol pair as every hand-authored version of this
# file before it (webots_rbx_bridge.py) -- see that file's own module
# docstring for the full deploy/protocol design this doesn't change at all.

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023a/projects/objects/floors/protos/Floor.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023a/projects/appearances/protos/Grass.proto"

WorldInfo {{
  basicTimeStep 16
  gpsCoordinateSystem "local"
}}
Viewpoint {{
  orientation -0.3 0.85 0.35 1.0
  position -1.2 -1.6 1.3
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
Floor {{
  size 6 6
  appearance Grass {{
  }}
}}
Robot {{
  translation 0 0 {wheel_radius:.6f}
  supervisor TRUE
  children [
    DEF BODY Shape {{
      appearance PBRAppearance {{
        baseColor 0.917647 0.145098 0.145098
        roughness 1
        metalness 0
      }}
      geometry Box {{
        size {chassis_l:.6f} {chassis_w:.6f} {chassis_h:.6f}
      }}
    }}
{wheels_vrml}
    GPS {{
      name "gps"
    }}
    InertialUnit {{
      name "imu"
    }}
    DEF ROBOT_CAM Camera {{
      translation {robot_cam_pos[0]:.6f} {robot_cam_pos[1]:.6f} {robot_cam_pos[2]:.6f}
      name "camera"
      width {IMAGE_WIDTH}
      height {IMAGE_HEIGHT}
      fieldOfView {fov_rad:.6f}
    }}
    DEF ROBOT_CAM_DEPTH RangeFinder {{
      translation {robot_cam_pos[0]:.6f} {robot_cam_pos[1]:.6f} {robot_cam_pos[2]:.6f}
      name "camera_depth"
      width {IMAGE_WIDTH}
      height {IMAGE_HEIGHT}
      minRange {DEPTH_MIN_RANGE}
      maxRange {DEPTH_MAX_RANGE}
    }}
    DEF SCENE_CAM Camera {{
      translation {SCENE_CAM_POS[0]:.6f} {SCENE_CAM_POS[1]:.6f} {SCENE_CAM_POS[2]:.6f}
      rotation 0 1 0 {SCENE_CAM_PITCH_RAD:.6f}
      name "camera_chase"
      width {IMAGE_WIDTH}
      height {IMAGE_HEIGHT}
      fieldOfView {fov_rad:.6f}
    }}
    DEF SCENE_CAM_DEPTH RangeFinder {{
      translation {SCENE_CAM_POS[0]:.6f} {SCENE_CAM_POS[1]:.6f} {SCENE_CAM_POS[2]:.6f}
      rotation 0 1 0 {SCENE_CAM_PITCH_RAD:.6f}
      name "camera_chase_depth"
      width {IMAGE_WIDTH}
      height {IMAGE_HEIGHT}
      minRange {DEPTH_MIN_RANGE}
      maxRange {DEPTH_MAX_RANGE}
    }}
  ]
  name "rover"
  boundingObject USE BODY
  physics Physics {{
    density -1
    mass {weight_kg:.6f}
  }}
  controller "webots_rbx_bridge"
}}
"""


def main():
    model_name = sys.argv[1] if len(sys.argv) > 1 else "generic_rover"
    dims = loadDimensions(model_name, DEFAULT_DIMENSIONS)
    wbt_text = buildRoverWbt(dims)
    with open(OUTPUT_PATH, "w") as f:
        f.write(wbt_text)
    print("generate_rover_wbt: wrote %s from %s dimensions" % (OUTPUT_PATH, model_name), flush=True)


if __name__ == "__main__":
    main()
