# Rover Wheel Independence ("Crab Steering")

## What was requested

Live, 2026-09-04: "there should also be a setting for robot wheel
independence, meaning the robot (like a rover) can move to the side without
its base moving, where only the wheels need to move a certain direction.
the base can still face the same way. this is what the first robot robots
are like usually - both the base and wheels rotate individually of each
other. if its disabled, it will just work normally."

This is a 4-wheel-independent-steering ("crab steering") rover: every wheel
can point in the direction of travel independent of the chassis heading, so
the rover can translate sideways/diagonally without its body yawing.
Disabled (default) must be today's exact skid-steer behavior, byte-for-byte
unchanged.

## What the rover looked like before this

- `sim_container/scripts/generate_model_sdf.py`'s `buildRoverSdf`: one
  `revolute` joint per wheel (Y axis, spin only) directly from `base_link`.
  No steering joint anywhere.
- Drive plugin: `libgazebo_ros_diff_drive` (skid-steer). Reads
  `geometry_msgs/Twist` on `cmd_vel`, drives every left-side joint
  identically and every right-side joint identically (two independent
  speeds, not four). No `linear.y` support at all.
- `rbx_sim_node.py`'s velocity command path is `linear_x`/`angular_z` only,
  end to end; the rover is treated as non-holonomic as a load-bearing
  assumption elsewhere (timeout sizing, etc.) -- unaffected by this work,
  since `libgazebo_ros_planar_move` (see below) reads the same `cmd_vel`
  message type and just also honors its `linear.y` field when it's nonzero.

## Design

Additive only. A new curated dimension field, `wheel_independence_enabled`
(0/1, default 0), in `ROBOT_DIMENSION_FIELDS` (`Nepi_IF_Sim.js`) --
persisted the same way every other rover dimension already is, no new RUI
widget. `buildRoverSdf` branches on it:

**Disabled (default):** exactly today's SDF. Confirmed by diffing generator
output before/after this change with the field explicitly 0 -- byte-for-byte
identical.

**Enabled:**
- Each wheel gets a new "steer hub" link (negligible mass/inertia, same
  convention `camera_link` already uses for non-structural helper links)
  inserted between `base_link` and the wheel: `base_link` ->
  (`{wheel}_steer_joint`, revolute, Z axis) -> `{wheel}_hub` ->
  (`{wheel}_joint`, revolute, Y axis -- same joint name/axis the disabled
  case already uses) -> `{wheel}` link. SDF link `<pose>` is absolute in
  the model frame regardless of joint topology, so wheel poses themselves
  don't need to change.
- Wheel-ground friction (`mu`/`mu2`) is dropped to near-zero (0.05/0.05,
  vs. 1.5/0.2 in the disabled case) for this mode only. Real locomotion
  comes entirely from `libgazebo_ros_planar_move` in this mode (a
  kinematic body-frame velocity, no wheel-ground traction involved at
  all), so ground friction on the wheels serves no locomotion purpose here
  and only fights the steering joint's own drive (confirmed live: default
  friction left the steering joint essentially unable to turn against
  drag from the body being moved by planar_move regardless of what the
  wheel itself wanted to do).
- Drive plugins: `libgazebo_ros_diff_drive` is replaced by
  `libgazebo_ros_planar_move` (a stock Gazebo plugin -- real body-frame
  x/y/yaw kinematics from the same `cmd_vel` Twist, natively understands
  `linear.y` unlike diff_drive) plus a new plugin,
  `nepi_crab_steer_plugin` (this repo's own, see below), which purely
  animates the 4 wheel corners to visually steer+spin consistent with that
  same motion. Two independent `cmd_vel` subscribers on one topic is fine.

## `nepi_gazebo_plugins` (new catkin package)

`sim_container/nepi_gazebo_plugins/` -- one Gazebo `ModelPlugin`,
`crab_steer_plugin.cpp` (`libnepi_crab_steer_plugin.so`):

- Subscribes to `cmd_vel` (namespaced via `<robotNamespace>`, matching
  `planar_move`'s own convention -- see "what went wrong" below for why
  this matters).
- Each `WorldUpdateBegin` tick: computes `atan2(vy, vx)` as the target
  steer angle (held at its last value while the rover is stopped, so a
  wheel pointed sideways doesn't visibly snap back to straight-ahead) and
  teleports each steer joint there with `Joint::SetPosition()`; drives each
  spin joint's velocity with `Joint::SetParam("vel"/"fmax", ...)` -- ODE's
  built-in velocity motor, the same mechanism `libgazebo_ros_diff_drive`
  itself already uses elsewhere in this codebase for spin.
- The steer angle's sign is empirically negated relative to a plain
  `atan2(vy, vx)` -- confirmed live that this joint's own measured
  `Position()` comes out inverted (a commanded +90 degrees measured back
  as -90), most likely ODE's own child-relative-to-parent convention for
  this specific axis/parent-child pairing, not a math error.

### What went wrong before it worked (kept here so the next reader doesn't repeat the detour)

Every early attempt (raw `SetPosition` on `WorldUpdateBegin`, Gazebo's own
`JointController::SetPositionTarget`/`Update` with various PID gains, a
manual proportional-torque `SetForce` loop, running on `WorldUpdateEnd`
instead of `Begin`) read back an unchanged, near-zero steering angle no
matter what was tried. Debug logging (a `gzmsg` print of the plugin's own
received `vx`/`vy` each tick) showed the real root cause: this plugin's
`cmd_vel` subscriber had no `<robotNamespace>`, so `ros::NodeHandle()`
resolved `cmd_vel` against the GLOBAL namespace (`/cmd_vel`), while
`planar_move` (mounted alongside) built its own `NodeHandle` WITH
`robotNamespace` applied and correctly subscribed to `/rover/cmd_vel` --
the two plugins were listening on two entirely different topics the whole
time, so `crab_steer_plugin` never received a real command to act on. Once
`<robotNamespace>/rover</robotNamespace>` was added to its own SDF block
(matching `planar_move`'s), the very first (simplest) implementation
worked immediately and reliably.

## Live verification (this dev machine, not the sim VM)

This dev machine has `gazebo11`/`ros-noetic-gazebo-plugins`/
`ros-noetic-gazebo-ros-control` installed locally, so the whole stack was
built and exercised directly here (`cmake`/`make` against the system
Gazebo+ROS, then a real headless `gzserver` via
`roslaunch gazebo_ros empty_world.launch` with the generated
`wheel_independence_enabled: 1` model spawned in a minimal world) --
not just a compile check. Confirmed via `/gazebo/get_model_state` and
`/gazebo/get_joint_properties`, commanding a pure `linear.y = 0.5` Twist on
a freshly-spawned model:

- Steering: `front_left_wheel_steer_joint` reaches and holds -90 degrees
  (the correct orientation for the sign convention above) within 0.5s and
  stays there for the whole 5+ second hold, across multiple repeated runs.
- Spin: nonzero (~2 rad/s) while commanded.
- Body: over 4 simulated seconds, `y` moved by 1.3-2.0m (matching the
  commanded speed), `x` stayed within a few cm of 0, and yaw drifted only a
  few degrees (typically under 5) -- i.e. the rover translates sideways
  without its body meaningfully rotating, confirmed via real telemetry
  across many repeated test runs, not assumed from code reading.

This was NOT verified on the actual sim VM this team deploys Gazebo to for
real use (that's a separate machine from the NEPI device, reachable only
by SSH, which was down this session) -- the plugin needs to be built there
too (a scoped `catkin build nepi_gazebo_plugins` on that machine's own
workspace) before this feature can actually run in the team's real
deployment. The `generate_model_sdf.py` and RUI changes are plain
Python/JS and need no separate build once combined into that workspace;
only the new C++ package needs compiling per target machine.
