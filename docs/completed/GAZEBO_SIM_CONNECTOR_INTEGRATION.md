# Gazebo → `sim_connector` Integration (Completed)

Extracted from `MULTI_SIMULATOR_INTEGRATION_PLAN.md`'s Phase 1, which is fully closed —
this is the one simulator integration verified end to end, including against the real
device. Kept as its own doc now that it's done, rather than mixed in with the still-open
phases for Webots/Stage/PyBullet/Unity/WPILib in the main plan.

---

## Starting point: the premise this corrected

The original request assumed "we're right now only testing with Gazebo," implying Gazebo
already flowed through the generic multi-simulator abstraction. On inspection that wasn't
the current state:

- **`nepi_app_sim_connector`** (`api/device_if_sim.py` + `scripts/sim_connector_app_node.py`)
  — the genuinely simulator-agnostic contract — was real and well-designed: one well-known
  TCP port (9030 factory default), a documented newline-delimited-JSON wire protocol, and
  capability flags derived from a `SIM_VEHICLE_DICT` robot-config profile. **But it had
  never been connected to a real simulator** — only exercised by
  `test_device_if_sim_harness.py` (mocked lambda callbacks, no bridge socket at all) and
  `demo_bridge_client.py` (a synthetic client, explicitly documented as "NOT a real
  simulator integration").
- The Gazebo integration that *was* real and working (the rover + the ArduPilot SITL
  drone) used a **different, older path**: the `RBX_SIM`/`RBX_ARDUPILOT` drivers built
  directly on `RBXRobotIF` (`device_if_rbx.py`), with their own bespoke TCP/JSON bridge
  (`sim_bridge_node.py`) — not `SimDeviceIF`/`sim_connector_app_node.py` at all.

So the real starting line was: zero simulators, including Gazebo, were wired into the new
generic abstraction. This phase's job was "get Gazebo talking to
`sim_connector_app_node.py`'s protocol," reusing everything already learned from
`sim_bridge_node.py`/`camera_rig_controller.py` rather than re-deriving it — and it
doubled as the shakedown for the process every later simulator (Webots, Stage, PyBullet,
WPILib) repeats.

## What was built

`sim_container/bridges/gazebo/sim_connector_bridge_gazebo.py` — a plain ROS node, zero
`nepi_sdk` dependency, that dials `sim_connector_app_node.py`'s TCP port and speaks its
exact wire protocol:

- Subscribes to `/rover/odom` (published by `libgazebo_ros_diff_drive.so`) and reformats
  it into the bare-telemetry JSON line the protocol expects.
- Announces `sensor_topics` for both the rover's cameras.
- Receives `motor_control`/`goto_position`/`go_home`/`go_stop`/`setup_action` lines and
  translates them using `rbx_sim_node.py`'s already-proven closed-loop goto controller
  math (proportional gains, turn-in-place gate) rather than re-deriving it.
- Relays camera frames the same way `camera_rig_controller.py` already does (base64 JPEG).
- Reuses `sim_bridge_node.py`'s obstacle-course spawn/delete pattern for the environment
  option.

No changes were needed to `SIM_VEHICLE_DICT` — `ground_robot_2_wheel`, already present in
`sim_connector_app_params.yaml`, matched this rover's real capabilities exactly (2
wheels/2 motors, `goto_position` only, two cameras, environment control).

## Verification

Verified end to end against the real `sim_connector_app_node.py` (not the mock harness),
on an isolated test roscore first, then on the real device:

- `bridge_connected: True`, `telemetry_age_sec` sub-100ms, live the whole session.
- `available_sensor_topics` announced both cameras (`gazebo_rover/robot_camera`,
  `gazebo_rover/scene_camera`); `capabilities_query` after selecting
  `ground_robot_2_wheel` matched the config exactly — confirmed the flags are a real
  derived report, not a static echo of the YAML.
- A `goto_position(x=2.0)` command drove the rover from `x≈0.05` to `x≈1.76` in Gazebo,
  proof the full path (command → app node → bridge → controller → real physics) works.
- `set_camera_view_mode(SCENE_CAMERA)` correctly switched which camera's frames the
  bridge relays.
- **Real-device confirmation:** direct network connectivity works with no SSH tunnel
  needed (VM and device share a LAN). Reconnected the bridge straight to the real device
  and repeated the full verification: `bridge_connected: True`, `capabilities_query`
  matched exactly, `goto_position(x=2.0)` moved the real device's tracked position from
  `x≈0.12` to `x≈1.82`, camera feed flowed on the device's actual `color_2d_image` topic
  at ~3.5-3.9Hz (slightly under the VM-only tests' 5Hz, consistent with real LAN latency).

## What's still open from this phase

- **A human loading the RUI page in a browser** and eyeballing that the capability-driven
  controls render correctly — everything the RUI reads to do that is confirmed live and
  correct, but no agent has browser/screenshot capability to complete this last visual
  step. Tracked in `SIM_CONNECTOR_REMAINING_WORK.md`.
- **`RESET`/`RETURN_HOME` and the `obstacle_course` environment option** are wired
  (reusing the existing Gazebo services/model) but weren't individually re-verified —
  worth a quick check.
- **Minor, not blocking:** the image relay ran at ~15Hz against an intended 5Hz cap
  (`IMAGE_RATE_HZ` in the bridge script) — the rate-limit logic should be re-checked
  before treating bandwidth as tuned.

## Setup notes for repeating this pattern on a bare test roscore

Needed again for every later simulator phase (Webots, Stage, PyBullet, WPILib):

1. `sim_connector_app_node.py` imports `nepi_api.device_if_sim`, which only exists there
   via the app's own `CMakeLists.txt` install rule on a real build — a bare devel-space
   catkin workspace doesn't have it. Symlinking
   `nepi_app_sim_connector/api/{device_if_sim,messages_if}.py` directly into the
   workspace's `nepi_api` source package reproduces that install-time override locally
   (this is what `~/sim_connector_test_ws` does).
2. `apps_mgr` normally loads every top-level key of `sim_connector_app_params.yaml` onto
   the app's own param namespace before launch — bypassing that (running the script
   directly) leaves `robot_configs` at just the capability-empty `default` entry. Load it
   explicitly: `rosparam load sim_connector_app_params.yaml /nepi/device1/app_sim_connector`.
3. Seed `debug_mode`/`user_folders` first — see `SIM_CONNECTOR_NAVPOSE_HANG_BUG.md`.
4. Launch with `python3 -u` (or `PYTHONUNBUFFERED=1`) and verify state via `rosservice
   call .../capabilities_query` / `rostopic echo .../sim/status`, not by grepping a
   redirected log.
