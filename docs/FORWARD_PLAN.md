# NEPI Drone Simulator — Forward Plan

A straightforward, ordered plan to move the SITL + motor-control work forward, with
owners called out. Deep detail lives in the companion docs; this is the "what next,
who does it" summary.

- Detail refs: [SIMULATOR_DEV_GUIDE.md](SIMULATOR_DEV_GUIDE.md) ·
  [SITL_IMPLEMENTATION_PLAN.md](SITL_IMPLEMENTATION_PLAN.md) ·
  [MOTOR_CONTROL_RATIOS.md](MOTOR_CONTROL_RATIOS.md)

---

## The goal, in one line

Drive the **ArduPilot SITL + Gazebo** simulator through the NEPI **RBX driver** —
autonomous control first, then direct per-motor control — with nav/pose feedback
flowing back into NEPI.

## What "motor controls" means (so there's no confusion)

Each motor is commanded as a **speed/power ratio, 0.0–1.0** (`0` = off, `1` = full).
It is **not** degrees and **not** a rotation count. NEPI sends `(motor_ind,
speed_ratio)`; the **driver/sim converts** that ratio into real actuator units and
reports the resulting motion back as nav/pose. Suraj's job: make a 0–1 value spin
the right motor in SITL.

---

## Suraj — do these in order

1. **Environment up.** WSL / Ubuntu 20.04 + ArduPilot SITL + Gazebo. Run SITL with
   `--no-mavproxy` (keeps TCP 5760 free for mavros).
   - *Done when:* `nc -z 127.0.0.1 5760` reports the port open while SITL runs.

2. **Connect RBX driver to SITL.** Add the `SITL` connection option (params + discovery
   only; the node doesn't change). Set `fake_gps = False` for SITL.
   - *Done when:* `mavlink_sitl/state` shows `connected: True` and the `ardupilot_sitl`
     device appears in the RUI. See [SITL plan §3](SITL_IMPLEMENTATION_PLAN.md).

3. **Autonomous flight via the RUI.** GUIDED → ARM → TAKEOFF → GoTo / Go Home.
   (Asher will walk you through the RUI procedure.)
   - *Done when:* the sim vehicle moves in Gazebo on a GoTo, no callback errors.

4. **Document sim nav/pose data.** List what SITL/mavros publishes (position,
   orientation, velocity, GPS, EKF) and map it to the NEPI messages the app needs.
   - *Done when:* a short markdown list of available vs. required nav/pose messages
     exists (feeds Jason's mock app).

5. **Motor controls — prove one motor from the CLI first.** With Asher's help, find the
   correct **MAVROS actuator/motor command** for ArduPilot and drive a single motor over
   the command line before writing driver code.
   - *Done when:* one motor visibly spins in Gazebo from a CLI command.

6. **Motor controls — implement the driver hooks.** In `rbx_ardupilot_node.py`:
   - `getMotorControlRatios() -> list[float]` (length = motor count; return current ratios)
   - `setMotorControlRatio(motor_ind, speed_ratio)` (wrap the MAVROS command from step 5;
     convert 0–1 → actuator units; clamp to `[0,1]`)
   - `manualControlsReady()` (already exists; true only in MANUAL mode)
   - Then uncomment lines [325-327](../src/nepi_drivers/rbx_drivers/rbx_ardupilot_node.py#L325-L327)
     to pass all three into `RBXRobotIF`.
   - Re-publish at ~50 Hz for any sustained command (ArduPilot fails safe otherwise).
   - *Done when:* commanding `set_motor_control` over the topic moves the motor in
     Gazebo and `current_motor_control_settings` in the RBX status updates.

> **Note on the RUI:** selecting "Manual" appears automatically once the driver reports
> the capability — no RUI code needed for that. But there is **no per-motor slider in the
> UI today**, so command motors over the **ROS topic / CLI** for now. Do **not** build a
> slider yet; that's a separate front-end task owned by Asher (see below).

---

## Asher — supporting items

- **Teach the RUI process.** Walk Suraj through arming + autonomous nav in the RUI
  (GUIDED/ARM/TAKEOFF/GoTo).
- **RBX connect.** Confirm the RBX-connect path Suraj needs for the SITL device.
- **Motor-control CLI demo.** Show Suraj how to publish a direct message to the mavros
  node so he can test individual motors before integrating (step 5 above).
- **Deployment script for `nepi_drones`** *(currently the main blocker)*. Configure it to
  override the base source in the right folders, using the **Ocean Aero** config as the
  reference pattern. Until this lands, Suraj's deploy → `nepi build` → restart loop isn't
  clean.
- **Per-motor slider — decision, not yet a task.** If/when a UI to drag each motor's
  speed is wanted, it's an add to the RBX Controls "Manual" panel
  ([NepiDeviceRBX-Controls.js:411](../../nepi_engine_ws/src/nepi_rui/src/rui_webserver/rui-app/src/NepiDeviceRBX-Controls.js#L411)).
  Not needed to get motor control working; decide after the topic path is proven.

## Jason — as needed

- Architecture / MAVROS command guidance; the "mock app" for the simulator.

---

## Command-line cheatsheet (pulled from the 2023 ArduPilot tutorial)

These are the raw `rostopic`/`rosservice` commands from the old
*Autopilot Interfacing and Automation (Ardupilot)* tutorial — the same "direct
messages to the mavros node" Asher offered to demo. **Ignore the tutorial's
automation scripts** (deprecated per the meeting); only the raw commands below are
current.

**Two adaptations from the tutorial text:**
- Tutorial namespace is `/nepi/s2x/pixhawk_mavlink`. For the SITL setup it's
  `/nepi/device1/mavlink_sitl`. Set it once per terminal:
  ```bash
  source /opt/nepi/ros/setup.bash
  MAV=/nepi/device1/mavlink_sitl     # confirm real name with: rostopic list | grep mavlink
  ```
- The tutorial's arming/mode calls are shown as *introspection* stubs
  (`value: false`, `custom_mode: ''`). The usable versions are below.

**Discover what's available**
```bash
rostopic list | grep mavlink
rosservice list | grep mavlink
rostopic list | grep setpoint
```

**Confirm mavros is connected (Task 1)**
```bash
rostopic echo $MAV/state          # expect connected: True
```

**Read nav/pose data (Task 4 — the data to document)**
```bash
rostopic echo $MAV/global_position/global      # lat/lon/alt (WGS84)
rostopic echo $MAV/global_position/local       # ENU pose/odom
rostopic echo $MAV/global_position/compass_hdg # heading
# NEPI-side fused solution:
rosservice call /nepi/device1/nav_pose_query "query_time: {secs: 0, nsecs: 0}
transform: false"
```

**Arm / mode / takeoff (Task 3 — normally done via the RUI; CLI shown for reference)**
```bash
rosservice call $MAV/set_mode "base_mode: 0
custom_mode: 'GUIDED'"
rosservice call $MAV/cmd/arming "value: true"
rosservice call $MAV/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 10.0}"
```

**Autonomous setpoint moves (message types the tutorial confirms)**
```bash
rostopic pub $MAV/setpoint_position/local  geometry_msgs/PoseStamped ...
rostopic pub $MAV/setpoint_position/global geographic_msgs/GeoPoseStamped ...
rostopic pub $MAV/setpoint_raw/attitude    mavros_msgs/AttitudeTarget ...
# hit space+Tab after the message type to auto-fill the body
```

**Direct motor drive — the research lead (Task 5a)**
The tutorial only drives individual motors through the **Mission Planner GUI**
(SETUP/Optional Hardware/Motor Test, "Test motor A–D") — there is no CLI motor-drive
command in it. But it lists a mavros **actuator** topic as the CLI entry point:
```bash
rostopic list | grep -i actuator     # tutorial references $MAV/actuator_control
```
Confirm the exact topic + message ArduCopter actually accepts for per-motor drive
(`mavros_msgs/ActuatorControl`, or `MAV_CMD_DO_MOTOR_TEST` via `$MAV/cmd/command`) —
**this is your open research item.** Dev params from the tutorial to tame spin speed
while testing (set in Mission Planner / SITL params):
`MOT_PWM_MAX=1500`, `MOT_SPIN_ARM=0.03`, `MOT_SPIN_MAX=0.5`, `MOT_SPIN_MIN=0.15`.

**Fake GPS — SKIP for SITL** (SITL supplies its own GPS). Kept only for the no-GPS
hardware path:
```bash
rostopic pub -r 10 $MAV/hil/gps mavros_msgs/HilGPS '{header: auto, fix_type: 3, geo: {latitude: 47.6541, longitude: -122.31894, altitude: 0.005}, eph: 0, epv: 0, vel: 0, vn: 0, ve: 0, vd: 0, cog: 0, satellites_visible: 9}'
```

**NEPI motor command (after the driver hooks from step 6 are wired)**
```bash
rostopic pub -1 /nepi/device1/ardupilot_sitl/rbx/set_motor_control \
  nepi_interfaces/MotorControl "{motor_ind: 2, speed_ratio: 0.4}"
```

---

## Dependencies / blockers

- **Deploy script (Asher)** gates Suraj's clean iteration loop — highest-leverage unblock.
- **Fake GPS must be OFF for SITL** (it fights SITL's own simulated GPS/compass).
- **Motor commands only work in MANUAL mode + armed** (the readiness gate).

## Already handled (no action needed)

- The NEPI-side motor-control API (`device_if_rbx.py`) is complete and its bugs were
  fixed 2026-07-16 — Suraj builds against a clean `(motor_ind, speed_ratio)` interface.
- Capability auto-detection: supplying `setMotorControlRatio` is all it takes for "Manual"
  to show in the RUI. No RUI changes required for that.
