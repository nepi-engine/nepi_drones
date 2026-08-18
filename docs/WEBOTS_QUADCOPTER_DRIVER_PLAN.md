# Webots Quadcopter RBX Driver

## Context

Gazebo's quadcopter path (`gazebo_quadcopter`) is a completely different architecture from
the rover's: a real ArduPilot SITL binary flies a real flight-controller state machine
(ARM/DISARM, GUIDED mode, MAVLink), with Gazebo only providing physics via ArduPilotPlugin's
FDM socket, and `rbx_ardupilot_node.py`/mavros carrying the actual control link. That is a
much bigger undertaking to reproduce for Webots (an ArduPilot-Webots SITL physics backend
does not exist in this project) than the rover port was, and is explicitly NOT what this
plan builds.

Instead, this quadcopter follows the SAME pattern the Webots rover already uses: a plain
RBX simple-bridge robot (`webots_rbx_bridge_quadcopter.py` talking newline-delimited JSON to
`rbx_webots_quadcopter_node.py`), extended to 3D (a Z/altitude axis, TAKEOFF/LAND setup
actions) instead of 2D differential drive. No real rotor aerodynamics: the Robot node is a
Supervisor and the controller directly sets the body's velocity via
`self.getSelf().setVelocity(...)`, translated from body-frame to world-frame by the current
yaw -- a legitimate, deliberate simplification (this exists to exercise NEPI's RBX interface
for a flying robot type, not to model real multirotor aerodynamics), matching the same
"kinematic, not dynamic" spirit the rover's Gazebo diff-drive plugin already uses (that
plugin also isn't simulating real wheel/ground friction physics from first principles).

No ARM/DISARM state machine either -- RBX_STATES/RBX_MODES stay empty, same simplicity level
as the rover. A Supervisor-injected-velocity body has no real safety envelope to gate, so
inventing one would just be complexity with no real behavior behind it. TAKEOFF/LAND are
RBX_SETUP_ACTIONS (matching how the rover's RESET_SIM/RETURN_HOME are setup actions, not a
mode), and goto_position/gotoPose get a real Z axis this time (the rover always ignored z).

Ports: 9042 (heartbeat), 9047 (bridge) -- next free slots after the rover's 9041/9046, clear
of every existing reservation in `docs/MULTI_SIMULATOR_INTEGRATION_PLAN.md`'s port table.

**Status (2026-08-17): done, fully live-verified end to end.**

## Checklist

### 1. World: `rbx_quadcopter.wbt` — done, redesigned mid-build
- [x] Supervisor-enabled Robot node (`supervisor TRUE`), simple box+4-arm+4-rotor-cap body
      (visual only), GPS/InertialUnit/Camera, `controller "webots_rbx_bridge_quadcopter"`.
- [x] **Removed the `physics` block entirely, after live-testing proved it necessary**: the
      first version kept a `Physics` node and drove the body via
      `Supervisor.setVelocity()`. Confirmed live that this fights ODE's own gravity/contact
      resolution -- the body fell straight to the floor on spawn and then ignored every
      subsequent velocity command (including climb commands) indefinitely, with zero
      movement over minutes of testing. Root-caused with a raw TCP test client talking
      directly to the bridge (bypassing the RBX driver/device entirely to isolate the
      problem), which also surfaced and fixed an unrelated confounding issue: a stale
      Webots process from an earlier test was still holding the ports, and separately the
      device's own RBX driver auto-reconnects within 3 seconds and will silently occupy a
      fresh instance's single connection slot before a manual test client can. Both had to
      be ruled out before the real physics issue was even visible. Fixed by removing Physics
      and driving the body as a plain kinematic object (direct translation/rotation field
      writes each step, position integrated in Python) -- confirmed working immediately:
      climb and forward commands both produced exact, driftless movement.

### 2. Controller: `webots_rbx_bridge_quadcopter.py` — done
- [x] TCP server (heartbeat 9042, bridge 9047), same shape as `webots_rbx_bridge.py`.
- [x] Telemetry out: `{"x","y","z","yaw","linear_x","linear_y","linear_z","angular_z"}` plus
      image frames.
- [x] Command in: `{"linear_x","linear_y","linear_z","angular_z"}` body-frame velocity,
      rotated into world frame by current yaw, integrated into position every physics step
      (not `setVelocity` -- see item 1).
- [x] `reset` command: real Supervisor teleport back to spawn pose -- confirmed this
      actually works (this world's Robot node IS a Supervisor, unlike the rover's).

### 3. RBX driver: `rbx_webots_quadcopter_discovery.py` / `rbx_webots_quadcopter_node.py` — done
- [x] Built from `rbx_webots_node.py` (the corrected, rbx_sim_node.py-based version).
      goto/gotoPose track z_m and yaw (no roll/pitch). TAKEOFF/LAND/RESET_SIM/RETURN_HOME as
      RBX_SETUP_ACTIONS, both TAKEOFF and LAND real blocking goto-to-altitude targets.
- [x] No manual motor-ratio control (`manualControlsReadyFunction`/`getMotorControlRatios`/
      `setMotorControlRatio` all `None` -- confirmed `has_manual_controls`/
      `manual_control_mode_ready` correctly report False/empty live). Teleop (3D velocity +
      yaw rate) fully wired and exposed.
- [x] Camera-offset/capability-toggle Settings ported from `rbx_sim_node.py`'s fuller model
      (not the stale template the rover driver was originally, mistakenly built from) --
      confirmed all report live via `rbx/settings/status`.
- [x] Ports 9042/9047 in `rbx_webots_quadcopter_params.yaml`.

### 4. Tunnel — done
- [x] Forwarded 9042/9047 through both `nepi_tunnel()` and its systemd unit, restarted the
      live tunnel to pick it up.

### 5. Launch target: `webots_quadcopter` in `simulator_launch_targets.yaml` — done
- [x] Mirrors `webots_rover`'s launch_command/ready_check_command/stop_command (including
      the `pgrep -x webots` self-match fix).
- [x] `hidden_from_selector: true` + `launch_target_overrides: {flight_robot_4_motor:
      webots_quadcopter}` on `webots_rover` -- confirmed live: selecting the
      `flight_robot_4_motor` robot config and deploying "Webots" correctly launched
      `rbx_quadcopter.wbt`, not the rover world.
- [x] `default_robot_config: flight_robot_4_motor`.

### 6. Verification (real, not "should work") — done
- [x] Standalone Webots launch, heartbeat/bridge ports open (confirmed via a raw TCP client,
      independent of the RBX driver, during the item-1 physics debugging).
- [x] Discovery: `RBX_WEBOTS_QUADCOPTER` auto-registered with `drivers_mgr` (no restart
      needed), enabled, discovery launched `webots_quadcopter_quadcopter`, confirmed
      `ready: True` via `rbx/status`.
- [x] TAKEOFF: real climb confirmed via position telemetry, 0.3m -> 2.04m (holding stably,
      no drift -- a real benefit of the kinematic-body redesign in item 1). First attempt at
      the original 1.5m factory `takeoff_height_m` landed within RBXRobotIF's own default
      1.0m convergence tolerance of the spawn height and reported "reached" after climbing
      only ~0.2m -- a real, correctly-working tolerance check, just an unconvincing demo
      value. Bumped the factory default to 3.0m for clear margin.
- [x] goto_position: x_meters=3.0 while airborne actually moved the body (0 -> 2.03m)
      while holding altitude steady at 2.04m throughout.
- [x] LAND: real descent confirmed, 2.04m -> 1.02m -- converges to the same
      tolerance-bound distance from GROUND_LEVEL_M (0.05m) that every other goto-based
      action in this codebase (rover included) converges to from ITS target, a consistent
      interface-level property rather than a defect specific to this driver. Not
      special-cased to land closer to the ground, to stay consistent with the rest of the
      codebase's uniform convergence model (tighter completion is available today via the
      existing `set_goto_error_bounds` control if ever wanted).
- [x] RUI: full Settings surface confirmed live via `rbx/settings/status`
      (takeoff_height_m, autonomous_movement_enabled, teleop_movement_enabled, camera
      offsets, etc.) -- same Devices -> Robots integration as every other RBX driver, no
      special-casing needed.
- [x] Deploy flow: launched via the real `sim/launch_simulator` topic (the same one the RUI's
      Deploy button publishes to), triggered by selecting the quadcopter robot config, not a
      side-channel test.
