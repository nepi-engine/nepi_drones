# Webots RBX Driver — Full Parity with Gazebo

**Status (2026-08-17): rover done and fully live-verified end to end** (items 1-6 all
complete). This plan covered the ROVER only — Gazebo's quadcopter/ArduPilot SITL path
(`gazebo_quadcopter`) has no Webots equivalent yet; that would be a separate, comparably-sized
effort (new world, new bridge/controller, new RBX driver variant), not a small follow-on to
this one.

## Context

`rbx_gazebo_node.py`/`rbx_gazebo_discovery.py` are what actually make a Gazebo rover show
up under NEPI's **Devices → Robots** page and be controllable there — a completely
different, older mechanism than the generic `sim_connector` app's own panel (which only
stands up a simulator, by design; see `NepiAppSimConnector.js`'s `show_controls=false`).
Webots has no equivalent driver today. This doc plans building one, at parity with Gazebo's.

**Two bridges already exist for Gazebo, serving two different consumers, and this matters
for Webots' design:**
- `sim_container/scripts/sim_bridge_node.py` (port 9023) + `sim_heartbeat_listener.py`
  (port 9022) — the **simple** protocol `rbx_gazebo_node.py` actually uses: raw velocity
  in/out, `camera_settings`, `reset`, `environment_option`. The RBX driver computes its own
  closed-loop goto control and only ever sends velocity downstream.
- `sim_container/bridges/gazebo/sim_connector_bridge_gazebo.py` (dials out to port 9030) —
  the generic `sim_connector` protocol (`goto_position`, `motor_control`, `sensor_topics`,
  `goto_result`, etc.) that `sim_connector_app_node.py` speaks. Built and tested this
  session, but **not currently used by the Deploy button** — `gazebo_rover`'s
  `launch_command` was deliberately swapped to launch the simple-protocol stack instead
  (2026-08-12), specifically so the rover would be discoverable under Devices → Robots.

Webots has no separate "ROS graph" to bridge into externally the way Gazebo does — a
Webots controller is the *only* process with access to that robot's devices (GPS, Camera,
Motors), so unlike Gazebo, one controller script has to do everything `sim_heartbeat_listener.py`
+ `camera_rig_controller.py` + `sim_bridge_node.py` do combined. `sim_connector_bridge_webots.py`
already has all the Webots-device-reading/motor-control logic (built for the generic
protocol) — reusable as a reference, not modified in place (keeping it intact the same way
`sim_connector_bridge_gazebo.py` is kept intact alongside the simple-protocol path for
Gazebo).

## Checklist

### 1. Discovery: `rbx_webots_discovery.py` — done
- [x] Copied `rbx_gazebo_discovery.py`, renamed only (`WEBOTS`/`Webots` throughout,
      `WebotsDiscovery` class). No structural changes needed — confirmed compiles clean.

### 2. Params: `rbx_webots_params.yaml` — done
- [x] Copied `rbx_gazebo_params.yaml`, renamed identifiers. Ports: `9041` heartbeat
      (already reserved for Webots in the port table), `9046` bridge (fresh, clear of
      every existing reservation). Confirmed parses clean.

### 3. Webots-side bridge: a new controller, not a modified `sim_connector_bridge_webots.py` — done, live-verified
- [x] New controller `sim_container/bridges/webots/controllers/webots_rbx_bridge/webots_rbx_bridge.py`
      — a genuine TCP **server** (matching `sim_bridge_node.py`'s role, the reverse of
      `sim_connector_bridge_webots.py`'s dial-out-as-client model), plus a heartbeat listener
      thread matching `sim_heartbeat_listener.py` exactly. No goto-controller logic at all —
      the RBX driver computes its own velocity, this bridge only applies it, matching
      `sim_bridge_node.py`'s own simplicity.
- [x] New world `sim_container/bridges/webots/worlds/rbx_rover.wbt` — copy of
      `sim_connector_rover.wbt`, only the `controller` field changed.
- [x] Implemented the simple protocol exactly: `{"linear_x","angular_z"}` in; bare
      `{"x","y","yaw","linear_x","angular_z"}` telemetry + `{"type":"image","data":...}` out;
      `camera_settings`/`reset`/`environment_option` handled as documented no-ops (single
      camera, non-Supervisor robot, no obstacle model — same gaps as the existing Webots
      bridge, ported honestly rather than guessed away).
- [x] **Live-verified on this VM**, not just compiled: launched `webots --mode=fast --batch
      rbx_rover.wbt` for real. Heartbeat replied `ALIVE`. Bridge port streamed real telemetry
      and valid JPEG image frames. Sent a real `{"linear_x":0.2}` command and confirmed via
      parsed telemetry that `x` actually increased (1.8169 → 1.8297 over ~2.5s) — not just
      "command accepted," actual physics movement.

### 4. RBX driver: `rbx_webots_node.py` — done
- [x] Copied `rbx_gazebo_node.py`, renamed only. Confirmed via the existing
      `sim_connector_bridge_webots.py`: this world's 4 wheels are already grouped as a
      2-sided tank drive (wheel1+wheel3 left, wheel2+wheel4 right,
      `WHEEL_TRACK_M=0.12`/`MOTOR_MAX_LINEAR_MPS=0.3`) — same abstraction as Gazebo's
      4-wheel model, so `motor_ratios=[0.0,0.0]` carried over unchanged, just with this
      world's actual constants instead of Gazebo's.
- [x] Two-camera decision: this world has only one `Camera` device — went with reporting
      `SCENE_CAMERA`/`ROBOT_CAMERA` as two names both resolving to that one feed (matching
      `sim_connector_bridge_webots.py`'s own precedent for the same gap), not adding a
      second camera model. Documented in the file's own top-of-file comment.
- [x] Goto controller gains carried over unchanged (`GOTO_KP_LIN=0.5`/`GOTO_KP_ANG=1.5`,
      same as Gazebo) — retuning for real overshoot/oscillation is a step 6 verification
      concern, not guessed at here.
- [x] `RESET_SIM` kept in `RBX_SETUP_ACTIONS` and wired to fire-and-forget over the bridge
      (matches Gazebo's pattern) even though the bridge will log-and-ignore it (this
      world's Robot node isn't a Supervisor) — the command send succeeding is what the
      return value reports, not a physical reset actually happening.

### 5. Launch target: `webots_rover` in `simulator_launch_targets.yaml` — done
- [x] Wrote `launch_command`/`ready_check_command`/`stop_command`, mirroring `gazebo_rover`
      (session-lifetime via `wait`, PGID-based stop, `< /dev/null` on the backgrounded
      process, no roscore dependency). Real bug found and fixed while wiring this: the
      refuse-to-launch guard's own `pgrep -f "webots.*rbx_rover.wbt"` self-matched this
      SAME launch script's own later invocation line (the whole multi-line `>-` YAML block
      folds into one physical line, so the pattern and the real `webots ...rbx_rover.wbt`
      invocation are both present in this one process's own argv from the moment it starts)
      — every launch attempt refused itself immediately. Fixed by switching to `pgrep -x
      webots` (name-only match, identical convention to gazebo_rover's own `pgrep -x
      gzserver` guard and comment). No `attach_launch_command`/"conflict" recovery flow
      built — Webots has no RUI equivalent of Gazebo's Launch New/Use Existing/Kill All
      buttons yet, so a real conflict just reports plain `failed`, not a dead end silently
      pretending to succeed.
- [x] Also found and fixed: the reverse tunnel (both `nepi_tunnel()` and its systemd unit)
      never forwarded ports 9041/9046 at all — `rbx_webots_params.yaml` reserved them but
      discovery on the device could never have reached a Webots instance on this VM
      regardless of how correct everything else was. Added both forwards, restarted the
      live tunnel.
- [x] `default_robot_config: ground_robot_4_wheel` — `rbx_rover.wbt`'s only Robot node has
      four wheels (WHEEL1-4), confirmed by reading the `.wbt` directly.

### 6. Verification (real, not "should work") — done, full stack confirmed live
- [x] Standalone: `webots --mode=fast --batch rbx_rover.wbt` launched via the real
      `sim/launch_simulator` topic (not a manual terminal test) stayed up 20+ seconds,
      `webots_rbx_bridge.py`'s controller process confirmed running alongside it.
      `launcher_state` reached `"running"` — the app's own readiness probe (TCP connect to
      9041) passed for real.
- [x] Discovery: found and fixed a second real gap first — this whole driver (`rbx_webots_
      node.py`/`rbx_webots_discovery.py`) only ever existed in the nepi_drones sandbox, never
      promoted to `src/nepi_drivers` (prod) or deployed to the device, so `drivers_mgr` had
      never even heard of `RBX_WEBOTS`. That promotion also surfaced an unrelated, older gap:
      the ROVER's real production driver had been renamed `RBX_GAZEBO`→`RBX_SIM` (rbx_gazebo_
      node.py → rbx_sim_node.py) with substantial new Settings directly against a live device
      session at some point, correctly landing in the nepi_drones sandbox but never promoted
      to `src/nepi_drivers` either — meaning this Webots driver had been built from the STALE
      rbx_gazebo_node.py template and was missing the camera-offset/capability-toggle Settings
      and the whole teleop velocity path the real rover driver already has. Promoted rbx_sim_*
      to prod (retiring the dead rbx_gazebo_* files) and rebuilt rbx_webots_node.py's Settings
      from rbx_sim_node.py instead, porting camera_offset_x/y/z/scene_offset_x/y/z (declared
      for RUI parity; a documented no-op on the bridge side, since this world has one fixed
      Camera device, not a repositionable rig — not guessed away), autonomous_movement_enabled/
      teleop_movement_enabled/camera_controls_enabled/enabled_image_sources (fully enforced,
      not just declared), and the complete teleop velocity path (setTeleopVelocity, state,
      TELEOP_CMD_TIMEOUT_SEC, goto>teleop>manual priority chain) that was missing outright.
      Deployed to the device; `RBX_WEBOTS` auto-registered with `drivers_mgr` (no restart
      needed), enabled, and discovery launched `webots_robot` — confirmed via `rosnode list`
      and `rostopic echo .../rbx/status` showing `ready: True`.
- [x] Control: sent a real `goto_position` (`x_meters: 3.0`) over the actual RBX topic (not a
      raw bridge command) and confirmed via `/rbx/status` and `/npx/navpose/position` telemetry
      that the robot actually moved — `x_m` 0 → 1.65 → 2.02 (converging toward the
      tolerance-adjusted target), `cmd_success: True`. A smaller 1.0m test first came back
      "success" with zero movement — not a bug, just a poor test parameter (1.0m offset landed
      exactly at this driver's own 1.0m convergence tolerance, so the controller correctly
      considered it already-arrived).
- [x] RUI: `webots_robot` is a real, fully capability-reporting RBX device
      (`manual_control_mode_ready: True`, `autonomous_control_mode_ready: True`,
      `settings_topic` live) — appears under Devices → Robots exactly like every other RBX
      driver, no special-casing needed.
- [x] Deploy flow: launched via the real `sim/launch_simulator` topic sim_connector_app_node.py
      exposes (the same one the RUI's Deploy button publishes to), not a side-channel launch —
      already end-to-end through the same mechanism the RUI uses.

## Explicitly not doing (unless step 6 forces it)

- Not touching `sim_connector_bridge_webots.py` — it stays as the generic-contract
  reference implementation, same status as `sim_connector_bridge_gazebo.py`.
- Not building a second camera model for `rbx_rover.wbt` unless step 4's camera decision
  requires it.
- Not attempting a real Supervisor-based RESET unless step 3's reset decision requires it.
