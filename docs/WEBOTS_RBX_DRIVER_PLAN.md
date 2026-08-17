# Webots RBX Driver — Full Parity with Gazebo

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

### 5. Launch target: `webots_rover` in `simulator_launch_targets.yaml`
- [ ] Write `launch_command`/`ready_check_command`/`stop_command`/`attach_launch_command`,
      mirroring `gazebo_rover`'s exactly (session-lifetime-scoped via `wait`, PGID-based
      stop, refuse-to-launch guard against a second Webots instance, `< /dev/null` on every
      backgrounded process, roscore NOT needed here since Webots' controller has zero ROS
      dependency — confirm nothing else on this launch path assumes one).
- [ ] `default_robot_config`: whichever of `ground_robot_2_wheel`/`ground_robot_4_wheel`
      actually matches `rbx_rover.wbt`'s wheel count (the existing Webots world was adapted
      from Webots' 4-wheel tutorial robot per Phase 2's notes — confirm before picking).

### 6. Verification (real, not "should work")
- [ ] Standalone: launch Webots with `rbx_rover.wbt` directly (`webots --mode=fast`), confirm
      no crash, confirm the heartbeat and bridge ports open.
- [ ] Discovery: confirm `rbx_webots_discovery.py` detects it and launches `rbx_webots_node.py`
      — check `rosnode list` for the new node, `rostopic echo .../rbx/status` for real data.
- [ ] Control: goto commands move the real Webots robot (verify via its own telemetry, same
      rigor as every other phase this project has used — position before/after, not just
      "command accepted").
- [ ] RUI: confirm the robot appears under Devices → Robots and its controls work — this is
      the actual point of choosing full parity over panel-only.
- [ ] Wire `webots_rover`'s `launch_command` into the Sim Connector app's Deploy flow and
      confirm end to end: Deploy → Webots launches → robot appears in Devices → Robots →
      controllable there, Sim Connector panel itself shows no manual controls (by design).

## Explicitly not doing (unless step 6 forces it)

- Not touching `sim_connector_bridge_webots.py` — it stays as the generic-contract
  reference implementation, same status as `sim_connector_bridge_gazebo.py`.
- Not building a second camera model for `rbx_rover.wbt` unless step 4's camera decision
  requires it.
- Not attempting a real Supervisor-based RESET unless step 3's reset decision requires it.
