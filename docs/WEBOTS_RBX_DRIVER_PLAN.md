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

### 1. Discovery: `rbx_webots_discovery.py`
- [ ] Copy `rbx_gazebo_discovery.py` as the starting point — its structure (purge → probe →
      launch, heartbeat-with-ALIVE-reply pattern, backoff) needs no Webots-specific changes,
      only renaming (`PKG_NAME`, `DEVICE_ID`, log names).
- [ ] Confirm the discovery options schema (`host`, `heartbeat_port`, `bridge_port`) matches
      what `rbx_webots_params.yaml` (below) declares.

### 2. Params: `rbx_webots_params.yaml`
- [ ] Copy `rbx_gazebo_params.yaml`, rename identifiers, pick heartbeat/bridge ports not
      already used in the 902x block (`docs/MULTI_SIMULATOR_INTEGRATION_PLAN.md`'s port
      table already reserves `9041` for Webots utility use — use that plus one more).

### 3. Webots-side bridge: a new controller, not a modified `sim_connector_bridge_webots.py`
- [ ] New controller directory `sim_container/bridges/webots/controllers/webots_rbx_bridge/webots_rbx_bridge.py`.
- [ ] New world file `sim_container/bridges/webots/worlds/rbx_rover.wbt` — same robot/geometry
      as `sim_connector_rover.wbt` (reuse via copy, don't re-author), `controller` field set
      to `"webots_rbx_bridge"` instead.
- [ ] Implement the **simple** protocol (matching `sim_bridge_node.py`'s wire shape exactly,
      not the generic sim_connector one): serves a heartbeat port (`ALIVE` reply) and a
      bridge port taking `{"linear_x":...,"angular_z":...}` in, replying bare telemetry
      `{"x","y","yaw","linear_x","angular_z"}` plus `{"type":"image",...}` frames, and
      handling `{"type":"camera_settings"}` / `{"type":"reset"}` / `{"type":"environment_option"}`.
      Port most of this from `sim_connector_bridge_webots.py`'s existing device-reading/motor
      code — the wire protocol is what's different, not the Webots API calls.
- [ ] `reset`: Webots' `Robot` node isn't a `Supervisor`, so — same documented limitation as
      the existing Webots bridge — this is a logged no-op unless the world's robot is made a
      `Supervisor`. Match `rbx_gazebo_node.py`'s `RESET_SIM` action either way (real teleport
      if feasible, otherwise an honest no-op — decide once this step is reached, don't guess now).
- [ ] `environment_option`: this world has no obstacle-course model — honest no-op, matching
      the existing Webots bridge's documented gap.

### 4. RBX driver: `rbx_webots_node.py`
- [ ] Copy `rbx_gazebo_node.py` as the starting point. Expect minimal logic changes — the
      whole point of matching the wire protocol in step 3 is that this file needs only
      renaming (class name, `PKG_NAME`, log strings) plus retuning the goto controller's
      proportional gains for Webots' physics-time stepping if it overshoots/oscillates
      (`MULTI_SIMULATOR_INTEGRATION_PLAN.md`'s Phase 2 already flagged this as a real
      possibility, not confirmed either way).
- [ ] Confirm `SCENE_CAMERA`/`ROBOT_CAMERA` two-camera convention: the existing Webots world
      has only one `Camera` device (per Phase 2's own documented gap) — either add a second
      camera to `rbx_rover.wbt`, or honestly report only one camera the way the existing
      bridge does (`SCENE_CAMERA`/`ROBOT_CAMERA` both resolving to the same feed) — decide
      once this step is reached.

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
