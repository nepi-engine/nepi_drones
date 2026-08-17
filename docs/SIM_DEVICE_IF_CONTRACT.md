# NEPI ↔ Simulator Interface Contract

## Status

This contract is **built and in production use**, not a proposal. `device_if_sim.py` +
`sim_connector_app_node.py` (`src/nepi_apps/nepi_app_sim_connector/` in the real
`nepi_engine_ws` tree) implement everything described below, and it has been verified
end-to-end against a real simulator (Gazebo) including on the real device — see
`MULTI_SIMULATOR_INTEGRATION_PLAN.md` for current per-simulator status and
`SIM_CONNECTOR_REMAINING_WORK.md` for what's actually still open.

This document is kept as the **reference for the contract itself** — what data flows
which direction, what the capability/status fields mean, and the rule for adding a new
simulator. It intentionally does not contain build history or phase-by-phase logs
anymore; those live in git history and in `MULTI_SIMULATOR_INTEGRATION_PLAN.md`.

**The one rule that matters most:** adding a new simulator should only ever mean writing
a new bridge script (speaking the wire protocol below) plus a new entry in
`sim_connector_app_params.yaml`'s robot-config list. It should never require changing
`device_if_sim.py`, `sim_connector_app_node.py`, or the RUI components. If a new
simulator genuinely can't be expressed this way, that's a real gap in the contract worth
writing up deliberately — not something to patch around per-simulator.

Real hardware, RoboRIO/FRC hardware integration, and non-ROS wire protocols (CAN,
NetworkTables, I2C/SPI, serial) are explicitly out of scope for this contract.

---

## Data flow contract

### Into NEPI (from simulator)

| Data | Mechanism |
|---|---|
| Position/orientation (NavPose) | `getNavPoseCb` → real `NPXDeviceIF` construction inside `device_if_sim.py`, publishing standard `NavPose` messages — the same shared mechanism RBX/IDX/NPX device types already use. |
| Image frames | Standard `sensor_msgs/Image`, republished from whatever the bridge sends as base64 JPEG. |
| List of available sensor topics (typed: topic + msg type) | `available_sensor_topics` — a live list of `(topic_name, msg_type)` pairs, re-derived on every status publish, never cached past construction. `has_camera`/`available_image_topics`/`active_image_topic` are filtered views over this one list, not independently tracked. |
| Capability/status report | `SimCapabilitiesQuery` service (queried once) + `SimStatus`/`SimInfo` topics (published continuously). |
| Connection/telemetry health | `bridge_connected` (bool) + `telemetry_age_sec` (float) — distinguishes "socket alive" from "telemetry actually current," since a stalled-but-connected bridge is a real failure mode a bare heartbeat can't catch. |

### Out of NEPI (to simulator)

| Command | Mechanism |
|---|---|
| Motor/wheel control ratios | `setMotorControlRatio(motor_ind, ratio)` — same pattern as RBX's manual motor control. |
| Goto/setpoint commands | `gotoPositionFunction`/`gotoPoseFunction`/`gotoLocationFunction` — a driver only implements the ones that apply to its vehicle; unset ones report `has_goto_* = False`. |
| Setup/go actions (e.g. reset-to-spawn, return home) | `setup_actions`/`go_actions` lists + index-based callbacks. |
| Image topic selection | Select from the reported `available_image_topics` list, not a blind topic-name string. |
| Camera view/rig control | `setCameraViewModeFunction` — a named mode string (e.g. `SCENE_CAMERA`/`ROBOT_CAMERA`), reported valid options via `available_camera_view_modes`. |
| Environment/obstacle control | `setEnvironmentOptionFunction` — toggles a named, simulator-reported option (e.g. `obstacle_course`). |

---

## `device_if_sim` design

Deliberately `device_if_rbx.py`'s proven shape, generalized rather than reinvented:

- **Constructor-injection contract.** A driver hands `SimDeviceIF` plain Python callback
  functions; each one's `None`-ness decides the matching `has_*` capability flag —
  computed once by default, but `apply_capability_profile()` allows re-deriving the full
  capability set when the operator switches which robot config is active (a real
  extension beyond RBX's construction-time-only model, needed because one generic app
  instance can be pointed at different simulated vehicles over its lifetime).
- **Same declarative wiring** — `CONFIGS_DICT`/`PARAMS_DICT`/`SRVS_DICT`/`PUBS_DICT`/`SUBS_DICT`
  handed to `NodeClassIF`.
- **Same two-tier status model** — an on-demand `SimInfo` identity report, a timed
  `SimStatus` operational report.
- **Same capabilities-as-a-service model** — `SimCapabilitiesQuery`, a cached, statically
  (or profile-)derived struct, not computed per-request.

### Capability/status fields beyond what RBX already has

RBX's existing flags (`has_manual_controls`, `has_goto_pose/position/location`,
`has_go_home/go_stop/set_home`, `has_battery_feedback`, etc.) all carry over unchanged.
`SimDeviceIF` adds:

| Field | Type | Purpose |
|---|---|---|
| `has_wheels` / `wheel_count` | bool / int | Ground-vehicle UI layout (how many sliders to render). |
| `has_motors` / `motor_count` | bool / int | Generalizes past wheels (e.g. a multi-rotor's per-motor test). |
| `available_sensor_topics` | `SensorTopicInfo[]` (`topic_name`, `msg_type`) | The generalized, typed topic list every other camera/sensor field is filtered from. |
| `has_camera` / `available_image_topics` / `active_image_topic` | bool / string[] / string | The camera-specific projection of the list above. |
| `has_camera_view_control` / `available_camera_view_modes` | bool / string[] | Named camera-rig view modes (see Camera configuration below). |
| `has_environment_controls` / `available_environment_options` | bool / string[] | Live, simulator-reported world/environment toggles. |
| `bridge_connected` / `telemetry_age_sec` | bool / float | Connection health, distinct from capability. |

### Capability → UI control mapping

The RUI renders purely from these flags — a driver reporting fewer capabilities produces
fewer controls, with zero RUI code changes needed per vehicle:

| Capability flag(s) | UI control |
|---|---|
| `has_manual_controls` + `wheel_count`/`motor_count` | Per-motor/wheel sliders |
| `has_goto_position` / `has_goto_pose` / `has_goto_location` | Goto input fields |
| `has_go_home` / `has_set_home` | Home / set-home buttons |
| `available_image_topics` (non-empty) | Camera selector + live image pane |
| `has_camera_view_control` + `available_camera_view_modes` | Camera view-mode selector |
| `has_environment_controls` + `available_environment_options` | Environment toggle button(s) |
| Everything `False`/empty | No control at all — not a special case, just what falls out of the flags |

---

## Two worked example systems

| Capability | Ground rover (Gazebo, verified) | Flight vehicle (drone-shaped) |
|---|---|---|
| `has_wheels` / `wheel_count` | `True` / 2 | `False` / 0 |
| `has_motors` / `motor_count` | same as wheels | `True` / 4+ |
| `has_goto_position` | `True` (local ENU) | `True` |
| `has_goto_location` (global lat/lon) | `False` — no GPS reference | `True` |
| `has_goto_pose` (attitude-only) | `False` | `True` |
| `has_camera` / cameras reported | `True` — both `scene_camera` and `robot_camera` | driver-dependent |
| `has_environment_controls` | `True` (obstacle course) | driver-dependent |

Both fit the same contract with zero changes to `device_if_sim.py` itself — only the
driver/bridge's own robot-config entry differs.

---

## Camera configuration

**List population:** capability/status-report-driven at runtime, not a static config
file — a pre-declared list can't reflect reality when a simulator's active topics change
mid-session (a second robot spawning, a camera model changing). `available_sensor_topics`
is re-scanned and re-published on every status tick.

**The two-camera convention (in production use):** every simulator bridge exposes exactly
two named cameras through that same generic mechanism:

| Camera | Purpose | Reference frame |
|---|---|---|
| `scene_camera` | Third-person view | Offset from the vehicle's body/center frame |
| `robot_camera` | Onboard/FPV view | The vehicle's own body frame |

Selecting which one streams reuses the existing `active_image_topic` selector.
`setCameraViewModeFunction`/`available_camera_view_modes` is a named-mode selector (e.g.
`SCENE_CAMERA`/`ROBOT_CAMERA`); **live pose/offset adjustment of either camera is
deliberately not given a wire shape yet** — it needs real usage patterns to clarify what's
actually needed before designing that control, not a speculative guess.

---

## Deferred / explicitly out of scope

- **Sim time / pause-step control** — not needed for the current build; revisit only if
  a real requirement for it shows up.
- **Live camera pose/offset adjustment** — see Camera configuration above.
- **Real-hardware / RoboRIO / FRC translator** — a different problem (a real device's
  native protocol, not a simulator), owned separately if it happens.
- **Robot-spec manifest file in `nepi_storage`** — auto-configuring a simulator from a
  known robot definition, rather than a hand-maintained robot-config entry.
- **Setup/install automation script** — installing a given simulator's tooling
  automatically.
