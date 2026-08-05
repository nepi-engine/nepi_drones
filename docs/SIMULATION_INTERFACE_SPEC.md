# NEPI ↔ Simulator Interface Spec & Implementation Plan

> **Note: how this document is organized.** One document, contract and build plan together
> — matching how `sim_container/ROVER_GAZEBO_BRIDGE_IMPL_PLAN.md` (renamed 2026-08-05 from
> `UNIVERSAL_SIMULATOR_IMPL_PLAN.md`; the rover Gazebo bridge) was always written: reasoning
> and phased checklist in the same place, not split across files. It's a chain of decisions,
> each one setting up the next, not a flat list:
> 1. **Data flow contract** is the foundation — it splits everything into "what comes into
>    NEPI" vs. "what goes out to the simulator," and for each item says whether it reuses
>    an existing RBX mechanism or needs something new.
> 2. **`device_if_sim` design** turns that raw material into a concrete interface, copying
>    `device_if_rbx`'s proven trick: a driver/app hands in callback functions, and whichever
>    ones are `None` vs. real functions decides the `has_*` capability flags. Capabilities
>    are decided once, at construction time, and cached — never computed dynamically.
> 3. **Capability → UI mapping** makes the payoff explicit: the RUI never needs new code
>    when a driver's capabilities change; it just renders whatever the flags say exist. This
>    is why "no controls at all, other than picking a motor" isn't a special case — it's
>    just what happens when most flags are `False`.
> 4. **Two worked examples** stress-test #2: if the same contract can honestly describe
>    both a wheeled rover and a flying drone with no per-vehicle special-casing, it's
>    general enough. If it couldn't, that would mean the design is still overfit to one
>    shape.
> 5. **Camera configuration + packaging-as-an-app** resolve the two open questions and make
>    the contract concrete enough to actually build against.
> 6. **Implementation Plan** is the phased, testable build checklist for everything above —
>    Phase/Step/Verification format, same as the rover bridge plan. Read the contract
>    sections first; the plan assumes their decisions as given.
> 7. **Deferred / future work** marks what's explicitly *not* being built this pass, so an
>    omission doesn't get mistaken for an oversight.
>
> **Decisions made 2026-08-04** (both were flagged open questions as of the previous draft):
> sensor topics beyond images are now a decided, typed contract (see the data flow table and
> [Camera configuration](#camera-configuration)); sim time / pause-step control is decided
> **out of scope for now** and moved to [Deferred / future work](#deferred--future-work).
>
> **Decisions made 2026-08-05, from a team meeting — read this before anything below, it
> changes who builds what:**
> 1. **`device_if_sim.py` (the generic contract implementation) is now owned by the
>    NEPI-core team, not this repo.** This repo's own Phase 1/2 attempt at it (built
>    2026-08-04, fully tested) is **archived, not deleted**, at
>    `sim_container/sim_old_plan/app_sim_connector/` — kept for reference in case the NEPI-core team's
>    version reuses pieces (the wire protocol ideas, the `SIM_VEHICLE_DICT` per-deployment
>    config pattern), but it is **no longer the active plan** and nothing in this repo should
>    build on it going forward. This repo's job now is the **driver-level two-camera
>    contract** below, to be built *on top of* whatever `device_if_sim.py` the NEPI-core team
>    delivers — see the rewritten [Implementation Plan](#implementation-plan) for what that
>    means for Phases 1/2 there.
> 2. **Two cameras, defined at the driver level:** `scene_camera` (third-person) and
>    `robot_camera` (onboard/FPV) — see [Camera configuration](#camera-configuration), fully
>    rewritten for this.
> 3. Until the NEPI-core team's `device_if_sim.py` lands, there isn't much for this repo to
>    build — most of the two-camera work depends on knowing what that contract actually
>    exposes. See the Implementation Plan's new checklist for exactly what's blocked vs. not.
>
> Read top to bottom in that order — later sections assume the earlier ones as given.

## Purpose & scope

This is a **requirements/interface contract with an attached build plan**, not a finished
implementation. It defines the contract between NEPI and *any* simulator (Gazebo today,
others later) so that:

- The simulation side (this repo, `nepi_drones`) knows exactly what to send and expose.
- The NEPI-core side **(a separate team — decided 2026-08-05, no longer "or a future pass in
  this same repo")** builds the generic `device_if_sim.py` implementation against this fixed
  target, without needing to understand Gazebo, ArduPilot, or any other simulator directly.
- Any simulator can connect to that implementation easily — see
  [Packaging: a NEPI App](#packaging-a-nepi-app-not-per-simulator-drivers) — without NEPI
  needing bespoke code written for each new simulator that shows up.

**Division of labor, decided 2026-08-05:** the NEPI-core team owns `device_if_sim.py` itself
going forward — this repo's own earlier attempt at it is archived at
`sim_container/sim_old_plan/app_sim_connector/` (see the top note above). This repo's ongoing scope is the
**driver-level camera contract** (two cameras — `scene_camera`/`robot_camera` — plus their
controls, see [Camera configuration](#camera-configuration)) that will be built *on top of*
whatever `device_if_sim.py` the NEPI-core team delivers, not a replacement for it.

**Explicitly out of scope for now:** real hardware, RoboRIO/FRC integration, and any
non-ROS wire protocol (CAN, NetworkTables, I2C/SPI, serial). That work is deferred to
later and owned elsewhere — see [Deferred work](#deferred--future-work) at the bottom. A
prior draft of this doc (`RBX_EXTERNAL_HARDWARE_INTERFACES.md`, removed) surveyed those
protocols in depth; that detail is preserved in git history if it's needed again, but none
of it should be treated as current direction.

Everything below is grounded in the existing, proven `device_if_rbx.py` pattern
(`RBXRobotIF`) — this doc mostly asks "what does the simulator case need that RBX
doesn't already have," rather than inventing a new architecture from scratch.

---

## Data flow contract

Two directions, defined independently of which simulator is on the other end.

### Into NEPI (from simulator)

| Data | Mechanism | New, or reuse existing? |
|---|---|---|
| Position/orientation (NavPose) | `getNavPoseCb` → `NavPose` message, published via the existing `NPXDeviceIF` (shared by RBX/IDX/NPX device types) | **Reuse as-is.** No new pose protocol needed — `device_if_rbx.py` already delegates NavPose publishing entirely to `NPXDeviceIF` whenever a driver supplies `getNavPoseCb` (`device_if_rbx.py:868-878`). `NavPose.msg` already carries both local ENU (`x_m,y_m,z_m,roll_deg,pitch_deg,yaw_deg`) and global (`latitude,longitude,altitude_m`) fields, each gated by a `has_*` bool, so it covers a wheeled rover (local only) and a drone (both) without modification. |
| Image frames | Existing `sensor_msgs/Image` topic subscription pattern | **Reuse.** Same shape as any camera-bearing RBX driver today. |
| **List of available sensor topics (typed: topic + msg type)** | New — see [Camera configuration](#camera-configuration) | **New, decided 2026-08-04.** RBX today hard-codes a single `image_source` param resolved via `nepi_sdk.find_topic()` (one topic, one match, `device_if_rbx.py:2123-2146`). A simulator can have several cameras (chase cam, onboard cam, etc.), so the interface needs a *list*, not a single string — and rather than an images-only list, it's a typed `(topic, msg_type)` list from the start, so a future sensor modality (lidar, depth, IMU) is just another entry, not a new field/contract change. |
| Capability/status report | `device_if_sim`'s own capabilities-query service + status topic | **New message type, same mechanism as RBX** (see below). |
| **Connection/telemetry health** | New status field, e.g. `telemetry_age_sec` or `bridge_connected` | **New.** The existing heartbeat-port pattern (`rbx_sim_discovery.py`) proves the socket is *alive*, but not that telemetry is *current*. A stalled-but-connected bridge (Gazebo paused, a hung callback) should be distinguishable from a healthy one. |

### Out of NEPI (to simulator)

| Command | Mechanism | New, or reuse existing? |
|---|---|---|
| Motor/wheel control ratios | `setMotorControlRatio(motor_ind, ratio)` / `MotorControl` message | **Reuse as-is** — this is exactly the manual-control pattern already proven against the Gazebo rover. |
| Goto/setpoint commands | `gotoPositionFunction`/`gotoPoseFunction`/`gotoLocationFunction` | **Reuse.** A driver only implements the ones that make sense for its vehicle (a rover has no `gotoLocation`; that's fine — it reports `has_goto_location = False`). |
| Setup actions (e.g. reset-to-spawn) | `setup_actions` list + `setSetupActionIndFunction` | **Reuse** — same mechanism as `RESET_SIM` in both existing drivers today. |
| Image topic selection | New `set_image_topic`-style command, but selecting from the *reported list* rather than typing a topic name blind | **New selector semantics on an existing mechanism** — see [Camera configuration](#camera-configuration) below. |
| **Camera view/rig control** | New — e.g. `setCameraViewModeFunction(mode_str)` | **New, but already implemented at the bridge level.** `sim_bridge_node.py` already handles a `{"type":"camera_settings","view_mode":...}` message that changes the chase-cam's follow behavior. This is distinct from *which image topic is active* — it's a virtual-camera-rig setting (follow angle/distance), not a topic selector — and wasn't previously reflected as its own capability. |
| **Environment/obstacle control** | New — e.g. `setup_actions` entry or a dedicated `setEnvironmentOptionFunction(option, enabled)` | **New, but already implemented at the bridge level.** `sim_bridge_node.py` already handles `{"type":"obstacle_course","enabled":bool}`, spawning/deleting an obstacle model in the world. There was previously no capability flag or command describing "change what's in the world" at all — only `RESET_SIM` (reset the *existing* world) was captured. |

The genuinely new things at the data-flow level are the **image-topic list + selector**,
**camera view/rig control**, **environment/obstacle control**, and **connection health
reporting**. Everything else is a direct reuse of what `device_if_rbx.py` already does.

---

## `device_if_sim` design

Rather than inventing a new pattern, `device_if_sim` should be `device_if_rbx`'s proven
shape, generalized:

- **Same constructor-injection contract.** A driver/app hands `device_if_sim` plain Python
  callback functions; each parameter being `None` vs. a real function is what decides the
  matching `has_*` capability flag — identical to `device_if_rbx.py:358-440`.
- **Same declarative wiring.** `CONFIGS_DICT`/`PARAMS_DICT`/`SRVS_DICT`/`PUBS_DICT`/`SUBS_DICT`
  handed to `NodeClassIF`, which does the actual ROS registration — no driver ever touches
  ROS topics/services directly.
- **Same two-tier status model**: an on-demand "info/identity" report (mirrors
  `DeviceRBXInfo`) plus a timed "operational status" report (mirrors `DeviceRBXStatus`,
  published at a fixed rate — RBX uses 2 Hz, `device_if_rbx.py:120`).
- **Same capabilities-as-a-service model**: a `capabilities_query` service returning a
  cached, statically-decided capability struct — not a topic, not computed per-request.
- **Error reporting and data recording are inherited, not new work.** Because
  `device_if_sim` reuses `device_if_rbx`'s status/save-data machinery, `errors_current`/
  `errors_prev`/`last_error_message` (already in `DeviceRBXStatus`) and the `SaveDataIF`
  recording hooks come along automatically. The NEPI-core team should not need to build
  these — only wire the new fields below into the same struct. **Not yet actually wired in
  the Phase 1 build below** — see that phase's scope note.

### New capability/status fields this device type needs

RBX's existing capability flags (`has_manual_controls`, `has_goto_pose`,
`has_goto_position`, `has_goto_location`, `has_go_home`, `has_go_stop`, `has_set_home`,
`has_battery_feedback`, etc. — full list in `RBXCapabilitiesQuery.srv`) all carry over
unchanged and apply equally to a simulated vehicle. `device_if_sim` adds:

| Field | Type | Purpose |
|---|---|---|
| `has_wheels` | bool | Distinguishes a ground vehicle from a flight vehicle for UI layout purposes. |
| `wheel_count` | int | How many independent wheel/track outputs exist (drives how many sliders the UI renders). |
| `has_motors` | bool | Generic motor-output capability — already effectively `has_manual_controls`, but named explicitly for clarity in a doc a non-RBX-familiar reader will consume. |
| `motor_count` | int | Same purpose as `wheel_count`, generalized past wheels (e.g. a multi-rotor's individual motor test, which already exists in the ArduPilot driver). |
| `available_sensor_topics` | `SensorTopicInfo[]` (`topic_name` + `msg_type` string pair) | **Decided 2026-08-04.** The generalized, typed list described in [Camera configuration](#camera-configuration) — every sensor topic the simulator currently exposes, not just cameras. |
| `has_camera` | bool | Whether any entry in `available_sensor_topics` has `msg_type == 'sensor_msgs/Image'`. Derived, not separately tracked. |
| `available_image_topics` | string[] | The camera-specific projection of `available_sensor_topics` (filtered to `sensor_msgs/Image` entries) — kept as its own field because the RUI's camera selector wants plain topic names, not the general typed list. |
| `active_image_topic` | string | Which of `available_image_topics` is currently selected/streaming. |
| `has_camera_view_control` | bool | Whether the camera view/rig (follow angle, distance) can be changed — not the same as switching image topics. |
| `available_camera_view_modes` | string[] | Reported list of valid `view_mode` values, mirroring the image-topic list pattern rather than requiring the app to guess valid strings. |
| `has_environment_controls` | bool | Whether the world/environment can be reconfigured live (e.g. obstacle course toggle), distinct from `RESET_SIM` which only resets the *existing* world. |
| `available_environment_options` | string[] | Reported list of environment toggles the simulator actually supports (e.g. `["obstacle_course"]`), so this generalizes past one hardcoded option. |

### Capability → UI control mapping

Same principle RBX already proves: the RUI shows/hides controls purely based on which
capability flags come back `True`, with zero RUI-side code changes needed when a driver's
capability set changes. Concretely:

| Capability flag(s) | UI control produced |
|---|---|
| `has_manual_controls` + `wheel_count`/`motor_count` | Per-motor/wheel slider(s), matching the existing RBX motor-slider UI |
| `has_goto_position` / `has_goto_pose` / `has_goto_location` | Goto input fields, exactly as ArduPilot's driver already exposes |
| `has_go_home` / `has_set_home` | Home button / set-home button |
| `available_sensor_topics` (non-`sensor_msgs/Image` entries) | Reserved for future non-camera sensor UI (lidar/depth/IMU panes) — no control exists yet, but the data is already there when one is built |
| `available_image_topics` (non-empty) | Camera selector dropdown + live image pane |
| `has_camera_view_control` + `available_camera_view_modes` | Camera view-mode selector (follow angle/distance) |
| `has_environment_controls` + `available_environment_options` | Environment toggle button(s) (e.g. obstacle course on/off) |
| Everything `False`/empty | No control at all — the CEO's "could be none, other than selecting which motor to run" case falls out naturally rather than needing special-casing |

---

## Two worked example systems

Chosen deliberately to be as different as possible, to stress-test that the contract
doesn't overfit to one vehicle shape.

| Capability | Gazebo generic rover | ArduPilot SITL drone |
|---|---|---|
| `has_wheels` / `wheel_count` | `True` / 2 (differential drive) | `False` / 0 |
| `has_motors` / `motor_count` | same as wheels in this case | `True` / 4+ (per-motor test already implemented) |
| `has_manual_controls` | `True` | `True` |
| `has_goto_position` | `True` (local ENU only) | `True` |
| `has_goto_location` (global lat/lon) | `False` — no GPS reference for a local rover | `True` |
| `has_goto_pose` (attitude-only) | `False` — not meaningful for a ground vehicle | `True` |
| NavPose fields populated | `has_position` only | `has_position` **and** `has_location`/`has_altitude` |
| `has_camera` / `available_image_topics` | `True` / one chase-cam topic today | `False` today (no camera wired up yet) |
| `available_sensor_topics` | one entry, `('...chase_cam/image_raw', 'sensor_msgs/Image')` today | empty today |
| `has_camera_view_control` | `True` (chase-cam follow mode already exists) | `False` today |
| `has_environment_controls` | `True` (obstacle course toggle already exists) | `False` today |
| `has_set_home` / `has_go_home` | `True` (reset-to-spawn) | `True` |

Both fit the same `device_if_sim` contract with no changes to the contract itself — only
the driver's constructor call differs in which arguments are real functions vs. `None`.
**Phase 1 of the Implementation Plan below uses these exact two columns as its test cases.**

---

## Camera configuration

**How the list gets populated (2026-08-04 decision, still current): capability/status-
report-driven at runtime, not a static config file.** A pre-declared config file can't
reflect reality — which cameras exist depends on what's actually spawned in the simulator at
that moment (a multi-robot world might have 0, 1, or several camera-bearing models). The
mechanism to reuse already exists in this codebase: `ai_if_detector.py` maintains
`self.available_image_topics`, refreshed on a timer via
`nepi_sdk.find_topics_by_msg('Image', ...)` — a live scan of the ROS graph for
`sensor_msgs/Image`-typed topics, filtered by an exclude-list, then reported in that node's
own status message. Generalized to `nepi_sdk.find_topics_by_msgs()` across whatever message
types a simulator's bridge declares it might expose, reporting the full typed
`available_sensor_topics` list in status; `has_camera`/`available_image_topics`/
`active_image_topic` are the `sensor_msgs/Image` filter over that one list, not a second
independent scan — this mechanism itself is unchanged by the 2026-08-05 decisions below.

**Which cameras get populated into that list — decided 2026-08-05, this is new:** every
driver defines exactly **two** cameras, not an arbitrary simulator-declared set:

| Camera | Purpose | Reference frame | Default pose | Modifiable? |
|---|---|---|---|---|
| `scene_camera` | Third-person view | The robot's center/body frame | A default offset relative to that center — e.g. **3 m up, 2 m back, angled down slightly** (placeholder numbers from the meeting, not final tuned values) | **Yes — all of it** (offset and angle), decided explicitly in the meeting. |
| `robot_camera` | Onboard/FPV view | A "main reference frame" — a canonical per-vehicle body frame still needs picking per driver (e.g. whatever `base_link`-equivalent frame that vehicle already has) | Coincident with (zero offset from) the main reference frame unless configured otherwise | **Hoped for, not yet a firm decision** — the meeting's own framing was "hopefully the camera position can be modified relative to that [frame]," weaker than `scene_camera`'s explicit "these are all modifiable." Treat as the intent, confirm before relying on it. |

Both still surface through the *same* generic `available_sensor_topics`/
`available_image_topics`/`active_image_topic` mechanism above, as two named entries — the
two-camera decision is a **driver-level convention for which cameras a driver populates that
generic list with**, not a change to `device_if_sim.py`'s own contract shape. Selecting which
one streams still reuses the existing `active_image_topic`/`set_active_image_topic`
selector unchanged.

**Controls needed (2026-08-05):** a set of controls for both cameras — at minimum,
(a) selecting which camera is active (reuses the existing selector, no new mechanism), and
(b) adjusting each camera's pose (offset + angle) live, for the ones marked modifiable above.
(b) is genuinely new: the existing `setCameraViewModeFunction`/`available_camera_view_modes`
mechanism is a single string label (e.g. `"FIRST_PERSON"`/`"THIRD_PERSON"`), not a
structured multi-axis pose — whether it gets extended, replaced, or paired with a new
pose-adjustment control is a decision to make **once the NEPI-core team's `device_if_sim.py`
is in hand and its actual camera-control surface is known**, not before. Don't guess at a
wire shape for this yet.

**ArduPilot driver implementation note (2026-08-05):** the ArduPilot driver's camera image
acquisition will subscribe to a CV2-image callback from some publisher — i.e. an
OpenCV-image-in-a-ROS-callback pattern, the same shape `rbx_ardupilot_node.py`'s existing
image handling already uses — rather than any other transport. Which of the two cameras
(or both) this applies to first is an implementation detail for whoever picks up that driver
work, not decided here.

---

## Packaging: a NEPI App, not per-simulator drivers

The goal is for any simulator to be able to connect **easily** — meaning connecting a new
simulator should require little-to-no new NEPI-side code, only a small adapter on the
simulator side. The right shape for this is a **NEPI App** (following the existing
`nepi_apps/` convention — `scripts/`, `api/`, `params/`, `msg/`/`srv/`, `rui/`), not one
bespoke RBX driver per simulator discovered through the normal hardware-discovery loop.
**Target shape only, as of 2026-08-05** — this is the app structure the NEPI-core team is
building toward for `device_if_sim.py`/`sim_connector_node.py`; this repo built a first pass
of it (now archived, see the top-of-document note) but is not the one building the real
version going forward:

```
nepi_apps/app_sim_connector/            # a NEPI App, not a driver
├── scripts/
│   └── sim_connector_node.py           # hosts ONE generic device_if_sim listener
├── api/
│   └── device_if_sim.py                # the generic contract, reused by every simulator
├── params/
│   └── sim_connector_params.yaml       # listen port, defaults
├── msg/ srv/
│   └── SimCapabilitiesQuery.srv, SimStatus.msg, SimInfo.msg, SensorTopicInfo.msg
│       # SensorTopicInfo.msg: topic_name (string) + msg_type (string) -- the
│       # 2026-08-04 typed sensor-topics decision, reused across caps/status
└── rui/
    └── SimConnector-Controls.js        # capability-flag-driven controls (sliders, camera selector, env toggles)
```

The app hosts a single, generic, well-known connection surface — a generalized version of
the JSON/TCP protocol `sim_bridge_node.py` already speaks — that any simulator's own bridge
script dials into. **Connecting a new simulator then means writing a small bridge script
on the simulator side that speaks this protocol** (exactly what `sim_bridge_node.py` does
for Gazebo today), not writing a new discovery-driven driver inside NEPI. This mirrors the
rule already proven for `device_if_rbx.py`: never touch the generic interface to support
new hardware — only a genuinely new *capability* justifies that. Here it extends one step
further: never touch NEPI at all to support a new *simulator*, only to support a new
*capability*.

Existing per-simulator RBX drivers (`rbx_ardupilot_node.py`, `rbx_sim_node.py`) can
either be absorbed into this app over time or continue operating alongside it — that
migration decision is separate from this spec and doesn't block it (see Phase 5 below).

---

## Mockup: illustrative sketch (superseded twice over — see note)

This sketch is kept for historical/review context (it's what got reviewed and approved
before Phase 1 was built). It was first superseded by this repo's own real Phase 1 build —
and as of the 2026-08-05 decision that the NEPI-core team now owns `device_if_sim.py`, that
Phase 1 build has itself been **archived** (kept for reference, not deleted — see the top
note) to `nepi_drones/sim_container/sim_old_plan/app_sim_connector/api/device_if_sim.py`. Neither is the
current source of truth for the contract going forward; read whatever `device_if_sim.py` the
NEPI-core team actually delivers once received.

```python
# api/device_if_sim.py — sketch only

class SimDeviceIF:
    def __init__(self,
                 device_info,
                 # existing RBX-style callbacks, all optional (None = unsupported)
                 getNavPoseCb=None,
                 setMotorControlRatio=None, getMotorControlRatios=None,
                 gotoPositionFunction=None, gotoPoseFunction=None, gotoLocationFunction=None,
                 goHomeFunction=None, goStopFunction=None,
                 setup_actions=None, setSetupActionIndFunction=None,
                 # new for sim
                 wheel_count=0, motor_count=0,
                 # decided 2026-08-04: typed, general -- returns a list of
                 # (topic_name, msg_type) pairs, not images-only
                 getAvailableSensorTopicsFunction=None,
                 setActiveImageTopicFunction=None,
                 setCameraViewModeFunction=None, available_camera_view_modes=None,
                 setEnvironmentOptionFunction=None, available_environment_options=None,
                 getTelemetryAgeFunction=None):
        ...
        self.caps_report.has_wheels = wheel_count > 0
        self.caps_report.has_motors = motor_count > 0
        self.caps_report.has_manual_controls = setMotorControlRatio is not None
        # has_camera / available_image_topics are filtered from the one typed
        # list, not a second independent capability
        sensor_topics = getAvailableSensorTopicsFunction() if getAvailableSensorTopicsFunction else []
        image_topics = [t for t, mtype in sensor_topics if mtype == 'sensor_msgs/Image']
        self.caps_report.has_camera = len(image_topics) > 0
        self.caps_report.has_camera_view_control = setCameraViewModeFunction is not None
        self.caps_report.has_environment_controls = setEnvironmentOptionFunction is not None
        ...
        # same NodeClassIF(configs_dict=..., params_dict=..., ...) wiring as device_if_rbx.py
```

```python
# scripts/sim_connector_node.py — sketch only
# Hosts the generic listener; any simulator's bridge script (e.g. sim_bridge_node.py)
# connects to THIS app rather than NEPI running simulator-specific code.

class SimConnectorNode:
    def __init__(self):
        ...
        self.sim_if = SimDeviceIF(
            device_info=self.device_info,
            getNavPoseCb=self.getNavPose,
            setMotorControlRatio=self.setMotorControlRatio,
            getMotorControlRatios=self.getMotorControlRatios,
            gotoPositionFunction=self.gotoPosition,
            wheel_count=2,
            getAvailableSensorTopicsFunction=self.getAvailableSensorTopics,
            setActiveImageTopicFunction=self.setActiveImageTopic,
            setCameraViewModeFunction=self.setCameraViewMode,
            setEnvironmentOptionFunction=self.setEnvironmentOption,
        )

    def getAvailableSensorTopics(self):
        # decided 2026-08-04: typed and general, not images-only. Real signature,
        # confirmed against nepi_sdk.py: find_topics_by_msgs(msg_type_list) returns
        # a (topics_list, msg_types_list) pair of PARALLEL lists, not a list of
        # tuples -- zip them into the (topic_name, msg_type) pairs device_if_sim
        # expects; the caller then filters for 'sensor_msgs/Image' to build the
        # camera list.
        topics, msg_types = nepi_sdk.find_topics_by_msgs(['Image', 'LaserScan', 'Imu'])
        return list(zip(topics, msg_types))
```

---

## Implementation Plan

Phased, testable build checklist for everything decided above — Phase/Step/Verification
format, same as `sim_container/ROVER_GAZEBO_BRIDGE_IMPL_PLAN.md` (renamed 2026-08-05; the
plan that shipped the rover Gazebo bridge). Assumes the contract sections above (including
the 2026-08-04 sensor-topics/sim-time decisions and the 2026-08-05 camera/ownership
decisions) as given.

> **Superseded 2026-08-05 — read before touching Phases 1/2 below.** Phases 1 and 2 were
> this repo's own attempt at `device_if_sim.py`/`sim_connector_node.py`, fully built and
> tested 2026-08-04. The team meeting that day decided the NEPI-core team owns that generic
> contract implementation going forward, not this repo. That work is **archived, not
> deleted**, at `sim_container/sim_old_plan/app_sim_connector/` — their own Verification sections below
> are left completely intact as an accurate historical record of what was built and tested,
> but neither phase is the active plan anymore, and nothing new should build on the archived
> copy. **The active plan starts at the new "Two-Camera Driver Contract" phase below,** which
> is itself blocked on receiving the NEPI-core team's `device_if_sim.py` — see that phase's
> own checklist for exactly what is and isn't blocked.

**Sandbox rule (unchanged from every prior phase of this project):** all new code lives in
`nepi_drones` — this repo — never directly in the `nepi_engine_ws` submodules
(`src/nepi_apps`, `src/nepi_engine`, `src/nepi_interfaces`). Those are "main things" and
are not touched by this plan. Where real ROS-level testing needs compiled message/service
Python bindings, Phase 1 below uses a disposable scratch catkin workspace on this dev VM
(not `nepi_engine_ws`, not the remote device) — see Step 1.3. Deploying to the remote
device or the real `nepi_apps` tree is future work, not part of this plan, and would only
ever be done as a testing convenience the way `rbx_ardupilot_node.py` is `scp`-deployed
today — never a direct edit in place there.

### Recommended execution order

Each phase is its own implementation pass, not one continuous run — same reasoning as the
rover bridge plan: keeps any one pass small enough to debug, and matches how every prior
piece of this project actually got built (one dated session file per phase).

1. **Phase 1** *(archived 2026-08-05 — see the banner above)* — the foundational contract:
   new msg/srv types + `device_if_sim.py` itself, verified standalone with a tiny
   test-harness node. Historical record only; superseded by the NEPI-core team's own
   implementation once delivered.
2. **Phase 2** *(archived 2026-08-05 — see the banner above)* — `sim_connector_node.py`:
   the one generic app node that hosts a `device_if_sim` instance and a generalized
   TCP/JSON bridge listener, verified with a synthetic fake-bridge script. Historical
   record only, same reason as Phase 1.
3. **Two-Camera Driver Contract** *(new, 2026-08-05 — the active plan)* — define
   `scene_camera`/`robot_camera` at the driver level per the rewritten
   [Camera configuration](#camera-configuration) section, and their controls, to build on
   top of whatever `device_if_sim.py` the NEPI-core team delivers. See its own section
   below for the concrete, current checklist.
4. **Phase 3** — real end-to-end proof against the existing rover Gazebo sim (the only
   simulator with a working bridge script today), reusing `sim_bridge_node.py`/
   `camera_rig_controller.py` rather than building new sim-side assets. Still applies once
   the two-camera work and the NEPI-core team's contract are both in hand.
5. **Phase 4** — RUI controls, capability-flag-driven; now includes the two-camera pose
   controls once their wire shape is known (see Camera configuration).
6. **Phase 5** — migration decision for `rbx_sim_node.py`/`rbx_ardupilot_node.py` — explicitly
   out of scope per this spec's own Packaging section; listed here only so it isn't forgotten.
7. **Phase 6** — regression & edge-case checklist, once Phases 3-5 above are proven.

### Phase 1: Foundational Contract (`device_if_sim.py` + new message types)

#### Objective

Produce the generic, reusable `SimDeviceIF` class and its message/service types, proven
correct in isolation — same constructor-injection / capability-flag pattern as
`device_if_rbx.py`, verified against both of the Two Worked Examples above (rover-shaped
and drone-shaped capability sets) before anything simulator-specific is built on top of it.

#### Step 1.1: Define the new message/service types

File location: `nepi_drones/sim_container/sim_old_plan/app_sim_connector/msg/` and `srv/` (new sandbox
folder — mirrors the real `nepi_apps/<app>/msg,srv/` convention; nothing here touches the
real `nepi_interfaces` package).

- `SensorTopicInfo.msg` — `string topic_name`, `string msg_type`. The 2026-08-04 typed-list
  decision; reused inside both the capabilities and status messages below.
- `SimCapabilitiesQuery.srv` — response-only (mirrors `RBXCapabilitiesQuery.srv`'s shape):
  all of RBX's existing capability fields (`has_manual_controls`, `has_goto_position`, etc.
  — reused unchanged) plus the new fields from the "New capability/status fields" table
  above (`has_wheels`, `wheel_count`, `has_motors`, `motor_count`,
  `available_sensor_topics` (`SensorTopicInfo[]`), `has_camera`, `available_image_topics`,
  `active_image_topic`, `has_camera_view_control`, `available_camera_view_modes`,
  `has_environment_controls`, `available_environment_options`).
- `SimInfo.msg` — mirrors `DeviceRBXInfo.msg`'s on-demand identity report shape.
- `SimStatus.msg` — mirrors `DeviceRBXStatus.msg`'s timed operational report shape, plus the
  new **connection/telemetry health** field from the contract above (`bridge_connected`
  bool + `telemetry_age_sec` float32).

#### Step 1.2: Implement `device_if_sim.py`

File location: `nepi_drones/sim_container/sim_old_plan/app_sim_connector/api/device_if_sim.py`.

Follow `device_if_rbx.py`'s structure exactly (confirmed by direct reading of
`nepi_drones/src/nepi_api/device_if_rbx.py`, not assumed):

- Constructor takes plain Python callback functions, all defaulting to `None`; each one's
  `None`-ness decides a `has_*` flag on `self.caps_report`, computed once at construction
  time (`device_if_rbx.py:358-440` is the exact pattern to mirror).
- `getAvailableSensorTopicsFunction` (returns a list of `(topic_name, msg_type)` tuples) is
  the one new callback shape from the 2026-08-04 decision — `has_camera`/
  `available_image_topics` are filtered from its result, not independently tracked (see the
  Mockup for the exact filter).
- Same `CONFIGS_DICT`/`PARAMS_DICT`/`SRVS_DICT`/`PUBS_DICT`/`SUBS_DICT` dicts handed to
  `NodeClassIF` (`device_if_rbx.py:452-740`) — `SRVS_DICT` registers `capabilities_query`
  (returns the cached `SimCapabilitiesQuery` response) and `device_info_query`; `PUBS_DICT`
  registers `info`/`status`/`status_str`, latched, matching RBX's own topic names so the
  RUI's existing status-handling code needs minimal changes later.
- `STATUS_UPDATE_RATE_HZ = 2` (matches RBX) driving a `statusPublishCb` timer
  (`device_if_rbx.py:757`), refreshing `available_sensor_topics` from
  `getAvailableSensorTopicsFunction()` on every publish (not cached indefinitely — a
  simulator's live topic set can change, e.g. a second robot spawning mid-session).
- `initCb`/`resetCb`/`factoryResetCb` can start as the same pass-through stubs
  `device_if_rbx.py:899-931` uses today (RBX itself doesn't persist meaningful config yet
  either) — not a gap introduced by this plan.

**Built, 2026-08-04 — real scope actually delivered:** constructor-injection + full
capability derivation (including the typed sensor-topics mechanism); full
`CONFIGS_DICT`/`PARAMS_DICT`/`SRVS_DICT`/`PUBS_DICT`/`SUBS_DICT` NodeClassIF wiring;
`capabilities_query`/`device_info_query` services; `info`/`status`/`status_str` publishers
with live re-derivation on every `statusPublishCb` tick; the two genuinely new sim-specific
commands (`setCameraViewModeFunction`, `setEnvironmentOptionFunction`) fully wired.

**Explicit, documented gap — not an oversight:** `device_if_rbx.py`'s deep blocking-wait
goto convergence logic (`setpoint_position_local_body`/`setpoint_attitude_ned`/
`setpoint_location_global_wgs84` — NED/ENU/WGS84 conversions plus an error-bound polling
loop against `getNavPoseCb`) is **not** reimplemented. This contract calls goto/setpoint
commands "Reuse as-is," meaning the intent is to reuse that exact mechanism — but
replicating it correctly needs its own dedicated verification pass, and Phase 1's own test
cases below don't exercise goto execution at all (they test capability/status derivation
only). `device_if_sim.py`'s `gotoPoseCb`/`gotoPositionCb`/`gotoLocationCb` are therefore
thin delegators (fire the injected function, toggle `process_current`/`cmd_success`
bookkeeping the same way RBX does) **without** the convergence-polling wait. Tracked as
follow-up work for Phase 2/3, not silently dropped. Same reasoning for `SaveDataIF`/
`SettingsIF`/`Transform3DIF` integration: the fields these would populate exist on
`SimStatus.msg` (contract-compliant shape), but the shared machinery itself isn't wired in
this pass.

#### Step 1.3: Build in an isolated scratch catkin workspace (testing only)

Purpose: `SRVS_DICT`/`PUBS_DICT` need real, importable compiled message/service Python
classes — `NodeClassIF` will fail immediately at construction without them. Rather than
touch `nepi_engine_ws` or the remote device to get that, build a **throwaway** catkin
workspace on this dev VM, `~/sim_connector_test_ws/`, containing:
- Symlinks (not copies) of `nepi_interfaces`, `nepi_sdk`, `nepi_api` from
  `nepi_engine_ws/src/` — read-only references, never modified in place.
- A symlink of `nepi_drones/sim_container/sim_old_plan/app_sim_connector/`.
- **Real gap found and closed, not assumed away:** this dev VM's `/opt/ros/noetic` had
  never built or installed any NEPI-specific package before this pass (it previously only
  ever needed plain ROS + `gazebo_ros` + `mavros`). `nepi_sdk.py` itself imports
  `rospy_message_converter` (missing — installed via user-level `pip3 install --user`, no
  sudo needed) and `geographic_msgs` (missing, and this VM's `sudo` requires a password this
  session doesn't have — worked around by cloning `ros-geographic-info/geographic_info` and
  `ros-geographic-info/unique_identifier` (for `uuid_msgs`, a `geographic_msgs` dependency)
  from GitHub straight into the scratch workspace's `src/` and letting `catkin_make` build
  them alongside everything else — no system package install, no `nepi_engine_ws` change).

This workspace is disposable dev-VM scratch state, exactly like the scratch catkin
workspace `sim_container/scripts/test_camera_rework.sh` already spins up a plain
`gzserver`/`roscore` pair for — not a change to any tracked repo, and never deployed to the
remote device as part of this phase.

#### Phase 1 Verification & Test Cases — all run live, 2026-08-04, all PASS

- **Test Case 1.1 (Rover-shaped capabilities) — PASS.** `test_device_if_sim_harness.py rover`
  in the scratch workspace, callback set matching the "Gazebo generic rover" column
  (wheel_count=2, `getAvailableSensorTopicsFunction` wired to the real
  `nepi_sdk.find_topics_by_msgs` scan, `has_goto_location`/`has_goto_pose` both unset). With
  a real throwaway `sensor_msgs/Image` topic (`/test/chase_cam/image_raw`) published
  externally via `rostopic pub -r 2`, `rosservice call .../capabilities_query` returned an
  exact field-by-field match to that column, including `has_camera=True`,
  `available_image_topics=['/test/chase_cam/image_raw']`, and `available_sensor_topics`
  carrying the correct typed `(topic_name, msg_type)` pair — derived live from a real scan,
  not a canned stub.
- **Test Case 1.2 (Drone-shaped capabilities) — PASS.** Same harness, `drone` profile
  (wheel_count=0, motor_count=4, all three goto functions set, no image topic present this
  time). `capabilities_query` matched the "ArduPilot SITL drone" column exactly, including
  `has_camera=False`/`available_sensor_topics=[]` derived correctly from an empty real scan.
- **Test Case 1.3 (Status timer + connection health) — PASS.** `rostopic hz .../status`
  converged to ~2.1-2.2 Hz (matches the 2 Hz `STATUS_UPDATE_RATE_HZ` target). Publishing the
  throwaway `sensor_msgs/Image` topic externally while the rover-profile harness was already
  running flipped `has_camera`/`available_sensor_topics` in the live `status` topic without
  restarting the node — the concrete proof that `statusPublishCb` re-derives the list every
  tick rather than caching it. `bridge_connected=True`/`telemetry_age_sec=0.1` (from the
  harness's own stub functions) were present and populated.
- **Test Case 1.4 (Everything-`False`/empty case) — PASS.** `empty` profile (every optional
  callback `None`). `capabilities_query` returned all-`False`/empty with no crash; a
  `publish_status` trigger and a `go_home` command (no `goHomeFunction` injected) both
  no-opped safely (logged a warning via `update_error_msg`, did not raise) — the node stayed
  alive and responsive afterward.

**Two real bugs found and fixed during this verification, not assumed away:**
1. `device_if_sim.py` had no class-level `node_if = None` default (unlike
   `device_if_rbx.py`, which has exactly this for exactly this reason). `NodeClassIF`'s own
   `configs_dict` wrapper invokes the caller's `reset_callback`/`init_callback`
   **synchronously during its own construction** (confirmed via direct traceback) — before
   `self.node_if = NodeClassIF(...)` has finished assigning — so `resetCb` → `initCb` →
   `publish_status()` raised `AttributeError: 'SimDeviceIF' object has no attribute
   'node_if'` on the very first construction attempt. Fixed by adding the same class
   attribute `device_if_rbx.py` already uses, plus one additional missing `None`-guard in
   `publishInfo`'s final `publish_pub` call that had the same latent issue.
2. The status timer was registered against `self.statusPublishCb`, but the method is
   actually named `publishStatusCb` (matching the `publish_status`/`publishStatusCb` naming
   used for the SUBS_DICT entry) — a plain typo that `AttributeError`'d on the very next
   construction attempt after fixing bug 1. Fixed by correcting the timer registration to
   reference the real method name.

**One real, non-obvious `nepi_sdk` prerequisite discovered, not a bug in this code:**
`nepi_sdk.get_base_namespace()` (called from `MsgIF.__init__`, i.e. before anything else in
`SimDeviceIF` construction runs) busy-waits in a **`while` loop with no timeout at all** for
a currently-registered ROS node whose full name (a) contains the substring "nepi" and (b)
has at least 3 `/`-separated segments (e.g. `/nepi/device1/<node>`) — confirmed by direct
line-by-line bisection of `MsgIF.__init__` this session, isolating the hang to this exact
call. Any `nepi_sdk`-based node — not just this one — will hang forever with **no error
message at all** if run outside a properly `/nepi/<device>/...`-namespaced environment. Not
a defect in `device_if_sim.py`; documented here (and in
`test_device_if_sim_harness.py`'s own usage comment) so the next person testing any
`nepi_sdk`/`nepi_api`-based code standalone doesn't lose an hour rediscovering it. Fixed for
testing purposes by running the harness with `ROS_NAMESPACE=/nepi/device1` set.

### Phase 2: `sim_connector_node.py` + generalized bridge protocol

#### Objective

One generic app node hosting a single `SimDeviceIF` instance and a TCP/JSON listener that
any simulator's own bridge script can dial into — the piece that makes "connect a new
simulator" mean "write a small bridge script," not "write new NEPI code" (per the Packaging
section above).

#### Step 2.1: Define the generalized wire protocol

**Built, 2026-08-04.** Extends the shapes `sim_bridge_node.py` already proves work
(telemetry, `camera_settings`, `image`) with the new types this plan adds — same
newline-delimited-JSON, dispatch-by-`"type"`-key-presence convention `sim_bridge_node.py`
already uses (bare/no-`"type"` lines are telemetry, matching the existing precedent).
Full detail and rationale in `sim_connector_node.py`'s own module docstring; summary here:

In (simulator → NEPI):
- Bare telemetry (no `"type"` key), **generalized** past the rover-only
  `{"x","y","yaw",...}` shape to the full NavPose contract — every field optional, gated the
  same way `NavPose.msg` itself is (`x_m`/`y_m`/`z_m` ⇒ `has_position`,
  `roll_deg`/`pitch_deg`/`yaw_deg` ⇒ `has_orientation`, `latitude`/`longitude` ⇒
  `has_location`, `altitude_m` ⇒ `has_altitude`) — one shape fits both worked examples with
  no per-vehicle special-casing.
- `{"type":"sensor_topics","topics":[{"topic_name":...,"msg_type":...},...]}` — the
  simulator-side bridge announces its current live topic list; fed straight into
  `getAvailableSensorTopicsFunction`'s return value.
- `{"type":"environment_options","options":[...]}` — same idea for
  `available_environment_options`, generalizing the one hardcoded `obstacle_course` toggle.
- `{"type":"image","topic_name":...,"data":"<base64 jpeg>","stamp":...}` — existing shape,
  extended with `topic_name` (defaults to the active image topic if omitted) so multiple
  announced cameras are distinguishable; only the currently-*active* topic's frames are
  actually decoded/republished.

Out (NEPI → simulator), one shape per `SimDeviceIF` callback: `{"type":"motor_control",...}`,
`{"type":"goto_position"/"goto_pose"/"goto_location",...}` (field names matching
`GotoPosition.msg`/`GotoPose.msg`/`GotoLocation.msg` 1:1), `{"type":"go_home"}`/`{"type":"go_stop"}`,
`{"type":"setup_action"/"go_action","action":"<string>"}` (generalizes `RESET_SIM`-style named
actions), `{"type":"camera_settings","view_mode":...}` (reused as-is),
`{"type":"set_active_image_topic","topic_name":...}`, and
`{"type":"environment_option","option":...,"enabled":bool}` (generalizes the old hardcoded
`{"type":"obstacle_course","enabled":bool}` into a named option).

#### Step 2.2: Implement `sim_connector_node.py`

File location: `nepi_drones/sim_container/sim_old_plan/app_sim_connector/scripts/sim_connector_node.py`.
Owns the TCP server thread (same `settimeout(None)`-after-`init_node()` care documented in
`src/nepi_drivers/CLAUDE.md` and already applied in `sim_bridge_node.py`/
`camera_rig_controller_ardupilot.py`), and wires received lines into `SimDeviceIF`'s
callback contract — this node is the *only* place that understands the wire protocol;
`device_if_sim.py` itself stays protocol-agnostic, exactly as `device_if_rbx.py` never
knows about mavros or TCP bridges either.

**Built, 2026-08-04.** This app owns the listen socket (single active client, same model
`sim_bridge_node.py` already proves for this project's sim assets) — the *reverse*
connection direction from `rbx_sim_node.py`/`sim_bridge_node.py`'s existing client/server
relationship, per this section's own "any simulator's own bridge script dials into" framing:
this app is the one stable, well-known connection surface, not a per-simulator driver
reaching out. Factory listen port `9030` (next free port clear of `sim_container`'s existing
902x/576x allocations — see `sim_bridge_node.py`'s own port-allocation comment), configurable
per deployment via `SIM_VEHICLE_DICT` (Step 2.3 below).

#### Step 2.3: `params/app_sim_connector_params.yaml`

**Built, 2026-08-04 — real scope actually delivered, including one design decision beyond
what this step originally described:**

**Capability-timing resolution (a real design decision, not an oversight).**
`device_if_sim.py`'s contract — like `device_if_rbx.py`'s — decides capabilities **once at
construction**, cached, never recomputed (this is what makes the RUI's capability-flag-driven
rendering work at all). But a genuinely generic connector app can't know a specific
simulator's wheel/motor counts or which goto functions make sense until *after* a bridge
connects — by which point the Python process, and `SimDeviceIF`, are already constructed.
Rather than fight the "decided once" principle, the fields the contract table already calls
genuinely dynamic (`available_sensor_topics`, `available_environment_options`) are the only
ones live-refreshed from the bridge; everything else (`wheel_count`, `motor_count`, which
goto/setup/go functions exist, `available_camera_view_modes`) is a **per-deployment config
decision**, read once at startup from a new `SIM_VEHICLE_DICT` block in
`app_sim_connector_params.yaml` — the same shape as `rbx_sim_node.py`/`rbx_ardupilot_node.py`
each hardcoding their own vehicle's capabilities as class constants, except configurable per
instance since this app's code has to stay vehicle-agnostic. Factory defaults are the same
capability-empty profile Phase 1's registration test already verified safe (Test Case 1.4) —
an operator connecting a specific simulator edits this block to match that simulator's real
capabilities. `available_environment_options` is a partial exception: it's dynamic (bridge-
announced) but `device_if_sim.py`'s Phase 1 contract has no live-refresh callback for it
(unlike `available_sensor_topics`) — resolved by having `sim_connector_node.py` mutate
`self.sim_if.caps_report.available_environment_options` directly on each `environment_options`
line, without changing `device_if_sim.py`'s public contract.

**Known gap carried forward, not introduced here:** `device_if_sim.py` accepts `getNavPoseCb`
at construction and stores it, but — confirmed by direct reading, not assumed — never actually
calls it or wires it to an `NPXDeviceIF` instance anywhere in the Phase 1 file. NavPose
publishing is therefore not yet functional through `SimDeviceIF` at all, in either phase.
`sim_connector_node.py` populates `self.navpose_dict` correctly from bridge telemetry
regardless (ready for whenever this gets wired), but this is flagged here explicitly so it
isn't mistaken for new Phase 2 scope, or missed before Phase 3's real end-to-end proof (which
will need NavPose actually flowing to be meaningful).

#### Phase 2 Verification & Test Cases — all run live, 2026-08-04, all PASS

Both test cases were run against the real deployed app on the physical device (rsync +
scoped `catkin build app_sim_connector --profile=release`, same safe, isolated deploy pattern
already established for the Phase 1 registration test — not the scratch catkin workspace,
since this is protocol-level testing of the running node itself, not new message-type
compilation), using `test_synthetic_bridge.py` (new, throwaway, in
`app_sim_connector/scripts/`) as the fake simulator-side bridge, run as a plain client
process dialing into the app's own listen port — the reverse connection direction from
`sim_bridge_node.py` (a real server), per this app's Packaging-section role as the one stable,
well-known connection surface any simulator's bridge dials into.

- **Test Case 2.1 (Synthetic bridge, no real simulator) — PASS.** `test_synthetic_bridge.py
  --once` sent one `sensor_topics` line (two entries, one image + one lidar),  one
  `environment_options` line (`["obstacle_course","night_mode"]`), and one bare (no `"type"`
  key) telemetry line carrying both local ENU and global fields at once. `rosservice call
  .../capabilities_query` matched exactly: `available_sensor_topics` carried both typed
  entries verbatim, `has_camera=True`/`available_image_topics=['/synthetic/chase_cam/image_raw']`
  correctly filtered from the typed list (not the lidar entry), and
  `available_environment_options=['obstacle_course','night_mode']` — proving the direct
  `caps_report` mutation approach above actually works live, not just in theory.
- **Test Case 2.2 (Bridge disconnect/reconnect) — PASS.** With the synthetic script left
  running (pushing telemetry every 0.2s): `status.bridge_connected=True`,
  `telemetry_age_sec≈0.13`. Killed the script: within 3s, `bridge_connected=False`,
  `telemetry_age_sec` had grown to `≈3.9` (still counting up from the last real telemetry
  line, not reset) — the concrete proof of the "stalled-but-connected distinguishable from
  disconnected" design intent from the data flow table. Restarted the script: within 3s,
  `bridge_connected=True` again and `telemetry_age_sec` dropped back to `≈0.13`.

No new bugs found in this pass (Phase 1's two bugs are the only ones found across both phases
so far). Cleanly disabled afterward via the same `apps_mgr` `update_state` mechanism as Phase
1; confirmed via `ps aux` that the device's four independently-running apps
(`fake_gps`/`image_viewer`/`pan_tilt_auto`/`nav_sim`) kept their exact original PIDs
throughout, undisturbed.

### Two-Camera Driver Contract (new, 2026-08-05 — the active plan)

#### Objective

Define `scene_camera`/`robot_camera` at the driver level (per the rewritten
[Camera configuration](#camera-configuration) section above) and their controls, ready to
build on top of whatever `device_if_sim.py` the NEPI-core team delivers. This phase is
explicitly **not** about building a generic contract implementation — that's the NEPI-core
team's scope now (see the top-of-document decision) — it's about the driver-side camera
definitions and controls that plug into it.

#### Checklist — what's blocked vs. not

- [ ] **Blocked on the NEPI-core team's `device_if_sim.py`:** the pose-adjustment control
      shape for (b) in Camera configuration's "Controls needed" — can't finalize a wire
      shape without knowing what camera-control surface the delivered contract actually
      exposes (extends `setCameraViewModeFunction`? a new callback? something else?).
- [ ] **Blocked on the same delivery:** integrating/testing anything against a real
      `device_if_sim.py` instance at all — there's currently none to build against (the
      archived one in `sim_container/sim_old_plan/` is explicitly not to be used for this).
- [ ] **Blocked, lower priority:** deciding what (if anything) from
      `sim_container/sim_old_plan/app_sim_connector/` is worth carrying over once the real contract
      arrives (the wire-protocol shapes, the `SIM_VEHICLE_DICT` per-deployment config
      pattern) vs. fully superseded — worth a look once there's something to compare it
      against, not before.
- [ ] **Not blocked, could be drafted independently if useful:** picking each vehicle's
      "main reference frame" for `robot_camera` (e.g. ArduPilot's existing body frame) — a
      per-driver decision that doesn't depend on `device_if_sim.py`'s internals. Not started
      this pass; noted here so it isn't missed once work resumes.
- [ ] **Not blocked:** the ArduPilot CV2-image-callback wiring note from Camera
      configuration is an implementation detail for that specific driver, independent of the
      generic contract — also not started this pass.

#### Verification & Test Cases

Not yet defined — depends on the NEPI-core team's `device_if_sim.py` and its actual camera
control surface. Draft once that's in hand, following the same Phase/Step/Verification
rigor as every prior phase in this document (real `rosservice`/`rostopic` checks, not a
"should work" sign-off).

### Phase 3: Real end-to-end proof (existing rover Gazebo sim)

#### Objective

Prove the generic app actually works against a real simulator, not just synthetic test
lines — using the rover sim because it's the only one with a working, proven bridge script
today (`sim_bridge_node.py` + `camera_rig_controller.py`).

#### Step 3.1: Point `sim_connector_node.py` at the real rover bridge

Either extend `sim_bridge_node.py` to also emit the new `sensor_topics`/
`environment_options` lines from Step 2.1, or run `sim_connector_node.py` alongside it as a
second listener during this proof — whichever is less invasive to the already-verified
rover bridge is the one to pick; decide by inspection once Phase 2 code exists, not now.

#### Phase 3 Verification & Test Cases

- **Test Case 3.1 (Real capabilities from a real sim):** with `sim_rover_gazebo` running,
  confirm `capabilities_query` reports `has_wheels=True`/`wheel_count=2`,
  `has_camera=True` with the real `camera_rig` image topic inside `available_image_topics`,
  and `has_environment_controls=True` with `obstacle_course` inside
  `available_environment_options` — all read from the live sim, not hardcoded.
- **Test Case 3.2 (Live telemetry + image, same rigor as every prior phase):** subscribe to
  the new app's `status` and image topics while driving the rover with `move 5x` (see
  `testcommands`); confirm position changes match `/gazebo/model_states` and image frame
  hashes change during motion — the same hash/pose verification method used for every prior
  camera-rig phase, not a weaker "should work" check.

### Phase 4: RUI controls

`SimConnector-Controls.js`, rendering purely from capability flags per the Capability → UI
mapping table above — deferred to a session with RUI/JS context loaded
(`src/nepi_rui/CLAUDE.md`), not started in this pass.

### Phase 5: Migration decision (`rbx_sim_node.py` / `rbx_ardupilot_node.py`)

Explicitly out of scope per this spec's own Packaging section — noted here only so it isn't
mistaken for an oversight later. Not blocking Phases 1-4.

### Phase 6: Regression & Edge Case Checklist

| Scenario / edge case | Cause / mechanism | Solution / safeguard |
|---|---|---|
| `available_sensor_topics` changes size between two status publishes | A second robot spawns, or a camera topic disappears mid-session | `statusPublishCb` re-derives from a fresh `getAvailableSensorTopicsFunction()` call every publish (Step 1.2) — never caches past construction |
| Bridge connected but simulator paused/hung | The heartbeat-port pattern proves the socket is alive, not that telemetry is current | `bridge_connected` + `telemetry_age_sec` are two different signals precisely so this is distinguishable (Test Case 2.2) |
| A capability flag left `True` with an empty backing list | E.g. `has_camera=True` but `available_image_topics` empty (bridge announced a camera, then it vanished) | `has_camera` must be *derived* from the live list every publish, never set once and cached — same reasoning as the first row |
| New msg/srv types break `nepi_interfaces` conventions | Field naming drifts from `DeviceRBXStatus.msg`/`RBXCapabilitiesQuery.srv`'s existing plain-snake_case style | Confirmed by direct reading of both files — no `_str`/`_list` suffix invented where the RBX precedent doesn't use one |
| Scratch test workspace accidentally becomes a dependency | Someone points real code at `~/sim_connector_test_ws/` instead of `nepi_drones` | It is explicitly disposable dev-VM scratch state (Step 1.3) — the authoritative source is always `nepi_drones/sim_container/sim_old_plan/app_sim_connector/` |

### Summary Checklist for Implementer

- [x] **Contract decisions (2026-08-04):** sensor topics beyond images decided as a typed
      `SensorTopicInfo[]` list; sim time / pause-step control decided out of scope, moved to
      Deferred work.
- [x] **Phase 1 (2026-08-04, archived 2026-08-05):** `SensorTopicInfo.msg`/
      `SimCapabilitiesQuery.srv`/`SimInfo.msg`/`SimStatus.msg` + `device_if_sim.py`, verified
      against both worked examples and the empty/`False` case in an isolated scratch catkin
      workspace (`~/sim_connector_test_ws`, isolated ROS master on port 11411 to avoid
      interference from the still-running ArduPilot sim on the default port). Two real bugs
      found and fixed (missing `node_if` class default, a method-name typo in the status
      timer); see Phase 1's own Verification section above for full detail. **Superseded**
      2026-08-05 — the NEPI-core team owns `device_if_sim.py` now; this build is archived at
      `sim_container/sim_old_plan/app_sim_connector/`, historical record only.
- [x] **Phase 2 (2026-08-04, archived 2026-08-05):** `sim_connector_node.py` + generalized
      wire protocol (telemetry/sensor_topics/environment_options/image in;
      motor_control/goto_*/go_home/go_stop/setup_action/go_action/camera_settings/
      set_active_image_topic/environment_option out), verified against
      `test_synthetic_bridge.py` (no real simulator yet) on the real deployed app — both test
      cases pass, no new bugs found. Introduced the `SIM_VEHICLE_DICT` per-deployment
      capability-config resolution (see Step 2.3); flagged a known, not-yet-fixed
      `getNavPoseCb`-never-wired gap in `device_if_sim.py` carried over from Phase 1.
      **Superseded** 2026-08-05, same reason as Phase 1 — archived, historical record only.
- [ ] **Two-Camera Driver Contract (new, 2026-08-05):** `scene_camera`/`robot_camera`
      definitions + controls — see that phase's own checklist above for exactly what's
      blocked (most of it, pending the NEPI-core team's `device_if_sim.py`) vs. not (picking
      each vehicle's main reference frame, the ArduPilot CV2-callback wiring).
- [ ] **Phase 3:** real end-to-end proof against the existing rover Gazebo sim, same
      hash/pose verification rigor as every prior camera-rig phase. Depends on the
      Two-Camera Driver Contract phase and the received `device_if_sim.py`.
- [ ] **Phase 4:** RUI controls — deferred to a session with RUI/JS context; now includes
      the two-camera pose controls once their wire shape is known.
- [ ] **Phase 5:** `rbx_sim_node.py`/`rbx_ardupilot_node.py` migration decision — deferred,
      not blocking.
- [ ] **Phase 6:** full regression/edge-case table validated once Phases 3-5 above are proven.

Each phase should be verified before starting the next — see "Recommended execution order"
above.

### Reference: Concepts and Conventions

Background detail supporting the phases above — read on demand rather than up front.

#### Why a scratch catkin workspace instead of `nepi_engine_ws` or the remote device

Every prior phase of this project drew a hard line between sandbox (`nepi_drones`) and
"main things" (the real `nepi_engine_ws` submodules and the remote device's deployed
`/opt/nepi/...` tree). Plain Python driver scripts could always be tested by `scp`-deploying
a single file to the remote device's already-`catkin`-built environment (see every
`rbx_ardupilot_node.py` deploy this project has done). Brand-new `.msg`/`.srv` files are
different — they need `catkin`'s message-generation step to become importable Python
classes at all, and running that inside `nepi_engine_ws` or on the remote device would mean
adding untracked package folders inside repos/deployments the user explicitly asked not to
be touched except as a deploy target. A disposable scratch workspace on this dev VM gets a
real, working `rosservice`/`rostopic`-level test without that risk.

#### NEPI's existing `nepi_apps/` layout convention

Every app under `nepi_apps/` follows `scripts/`, `api/`, `params/`, `msg/`/`srv/`, `rui/` —
see the top-level `CLAUDE.md`'s "ROS Package Structure" section. `app_sim_connector` follows
this exactly; nothing about it is a new convention.

#### Testing & acceptance milestones

1. **Isolated unit-level proof (Phase 1):** `SimDeviceIF` alone, two worked-example configs,
   verified via real `rosservice call`/`rostopic echo` in the scratch workspace.
2. **Protocol-level proof (Phase 2):** the generic app's wire protocol, verified against a
   synthetic bridge script, no real simulator required.
3. **Real-simulator proof (Phase 3):** the same app, unmodified, driven by the existing
   rover Gazebo sim's real bridge — the actual test of "a new simulator connects with no
   NEPI-side code changes," since the rover bridge predates this app entirely.
4. **RUI integration (Phase 4):** capability-flag-driven controls render/hide correctly as
   flags change, with zero RUI code paths hardcoded to a specific simulator.

---

## Deferred / future work

Not part of this pass — noted so they aren't forgotten, not specified in detail:

- **Sim time vs. wall clock / pause-step control.** Gazebo can run faster/slower than real
  time or be paused entirely. There's currently no way for NEPI to query or control sim
  time — relevant if logged data ever needs to correlate to simulated rather than
  wall-clock time, or if a debugging step-by-step mode becomes useful. **Decided 2026-08-04:
  not needed for the current build.** NEPI does not need to control sim time right now;
  revisit if either of the above becomes a real requirement.
- **Real-hardware / RoboRIO translator.** Someone else's scope for later; when it happens,
  it plugs into the same `device_if_sim`/`device_if_rbx` contract as any other driver, just
  speaking NetworkTables/CAN instead of a simulator's native topics.
- **Robot-spec manifest file in `nepi_storage`.** A file describing a robot's capabilities
  so a simulator can auto-configure itself against a known robot definition, rather than
  the driver hardcoding capability counts.
- **Setup/install automation script.** Installs Gazebo and the correct supporting tooling
  automatically, to reduce new-environment setup friction.
