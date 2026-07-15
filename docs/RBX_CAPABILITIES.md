# NEPI RBX Capabilities Reference

A guide to the NEPI **RBX** (Robotic Base X) control interface, written for developers
building a client on top of it (for example a drone simulator app). It walks through
every capability the interface exposes, how a client discovers and drives them, and how
to add new ones on the driver side.

Everything here is grounded in the actual implementation:

- Interface class: `nepi_engine_ws/src/nepi_engine/nepi_api/src/nepi_api/device_if_rbx.py` (`RBXRobotIF`)
- Reference driver: `src/nepi_drivers/rbx_drivers/rbx_ardupilot_node.py`
- Messages / services: `nepi_engine_ws/src/nepi_interfaces/`
- SDK helpers: `nepi_engine_ws/src/nepi_engine/nepi_sdk/src/nepi_sdk/nepi_rbx.py`
- Web UI (a working client example): `nepi_engine_ws/src/nepi_rui/.../NepiDeviceRBX*.js`

---

## 1. What RBX is

RBX is NEPI's abstraction layer for any *robotic base* that can be commanded and can
report its state: multirotors, rovers, boats, ROVs, and so on. A concrete driver
(for example the ArduPilot/MAVLink node) wraps a real or simulated vehicle and presents
it to the rest of NEPI through a single, uniform ROS interface, `RBXRobotIF`.

The key design idea: **a driver advertises only the capabilities it actually supports.**
The interface is fixed and complete, but each driver "lights up" a subset of it by
passing (or not passing) implementation functions into the `RBXRobotIF` constructor.
A client is expected to *ask* what is supported (via the capabilities query) and adapt,
rather than assume everything is available.

This makes RBX an ideal target for a simulator: your sim driver can start by supporting
a handful of capabilities and grow over time, and any RBX client (including the NEPI web
UI) will present exactly the controls your sim declares.

---

## 2. Where RBX lives (namespaces and discovery)

Given a driver node running at namespace `<node_ns>` (for example
`/nepi/device/ardupilot`), the interface publishes everything under an `rbx`
sub-namespace:

```
<node_ns>/rbx/
```

A few subscriptions live elsewhere (noted below), but for a client the rule of thumb is:
**all RBX topics and services are under `<node_ns>/rbx/`.**

Discovery flow for a client:

1. Find RBX device nodes. Each reports `type = "RBX"` in its device-info response.
2. Call the capabilities query service to learn what the device supports.
3. Subscribe to `status` and `info` for live feedback.
4. Publish to the relevant command topics, honoring the handshake in section 7.

---

## 3. Quick start: the client interaction loop

For a sim app, the minimal working loop looks like this:

1. **Query capabilities** once at connect time (`rbx/capabilities_query`). Cache the
   `has_*` flags, the axis-control flags, and the `state_options` / `mode_options` /
   `setup_action_options` / `go_action_options` lists.
2. **Subscribe to `rbx/status`** (`DeviceRBXStatus`). This is your primary feedback
   channel: `ready`, `process_current`, `cmd_success`, `errors_current`, battery, and the
   manual/autonomous ready flags.
3. **Subscribe to `rbx/info`** (`DeviceRBXInfo`) for slower-changing config: current
   state index, mode index, home location, command timeout, error bounds.
4. **Send a command** only when `status.ready == True`. Commands are indices into the
   enumerated option lists (state/mode/actions) or typed setpoint messages
   (goto pose/position/location).
5. **Wait for completion**: `ready` goes `False` while the action runs, then back to
   `True`. Read `cmd_success` and `errors_prev` to judge the result.

That handshake is the single most important contract for a client. Section 7 covers it
in full.

---

## 4. Capability model at a glance

A driver enables each capability by supplying the matching constructor argument to
`RBXRobotIF`. If the argument is `None`, the capability flag is reported `False` and the
NEPI UI hides that control. If it is a function, the flag is `True`.

| Capability | Enabled by constructor arg | Reported flag (`capabilities_query`) | Client command topic |
|---|---|---|---|
| Battery feedback | `getBatteryPercentFunction` | `has_battery_feedback` | (read `status.battery`) |
| Manual motor control | `setMotorControlRatio` (+ `getMotorControlRatios`, `manualControlsReadyFunction`) | `has_manual_controls` | `rbx/set_motor_control` |
| Autonomous control | `autonomousControlsReadyFunction` | `has_autonomous_controls` | (gates the goto commands) |
| Set home | `setHomeFunction` (+ `getHomeFunction`) | `has_set_home` | `rbx/set_home`, `rbx/set_home_current` |
| Go home | `goHomeFunction` | `has_go_home` | `rbx/go_home` |
| Stop / hold | `goStopFunction` | `has_go_stop` | `rbx/go_stop` |
| Goto attitude (pose) | `gotoPoseFunction` | `has_goto_pose` | `rbx/goto_pose` |
| Goto body position | `gotoPositionFunction` | `has_goto_position` | `rbx/goto_position` |
| Goto global location | `gotoLocationFunction` | `has_goto_location` | `rbx/goto_location` |
| Per-axis support | `axisControls` (`AxisControls`) | `control_support` (x,y,z,roll,pitch,yaw) | (informs which axes are meaningful) |
| States | `states` + get/set index funcs | `state_options[]` | `rbx/set_state` |
| Modes | `modes` + get/set index funcs | `mode_options[]` | `rbx/set_mode` |
| Setup actions | `setup_actions` + set-index func | `setup_action_options[]` | `rbx/setup_action` |
| Go actions | `go_actions` + set-index func | `go_action_options[]` | `rbx/go_action` |

`control_support` (an `AxisControls` message) tells a client which of the 6 DOF axes the
vehicle can actually be commanded on. Commanded values for unsupported axes should be
treated as ignored.

---

## 5. Feedback interface (what a client reads)

### 5.1 Capabilities query (service) - `rbx/capabilities_query`

Service type `RBXCapabilitiesQuery`. This is the first thing a client should call.
Response fields:

```
string device_name, device_path, device_node_name
AxisControls control_support        # x,y,z,roll,pitch,yaw booleans
bool has_battery_feedback
bool has_manual_controls
bool has_autonomous_controls
bool has_set_home
bool has_go_home
bool has_go_stop
bool has_goto_pose
bool has_goto_position
bool has_goto_location
string[] state_options              # enumerated names, index-addressable
string[] mode_options
string[] setup_action_options
string[] go_action_options
string[] data_products              # e.g. ['image']
```

### 5.2 Device info query (service) - `rbx/device_info_query`

Service type `DeviceInfoQuery`. Returns identity fields plus `type = "RBX"`, the node
name and namespace. Use it to confirm a node is an RBX device and to display identity.

### 5.3 Info topic - `rbx/info` (`DeviceRBXInfo`, latched)

Slower-changing configuration and status. Key fields:

| Field | Meaning |
|---|---|
| `connected` | Driver has finished init and is talking to the vehicle |
| `standby` | Vehicle in standby |
| `state` | **Index** into `state_options` (current state) |
| `mode` | **Index** into `mode_options` (current mode) |
| `error_bounds` | Current goto tolerance (`ErrorBounds`) |
| `cmd_timeout` | Seconds before a goto/action is abandoned |
| `image_source` | Image topic overlaid with status; blank means black background |
| `image_status_overlay` | Whether status text is drawn over the image |
| `home_lat` / `home_long` / `home_alt` / `home_depth` | Current home location |

### 5.4 Status topic - `rbx/status` (`DeviceRBXStatus`, latched, ~2 Hz)

The primary live feedback channel. Key fields:

| Field | Meaning |
|---|---|
| `ready` | **True = idle/accepting commands; False = busy running an action.** The core gate. |
| `process_current` | Human-readable name of the running action, or `"None"` |
| `process_last` | Name of the previous action |
| `cmd_success` | Whether the last completed action met its error bounds before timeout |
| `battery` | Charge 0.0-1.0, or `-999` if `has_battery_feedback` is False |
| `errors_current` | Live setpoint error (`GotoErrors`) during an active goto |
| `errors_prev` | Final error of the last goto |
| `manual_control_mode_ready` | Manual (motor) control is available right now |
| `autonomous_control_mode_ready` | Autonomous (goto) control is available right now |
| `current_motor_control_settings` | `MotorControl[]`, current per-motor speed ratios |
| `last_cmd_string` | The last command as a reusable script line (see `nepi_rbx.py` automation) |
| `last_error_message` | Most recent error string (rejections, failures) |
| `navpose_frame_transform` | Transform from device native frame to `nepi_frame` |

### 5.5 Status string topic - `rbx/status_str` (`String`)

A pre-formatted text rendering of status, convenient for logging or overlays.

---

## 6. Command interface (what a client publishes)

All command topics are under `<node_ns>/rbx/`. Grouped by function:

### Configuration (non-blocking, apply immediately)

| Topic | Msg | Purpose |
|---|---|---|
| `set_goto_error_bounds` | `ErrorBounds` | Set goto tolerance: `max_distance_error_m`, `max_rotation_error_deg`, `min_stabilize_time_s` |
| `set_goto_timeout` | `UInt32` | Seconds before a goto/action is abandoned |
| `set_image_topic` | `String` | Choose the image the status overlay draws on |
| `enable_image_overlay` | `Bool` | Toggle the status text overlay |
| `set_process_name` | `String` | Set `process_current` (label an externally driven process) |
| `set_navpose_frame` | `String` | Select which navpose frame drives control math |
| `publish_status` | `Empty` | Force an immediate `status` publish |
| `publish_info` | `Empty` | Force an immediate `info` publish |

### State and mode (enumerated, by index)

| Topic | Msg | Purpose |
|---|---|---|
| `set_state` | `Int32` | Index into `state_options` (e.g. DISARM / ARM) |
| `set_mode` | `Int32` | Index into `mode_options` (e.g. STABILIZE / GUIDED / RTL) |

### Actions (enumerated, by index; blocking)

| Topic | Msg | Purpose |
|---|---|---|
| `setup_action` | `Int32` | Index into `setup_action_options` (e.g. TAKEOFF / LAUNCH). Runs to completion. |
| `go_action` | `Int32` | Index into `go_action_options`. Runs to completion. |

### Home

| Topic | Msg | Purpose |
|---|---|---|
| `set_home` | `GeoPoint` | Set home to an explicit lat/long/alt |
| `set_home_current` | `GotoLocation` | Set home to the current location |
| `go_home` | `Empty` | Return-to-home (blocking) |

### Autonomous goto (blocking; require autonomous control ready)

| Topic | Msg | Purpose |
|---|---|---|
| `goto_pose` | `GotoPose` | Command attitude: `roll_deg`, `pitch_deg`, `yaw_deg` |
| `goto_position` | `GotoPosition` | Command body-relative move: `x_meters` (fwd), `y_meters` (left), `z_meters` (up), `yaw_deg` |
| `goto_location` | `GotoLocation` | Command global move: `lat`, `long`, `altitude_meters`, `yaw_deg` |
| `go_stop` | `Empty` | Stop / hold position |

### Manual motor control (require manual control ready)

| Topic | Msg | Purpose |
|---|---|---|
| `set_motor_control` | `MotorControl` | Set one motor: `motor_ind`, `speed_ratio` (0.0 off - 1.0 max) |

---

## 7. The command handshake contract

This is the contract every client (including a sim) must follow. It applies to all
*blocking* commands: setup/go actions, go home, go stop, and the three goto commands.

**Before sending:** check `status.ready == True`. If a blocking command arrives while
`ready` is `False`, the interface **rejects it** and writes `"Another Command Process is
Active"` to `last_error_message`. It does not queue.

**While running:** the interface sets, in order:

1. `process_current` = the action name (for example `"GoTo Location"`)
2. `ready` = `False`
3. `cmd_success` = `False`
4. `errors_current` is reset, then updated live as the vehicle converges

**On completion:** the driver function returns (it blocks until the vehicle reaches the
setpoint within the error bounds, or until `cmd_timeout` elapses). The interface then
sets:

1. `process_last` = the action name, `process_current` = `"None"`
2. `cmd_success` = the driver's result (met error bounds before timeout?)
3. `ready` = `True`
4. `errors_prev` = the final error

So a client's per-command sequence is:

```
wait for status.ready == True
publish command
wait for status.ready to go False        # command accepted, running
wait for status.ready to go True again   # command finished
read status.cmd_success and status.errors_prev
```

"Success" specifically means: all supported translation errors dropped below
`max_distance_error_m` and rotation errors below `max_rotation_error_deg`, and stayed
there for `min_stabilize_time_s`, all before `cmd_timeout` seconds elapsed.

State and mode changes (`set_state`, `set_mode`) and the configuration commands are not
gated by this handshake; they apply immediately and are reflected in the next `info` /
`status` publish.

---

## 8. Coordinate frames and units

From the message definitions (`nepi_interfaces/msg/Goto*.msg`):

- **Goto Pose** (`GotoPose`): `roll_deg`, `pitch_deg`, `yaw_deg`, each `-180..180`.
  A value of **`-999` means "keep current"** for that axis.
- **Goto Position** (`GotoPosition`): body frame, `x_meters` = Forward, `y_meters` =
  Left, `z_meters` = Up. A value of **`0` means "keep current"** for that axis.
  `yaw_deg` is body-relative, `-180..180`, `0` = keep current yaw.
- **Goto Location** (`GotoLocation`): `lat`, `long`, `altitude_meters` (AMSL),
  `yaw_deg` (`-180..180`). A value of **`-999` means "keep current"**.
- **Errors** (`GotoErrors`): signed `x_m`, `y_m`, `z_m`, `heading_deg`, `roll_deg`,
  `pitch_deg`, `yaw_deg`.
- **Axis support** (`AxisControls`): booleans `x`, `y`, `z`, `roll`, `pitch`, `yaw`.
- **Error bounds** (`ErrorBounds`): `max_distance_error_m`, `max_rotation_error_deg`,
  `min_stabilize_time_s`.
- **Motor control** (`MotorControl`): `motor_ind`, `speed_ratio` 0.0-1.0.

Note the two different sentinels: position uses `0` to mean "hold this axis", while pose
and location use `-999`. Watch this when generating commands from a sim.

Internally the interface maintains both ENU and NED forms of the current pose, plus WGS84
and AMSL location, derived from the driver's navpose (see section 9 of the driver notes).
For a client, the frames above are what matter.

---

## 9. States, modes, and actions (the enumerations)

Four capabilities are **enumerated lists** defined by the driver and reported in the
capabilities query. A client references them **by index**.

- **States** (`state_options`): discrete operating states. Set with `set_state` (index).
  Current index is `info.state`.
- **Modes** (`mode_options`): flight/control modes. Set with `set_mode` (index). Current
  index is `info.mode`.
- **Setup actions** (`setup_action_options`): one-shot blocking setup operations. Trigger
  with `setup_action` (index).
- **Go actions** (`go_action_options`): one-shot blocking "go" operations. Trigger with
  `go_action` (index).

The driver defines these lists and provides the get/set index functions that translate an
index into a real vehicle command.

### ArduPilot reference values

The ArduPilot driver (`rbx_ardupilot_node.py`) declares:

```python
RBX_STATES        = ["DISARM", "ARM"]
RBX_MODES         = ["STABILIZE", "LAND", "RTL", "LOITER", "GUIDED", "RESUME"]
RBX_SETUP_ACTIONS = ["TAKEOFF", "LAUNCH"]
RBX_GO_ACTIONS    = []
```

So on that driver, "arm" is `set_state(1)`, "switch to GUIDED" is `set_mode(4)`, and
"take off" is `setup_action(0)`. `RESUME` is a special mode that returns to the previous
mode. There are no go actions. A sim can pick whatever lists make sense for its vehicle;
these are just the ArduPilot choices.

And it wires these capabilities into `RBXRobotIF`:

| Capability | ArduPilot |
|---|---|
| Battery feedback | enabled (`getBatteryPercent`) |
| Manual motor control | disabled (`setMotorControlRatio = None`) |
| Autonomous control | enabled |
| Set home / Go home | enabled |
| Go stop | enabled |
| Goto pose / position / location | all enabled |

---

## 10. Manual vs autonomous control gating

RBX distinguishes two control regimes, surfaced as `manual_control_mode_ready` and
`autonomous_control_mode_ready` in status:

- **Manual**: direct per-motor control via `set_motor_control`. Available only when the
  driver supplied `setMotorControlRatio` and `manualControlsReadyFunction()` returns True.
- **Autonomous**: the goto commands. `goto_location` is accepted only when
  `autonomousControlsReadyFunction()` returns True; `goto_position` is accepted only when
  `manual_control_mode_ready` is False (that is, when the vehicle is not in manual mode).

A client should read both flags from status and only offer the controls that are
currently ready. The NEPI web UI does exactly this.

---

## 11. Building a new RBX driver (and adding capabilities)

A driver's whole job is to construct one `RBXRobotIF` and hand it implementation
functions. The interface takes care of all the ROS plumbing (topics, services, the
handshake, status/info publishing, image overlay, navpose bridging).

### 11.1 The constructor contract

`RBXRobotIF.__init__` (in `device_if_rbx.py`) takes, among others:

```python
RBXRobotIF(
    device_info,                       # dict: name, path, serial_number, hw/sw version
    capSettings, factorySettings,      # SettingsIF plumbing
    settingUpdateFunction, getSettingsFunction,
    axisControls,                      # AxisControls: which DOF are supported
    getBatteryPercentFunction,         # -> float 0..1, or None
    states, getStateIndFunction, setStateIndFunction,
    modes, getModeIndFunction, setModeIndFunction,
    checkStopFunction,
    setup_actions, setSetupActionIndFunction,
    go_actions, setGoActionIndFunction,
    getHomeFunction=None, setHomeFunction=None,
    manualControlsReadyFunction=None,
    getMotorControlRatios=None, setMotorControlRatio=None,
    autonomousControlsReadyFunction=None,
    goHomeFunction=None, goStopFunction=None,
    gotoPoseFunction=None, gotoPositionFunction=None, gotoLocationFunction=None,
    getNavPoseCb=None,                 # -> navpose dict, drives control error math
    navpose_update_rate=10,
    msg_if=None,
)
```

### 11.2 Enabling or disabling an existing capability

This is the common case, and it needs **no interface changes at all**:

- **To enable** a capability, pass a real function for its constructor argument.
  For example, to support return-to-home, implement `goHome(self) -> bool` and pass
  `goHomeFunction = self.goHome`. `has_go_home` will report `True` and the UI shows the
  button.
- **To disable** it, pass `None` (the default). The flag reports `False` and clients hide
  the control.

Contract for the blocking functions (`goHome`, `goStop`, the three `goto*`, the action
setters): **block until the vehicle reaches the goal within the current error bounds, or
until `cmd_timeout` elapses, then return `True`/`False`.** The interface manages the
`ready` flag and `cmd_success` around your call; you just do the work and report the
outcome. Use the injected `checkStopFunction` to bail out early if a stop is requested.

The get/set index functions map between your enumerated list and the vehicle. For
example `setStateInd(1)` on ArduPilot arms the vehicle; `getStateInd()` returns `1` when
armed.

Minimal skeleton for a sim driver:

```python
class MySimRBXNode:
    RBX_STATES        = ["DISARM", "ARM"]
    RBX_MODES         = ["MANUAL", "GUIDED", "HOLD"]
    RBX_SETUP_ACTIONS = ["TAKEOFF"]
    RBX_GO_ACTIONS    = []

    def __init__(self):
        axis = AxisControls(x=True, y=True, z=True, roll=False, pitch=False, yaw=True)
        self.rbx_if = RBXRobotIF(
            device_info = self.device_info_dict,
            axisControls = axis,
            getBatteryPercentFunction = self.getBattery,
            states = self.RBX_STATES,
            getStateIndFunction = self.getStateInd,
            setStateIndFunction = self.setStateInd,
            modes = self.RBX_MODES,
            getModeIndFunction = self.getModeInd,
            setModeIndFunction = self.setModeInd,
            checkStopFunction = self.checkStop,
            setup_actions = self.RBX_SETUP_ACTIONS,
            setSetupActionIndFunction = self.setSetupActionInd,
            go_actions = self.RBX_GO_ACTIONS,
            setGoActionIndFunction = self.setGoActionInd,
            # start with autonomous goto only; leave manual motor control off
            autonomousControlsReadyFunction = self.autoReady,
            gotoLocationFunction = self.gotoLocation,   # enables has_goto_location
            gotoPositionFunction = self.gotoPosition,   # enables has_goto_position
            gotoPoseFunction = None,                    # not yet -> has_goto_pose False
            getHomeFunction = self.getHome,
            setHomeFunction = self.setHome,             # enables has_set_home
            goHomeFunction = self.goHome,               # enables has_go_home
            goStopFunction = self.goStop,               # enables has_go_stop
            getNavPoseCb = self.getNavPose,
            # ... settings plumbing ...
        )

    def gotoLocation(self, setpoint):   # [lat, long, alt_m, yaw_deg]
        # command the sim, then block until within error bounds or timeout
        return True   # cmd_success
```

To add support for attitude commands later, implement `gotoPose` and set
`gotoPoseFunction = self.gotoPose`. Nothing else changes.

### 11.3 Adding a genuinely new capability (interface change)

If you need a capability the interface does not have yet (a new command topic, a new
report flag), the change touches three layers. Follow the pattern the existing
capabilities already use:

1. **Messages/services** (`nepi_interfaces/`): add a message type if the command needs a
   new payload, and add a `has_<x>` flag and/or option list to `RBXCapabilitiesQuery.srv`
   (and `DeviceRBXStatus.msg` if it needs live feedback). Rebuild the interfaces package.
2. **Interface** (`device_if_rbx.py`):
   - Add a constructor argument for the driver's implementation function and set the
     `caps_report.has_<x>` flag based on whether it is `None`.
   - Add an entry to `SUBS_DICT` (a new command topic) or `SRVS_DICT`, pointing at a new
     callback.
   - Write the callback following the handshake pattern in section 7: reject if not
     `ready`, set `process_current` / `ready=False`, call the driver function, then
     restore `process_current="None"` / `cmd_success` / `ready=True`.
3. **Driver** (your node): implement the function and pass it into the constructor.
4. **Clients / UI** (optional): read the new `has_<x>` flag and render the control. See
   `NepiDeviceRBX-Controls.js` for how the web UI conditionally builds controls from the
   capabilities response, so your sim app can mirror that logic.

Because clients gate on the `has_*` flags, a new capability is backward compatible: older
clients simply ignore it.

---

## 12. File map

| Concern | File |
|---|---|
| Interface class (all topics, services, handshake) | `nepi_engine_ws/.../nepi_api/device_if_rbx.py` |
| Reference driver | `src/nepi_drivers/rbx_drivers/rbx_ardupilot_node.py` |
| Driver discovery | `src/nepi_drivers/rbx_drivers/rbx_ardupilot_discovery.py` |
| Capabilities service | `nepi_interfaces/srv/RBXCapabilitiesQuery.srv` |
| Status message | `nepi_interfaces/msg/DeviceRBXStatus.msg` |
| Info message | `nepi_interfaces/msg/DeviceRBXInfo.msg` |
| Goto / axis / error / motor messages | `nepi_interfaces/msg/Goto*.msg`, `AxisControls.msg`, `ErrorBounds.msg`, `MotorControl.msg` |
| SDK helpers / automation | `nepi_engine_ws/.../nepi_sdk/nepi_rbx.py` |
| Web UI client (example) | `nepi_engine_ws/.../rui-app/src/NepiDeviceRBX*.js` |
| SITL simulation setup | `src/docs/SITL_IMPLEMENTATION_PLAN.md` |
| Driver discovery details | `src/docs/DISCOVERY_EXPLAINED.md` |

---

## Appendix: capability decision cheat-sheet for a sim client

```
on connect:
    caps = call rbx/capabilities_query
    subscribe rbx/status, rbx/info

render controls:
    show state buttons      for each name in caps.state_options
    show mode buttons       for each name in caps.mode_options
    show setup buttons      for each name in caps.setup_action_options
    show go-action buttons  for each name in caps.go_action_options
    if caps.has_set_home:        show set-home / set-home-current
    if caps.has_go_home:         show go-home
    if caps.has_go_stop:         show stop
    if caps.has_goto_pose:       show attitude command
    if caps.has_goto_position:   show body-move command
    if caps.has_goto_location:   show global-move command
    if caps.has_manual_controls: show per-motor sliders
    if caps.has_battery_feedback: show battery gauge (else hide, status.battery == -999)

send any blocking command:
    require status.ready == True
    publish
    ready -> False (running) -> True (done); then read cmd_success, errors_prev
```
