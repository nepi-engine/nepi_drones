# RBX Manual Motor Control Ratios

How `getMotorControlRatios` and `setMotorControlRatio` work in an RBX driver, and how
manual motor control is reached from a robot's Device page in the NEPI web UI.

Related reading: [RBX_CAPABILITIES.md](RBX_CAPABILITIES.md) (sections 4 and 10 cover the
capability model and the manual/autonomous control gating).

---

## 1. What "motor control ratios" are

RBX exposes two control regimes:

- **Autonomous**: the `goto_*` commands (pose, position, location). The interface does the
  flight-control math for you.
- **Manual**: direct, per-motor commands. You tell the vehicle "motor 2 to 40% speed" and
  it does exactly that, with no autonomy in between.

A "motor control ratio" is a single motor's commanded speed as a fraction of its maximum:

- `0.0` = motor off
- `1.0` = motor at max
- values in between = proportional speed

Each motor has its own ratio. The full set of ratios (one per motor) is the vehicle's
manual-control state. There is no direction/reverse field in the base message; the ratio
is a magnitude of `[0.0, 1.0]`.

The wire type is [`MotorControl.msg`](../../nepi_interfaces/msg/MotorControl.msg):

```
uint8   motor_ind      # which motor (0-based index)
float32 speed_ratio    # 0-1, 0 -> off, 1 -> max
```

---

## 2. The two driver functions

A driver supplies these to `RBXRobotIF`. They are the only two things a driver has to
implement to get manual motor control; the interface handles all the ROS plumbing.

### `getMotorControlRatios()`

- **Signature**: `getMotorControlRatios() -> list[float]`
- **Returns**: an ordered list of the current speed ratios, one entry per motor. Index `i`
  in the list is motor index `i`.
- **Purpose**: two jobs at once.
  1. **Defines how many motors exist.** The interface uses `len(getMotorControlRatios())`
     as the motor count. That count is what bounds-checks incoming commands (a
     `motor_ind` past the end is rejected).
  2. **Reports current state.** The interface converts this list into the
     `current_motor_control_settings` field of the RBX status message (a `MotorControl[]`),
     so clients can see where each motor is right now.

### `setMotorControlRatio(motor_ind, speed_ratio)`

- **Signature**: `setMotorControlRatio(motor_ind: int, speed_ratio: float) -> None`
- **Purpose**: set one motor to a speed ratio. This is where the driver actually talks to
  the hardware (or sim). The interface calls it once per accepted command, for a single
  motor at a time.
- Supplying this function (non-`None`) is what turns the whole capability on. See section 4.

Both live in the driver. In this repo the ArduPilot RBX node has them as stubs today, see
[rbx_ardupilot_node.py:447](../src/nepi_drivers/rbx_drivers/rbx_ardupilot_node.py#L447):

```python
def setMotorControlRatio(self, motor_ind, speed_ratio):
    pass

def getMotorControlRatios(self):
    return []
```

---

## 3. The command path (what happens end to end)

```
client / UI  --MotorControl-->  <rbx node>/rbx/set_motor_control  (topic)
                                        |
                                        v
                               setMotorControlCb(motor_msg)      device_if_rbx.py
                                        |
                                        v
                               setMotorControl(new_motor_ctrl)   validates + gates
                                        |
                                        v
                               driver.setMotorControlRatio(motor_ind, speed_ratio)
                                        |
                                        v
                                   motor hardware / sim
```

The interface's `setMotorControl` runs these checks before it ever calls the driver
(see [device_if_rbx.py:1154](../../nepi_engine_ws/src/nepi_engine/nepi_api/src/nepi_api/device_if_rbx.py#L1154)):

1. `manualControlsReadyFunction()` must return `True` **and** `setMotorControlRatio` must
   be supplied. Otherwise the command is dropped with "Manual Controls not Ready".
2. `motor_ind` must be within range (`< len(getMotorControlRatios())`), else "out of range".
3. `speed_ratio` must be in `[0, 1]`, else "out of range".

Only after all three pass does it call `setMotorControlRatio(motor_ind, speed_ratio)`.

Feedback: the current ratios come back in the RBX status message field
`current_motor_control_settings` (a `MotorControl[]`), rebuilt from `getMotorControlRatios()`.

---

## 4. Wiring it into RBXRobotIF

A driver enables manual control by passing the functions to the `RBXRobotIF` constructor.
Whether `setMotorControlRatio` is `None` decides the capability flag:

| Constructor arg | If supplied | If `None` |
|---|---|---|
| `setMotorControlRatio` | `has_manual_controls = True` | `has_manual_controls = False` |
| `getMotorControlRatios` | motor count + status feedback | reports an empty motor list |
| `manualControlsReadyFunction` | gates whether commands are accepted right now | manual mode reported never ready |

To **enable** manual control on a driver:

```python
self.rbx_if = RBXRobotIF(
    ...
    manualControlsReadyFunction = self.manualControlsReady,  # -> bool
    getMotorControlRatios       = self.getMotorControlRatios, # -> list[float]
    setMotorControlRatio        = self.setMotorControlRatio,  # (ind, ratio)
    ...
)
```

To **disable** it (the current ArduPilot default, see
[rbx_ardupilot_node.py:325](../src/nepi_drivers/rbx_drivers/rbx_ardupilot_node.py#L325)):

```python
    manualControlsReadyFunction = None,
    getMotorControlRatios       = None,
    setMotorControlRatio        = None,
```

`has_manual_controls` is reported in the driver's `capabilities_query`, and that flag is
what the web UI reads to decide whether to offer manual control at all.

---

## 5. Setting it from the robot's Device page

In the NEPI web UI, open the robot under its **RBX** device, then the **Controls** tab
(rendered by
[NepiDeviceRBX-Controls.js](../../nepi_engine_ws/src/nepi_rui/src/rui_webserver/rui-app/src/NepiDeviceRBX-Controls.js)):

1. **Select Control Type.** The dropdown lists `Manual` only when the driver reports
   `has_manual_controls = True`. If the driver did not supply `setMotorControlRatio`,
   `Manual` will not appear at all.
2. **Choose `Manual`.** The page then shows a **Manual Ready** indicator, which mirrors
   the status field `manual_control_mode_ready` (driven by `manualControlsReadyFunction()`).
   It must read true before commands will be accepted.

> **UI note:** at present the Controls tab only exposes the `Manual` selection and the
> readiness indicator. There is no per-motor slider in the web UI yet, so actual
> `set_motor_control` commands are issued programmatically or over the topic (below). If a
> motor slider panel is added later, it will publish to the same `set_motor_control`
> topic described here.

### Commanding a motor over the topic

The topic is `<rbx node namespace>/rbx/set_motor_control`, type `MotorControl`. To set
motor `2` to 40% (ROS1 / rospy, which is what NEPI runs):

```bash
rostopic pub -1 /nepi/<device>/<rbx_node>/rbx/set_motor_control \
  nepi_interfaces/MotorControl "{motor_ind: 2, speed_ratio: 0.4}"
```

Publish one message per motor you want to change. Confirm it took effect by watching
`current_motor_control_settings` in the RBX status message. Remember the readiness gate in
section 3: if `Manual Ready` is false, the command is silently ignored (a warning is
logged, no motion happens).

---

## 6. Implementing the two functions on a new driver

Minimal shape for a driver with N motors:

```python
def __init__(self):
    self.motor_ratios = [0.0] * self.NUM_MOTORS   # current commanded state

def manualControlsReady(self):
    # True only when it is safe to accept direct motor commands
    # (armed, in a manual/passthrough mode, not mid-autonomous-goto, etc.)
    return self.is_armed and self.mode_is_manual

def getMotorControlRatios(self):
    # length defines the motor count; contents are current ratios
    return self.motor_ratios

def setMotorControlRatio(self, motor_ind, speed_ratio):
    # push to hardware, then remember it so getMotorControlRatios() reflects it
    self.motor_ratios[motor_ind] = speed_ratio
    self._send_motor_command(motor_ind, speed_ratio)
```

Notes:

- Keep `getMotorControlRatios()` cheap and always the right length. Bounds checking and
  status reporting both depend on that length being correct.
- Range clamping to `[0, 1]` is done by the interface before your function is called, but
  clamping again inside `_send_motor_command` is cheap insurance.
- `setMotorControlRatio` is called for **one** motor at a time. If you need an all-stop,
  expect one call per motor with `speed_ratio = 0.0`.

### Interface status (fixed 2026-07-16)

`device_if_rbx.py` had four defects in the motor-control path that were fixed on
2026-07-16, so you are building against a clean interface. For the record (and in case
you read older code / another copy):

- **Signature standardized on two positional args:** `setMotorControlRatio(motor_ind,
  speed_ratio)` ([device_if_rbx.py:1164](../../nepi_engine_ws/src/nepi_engine/nepi_api/src/nepi_api/device_if_rbx.py#L1164)).
  Write your driver function to that signature (the ArduPilot node stub already matches).
  The init-time reset block now uses the same two-arg call.
- **Init-time ordering** fixed: `setMotorControlRatio` is assigned before the manual-controls
  setup block, so enabling manual controls no longer raises `AttributeError` at construction.
- **Callback field** fixed: the `set_motor_control` callback now uses the `MotorControl`
  message directly (it previously read a non-existent `.data` field and crashed on every
  command).
- **Range check** fixed: the `[0,1]` guard now tests `speed_ratio`, not the whole message
  object.

Still good practice: clamp `speed_ratio` to `[0,1]` inside your driver as cheap insurance.
