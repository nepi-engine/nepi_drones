# NEPI Drone Simulator — Developer Task Guide

Your (Suraj's) sequenced task list from the Jul 15 check-in. Goal for this phase:
drive the **ArduPilot SITL + Gazebo** simulator through the NEPI **RBX driver** —
first autonomous control, then direct motor control — validating nav/pose feedback
along the way.

This guide is the "what to do, in order." Two companion docs hold the deep detail;
this file links into them at the right steps:

- [SITL_IMPLEMENTATION_PLAN.md](SITL_IMPLEMENTATION_PLAN.md) — how to wire the RBX
  driver to a SITL endpoint (discovery + params changes, ~25 lines).
- [MOTOR_CONTROL_RATIOS.md](MOTOR_CONTROL_RATIOS.md) — how the motor-control
  functions work end to end.
- [RBX_CAPABILITIES.md](RBX_CAPABILITIES.md) — the capability/gating model.

Scope for this phase (agreed in the meeting): **nav/pose data only.** No cameras,
no lights. Develop **in a local VM/WSL, outside the container** first; Dockerize
later once it works.

---

## The big picture (read once)

```
  ArduPilot SITL  (simulated flight controller, your VM)
        |  MAVLink over TCP 127.0.0.1:5760
        v
  mavros_node          (translates MAVLink <-> ROS; launched by the RBX discovery script)
        |  ROS topics/services
        v
  rbx_ardupilot_node.py   (the RBX driver node — where YOUR motor code goes)
        |  device_if_rbx.py  (RBX API — NEPI side, already built)
        v
  NEPI RUI / Argus     (GUIDED, ARM, TAKEOFF, GoTo, and motor controls)
```

Two things to internalize:

1. **You edit the driver, not the API.** The NEPI side (`device_if_rbx.py`) is
   done. Your work lives in `rbx_ardupilot_node.py` (and its discovery script /
   params). The API auto-detects what the driver supports and surfaces it in the
   RUI — no RUI changes needed on your end.
2. **Autonomous commands must be re-published at ~50 Hz.** ArduPilot times out and
   fails safe if the stream stops. The driver already loops for autonomous goto;
   keep that pattern for any sustained motor command too.

---

## Task 0 — Environment setup  *(your "Setup WSL" + "Figure out simulator" items)*

- [ ] Set up **WSL / Ubuntu 20.04** per the project's environment doc (confirm the
      exact doc with Asher/Jason).
- [ ] Build and run **ArduPilot SITL** (ArduCopter) + **Gazebo**. Full build/run
      commands are in [SITL_IMPLEMENTATION_PLAN.md §1 (Step 1)](SITL_IMPLEMENTATION_PLAN.md).
      Run SITL with `--no-mavproxy` so **TCP 5760 stays free for mavros**.
- [ ] **Acceptance:** `nc -z 127.0.0.1 5760 && echo open` reports the port open
      while SITL runs.

> Compatibility note (your "Figure out simulator" item): SITL is just a MAVLink
> endpoint; NEPI already speaks MAVLink via mavros, so there is no new transport to
> build. SITL provides its **own simulated GPS + compass**, so **fake GPS must be
> OFF** for SITL (it fights the sim sensors). This is different from the hardware
> bench setup where fake GPS is used.

---

## Task 1 — Connect the RBX driver to SITL

Point the RBX ArduPilot driver at the SITL endpoint instead of a serial Pixhawk.
The node itself does **not** change — all the work is in the discovery script +
params YAML. Follow [SITL_IMPLEMENTATION_PLAN.md §3 (Steps 3–5)](SITL_IMPLEMENTATION_PLAN.md):

- [ ] Add a `SITL` option to `rbx_ardupilot_params.yaml` `connection.options`.
- [ ] Add the `SITL` discovery branch + `launchSitlDeviceNode()` (forces
      `fcu_url = tcp://127.0.0.1:5760` and fake GPS off).
- [ ] In the RUI drivers panel, select `connection = SITL`, `fake_gps = False`.
- [ ] **Acceptance:**
      `rostopic echo -n1 /nepi/device1/mavlink_sitl/state` shows `connected: True`,
      and the `ardupilot_sitl` RBX device shows up in the RUI.

---

## Task 2 — Autonomous flight through the RUI  *(your "Test autonomous flight" item)*

Asher will walk you through the arming/nav procedure in the RUI (his action item
"Teach UI process"). All testing goes **through the RUI**, not the old automation
scripts (those are deprecated / non-functional — ignore them).

- [ ] Sequence: **GUIDED → ARM → TAKEOFF → GoTo Position / Go Home.**
- [ ] SITL needs ~20–40 s after boot to get a GPS fix and settle the EKF before it
      will arm — wait for a healthy EKF. (The hardware compass/GPS workarounds do
      **not** apply to SITL; it arms clean on stock params.)
- [ ] **Acceptance:** the simulated vehicle actually moves in Gazebo on a GoTo, no
      callback errors, "Autonomous Ready" latches green.
      Details + checklist: [SITL_IMPLEMENTATION_PLAN.md §4](SITL_IMPLEMENTATION_PLAN.md).

---

## Task 3 — Document the simulator's nav/pose data  *(your "Document navigation data" item)*

The nav/pose data the simulator produces has to be **published by the sim and
subscribed by the NEPI Nav Pose Manager** to close the feedback loop.

- [ ] List every nav/pose field SITL/mavros exposes (position, orientation,
      velocity, GPS fix, EKF status, etc.) — e.g. from
      `/nepi/device1/mavlink_sitl/global_position/*` and `.../local_position/*`.
- [ ] Map each to the NEPI message(s) the app needs, and note anything missing.
- [ ] **Deliverable:** a short markdown list of available vs. required nav/pose
      messages. This feeds the "mock app" Jason will set up for you to edit.

---

## Task 4 — Motor controls

This is the headline item, in two parts. **The NEPI API side is complete and was
cleaned up on 2026-07-16** (signature, range-check, and init bugs fixed), so you're
building against a clean interface. Full mechanics:
[MOTOR_CONTROL_RATIOS.md](MOTOR_CONTROL_RATIOS.md).

### 4a — Prove one motor from the command line first  *(before writing driver code)*

Ask Asher to demo sending a direct message to the mavros node. Confirm a single
motor spins in the simulator before wiring anything into the driver. This isolates
"can MAVLink drive a motor" from "is my driver code right."

- [ ] Identify the correct **MAVROS command/topic for direct motor/actuator
      override** to ArduPilot (research `mavros` actuator control / RC-override /
      motor-test paths). This is your open research item.
- [ ] Drive one motor over the CLI; watch it move in Gazebo.

### 4b — Implement the three driver hooks

The RBX node already has these as **stubs** and currently passes `None` for all
three into `RBXRobotIF` — that's why motor controls don't appear yet:

- Stubs: [rbx_ardupilot_node.py:447-451](../src/nepi_drivers/rbx_drivers/rbx_ardupilot_node.py#L447)
- Passed as `None`: [rbx_ardupilot_node.py:325-327](../src/nepi_drivers/rbx_drivers/rbx_ardupilot_node.py#L325-L327)
- `manualControlsReady()` already exists and returns true only in **MANUAL** mode:
  [rbx_ardupilot_node.py:574](../src/nepi_drivers/rbx_drivers/rbx_ardupilot_node.py#L574)

Do this:

- [ ] **`getMotorControlRatios() -> list[float]`** — return current ratios, one per
      motor. Its **length defines the motor count** the API bounds-checks against.
      Start with a stub returning `[0.0, 0.0, 0.0, 0.0]` — just returning a non-empty
      list is what makes the motor controls appear in the RUI.
- [ ] **`setMotorControlRatio(motor_ind, speed_ratio)`** — the real work. Wrap the
      MAVROS command you proved in 4a. Note the **canonical signature is two args**
      `(motor_ind, speed_ratio)`; convert the NEPI `0.0–1.0` ratio into whatever
      units the MAVROS command wants (PWM µs, normalized -1..1, etc.), and clamp to
      `[0,1]` defensively.
- [ ] **`manualControlsReady()`** — already returns true only in MANUAL mode; make
      sure the vehicle is armed + in MANUAL when you test, or commands are dropped.
- [ ] Wire all three into the `RBXRobotIF(...)` constructor (uncomment lines
      [325-327](../src/nepi_drivers/rbx_drivers/rbx_ardupilot_node.py#L325-L327)).
- [ ] If a motor command must be **sustained**, re-publish at ~50 Hz (same reason as
      autonomous goto — ArduPilot fails safe otherwise).

### 4c — Test end to end

- [ ] Confirm `Manual` control type appears in the RUI (RBX device → Controls tab).
      No RUI code changes should be needed — it's capability-driven. If it doesn't
      show, coordinate with Asher.
- [ ] Command a motor over the topic and watch `current_motor_control_settings` in
      the RBX status update, and the motor move in Gazebo:
      ```bash
      rostopic pub -1 /nepi/<device>/ardupilot_sitl/rbx/set_motor_control \
        nepi_interfaces/MotorControl "{motor_ind: 2, speed_ratio: 0.4}"
      ```

---

## Task 5 — Deploy & build workflow

Two commands, don't mix them up:

- **`nepi deploy`** — sends your code from your dev machine to the device.
- **`nepi build`** — compiles the source **on the device**.

The `nepi_drones` repo is meant to **override** the base source in the designated
folders when deployed — same override pattern as the **Ocean Aero** customer repo.

> **Dependency / current blocker:** the deploy script for `nepi_drones` is **not
> fully configured yet** — Asher owns updating it (his "Update Deployment Script"
> action item, using the Ocean Aero config as the reference). Until that lands, your
> deploy→rebuild→iterate loop won't be clean. Check with Asher on status before
> relying on it; for early dev you can edit the deployed `/opt` copies directly to
> test, but a real `nepi build` from `/home/production/nepi_engine_ws/...` source is
> the source of truth (the next build overwrites `/opt`).

- [ ] Confirm with Asher when the `nepi_drones` deploy script is ready.
- [ ] Verify a full loop: edit driver → `nepi deploy` → `nepi build` → restart →
      change is live.

---

## Quick reference

| Thing | Where |
|---|---|
| RBX driver node (your code) | [rbx_ardupilot_node.py](../src/nepi_drivers/rbx_drivers/rbx_ardupilot_node.py) |
| Discovery / params (SITL wiring) | `rbx_ardupilot_discovery.py`, `rbx_ardupilot_params.yaml` |
| Motor message contract | [MotorControl.msg](../../nepi_interfaces/msg/MotorControl.msg) — `uint8 motor_ind`, `float32 speed_ratio` |
| SITL endpoint | `tcp://127.0.0.1:5760` (run SITL `--no-mavproxy`) |
| Motor control topic | `<device ns>/ardupilot_sitl/rbx/set_motor_control` |
| Deep dives | [SITL plan](SITL_IMPLEMENTATION_PLAN.md) · [Motor control](MOTOR_CONTROL_RATIOS.md) · [Capabilities](RBX_CAPABILITIES.md) |

## Who to ask

- **RUI arming/nav walkthrough, RBX connect, deploy script, motor-control CLI
  demo:** Asher.
- **Architecture / MAVROS command questions:** Jason (and Asher).
