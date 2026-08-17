# Sim Connector — What's Left, and Where to Start

A single, current answer to "what hasn't been implemented yet." Pulls together the real
implementation status (`MULTI_SIMULATOR_INTEGRATION_PLAN.md`'s status table) with a direct
code-level check of `device_if_sim.py`/`sim_connector_app_node.py` in the real
`nepi_engine_ws` tree (not the sandbox), so this reflects what's actually there, not what
an earlier plan assumed would be there. See `SIM_DEVICE_IF_CONTRACT.md` for the contract
itself.

---

## What's actually done (don't redo this)

- **The generic contract is real and working**: `device_if_sim.py` + `sim_connector_app_node.py`
  (`src/nepi_apps/nepi_app_sim_connector/`) — capability-flag derivation from a per-robot
  config, a real TCP/JSON wire protocol on port 9030, and a substantial RUI (motor/goto
  controls, camera selector + viewer, camera-view-mode dropdown, environment toggles,
  robot-config selector, plus an SSH-based remote auto-launch panel).
- **Gazebo is fully wired into that generic contract**, not just the old RBX-specific path
  — `sim_container/bridges/gazebo/sim_connector_bridge_gazebo.py`, verified end-to-end
  including against the real device: goto commands move the real tracked position, both
  `scene_camera` and `robot_camera` are announced and streamable, environment options and
  reset actions are wired.
- **Webots, ROS Stage, and PyBullet bridges are built and verified on the dev VM** (not yet
  confirmed on the real device or in a browser — see below).
- **WPILib HAL Sim (the FRC-adjacent simulator) bridge is built and verified on the dev VM**
  too — goto works, RESET works, same "VM done, device/RUI unconfirmed" status as the three
  above.
- **Unity is blocked on licensing, not engineering** — needs an interactive Unity account
  sign-in, which can't be automated. Not a code task; needs you specifically.

---

## Real gaps — actual work left, as granular checklists

### 1. Environment-option control is enable-only — **code fix done, live verification still open**

`sim_connector_app_node.py`'s `setEnvironmentOption` always sent `enabled: True`
regardless of caller intent, and `device_if_sim.py`'s `setEnvironmentOptionCb` had no
boolean parameter at all in its ROS subscriber. There was no way to turn an environment
option *off* — only on.

- [x] Read `device_if_sim.py`'s `setEnvironmentOptionCb` and its `SUBS_DICT` entry —
      confirmed `std_msgs/String`, one field.
- [x] **Chose not to add a new compiled message type** (would need a workspace-wide
      `catkin` rebuild to become importable). Instead: `setEnvironmentOptionCb` now
      JSON-decodes `msg.data` as `{"option": "<name>", "enabled": <bool>}` on the
      existing `std_msgs/String` topic, falling back to treating a bare non-JSON string
      as an enable (backward compatible with any other caller still on the old format).
- [x] `setEnvironmentOptionFunction` now called as `(option, enabled)`, not just
      `(msg.data)`.
- [x] `sim_connector_app_node.py`'s `setEnvironmentOption(option, enabled=True)` now
      forwards the real bool into the wire protocol's `environment_option` JSON line
      (previously hardcoded `True`).
- [x] **Found and fixed a second, independent bug while checking the bridges**:
      `sim_connector_bridge_gazebo.py` only ever called `self.setObstacleCourse(True)`,
      gated behind `and msg.get("enabled", True)` — meaning a `False` value was silently
      dropped before ever reaching the toggle function, which itself already supported
      both directions correctly. Fixed to call `setObstacleCourse(bool(msg.get("enabled", True)))`
      unconditionally.
  - [ ] Webots/Stage/PyBullet/WPILib bridges not checked for the same bug — do this
        before relying on environment toggles for those simulators.
- [x] RUI (`Nepi_IF_Sim-Controls.js`) environment buttons now toggle real on/off state
      and send the JSON-encoded `{option, enabled}` string, with the button label
      reflecting current state (`"obstacle_course (on)"` / `"(off)"`).
  - Known limitation, not fixed here: the on/off state is tracked client-side only —
    `SimStatus.msg` has no field reporting which environment options are actually active
    server-side, so a page reload resets the button to "off" regardless of the sim's
    real state. Would need a new status field to fix properly; not done since it's a
    step beyond just closing the bug.
- [ ] **Live verification not done** — no running Gazebo/ROS environment was available
      in this pass. All of the above was verified by direct code reading, `py_compile`
      on the three Python files, and manual JS review — not by actually toggling
      `obstacle_course` in a real sim. Do this before considering item 1 closed:
      toggle on, confirm it appears in Gazebo; toggle off through the same control,
      confirm it's actually removed.

### 2. Goto commands don't report success/failure back to NEPI's status

`gotoPoseCb`/`gotoPositionCb`/`gotoLocationCb` in `device_if_sim.py` were fire-and-forget
— they never set `self.status_msg.cmd_success`, unlike every other command type.

- [x] **Decision: worth fixing now**, not punting — silently unable to report goto
      failure is a real correctness gap once more than one simulator's bridge has a real
      closed-loop controller, not just a Gazebo-specific curiosity.
- [x] Defined a new incoming wire-protocol line: `{"type":"goto_result","success":bool}`
      — documented in `sim_connector_app_node.py`'s own wire-protocol docstring.
- [x] Added `device_if_sim.py`'s `report_goto_result(success)`, called by the hosting
      node when that line arrives, setting `cmd_success` asynchronously (there's no
      per-command ID in the protocol, so this always applies to "whatever goto is most
      recently pending" — correct as long as a bridge only tracks one goto target at a
      time, which every existing bridge's controller design already assumes).
- [x] Wired `sim_connector_app_node.py`'s `processBridgeLine` to call it on `goto_result`.
- [x] Updated the Gazebo bridge's `controlTickCb` to actually send
      `{"type":"goto_result","success":true}` at the exact point it already logged
      "goto target reached" internally — that log line existed before, the wire send
      didn't.
  - [ ] Webots/Stage/PyBullet/WPILib bridges don't send this yet — each has to be
        checked for whether it has an equivalent convergence-detection point and wired
        similarly. Not done here; do before relying on goto success/failure reporting
        for those simulators. This is backward compatible either way — a bridge that
        never sends `goto_result` just leaves `cmd_success` untouched, same as before.
  - [ ] No failure path is actually exercised anywhere yet — every bridge only ever
        sends `success: true`. Worth deciding what "goto failed" even means for a given
        simulator (unreachable target? timeout? collision?) before wiring a false-case
        send.
- [ ] **Live verification not done** — same caveat as item 1: no running Gazebo/ROS
      environment was available in this pass. Verified by code reading and
      `py_compile` only. Before considering this closed: send a `goto_position`,
      confirm `cmd_success` flips `True` in `sim/status` once the rover visibly arrives.

### 3. Minor: Gazebo's image relay rate cap — re-checked, looks correct now

Previously reported running at ~15Hz against an intended 5Hz cap (`IMAGE_RATE_HZ`).

- [x] Read `senderLoop`'s gating logic: `if now - last_image >= 1.0 / IMAGE_RATE_HZ`
      (0.2s) inside a loop that itself sleeps `1.0 / TELEMETRY_RATE_HZ` (0.1s,
      `TELEMETRY_RATE_HZ = 10.0`) per iteration. As written today, this cannot exceed
      10Hz outer-loop-wise, and the 0.2s image gate should reliably cap it to 5Hz — the
      logic reads as correct, not buggy.
- [ ] **Could not reproduce or confirm the original ~15Hz measurement** — no live
      Gazebo/ROS environment available in this pass to re-run `rostopic hz` against.
      Left as-is rather than "fixing" code that looks correct by static reading; either
      the constants were already tuned since that measurement was taken, or it needs a
      live re-measurement to actually catch whatever's really happening. Don't assume
      this is closed — confirm with `rostopic hz` next time a real sim is running.

### 4. Human-in-the-loop verification (not code, can't be done by an agent)

- [ ] Load the RUI in a browser against the real device and visually confirm the
      capability-driven controls render correctly for the Gazebo `ground_robot_2_wheel`
      config (wire-level behavior already confirmed — this is purely the visual check).
- [ ] Re-verify `RESET`/`RETURN_HOME` and the `obstacle_course` toggle individually for
      Gazebo (wired, but not individually re-checked in the last verification pass).
- [ ] Repeat Gazebo's real-device confirmation steps for Webots: load the RUI, confirm
      controls render, confirm on-device behavior matches the VM-side verification.
- [ ] Same for ROS Stage.
- [ ] Same for PyBullet.
- [ ] Same for WPILib HAL Sim.

### 5. A decision on old, now-superseded code — **partly done**

`rbx_gazebo_node.py`/`rbx_gazebo_discovery.py` (the old `RBXRobotIF`-based Gazebo driver)
is not used by anything the new Gazebo bridge relies on.

- [x] Took the conservative middle ground rather than deciding retire-vs-keep
      unilaterally: **removed the two dead camera methods**
      (`get_sensor_topics()`/`get_camera_reference_frames()` — confirmed never called
      anywhere, never wired into the `RBXRobotIF` constructor) since leaving working-
      looking-but-unreachable code around was the actively misleading part. Left a
      comment pointing at the real two-camera mechanism (the generic sim_connector
      path) and at this checklist item for the still-open bigger decision. The
      constants they used (`ROBOT_MAIN_REFERENCE_FRAME`, `SCENE_CAMERA_DEFAULT_OFFSET_M`,
      `SCENE_CAMERA_DEFAULT_TILT_DEG`) were left in place — they're inert, not
      misleading, and encode real tuning values worth keeping if this driver ever does
      get migrated onto `SimDeviceIF`.
- [ ] **Still open, genuinely your call:** retire this driver entirely now that Gazebo
      goes through the generic contract, or keep it as a fallback/for some other
      reason? Not decided here — a whole-file deletion (including its discovery script
      and params YAML, and whatever `apps_mgr`/`drivers_mgr` registration references
      it) is a bigger, more consequential change than the dead-code trim above, and
      worth your explicit sign-off rather than an agent's unilateral guess, especially
      since this driver may already be deployed on the real device.

---

## Explicitly not next steps (deferred on purpose)

- Multi-robot variants for Webots/Stage/PyBullet/WPILib — Gazebo already has one; the
  others get a single-vehicle proof first, by design.
- Live camera pose/offset adjustment — deliberately left without a wire shape until real
  usage clarifies what's actually needed; don't design this speculatively.
- Real hardware / RoboRIO / FRC hardware integration — a different problem (a real
  device's native protocol, not a simulator), always out of scope here.
- Any change to `device_if_sim.py`'s core contract to support a new simulator — per its
  own stated rule, a new simulator should only ever need a new bridge script + a new
  robot-config entry.

---

## Suggested order to actually start in

1. **Fix the environment-option enable/disable bug first.** It's small, self-contained,
   and directly unblocks the "user can turn things on and off" goal that motivates this
   whole effort.
2. **Fix or explicitly punt on the goto `cmd_success` gap** — at minimum, decide and write
   down whether this matters before more simulators get built on top of an interface that
   silently can't report goto failure.
3. **Do the RUI visual-confirmation pass** for Gazebo — cheap, and likely to surface real
   UI gaps in exactly the "what's shown/editable" area that matters most, before sinking
   more time into bridges whose controls haven't been eyeballed yet.
4. **Decide on `rbx_gazebo_node.py`** — a quick call, removes ongoing confusion between two
   parallel Gazebo integration paths.
5. **Work through the remaining on-device + RUI confirmations** for Webots, Stage,
   PyBullet, and WPILib, one at a time.
6. **Unity, whenever you're ready to personally do the account sign-in** — not blocking
   anything else.
