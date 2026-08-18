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
- **Webots and PyBullet bridges are built and verified on the dev VM** (not yet
  confirmed on the real device or in a browser — see below).
- **ROS Stage support was retired 2026-08-17** — not a priority; removed entirely (bridge
  script, robot config, launch target). See `MULTI_SIMULATOR_INTEGRATION_PLAN.md`'s §6 if
  the history is ever needed again.
- **WPILib HAL Sim (the FRC-adjacent simulator) bridge is built and verified on the dev VM**
  too — goto works, RESET works, same "VM done, device/RUI unconfirmed" status as the three
  above.
- **Unity is blocked on licensing, not engineering** — needs an interactive Unity account
  sign-in, which can't be automated. Not a code task; needs you specifically.

## Live testing round (2026-08-17) — real bugs found and fixed

Live testing against the real device (not just VM-side code reading) surfaced several
real bugs beyond the three below, all now fixed and deployed:

- **The actually-deployed RUI had `show_controls={false}` hardcoded** in
  `NepiAppSimConnector.js` — a second, more severe cause of "no motor/goto/camera/
  environment controls visible" than the capability-empty-default-config explanation
  below. This wasn't a mismatch between `nepi_apps`'s source copy and what's live — the
  *deployed* copy had drifted to `false` while the source had `true`. Fixed by
  redeploying the correct copy.
- **Deploy button ordering** — fixed (see item in the checklist history), deployed.
- **Quadcopter launched Gazebo before ArduPilot SITL** — swapped (SITL first), verified
  safe by reading the actual ArduPilot/Gazebo FDM socket source (both sides are
  independent, retry-based, no ordering dependency) — not yet live-verified after the
  swap.
- **Killed rover stayed listed in the RUI's Devices page** — root-caused to
  `sim_connector_app_node.py`'s own dangling status-topic subscription (see the fixed
  item below), deployed.
- **"4-Wheel Rover" launch appeared to open an empty world** — investigated at length;
  the committed launch-target config, world file, and model were all confirmed correct.
  The actual cause: a leftover `roscore`/`gzserver` from an earlier, unrelated manual
  test session on the dev VM was still running (or its stale `gzclient` window still
  visible) when the launch was attempted, and `gazebo_rover`'s launch command correctly
  refuses to start a second Gazebo instance when one is already running (to avoid
  silently attaching to the wrong world) — it's very likely what looked like "empty
  world" was actually that leftover session, not a real config/code bug. Cleaned up the
  leftover process; **worth simply retrying this launch** before assuming anything else
  is wrong.
- **`apps_mgr` does not auto-relaunch a killed app** — learned the hard way while
  restarting `app_sim_connector` to pick up code changes: killing the node did not
  bring it back within any reasonable window, contrary to what its own docs implied.
  Had to relaunch it manually. Worth knowing for next time, not something to "fix"
  necessarily — just don't assume killing an app node self-heals.
- **Manually relaunching a NEPI app node needs `LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1`
  set** on this device, or a lazy `import open3d` deep in `ReadWriteIF`'s init crashes
  with a static-TLS allocation error. Whatever normally launches these nodes sets this;
  a raw manual launch doesn't. Worth understanding whether this should be baked into the
  node's own startup rather than relying on the launcher's environment, but not changed
  here since it's outside this pass's scope.

All of the above (except the still-open live-goto/environment-toggle verification and
the quadcopter reorder) are deployed to the real device as of this round — the RUI
bundle was rebuilt and pushed, `sim_connector_app_node.py`/`device_if_sim.py` were
live-synced into the running container, and the app was restarted and confirmed healthy
via its own status topic.

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
  - [ ] Webots/PyBullet/WPILib bridges not checked for the same bug — do this before
        relying on environment toggles for those simulators (PyBullet and WPILib both
        report zero environment options today, so this may be moot for them).
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
- [x] **Webots/PyBullet/WPILib bridges now send it too.** All had the exact same gap
      (a "goto target reached" log/print with no corresponding wire message) — fixed
      identically in `sim_connector_bridge_pybullet.py`, `sim_connector_bridge_webots.py`,
      and WPILib's `robot.py` (this one uses `shared_state`/`self.sock_lock` instead of
      the other two's shared class shape, but the fix is the same: send `goto_result`
      right where it already logs convergence). ROS Stage's bridge got this same fix
      too, before it was retired shortly after — see the note above.
- [x] **Found and fixed a real thread-safety gap this introduced.** All five bridges'
      `sendLine()` only locked `self.sock`'s *assignment* (connect/disconnect), not the
      actual `sendall()` call — fine when only one thread (the main sender loop) ever
      wrote to the socket, but adding a `goto_result` send from the goto-control
      thread/timer means two threads can now write to the same socket concurrently.
      Moved the lock to wrap the actual `sendall()` inside `sendLine()` itself (rather
      than requiring every call site to remember to acquire it) across all five bridges.
- [ ] No failure path is exercised anywhere yet — every bridge only ever sends
      `success: true`. Worth deciding what "goto failed" even means per simulator
      (unreachable target? timeout? collision?) before wiring a false-case send.
- [ ] **Live verification still open** for the underlying goto_result plumbing itself
      (separate from the thread-safety fix, which is a static-reasoning fix, not
      something that needs a live race to reproduce): send a `goto_position`, confirm
      `cmd_success` flips `True` in `sim/status` once the rover visibly arrives.

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
- [ ] Same for PyBullet.
- [ ] Same for WPILib HAL Sim.

### 5. A decision on old, now-superseded code — **done, removed 2026-08-18**

`rbx_gazebo_node.py`/`rbx_gazebo_discovery.py`/`rbx_gazebo_params.yaml` (the old
`RBXRobotIF`-based Gazebo driver) deleted outright, per explicit sign-off. Confirmed
before deleting: never promoted past `nepi_drones` (no copy in `src/nepi_drivers`), never
deployed to the real device (`/opt/nepi/nepi_engine/lib/nepi_drivers/` has no matching
files), and its default ports (9022/9023) collided with RBX_SIM's own rover-single slot
— a real, if never-yet-triggered, footgun. `docs/ROS_TOPICS_AND_SERVICES.md` updated to
match (driver count, port table, the two-spellings-of-reset note).

---

## Explicitly not next steps (deferred on purpose)

- Multi-robot variants for Webots/PyBullet/WPILib — Gazebo already has one; the
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
4. ~~Decide on `rbx_gazebo_node.py`~~ — done, removed.
5. **Work through the remaining on-device + RUI confirmations** for Webots, PyBullet,
   and WPILib, one at a time — Webots first, per current priority.
6. **Unity, whenever you're ready to personally do the account sign-in** — not blocking
   anything else.
