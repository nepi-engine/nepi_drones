# Multi-Simulator Integration Plan — Webots, ROS Stage, PyBullet, Unity, WPILib HAL Sim

Phased, testable plan to plug five more simulators into NEPI's generic simulator contract
(`nepi_app_sim_connector`), one at a time, each fully installed/tested/debugged before moving
to the next. See `SIM_DEVICE_IF_CONTRACT.md` for the contract itself (data flow, capability
fields, the two-camera convention) — this document is the phased build/status log, not the
contract reference.

All work happens in `nepi_drones` (this repo), not the main `nepi_engine_ws` submodules — see
`SIMULATION_OVERVIEW.md` for why. Environment: the dev VM (`suraj-vm`), the
same one running the existing Gazebo/SITL work — see resource inventory below.

---

## 0. Correcting a premise before planning against it

**Done — see `completed/GAZEBO_SIM_CONNECTOR_INTEGRATION.md`.** That doc covers the
premise-correction (Gazebo was not actually wired into the generic `sim_connector`
contract when this plan started, despite an initial assumption that it was) and the full
Phase 1 writeup. Kept here only as a pointer since Phases 2-6 below build on the same
"one phase per simulator" process that Phase 1 established.

---

## 1. Blocking issue, now resolved: the NavPose "hang"

`docs/completed/SIM_CONNECTOR_NAVPOSE_HANG_BUG.md` originally documented an open, 100%-reproducible hang
inside `NPXDeviceIF` → `NavPoseIF` → `NodeServicesIF`. Investigated 2026-08-07 from two
directions at once (see that doc's full write-up, including its addendum) with a reconciled
answer:

- **On a real, fully-running device, there is no hang at all** — it was a stdout-buffering
  illusion (unflushed `RosStreamHandler` output under a redirected, non-TTY stdout, worsened by
  test scripts `kill -9`-ing a process that looked stuck). Confirmed via a live thread-stack
  dump showing the process already parked cleanly in `rospy.spin()`, and via direct
  `rosservice`/`rostopic` calls against the "stuck" process all responding normally.
- **In a bare/isolated test roscore with no `config_mgr`** (exactly the kind of minimal
  environment this plan's bridge-development phases use before touching a real device), the
  *same* underlying call chain **does genuinely block** — confirmed independently with the same
  stack-dump technique, catching the process stuck for 100+ seconds at
  `nepi_system.get_user_folders()` → `wait_for_param(timeout=1000)`, because nothing in a bare
  roscore ever sets the `user_folders` param a real device's `config_mgr` would provision
  automatically. Pre-seeding that param (and `debug_mode`, a related, already-documented case)
  let a full `SimDeviceIF` construction — `NavPoseIF` included — complete in under 30 seconds.

**Practical takeaway for every phase below:** this was never a code defect blocking real
deployment, but it *is* a real setup step for the kind of bare-roscore bridge testing this plan
does. Before launching anything that constructs a `SimDeviceIF` against a throwaway/isolated
`roscore` (no `config_mgr` running), seed at minimum:

```bash
rosparam set <namespace>/debug_mode false
rosparam set <namespace>/user_folders "{data: /path/to/a/writable/scratch/dir}"
```

Skipping this doesn't break anything long-term, it just means the first `SimDeviceIF`
construction in that test session eats up to ~16.7 minutes (`user_folders`' 1000s timeout)
before proceeding on its own — annoying, not fatal, but seed the params and avoid the wait.

### Phase 0 — Steps (completed)

1. ~~Reproduce (or rule out) the hang on the dev VM~~ — done; see above and the bug doc's
   addendum.
2. ~~Root-cause it~~ — done (missing-param `wait_for_param` block, not a `NodeServicesIF`
   defect).
3. ~~Document the finding~~ — done, in `completed/SIM_CONNECTOR_NAVPOSE_HANG_BUG.md`.
4. **Get `sim_connector_app_node.py` to a confirmed-running state** on the VM, listening on its
   factory port, with `demo_bridge_client.py` able to connect and see `bridge_connected: true`
   in `sim/status` — this is the baseline every later phase's "point a real bridge at it" step
   compares against, and is folded into Phase 1 below since it needs the same param-seeding
   step just established.
   - *Done when:* `rostopic echo .../sim/status` shows `bridge_connected: true` and live
     telemetry with `demo_bridge_client.py --profile rover` running, on this VM.

---

## 2. Environment inventory (checked directly on `suraj-vm`)

| Resource | Value | Implication |
|---|---|---|
| OS | Ubuntu 20.04.6, ROS Noetic | Matches existing Gazebo/rover work |
| CPU | 4 cores | Already contended when Gazebo GUI + bridge + roscore run together (see `feedback-sitl-sim-gotchas`) — expect the same or worse with a second simulator's process tree |
| RAM | 7.8 GiB (4.8 GiB used at idle) | Tight headroom; run one simulator at a time, never two concurrently |
| Disk | 24 GiB free of 59 GiB | **Constrains Unity** (a single Editor install is commonly 10-15+ GiB) — budget disk explicitly before installing it, see Phase 5 |
| GPU | VMware SVGA II (software rendering only, no CUDA/GL acceleration) | Gazebo already runs like this today (documented as slow-but-workable). Webots and Unity both degrade further under pure software rendering — expect low frame rates, not a blocker for headless/physics-only testing |
| Display | `DISPLAY=:0` set, no `Xvfb` installed | GUI apps can run, but install `Xvfb` if we want headless CI-style runs later |
| Existing scratch workspace | `~/sim_connector_test_ws` (catkin, already built once) | Reuse this for `nepi_app_sim_connector` iteration rather than creating a second one |

**Standing rule carried over from the Gazebo work** (`feedback-sitl-sim-gotchas`): always fully
tear down a simulator's whole process tree after testing it — don't leave it running "in case
it's useful," and watch for cleanup traps that `pkill` by bare process name (they can kill an
unrelated `rosmaster`/`roscore` you have running for something else).

---

## 3. The playbook every phase below follows

Each phase (one per simulator) runs these steps in order, and **does not start the next
phase until every step here is genuinely done and debugged** — not just "install completed
without error."

1. **Install.** Package manager where one exists (`apt`, `pip`); note exact versions installed
   in this doc's per-phase section as we go (living doc, update in place).
2. **Standalone smoke test — no NEPI involved.** Get the simulator itself running and doing
   something visible (a robot moving under a scripted or manual command) using only that
   simulator's own tools. This isolates "is the simulator installed correctly" from "is our
   bridge correct" — if step 4 fails, step 2 having passed already rules out half the possible
   causes.
3. **Write the bridge script.** One new small script under
   `sim_container/bridges/<sim_name>/`, following `demo_bridge_client.py`'s shape and the exact
   wire protocol documented at the top of `sim_connector_app_node.py`. Default to **zero ROS
   dependency** where the simulator's own native scripting API can supply everything needed
   (pose, sensor list, motor/goto commands) — this matches the design point `demo_bridge_client.py`
   already establishes and keeps the bridge portable to wherever that simulator actually runs.
   ROS Stage is the one exception (see Phase 3) since its entire interface *is* ROS topics.
4. **Connect the bridge to `sim_connector_app_node.py`.** Point it at the app's listen port
   (assign a new port per the block convention below — don't reuse 902x, that's the
   Gazebo/ArduPilot block). Confirm in this order, each is a real go/no-go gate:
   - `bridge_connected` flips `true` in `sim/status`.
   - Telemetry visibly updates (`rostopic echo .../sim/status`, watch `available_sensor_topics`,
     and via `NavPose` if `getNavPoseCb` is wired — position numbers actually changing, not
     frozen).
   - `available_sensor_topics`/`available_environment_options` populate if the simulator
     reports any.
5. **Add a matching robot config** to `sim_connector_app_params.yaml`'s `SIM_VEHICLE_DICT` (new
   named entry — e.g. `webots_wheeled_robot`), matching that simulator's actual capabilities
   (wheel/motor count, which goto variants make sense, camera view control, environment
   control). Select it via `select_robot_config` and confirm the capability flags in
   `capabilities_query` actually flip to match — this is the point where a wrong flag would
   silently lie to the RUI, so check it explicitly rather than assuming.
6. **Full functional test through the real control surface**, not just topic-level checks:
   - Drive/fly the vehicle via `goto_position`/`goto_pose`/`motor_control` as appropriate and
     visually confirm it moves correctly in the simulator's own viewport.
   - If a camera exists, confirm a live image arrives on the republished `color_2d_image`-style
     topic and is viewable (`rqt_image_view` or similar).
   - Exercise every `setup_action`/`go_action`/environment-option this simulator's config
     declares — a declared-but-untested action is not "done."
   - Exercise the RUI itself if time allows (Devices page rendering the capability-flag-driven
     controls) — this is the actual end-user-visible payoff and the strongest signal that the
     capability flags are correct, not just that the wire protocol is.
7. **Debug to closure, not just to "it mostly works."** Any silent failure, wrong unit
   conversion, or flag mismatch found in step 6 gets fixed and re-verified before moving on —
   matching the standard already set by the rover/ArduPilot work's own bug list (wheel
   acceleration, timeout tuning, per-instance topic collisions, etc. in `SIMULATION_OVERVIEW.md`).
8. **Document.** Add a "Phase N — `<sim>`: done" section here with what was built, the port
   assignments, and any simulator-specific gotchas found — same spirit as
   `SIMULATION_OVERVIEW.md`'s file-by-file walkthrough. Update the status table in §10.
9. **Clean up.** Kill every process this phase started (simulator, bridge script, any roscore
   it needed) before starting the next phase's install — don't let a leftover Webots process
   compete with PyBullet's for the same 4 cores/8 GB.

### Port block for this plan

Existing blocks: `5760`/`5771` (MAVLink), `9021`–`9027` (Gazebo/ArduPilot utility sockets),
`9030` (the `sim_connector_app_node.py` bridge listen port itself — shared by every simulator in
this plan, since they all dial the *same* app node one at a time).

Per-simulator bridges don't need their own inbound port (they're TCP *clients* of 9030), but
several will want a local reset/heartbeat-style utility socket like Gazebo's. New block,
avoiding collision with the existing one:

| Simulator | Utility port(s) | Notes |
|---|---|---|
| Gazebo (new sim-connector bridge) | reuse `9023` conceptually, but as a distinct process from the RBX_SIM path — see Phase 1 | Don't literally share the port with `sim_bridge_node.py` if both might run; pick `9040` if a second listener is needed |
| Webots | `9041` (reset/heartbeat, if needed) | Webots controller likely doesn't need one — it dials out only |
| ROS Stage | `9042` | |
| PyBullet | `9043` | |
| Unity | `9044` | |
| WPILib HAL Sim | `9045` | |

---

## 4. Phase 1 — Gazebo → the new `sim_connector` contract

**Completed — see `completed/GAZEBO_SIM_CONNECTOR_INTEGRATION.md`** for the full goal,
what was built (`sim_container/bridges/gazebo/sim_connector_bridge_gazebo.py`),
verification detail (including real-device confirmation), and what's still open from
this phase (a human visually confirming the RUI page, two individual re-checks, one
minor rate-limit bug) — the last two are tracked in `SIM_CONNECTOR_REMAINING_WORK.md`.

---

## 5. Phase 2 — Webots

**Why second:** open-source, free, no licensing friction, and (unlike Stage) supports full 3D
+ camera, making it the closest analog to Gazebo — the natural next step in difficulty.

1. **Install.** `apt` package `webots` if available in a configured repo, otherwise the
   `.deb` from Cyberbotics' release page downloaded once and installed locally — record the
   exact version. Note: recent Webots releases (R2023a+) primarily target **ROS2**; since this
   VM is ROS1 Noetic, **do not** route through a ROS bridge — use Webots' own native Python
   **Controller API** (`from controller import Robot`) directly, which has zero ROS dependency
   and matches this plan's "prefer no ROS" default anyway.
2. **Standalone smoke test.** Load one of Webots' built-in sample worlds with a wheeled robot
   (e.g. the e-puck or a generic differential-drive sample), run its default controller, confirm
   it moves in the Webots GUI. This validates the install and the display pipeline (software
   rendering, per §2) independent of anything NEPI-related.
3. **Build/adapt a world.** Either reuse a stock wheeled-robot world or build a minimal one
   (differential drive, one camera device) — don't attempt to port the Gazebo SDF model
   directly, Webots uses its own `.wbt`/PROTO format.
4. **Bridge script**, `sim_container/bridges/webots/sim_connector_bridge_webots.py`, run as a
   Webots **Robot controller** (launched by Webots itself, not externally):
   - Reads the robot's `GPS`/`Compass`/`InertialUnit` device nodes (or, if the sample robot
     lacks them, add minimal ones to the PROTO/world) each control step, converts to the same
     bare-telemetry JSON shape as Phase 1.
   - Reads a `Camera` device, JPEG-encodes frames (Webots' `Camera.getImage()` returns raw
     RGBA — needs a numpy/cv2 conversion step the Gazebo bridge doesn't, since Gazebo's camera
     plugin already hands JPEG-able frames via `cv_bridge`).
   - Writes `Motor.setVelocity()` per wheel in response to `motor_control`; implements the same
     turn/drive/turn goto controller pattern as Phase 1 for `goto_position` (Webots physics-time
     stepping is different from Gazebo's — verify the controller's proportional gains still
     converge at Webots' default `basicTimeStep`, retune if the robot overshoots or oscillates).
   - Dials `127.0.0.1:9030` as a TCP client from inside the controller process (Webots
     controllers are plain processes with normal socket access — no special API needed for
     this part).
5. **Robot config:** `webots_wheeled_robot` in `SIM_VEHICLE_DICT`.
6. **Full functional test** per the playbook — including confirming Webots' own step-time
   control doesn't stall or desync the telemetry rate under software rendering.
7. **Document + clean up.**

*Done when:* a Webots-simulated wheeled robot is driveable and viewable through the same RUI
page Phase 1 proved, with no code changes to `device_if_sim.py`/`sim_connector_app_node.py`.

### Phase 2 — Completed 2026-08-07 (VM-side; RUI/on-device confirmation still open)

**Install:** Webots **R2025a** (the current latest release) installs cleanly via its official
`.deb`, but its binary requires `GLIBC_2.34`/`GLIBCXX_3.4.29` — this VM's Ubuntu 20.04.6 only
ships glibc 2.31, so `webots-bin` failed immediately with unresolved symbol errors. Not fixable
locally (upgrading system glibc is not something to attempt). **Webots R2023a installs and
runs cleanly** on this VM instead — removed R2025a, installed R2023a. One unrelated packaging
gap on either version: `libsndio.so.7` was missing even after `libsndio7.0` installed (only
`libsndio.so.7.0` existed, no versioned symlink) — fixed with a manual
`ln -s libsndio.so.7.0 libsndio.so.7` + `ldconfig`. **Takeaway for future VM rebuilds: pin to
R2023a (or verify glibc ≥2.34 first) rather than "always grab latest."**

**World + robot:** built `sim_container/bridges/webots/worlds/sim_connector_rover.wbt`, adapted
directly from Webots' own tutorial 4-wheel robot
(`projects/samples/tutorials/worlds/4_wheels_robot.wbt`) rather than authored from scratch —
same body/wheel geometry and `wheel1`-`wheel4` `RotationalMotor` naming, with `GPS`,
`InertialUnit`, and one `Camera` device added as direct children of the `Robot` node for the
NavPose/image side of the contract.

**Bridge:** `sim_container/bridges/webots/controllers/sim_connector_bridge_webots/
sim_connector_bridge_webots.py` — a genuine Webots **Robot controller** (launched by Webots
itself via the world file's `controller` field), using only Webots' native Python Controller
API plus a plain TCP socket. **This is the first bridge in the plan with zero ROS dependency on
the simulator side at all** — Gazebo's bridge is necessarily a ROS node since that's Gazebo's
own interface; this one proves the contract doesn't require ROS on that end. Reused the exact
same closed-loop goto controller shape as the Gazebo bridge (proportional gains, turn-in-place
gate) — the control law doesn't care which physics engine is underneath.

**Verified end to end**, same rigor as Phase 1:
- Standalone smoke test: Webots ran the world+controller with no crash under this VM's
  software-rendering GPU (`VMware, Inc.` — Webots auto-disabled shadows/AA/ambient occlusion
  and printed a below-minimum-requirements warning, but ran fine at `--mode=fast`).
- `bridge_connected: True`, live telemetry, `available_sensor_topics` announced
  `webots_rover/camera` correctly.
- Reused `ground_robot_2_wheel` unmodified (same as Phase 1) — `capabilities_query` matched
  exactly.
- `goto_position(x=1.0)` drove the rover from `x≈0.0` to `x≈0.95` in real Webots physics,
  confirmed via the live `NavPose` topic (`has_position: True`, `x_m: 0.95`).
- Camera relay confirmed at the **correct** 5Hz (`rostopic hz` measured 4.92-5.03Hz) — unlike
  Phase 1's Gazebo bridge, which measured ~15Hz against the same intended 5Hz cap. Worth a
  side-by-side comparison of the two bridges' sender-loop timing if that discrepancy is ever
  chased down; not blocking either way.

**Known, deliberate gaps** (documented, not oversights — see the bridge script's own
docstring): `RESET` setup action is a logged no-op (this `Robot` node is not a `Supervisor`, so
it cannot teleport itself — making it one would be a legitimate follow-up, not attempted this
pass) and `environment_option` is a logged no-op (this world has no obstacle-course model).
`RETURN_HOME` works normally (drives under closed-loop control, no teleport needed). Only one
camera exists on this world, so `SCENE_CAMERA`/`ROBOT_CAMERA` both resolve to it — acceptable
for proving the contract, not full per-sim feature parity.

**Still open before Phase 2 is fully closed:** RUI-level confirmation on the real device (same
deferral as Phase 1, same reason).

---

## 6. Phase 3 — ROS Stage (`stage_ros`)

**Why third:** genuinely simplest remaining case (2D-only, no camera, native ROS topics) — a
good "prove the contract handles a minimal-capability vehicle honestly" case before the two
higher-risk stretch items.

1. **Install.** `sudo apt install ros-noetic-stage-ros`. Record version.
2. **Standalone smoke test.** Launch one of `stage_ros`'s stock worlds (`willow-erratic.world`
   or similar) via `rosrun stage_ros stageros <world>`, drive the robot manually with
   `rostopic pub .../cmd_vel`, confirm motion in Stage's own 2D viewer.
3. **Bridge script**, `sim_container/bridges/stage/sim_connector_bridge_stage.py` — **this one
   is a ROS node**, not a zero-dependency script, since Stage's entire interface already is ROS
   topics (`/cmd_vel` in, `/odom` + `/base_scan` out) — writing a non-ROS bridge here would mean
   re-implementing a socket layer Stage doesn't need. Subscribes `/odom`, republishes telemetry;
   subscribes nothing extra for sensors — announce `/base_scan` as a `sensor_msgs/LaserScan`
   entry in `sensor_topics` (this is the first non-camera entry in `SCAN_MSG_TYPES` we'll
   actually exercise, worth confirming `has_camera` correctly stays `False` while some other
   sensor-topic-driven UI element still reflects the LaserScan being present).
4. **Robot config:** `stage_ground_robot` — `has_goto_position=True`, no camera capability, no
   `goto_pose`/`goto_location` (Stage is 2D — no meaningful pitch/roll/altitude).
5. **Full functional test:** goto drives the Stage robot; confirm the RUI correctly renders "no
   camera view" state cleanly (this is the interesting case for this phase — most other sims in
   this plan do have a camera, so this is where a camera-optional code path actually gets
   exercised).
6. **Document + clean up.**

*Done when:* a Stage robot is driveable through the RUI with an honestly-reported
no-camera/no-3D capability set — the point of this phase is confirming the contract degrades
correctly, not just that it supports everything.

### Phase 3 — Completed 2026-08-07 (VM-side; RUI/on-device confirmation still open)

**Install:** `sudo apt install ros-noetic-stage-ros` — clean, no version conflicts, no manual
build needed (unlike Webots' glibc issue in Phase 2).

**Standalone smoke test found a real, Stage-specific gotcha, not a config error:** driving the
stock `willow-erratic.world` robot with a single `rostopic pub -1 /cmd_vel ...` barely moved it
(6cm over 3 commanded seconds at 0.3 m/s). Confirmed via `/base_pose_ground_truth` (matched
`/odom` exactly, ruling out odometry noise) that this was real, not measurement error. Root
cause: **Stage decays commanded velocity toward zero if `/cmd_vel` isn't refreshed
continuously** — the opposite of Gazebo's `libgazebo_ros_diff_drive.so`, which latches the last
command indefinitely. Continuous 10Hz publishing moved the robot as expected (less than the
full 0.3 m/s due to Stage's modeled acceleration limit, which is expected/correct, not a bug).
**No design change was needed** — the Gazebo and Webots bridges already re-publish a velocity
command every control tick regardless of idle/active state (for their own, different reasons);
this just confirms that habit is the right general default, not a Gazebo-only caution.

**Bridge:** `sim_container/bridges/stage/sim_connector_bridge_stage.py` — a plain ROS node,
the one bridge in this plan where that's correct by design rather than a fallback, since
Stage's entire interface already is ROS topics (`/cmd_vel`, `/odom`, `/base_scan`). Reused the
same closed-loop goto controller shape as the other two bridges.

**New robot config, not a reuse this time:** `stage_ground_robot`, added to
`sim_connector_app_params.yaml` — genuinely different from `ground_robot_2_wheel`
(`has_camera_view_control: false`, `has_environment_controls: false`), because this world has
neither a camera nor an environment toggle. This is the phase that actually exercises the "a
sensor topic exists but has_camera correctly stays False" path the plan called out as the point
of doing Stage third: `available_sensor_topics` correctly reports
`stage_robot/base_scan` (`sensor_msgs/LaserScan`) while `has_camera`/`available_image_topics`
stay `False`/`[]` — confirmed directly via `capabilities_query`, not assumed.

**Verified end to end:** `bridge_connected: True`, live telemetry, correct capability
degradation (above), and `goto_position(x+1.0)` moved the robot from `x≈-11.82` to `x≈-11.07`
(most of the way to the 1.0m target given the tolerance) with "goto target reached" logged.

**Deliberately not implemented this pass:** actual `LaserScan` data relay over the bridge wire
(only the topic's existence/type is announced — no lidar streaming protocol exists in the
current wire contract, and adding one is a genuine "new capability" change, not a bridge-local
decision) and any setup actions (no reset-to-spawn mechanism was readily available for Stage's
default world without deeper Stage-specific work).

**Still open before Phase 3 is fully closed:** RUI-level confirmation on the real device (same
deferral as Phases 1-2).

---

## 7. Phase 4 — PyBullet

**Why fourth:** pure-Python, pip-installable, no GPU/ROS dependency at all — but genuinely
different physics/rendering pipeline from anything else in this plan, worth its own careful
pass rather than assuming "just like Webots but easier."

1. **Install.** `pip3 install pybullet` (record exact version — check it against Python 3.8,
   confirm no wheel-build issues on this VM's glibc/arch before assuming success).
2. **Standalone smoke test.** Run one of PyBullet's bundled examples (`pybullet.GUI` mode with
   `r2d2.urdf` or similar, driven by a scripted velocity command) and confirm the GUI window
   renders and the robot moves — this also tells us whether PyBullet's own OpenGL path tolerates
   this VM's software-rendering GPU any better/worse than Gazebo/Webots did.
3. **Bridge script**, `sim_container/bridges/pybullet/sim_connector_bridge_pybullet.py` — zero
   ROS dependency, a plain Python script that:
   - Runs the PyBullet physics loop directly (`p.stepSimulation()`), reading back the robot
     body's position/orientation each step for telemetry.
   - Renders a synthetic camera view via `p.getCameraImage()` (returns raw RGBA — same
     JPEG-encoding step as the Webots bridge) if we choose to model a camera-equipped robot;
     otherwise skip camera capability honestly, same as Phase 3.
   - Applies `motor_control`/`goto_position` commands via `p.setJointMotorControl2` (wheeled
     URDF) or direct velocity control, whichever the chosen robot model supports.
   - Dials `127.0.0.1:9030` directly (plain Python socket — this is the simplest bridge in the
     whole plan structurally, closest to `demo_bridge_client.py` but with real physics behind
     it instead of a scripted circle).
4. **Robot config:** `pybullet_wheeled_robot`.
5. **Full functional test** per the playbook.
6. **Document + clean up.**

*Done when:* a PyBullet-simulated robot is driveable/viewable through the RUI with no ROS
process involved on the simulator side at all — useful proof that the contract really doesn't
require ROS on the simulator end, only on the NEPI end.

### Phase 4 — Completed 2026-08-07 (VM-side; RUI/on-device confirmation still open)

**Install:** `pip install pybullet` — clean, ~103MB wheel, no build/compile step, no version
issues (unlike Webots' glibc problem in Phase 2).

**Standalone smoke test found a real technique gotcha, not a bug:** pushing the bundled
`r2d2.urdf` sample robot with `p.applyExternalForce()` barely moved it (a real, physically
modeled wheeled robot resists being shoved around by an external force at its base — sensible
physics, not broken). Switched to `p.resetBaseVelocity()` — directly commanding the base's
linear/angular velocity every step — confirmed to move exactly as commanded (0.5 m/s × 1s =
0.494m measured). This is the technique the bridge's goto controller actually uses.

**Bridge:** `sim_container/bridges/pybullet/sim_connector_bridge_pybullet.py` — pure Python,
zero ROS/`nepi_sdk` dependency, and structurally the simplest bridge in the plan so far: unlike
Gazebo/Webots/Stage, there's no separate simulator process to launch at all — PyBullet is a
library, so this one script's own main loop both steps physics and serves the bridge protocol.
Reused the same closed-loop goto controller shape as the other three bridges.

**Verified end to end**, including a capability Webots' bridge had to leave as an honest gap:
- `bridge_connected: True`, live telemetry, `pybullet_rover/camera` sensor topic announced.
- Reused `ground_robot_2_wheel` unmodified; `capabilities_query` matched.
- `goto_position(x=1.5)` moved the robot from `x≈0.0` to `x≈1.36` (confirmed via live `NavPose`).
- **`RESET` setup action is a real, working teleport here** (`p.resetBasePositionAndOrientation`)
  — confirmed live: position returned to `x≈0.0, y≈0.0` after firing it. PyBullet has no
  Supervisor-style restriction the way Webots does, so this is the first bridge in the plan
  where RESET isn't a documented gap.
- Camera relay (a synthetic chase-cam view via `p.getCameraImage()`, since stock `r2d2.urdf`
  has no onboard camera link) confirmed at the correct 5Hz (`rostopic hz` measured 4.92-4.94Hz).

**Still open before Phase 4 is fully closed:** RUI-level confirmation on the real device (same
deferral as Phases 1-3).

---

## 8. Phase 5 (stretch) — Unity Engine

**Flagged lower-confidence per the scoping discussion for this plan.** Two real risks, not
hypothetical ones, given this VM's actual measured resources (§2):

- **Disk.** A Unity Editor install is commonly 10-15+ GiB per version; this VM has 24 GiB free.
  Installing Unity Hub + one Editor version + a project could consume most or all remaining
  disk. **Gate:** check `df -h` again immediately before starting this phase (other phases will
  have consumed some space already) and get explicit confirmation before installing if headroom
  looks tight — this is exactly the kind of disk-filling action worth pausing on rather than
  attempting silently.
- **Licensing/headless activation.** Unity Personal is free but historically requires signing
  into a Unity ID through the Editor's own UI on first run, which typically needs a real browser
  session — awkward-to-impossible in a VM reached over SSH/VNC without walking through it
  interactively. Confirm whether Unity's current headless/CI activation flow (`unity-hub` CLI
  activation or a manual license file) actually works without a browser before assuming this
  phase is even startable.

1. **Feasibility check first** (before any install): confirm disk headroom and confirm a
   license-activation path that doesn't require an interactive browser login. If either fails,
   stop here, document why in this file's status table, and treat Unity as blocked rather than
   forcing it.
2. **Install** (if feasible) via Unity Hub CLI, smallest supported LTS Editor version, Linux
   Editor target.
3. **Standalone smoke test.** A default empty 3D scene with one primitive (a Capsule or Cube
   standing in for a robot chassis) moved by a simple `Rigidbody.MovePosition` script, confirm
   it runs (Play mode) and moves, under this VM's software-rendering GPU — expect very low
   frame rates; that's fine for a connectivity/protocol test, it doesn't need to be smooth.
4. **Bridge script** — a C# `MonoBehaviour` using `System.Net.Sockets.TcpClient`
   (Unity's own native socket API, zero ROS dependency, mirrors every other zero-ROS bridge in
   this plan) that:
   - Sends the GameObject's `transform.position`/`transform.rotation` as telemetry each
     `FixedUpdate`.
   - Applies incoming `motor_control`/`goto_position` as `Rigidbody` forces/velocity or a direct
     `transform` move, whichever is simpler to get working first (don't over-engineer vehicle
     physics for a connectivity proof).
   - Optionally streams a `Camera.Render()` texture as JPEG if time/performance allows; treat
     this as optional for the stretch phase rather than a hard requirement.
5. **Robot config:** `unity_generic_robot`.
6. **Full functional test** per the playbook, budgeted as best-effort given the performance
   ceiling already flagged.
7. **Document + clean up**, including an honest note on frame rate / usability if this phase
   completes, so a future reader knows whether this was a real working integration or a
   bare-minimum protocol proof.

*Done when:* either a working (however minimal) Unity↔NEPI connection is demonstrated, or a
clear, documented reason this phase is blocked (disk/licensing) is recorded instead of a
half-finished attempt.

### Phase 5 — Blocked at the feasibility gate, 2026-08-07 (as anticipated)

Checked both gates from step 1 before installing anything, per this phase's own instruction to
stop here rather than force it:

- **Disk:** 23 GiB free (after Phases 1-4's installs). Tight but plausibly enough for one
  minimal Editor + Linux Standalone target — not the blocking factor.
- **Licensing — the actual blocker.** Unity Personal requires activating through an interactive
  Unity ID sign-in (browser-based OAuth-style flow) on first run, or an offline manual-activation
  flow that still requires an actual Unity account (create an activation request, upload it via
  a browser to license.unity3d.com, download a `.ulf` response). Neither path is completable
  from this non-interactive session, and creating a Unity account with real credentials on the
  user's behalf is not something to do unilaterally — that's a decision for whoever owns the
  account, not an automatable install step.

**Stopped here rather than installing the multi-GB Unity Editor for something that couldn't be
exercised further afterward.** If this is worth pursuing, it needs the account owner to either
(a) do the interactive first-run activation themselves once the Editor is installed, or (b) hand
over an existing offline-activation `.ulf` file. Everything else in this phase's plan (the C#
`TcpClient` bridge shape, the feasibility-gated approach) still stands as the plan if/when
licensing is unblocked — no code was written this pass since there was nothing to test it
against.

---

## 9. Phase 6 (stretch) — WPILib HAL Sim (FIRST Robotics)

**Flagged lower-confidence.** WPILib's simulation stack (via `robotpy`, the Python
implementation) is built to simulate an FRC robot's *control system* — motor controllers,
encoders, gyros, NetworkTables — not a renderable 3D/2D world with a camera and a NavPose
concept the way Gazebo/Webots/PyBullet/Stage all are. There's no built-in "world" at all;
getting a vehicle that actually moves through space requires writing a custom
`PhysicsEngine` (robotpy's `pyfrc.physics` hook) that integrates simple differential-drive
kinematics ourselves — real code, not configuration.

1. **Install.** `pip3 install robotpy` (installs `wpilib`, HAL simulation bindings,
   `robotpy-wpimath`). Record exact version.
2. **Standalone smoke test.** Write the smallest possible robotpy `TimedRobot` subclass
   (`robot.py`) with two motor controllers on a differential drive, run it via
   `python3 robot.py sim` (robotpy's built-in simulation entry point), confirm the WPILib Sim
   GUI launches and shows motor output values responding to a scripted joystick/command input —
   this alone is the "does the toolchain work at all" gate, independent of any custom physics.
3. **Write a minimal `PhysicsEngine`.** Integrate `x_m`/`y_m`/`yaw_deg` each sim tick from the
   two simulated motor voltages using standard differential-drive kinematics (wheel radius,
   track width — pick arbitrary reasonable values, this is a from-scratch toy robot, not a
   model of anything physical). This is the piece that makes WPILib's otherwise
   world-less simulation produce a NavPose-shaped output at all.
   - *Done when:* running the sim and commanding equal forward voltage on both sides produces a
     monotonically increasing `x_m` (or equivalent) in a debug print, with no camera/vision
     component attempted yet.
4. **Bridge script**, `sim_container/bridges/wpilib/sim_connector_bridge_wpilib.py` — likely a
   small Python thread inside (or alongside) the `robot.py` sim process, reading the
   `PhysicsEngine`'s current pose each tick and dialing `127.0.0.1:9030` directly, same
   zero-ROS shape as the PyBullet bridge.
5. **Robot config:** `wpilib_ground_robot` — `has_camera=False` explicitly (no camera concept
   exists here unless a separate, unrelated vision-simulation add-on is brought in, which is out
   of scope for this phase), motor_count=2, `has_goto_position=True` only if the custom physics
   integration is trustworthy enough to drive a closed-loop goto controller against — otherwise
   scope this phase down to **manual motor control only** and say so explicitly rather than
   wiring a goto controller against physics we don't trust.
6. **Full functional test**, scoped to whatever capability set step 5 actually committed to.
7. **Document + clean up**, with explicit notes on which capabilities were deliberately left
   off (camera, goto) and why, so this doesn't read as an oversight later.

*Done when:* a robotpy-simulated differential-drive "robot" reports honest (likely
motor-control-only) capabilities and responds correctly to commands through the RUI, with a
clear written record of what was intentionally scoped out.

### Phase 6 — Completed 2026-08-07 (VM-side; RUI/on-device confirmation still open)

**Install found a real toolchain incompatibility, worked around, not ignored:** `pip install
robotpy` (current release, 2024.x) failed to build `robotpy-wpiutil` from source —
`gcc: error: unrecognized command line option '-std=c++20'` and `fatal error: span: No such
file or directory`. Root cause: this robotpy release requires C++20 (GCC 10+ / a libstdc++ with
`<span>`), and Ubuntu 20.04 ships GCC 9. **`pip install robotpy==2022.4.8` installs cleanly**
with prebuilt `manylinux` wheels for Python 3.8 — no compilation at all. Pinned to that rather
than chasing a compiler upgrade for a stretch-priority phase.

**No built-in world, as anticipated** — `sim_container/bridges/wpilib/physics.py` is what makes
this produce a NavPose-shaped output at all, using pyfrc's own bundled
`drivetrains.TwoMotorDrivetrain` kinematics helper (reused, not hand-derived) to integrate a 2D
pose from the two simulated PWM motor outputs each physics tick. One real gotcha found here:
`TwoMotorDrivetrain` takes `pint` `Quantity` objects, not plain floats — `0.5` raises
`ValueError: 0.5 must be a Quantity`; fixed with `0.5 * units.meters` /
`0.5 * units.mps` (`from pyfrc.physics.units import units`).

**Bridge:** `sim_container/bridges/wpilib/{robot.py,physics.py,shared_state.py}` — `robot.py` is
launched via `python3 robot.py sim`, WPILib/pyfrc's own simulation entry point, and owns both
the `TimedRobot` lifecycle and the same TCP-client/closed-loop-controller pattern every other
bridge in this plan uses, adapted to run its control tick inside `autonomousPeriodic` (a WPILib
periodic callback) instead of its own timer/thread, since motor outputs only take effect on a
scheduled tick while the robot is enabled. `wpilib.simulation.DriverStationSim` force-enables
autonomous mode at startup (no physical/virtual Driver Station or joystick involved) so periodic
motor commands actually reach the simulated PWM outputs.

**New robot config:** `wpilib_ground_robot` (no camera, no environment control, but a
**real, working** `RESET`) — deliberately not a reuse of Phase 3's `stage_ground_robot`, whose
empty `setup_actions` list reflects Stage genuinely having no reset mechanism. WPILib HAL Sim
has no such restriction (unlike Webots' Supervisor gate) — `physics.py` can call
`self.physics_controller.field.setRobotPose(...)` directly, via a `shared_state.py`
request-flag crossing the `robot.py`/`physics.py` boundary.

**Verified end to end**, including the scope decision to keep `goto_position` (not
motor-control-only, per this phase's own fallback criterion — the physics integration here is a
real, trustworthy kinematic model, not a guess):
- `bridge_connected: True`, live telemetry.
- `capabilities_query` after selecting `wpilib_ground_robot` matched exactly
  (`has_goto_position: True`, `has_camera: False`, `wheel_count: 2`).
- `goto_position(x=1.0)` moved the robot from `x≈0.0` to `x≈0.90` (confirmed via live `NavPose`),
  with "goto target reached" logged.
- `RESET` genuinely teleported the robot back to `x≈0.0, y≈0.0` — confirmed live, not assumed.

**Deliberately not implemented** (see the module docstrings): no camera (no such concept here
without a separate, unrelated vision-sim add-on) and no environment control (nothing to
toggle) — matching this phase's own pre-declared scope, not an oversight.

**Still open before Phase 6 is fully closed:** RUI-level confirmation on the real device (same
deferral as every earlier phase).

---

## 10. Status tracking

Update this table as each phase completes — the living summary of where things actually stand,
so a future session doesn't have to re-derive it from git history.

| Phase | Simulator | Status | Notes |
|---|---|---|---|
| 0 | NavPose hang bug | **Resolved 2026-08-07** | Not a real bug — missing-param `wait_for_param` block in bare test roscores only; see `completed/SIM_CONNECTOR_NAVPOSE_HANG_BUG.md` addendum. Seed `debug_mode`/`user_folders` params before bridge testing |
| 1 | Gazebo (new contract) | **Fully done 2026-08-07, incl. real device** | See `completed/GAZEBO_SIM_CONNECTOR_INTEGRATION.md` for full detail; only a human visually loading the RUI page remains, tracked in `SIM_CONNECTOR_REMAINING_WORK.md` |
| 2 | Webots | **VM-side done 2026-08-07** | R2023a (R2025a needs glibc ≥2.34, incompatible with this VM). Native Controller API, zero ROS. RUI/on-device confirmation still open |
| 3 | ROS Stage | **VM-side done 2026-08-07** | ROS-native bridge (correctly, not a fallback). Found: Stage needs continuous cmd_vel, unlike Gazebo's latching plugin. RUI/on-device confirmation still open |
| 4 | PyBullet | **VM-side done 2026-08-07** | Fully ROS-free on the sim side; RESET actually works (no Supervisor restriction, unlike Webots). RUI/on-device confirmation still open |
| 5 | Unity (stretch) | **Blocked 2026-08-07** | Disk OK (23GB free); licensing needs an interactive Unity ID sign-in or an existing `.ulf` file — needs the account owner, not automatable |
| 6 | WPILib HAL Sim (stretch) | **VM-side done 2026-08-07** | Pinned to robotpy 2022.4.8 (2024.x needs GCC 10+/C++20, unavailable here). goto_position works (not motor-control-only); RESET genuinely works. RUI/on-device confirmation still open |

---

## 11. Deferred / explicitly out of scope for this plan

- Multi-robot variants for any of the new simulators (Gazebo already has one; the others get a
  single-vehicle proof first).
- Any change to `device_if_sim.py`, `sim_connector_app_node.py`, or the RUI components — per
  `SIM_DEVICE_IF_CONTRACT.md`'s stated convention, a new simulator is only ever a new bridge
  script + a new `robot_configs` entry, never a core-contract change. If a phase in this plan
  discovers the contract genuinely can't express something a simulator needs, stop and write up
  the gap rather than patching the shared interface unilaterally.
- Real hardware / non-simulator targets for any of these toolchains (e.g. a physical FRC
  RoboRIO) — explicitly out of scope, matching `SIM_DEVICE_IF_CONTRACT.md`'s own scope note.
