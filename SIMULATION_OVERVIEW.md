# Simulation Work Overview

A from-scratch explanation of everything built in this repo over the past week: two
simulator "products" (a generic Gazebo rover, and an ArduPilot SITL drone), both wired
into real NEPI RBX drivers so they're controllable from NEPI's actual web UI exactly
like real hardware.

Nothing described here has been committed to git yet, and none of it exists in the
production `nepi_engine_ws` submodule — it all lives in this sandbox repo (`nepi_drones`)
for now.

---

## Part 1 — Orientation: what exists, what you can do

Two independent simulated "robots" exist, each drivable end-to-end from NEPI's real web
UI (the RUI) with no special-casing — the UI doesn't know or care that they're
simulations rather than real hardware:

1. **Generic rover sim** — a differential-drive ground rover, simulated in Gazebo,
   built entirely from scratch this week. Comes in a single-robot flavor and a
   two-robot flavor (`rover1`/`rover2` side by side). Controllable via
   `goto_position`/`goto_pose` commands, with a live first-person or third-person
   chase-cam video feed you can switch between as a driver setting.

2. **ArduPilot SITL drone sim** — a pre-existing ArduCopter flight simulator that
   already had full RBX integration (arm/disarm, flight modes, takeoff, goto, goto by
   GPS location) before this week. This week added the same camera-rig feature to it:
   a switchable first-person (gimbal-stabilized) or third-person chase-cam view.

**What you should be able to do, end to end, for either simulator:** run one bash
function to start the whole stack (Gazebo + bridge/discovery machinery + the reverse
tunnel to the real NEPI device), see the device show up automatically in the RUI's
Devices → Robot page, command it from the UI (goto/arm/takeoff), and watch its live
camera feed with a switchable viewpoint.

**Two different wiring approaches, worth understanding up front** because it explains
why the rover needed a lot more custom plumbing than ArduPilot did:

- The **rover** talks to NEPI over a **custom TCP/JSON bridge**. Gazebo's own ROS
  topics live on a separate machine (the dev VM) with its own separate ROS master —
  there's no shared ROS graph with the real NEPI device, so a hand-built bridge was
  necessary.
- **ArduPilot** talks over **mavros/MAVLink**, which is already a cross-machine wire
  protocol by design — mavros just needed the right port forwarded through the tunnel,
  no custom bridge code required.

---

## Part 2 — Concepts primer

- **RBX** is NEPI's device-type category for "robots" — things that move themselves
  (rovers, drones, boats) — as distinct from IDX (cameras), NPX (nav/GPS sensors), PTX
  (pan-tilt heads), LSX (lights).
- **Driver / Discovery / Node / Device**:
  - A **driver** is the whole three-file package for one specific piece of
    hardware/simulator (e.g. "the RBX_SIM driver").
  - **Discovery** is a small, always-running probe. NEPI's `drivers_mgr` calls its
    `discoveryFunction()` every 1–3 seconds to ask "is this thing actually present
    right now?" If yes, and nothing's running for it yet, discovery launches a node.
  - A **node** is the long-running ROS process that actually talks to the
    hardware/simulator continuously and registers with NEPI's standard interface so
    the rest of the platform (including the RUI) can control it uniformly.
  - A **device** is one instance of a connected/simulated robot (e.g. "rover1", "the
    ArduPilot SITL vehicle").
- **`RBXRobotIF`** (`nepi_api/device_if_rbx.py`, shared platform code neither driver
  modifies) is the interface class both drivers instantiate once. It owns all the ROS
  plumbing — status/capability publishing, goto/settings/state/mode command
  subscriptions, the blocking "did the goto converge" wait loop, navpose publishing —
  so each driver only supplies plain Python callbacks. Key defaults worth knowing:
  25-second default command timeout, 640x480-ish 2m/1° default goto tolerance, and a
  default image-input topic name of `color_2d_image` that it automatically searches
  for.
- **The two-machine setup**: a dev VM runs Gazebo/SITL; a separate, real NEPI device
  runs the actual ROS master, `drivers_mgr`, and the RUI. They're connected only by a
  **reverse SSH tunnel** forwarding specific raw TCP ports — never a shared ROS master.
  This is why so much of the rover code exists: anything that needs to cross that gap
  needs its own tiny protocol.
- **The "902x port block" convention**, so port numbers make sense wherever they show
  up: `9021` = Gazebo reset listener (ArduPilot), `9022`/`9023` = rover1
  heartbeat/bridge, `9024`/`9025` = rover2 heartbeat/bridge, `9026` = ArduPilot camera
  bridge. All forwarded through one shared `nepi_tunnel`.

---

## Part 3 — File-by-file walkthrough

### A. Simulation environment side (`sim_container/`)

#### 1. `models/generic_rover/model.sdf` + `model.config`
The simulated rover itself: a differential-drive chassis for Gazebo Classic 11 / ROS
Noetic.
- **Kinematics**: `base_link` (5kg box) + two driven wheels (`left_wheel_joint`,
  `right_wheel_joint`) + two frictionless caster spheres (mu=0) so the driven wheels
  fully determine motion.
- **Camera**: `camera_link`, fixed-jointed to `base_link` at offset (0.2, 0, 0.5),
  640×480, its own `libgazebo_ros_camera.so` plugin → `rover/camera/image_raw`. This is
  the rover's *own* built-in camera — separate from the camera-rig chase-cam feature
  (see #2).
- **Drive plugin**: `libgazebo_ros_diff_drive.so`, namespace `/rover`, subscribes
  `cmd_vel`, publishes `odom` (+TF), wheel separation 0.34m, wheel diameter 0.2m.
- **Gotcha found and fixed**: `wheelAcceleration` is deliberately `0.0` — a nonzero
  ramp made the rover crawl at ~1/10 commanded speed under this world's physics tuning.

#### 2. `models/camera_rig/model.sdf` + `model.config`
A standalone, camera-only Gazebo model used purely as a movable "chase-cam" — not
attached to the rover's (or drone's) kinematic tree at all.
- `<static>true</static>`: never falls, never reacts to collisions. It's teleported
  every frame by an external Python controller (see #8, #9).
- Repositioning is done via the **`/gazebo/set_model_state` topic** — confirmed by a
  direct test (spawn a probe model, drive it at a few Hz, watch for jitter) to move
  smoothly with no snap-back, unlike a fixed SDF joint, which would fight the physics
  solver.
- Reused unmodified by three different controllers: the single-robot rover, the
  multi-robot rover, and ArduPilot.

#### 3. `worlds/generic_rover.world`
The single-robot rover world (default `sim_rover_gazebo` workflow).
- Physics tuning deliberately differs from the ArduPilot world's: contact constraints
  are left at Gazebo defaults (`contact_max_correcting_vel=100`,
  `contact_surface_layer=0.001`) rather than the drone world's tuned values — those
  drone-tuned values would let a wheeled vehicle sink into the ground and never grip.
- `<include>`s `model://generic_rover` and `model://camera_rig` (spawned off to the
  side at first; the camera controller moves it into position once it sees odometry).

#### 4. `worlds/generic_rover_multi.world`
The two-robot variant (`rover1`/`rover2`), for `sim_rover_gazebo_multi`.

**The single most important design fact in this file**: the rover model XML is
**inlined twice**, not `<include>`d twice. This is a real, empirically-confirmed Gazebo
Classic 11 limitation that came up **three separate times** across this week's work:

> Gazebo Classic 11's `<include>` cannot override plugin parameters per instance — an
> `<include>`-level `<plugin>` block gets *appended*, not merged/substituted. Two
> `<include>`s of the same model produce two colliding plugin instances fighting over
> the same topics (confirmed directly: an SDF error, "Non-unique names detected in type
> plugin," for the diff-drive case; and worse, a *silent* collision for the camera_rig
> case — two `<include>`s of `camera_rig` produced only **one**
> `/camera_rig/camera/image_raw` topic feeding both instances, not two, since
> `<cameraName>`/`<frameName>` are literal strings baked into the model file, untouched
> by `<include>`'s own `<name>` override).

The fix, used consistently everywhere two instances of the same model are needed: full
inline duplicate `<model>` blocks with the plugin parameters hand-edited per instance
(`robotNamespace` `/rover1`/`/rover2`, `cameraName` `rover1/camera`/`rover2/camera`,
etc.). `publishOdomTF` is also turned off here (unlike the single-robot world) since two
robots would otherwise publish conflicting `odom→base_link` transforms.

#### 5. `scripts/sim_heartbeat_listener.py`
A tiny plain-TCP (not ROS) liveness probe, default port 9022.
- Listens on `127.0.0.1:<port>`, replies `ALIVE\n` on every connection, closes it.
- **Deliberately not a ROS node**: the remote NEPI device can't see this VM's ROS graph
  at all, so discovery can't check a ROS topic — it probes this raw socket through the
  reverse tunnel instead.
- The `ALIVE` reply is what actually matters, not just a successful connection — with
  `ssh -R` forwarding, a bare `connect()` can succeed against the far end's `sshd` even
  when nothing is listening on the VM. Reading the real reply is the only way to prove
  true liveness. This exact gotcha bit the project once and is now baked into the
  design.

#### 6. `scripts/sim_bridge_node.py`
The rover sim's ROS-side entry point for single-robot mode: publishes liveness, relays
velocity commands into Gazebo, and serves the TCP bridge the remote device's
`rbx_sim_node.py` talks to.
- `/sim/heartbeat` at 1Hz on a **wall-clock thread**, not a `rospy.Timer` — a ROS timer
  tracks sim time and would falsely read as "dead" if the sim is paused or running at a
  non-1x rate.
- Relays `/nepi/sim/cmd_vel` → `/rover/cmd_vel`.
- **TCP bridge server on `127.0.0.1:9023`**: one persistent client, newline-delimited
  JSON both directions. In: velocity commands or camera-settings messages. Out:
  odometry telemetry at 10Hz, plus (added this week) base64-JPEG camera frames relayed
  from `camera_rig_controller.py`.
- Camera settings received over the bridge are stored as plain ROS params
  (`/sim/camera/view_mode`, `offset_x/y/z`) — the only handoff mechanism available
  since this VM has zero visibility into the remote device's process memory.
- Socket `accept()` timeout is explicitly cleared (`settimeout(None)`), overriding a
  gotcha from `rospy.init_node()` (see the driver section below) — commands are
  legitimately sporadic and a recv timeout must not be mistaken for the client dying.

#### 7. `scripts/sim_bridge_multi_node.py`
The two-robot counterpart — one independent `RobotBridge` instance per robot slot in a
single process (`rover1`: bridge port 9023, reusing the single-robot port; `rover2`:
bridge port 9025). Camera settings go to a **distinct param namespace per robot**
(`/sim/camera/rover1/*` vs `/sim/camera/rover2/*`) so one robot's setting changes never
leak into the other's. Deliberately a separate file from the single-robot version, not
a generalization of it — zero regression risk to the already-verified single-robot
path, and only one of the two workflows is ever launched at a time.

#### 8. `scripts/camera_rig_controller.py` / `camera_rig_controller_multi.py`
Make the standalone `camera_rig` model track the rover in first-person or chase-cam
view, and produce the compressed image stream the bridge relays onward.
- **One position formula serves both view modes**: `target = rover_position + offset`,
  rotated into the rover's current heading (so the offset stays correct as the rover
  turns). Only the *orientation* logic differs between modes: FIRST_PERSON faces
  forward (same yaw as the rover); THIRD_PERSON computes a real look-at (yaw *and*
  pitch) back toward the rover.
- Runs a 20Hz pose-follow loop (fast/smooth, since it's purely visual) and a separate
  7Hz image loop (JPEG quality 60, via `cv2.imencode` — done here rather than in the
  bridge node because `ros-noetic-compressed-image-transport` isn't installed on this
  VM and this node is already OpenCV-facing).
- Not auto-started by any bash function — must be launched manually in a separate
  terminal/screen session after the base sim is up.
- `camera_rig_controller_multi.py` is the same idea, running one instance per robot
  slot in a single process, mirroring the bridge node's per-slot pattern, each reading
  its own robot's odom and its own param namespace.

#### 9. `scripts/camera_rig_controller_ardupilot.py`
The ArduPilot port of the same feature, adapted for a genuinely different vehicle.
- Reads pose from `/gazebo/model_states` (filtered for model `iris_demo`) rather than
  an odom topic — ArduPilot SITL has no native ROS odom on this VM.
- **A real design decision, not a copy-paste**: since a drone moves in full 3D (unlike
  the planar rover), FIRST_PERSON here is **yaw-only, gimbal-stabilized** — the camera
  stays level regardless of the airframe's roll/pitch, matching how real commercial
  inspection/survey drones use 3-axis gimbals (not FPV racing rigs, which would be
  rigidly slaved to the full airframe attitude). THIRD_PERSON extends the rover's
  look-at math to true 3D (real altitude delta included).
- Because ArduPilot has no equivalent of `sim_bridge_node.py` (MAVLink already carries
  telemetry/commands), this file runs its **own** minimal TCP JSON-lines server
  directly on port 9026, combining both roles the rover split across two files.

#### 10. `scripts/nepi_sitl_dev_env.sh` and `scripts/sim_rover_dev_env.sh`
The bash function libraries (sourced from `~/.bashrc`) that actually start/stop
everything.

**`sim_rover_dev_env.sh` (rover workflow):**
| Function | What it does |
|---|---|
| `sim_heartbeat_listener [port]` | Starts the heartbeat listener (default 9022) if not already running. |
| `sim_rover_gazebo` | One-command launcher: local roscore (if needed) → Gazebo with `generic_rover.world` → heartbeat listener → `nepi_tunnel` → `sim_bridge_node.py` in the foreground. Ctrl-C tears everything down. |
| `sim_rover_gazebo_multi` | Same shape, `generic_rover_multi.world`, two heartbeat listeners, `sim_bridge_multi_node.py`. Mutually exclusive with `sim_rover_gazebo` (shared ports). |

Neither camera controller script is auto-started — always a manual second-terminal
launch.

**`nepi_sitl_dev_env.sh` (ArduPilot workflow):**
| Function/alias | What it does |
|---|---|
| `nepi_gazebo` | Plain `gazebo` on the ArduPilot world, no ROS (legacy alias). |
| `nepi_sitl`/`sitl` | Launches `sim_vehicle.py` against a manually-started Gazebo. |
| `gz_reset_listener [port]` | Tiny listener (default 9021); on connection, does a *pose-only* Gazebo reset (`gz world -o`, deliberately not a full time reset, which would crash the connected SITL FDM socket). |
| `nepi_tunnel` | Idempotent `autossh` reverse tunnel (self-reconnecting, unlike plain `ssh`) forwarding every port this whole project uses — one tunnel serves both simulator workflows. |
| `sitl_gazebo` (alias `gazebo_sitl`) | One-command launcher: local roscore (a **new requirement this week** — the camera plugin and `/gazebo/model_states` both need the `gazebo_ros` API plugin, which needs a roscore; this workflow never used ROS before) → Gazebo via `rosrun gazebo_ros gazebo` → `gz_reset_listener` + `nepi_tunnel` → SITL in the foreground with a dedicated `--out=tcpin:0.0.0.0:5771` port so mavros can connect alongside MAVProxy. |
| `camera_rig_controller_ardupilot` | Manual launch, separate terminal. |

#### 11. `~/ardupilot_gazebo/worlds/iris_arducopter_cmac.world` + `models/iris_with_ardupilot/model.sdf`
The pre-existing ArduPilot drone model/world, living **outside this repo** on the dev
VM (same as the rest of the ArduPilot toolchain always has). This week added one new
`<include>model://camera_rig</include>` to the world file — a single instance, no
per-instance-namespacing concern here since ArduPilot SITL is single-vehicle by
construction. The drone model itself (593 lines) has four rotor pairs, an
`ArduPilotPlugin` FDM socket (ports 9002/9003 — a different range from the 902x
sim-utility block, don't confuse the two), and no camera of its own before this week.

---

### B. NEPI driver integration side (`src/nepi_drivers/rbx_drivers/`)

#### 12. RBX_SIM — `rbx_sim_params.yaml` / `rbx_sim_discovery.py` / `rbx_sim_node.py`

- **`rbx_sim_params.yaml`**: declares the driver's identity (`pkg_name: RBX_SIM`,
  `type: RBX`, `display_name: "Gazebo Simulated Robot"`), wires the node/discovery
  files together by name, and sets `process: CALL` (discovery runs as an in-process
  Python call, the standard pattern for every NEPI driver). Only one connection
  option: `SIMULATOR`.
- **`rbx_sim_discovery.py`**: probes the heartbeat port and reads back the real
  `ALIVE` reply (not just connection success — see #5's gotcha) for each of two
  hardcoded robot slots (`SIM_ROBOT_SLOTS`: rover1 on 9022/9023, rover2 on 9024/9025).
  On a live detection, launches one `rbx_sim_node.py` process per slot, passing device
  name/host/ports through the ROS param server.
- **`rbx_sim_node.py`**: holds a persistent TCP client to the bridge, sending velocity
  commands and receiving telemetry/images. **Because the rover has no onboard
  autopilot, this file implements its own goto controller** — turn to face the
  bearing, drive, final turn to the requested yaw, at 10Hz, using proportional gains.
  Exposes `gotoPosition`/`gotoPose` (no `gotoLocation` — no GPS reference); states,
  modes, and actions are all deliberately empty lists (no ARM/battery/flight-mode
  concept for a plain rover).
  - **Real bug found and fixed**: `RBXRobotIF`'s default 25-second command timeout was
    too short for a non-holonomic rover, which can legitimately need close to two
    ~180° turns plus travel time for one goto — a 6m goto once logged a false timeout
    even though the rover's own controller had actually converged ~2.5s earlier. Fixed
    by raising the timeout to 60s specifically for this driver.
  - **Real bug found and fixed (multi-robot phase)**: the camera image topic name
    RBXRobotIF searches for (`color_2d_image`) resolves against a namespace **shared
    by every driver node on the device**, not each node's own sub-namespace — with two
    rover instances running, both would publish to and read from the exact same
    topic, cross-talking their camera feeds together. Fixed by qualifying the topic
    name with each instance's own device name (`sim_rover1/color_2d_image` vs.
    `sim_rover2/color_2d_image`).
  - Camera settings: `camera_view_mode` (FIRST_PERSON/THIRD_PERSON) and
    `camera_offset_x/y/z`, following the same live-settings pattern as everything
    else on this driver.

#### 13. RBX_ARDUPILOT — `rbx_ardupilot_params.yaml` / `rbx_ardupilot_discovery.py` / `rbx_ardupilot_node.py`

- **`rbx_ardupilot_params.yaml`**: `pkg_name: RBX_ARDUPILOT`, connection menu of
  `SERIAL` (real hardware) or `SITL`, plus a `fake_gps` toggle (useful for real
  hardware indoors; auto-disabled for SITL since ArduPilot SITL simulates its own GPS).
- **`rbx_ardupilot_discovery.py`**: handles serial/TCP/UDP/SITL connection types in one
  function. For SITL, probes **only port 5771** (not both 5760 and 5771) — a
  deliberate choice: MAVProxy exposes two MAVLink ports, and probing both would make
  discovery see one "die" every cycle as mavros claims the other, thrashing the node
  in an endless kill/relaunch loop. On detection, launches both a `mavros_node`
  process (the actual MAVLink-to-ROS bridge) and the `rbx_ardupilot_node.py` RBX node.
- **`rbx_ardupilot_node.py`**: talks entirely through mavros (no direct TCP bridge like
  the rover) — waits for mavros's `state` topic, calls mavros services for
  arm/mode/takeoff/commands. Much richer RBX surface than the rover since there's a
  real autopilot underneath: states `DISARM`/`ARM`, modes
  `STABILIZE`/`LAND`/`RTL`/`LOITER`/`GUIDED`/`RESUME`, actions
  `TAKEOFF`/`LAUNCH`/`RESET_SIM`, and all three goto variants including
  `gotoLocation` (real GPS lat/lon/alt).
  - `RESET_SIM` is a SITL-only convenience: force-disarms, then pings the VM's
    `gz_reset_listener.py` (port 9021) to teleport the sim back to its spawn pose.
  - **Camera bridge is a second, fully independent connection** (port 9026) rather
    than piggybacking on mavros — MAVLink already carries telemetry/commands, so this
    is purely the missing camera channel, added this week.
  - A subtle design note: `manualControlsReady()` always returns `True` because
    ArduPilot's own motor-test command auto-arms the flight controller as a side
    effect — gating on "must be disarmed" here would create a self-inflicted deadlock
    where testing one motor locks out every other motor command until the test times
    out. ArduPilot's own internal safety checks are relied on instead.
  - Also surfaces the flight controller's own status-text messages (e.g. "Arm:
    Compass not healthy") into the RBX error message, so a failed command shows the
    real reason in the UI instead of a generic timeout.

**A gotcha both drivers' bridge/socket code has to work around**: `rospy.init_node()`
sets a process-global 60-second default socket timeout that silently applies to *any*
plain `socket.socket()` created afterward — including ones a node `accept()`s. A
long-idle raw connection can see `recv()` time out and be misread as the peer dying.
Both drivers set their own explicit, shorter timeouts on their sockets to sidestep
this. Documented in `src/nepi_drivers/CLAUDE.md`'s Known Constraints.

---

### C. Planning docs (historical context, not current-state truth)

#### 14. `UNIVERSAL_SIMULATOR_IMPL_PLAN.md`
The original 5-phase plan for the rover simulator effort. Contains a notable
self-correction: the first draft assumed NEPI and the simulator share one ROS master
over LAN; the real environment turned out to be two fully separate machines connected
only by a raw-TCP reverse SSH tunnel, which is why the custom bridge exists at all. The
code samples in this doc are early drafts — the actual built code diverged
substantially (bridge protocol, camera relay, multi-robot slots, threading details).
Treat it as intent/history, not documentation of what's actually running.

#### 15. `docs/SIMULATOR_DEV_GUIDE.md`
An earlier, ArduPilot-only task guide (predates the rover work entirely) covering the
original mavros/RBX hookup, autonomous flight through the RUI, and motor controls. It
explicitly scoped out cameras ("no cameras, no lights") — that scope note is now
superseded by this week's camera-rig work. Doesn't cover the rover/`sim_container` side
at all.

---

## Part 4 — Open threads

- **Nothing has been committed to git yet** — everything described above is
  uncommitted/untracked in this repo.
- One proposed (not applied) `CLAUDE.md` Decision Log entry is awaiting your call: a
  note that the ArduPilot SITL workflow now requires a roscore (previously
  MAVLink-only, permanently changed by the camera-rig work).
- An earlier, still-unresolved finding from the persistence testing phase: one deployed
  driver file didn't survive a real container-restart cycle on the remote device once,
  and the root cause was never confirmed. Worth re-checking file integrity (a quick
  md5sum comparison) after any future `nepicommit`, rather than assuming it's safe.
