# Phone Scan → Simulated Environment: Design & Implementation Plan

*Design plan only — no code written yet. Covers converting a phone LiDAR/IMU/video scan
of a real obstacle course into a simulated environment the rover sim can load, so an FRC/
FTC team can test their autonomous-challenge course without hand-authoring a world file.
Revised after catching up on ~76 commits of unrelated sim-connector/Webots/ArduPilot work
that landed since this plan was first drafted — see §2 for what changed and why it matters
here.*

Grounded in:

- Raw scan data: `nepi_office_strayscan/779206be34/` (this repo, uncommitted — see §8)
- RBX-level environment mechanism (static, older): `src/nepi_drivers/rbx_drivers/rbx_sim_node.py`,
  `sim_container/scripts/sim_bridge_node.py`
- App-level environment mechanism (dynamic, current work-in-progress):
  `src/nepi_apps/nepi_app_sim_connector/api/device_if_sim.py`,
  `sim_container/bridges/gazebo/sim_connector_bridge_gazebo.py`
- Current obstacle model: `sim_container/models/obstacle_course/model.sdf`
- Upload precedent: `src/nepi_apps/nepi_app_sim_connector/rui/Nepi_IF_Sim.js` ("Upload
  Robot Config")
- Status/architecture docs (read in full for this revision):
  `docs/SIM_DEVICE_IF_CONTRACT.md`, `docs/SIM_CONNECTOR_REMAINING_WORK.md`,
  `docs/MULTI_SIMULATOR_INTEGRATION_PLAN.md`, `docs/SIMULATOR_AUTO_LAUNCH_PLAN.md`,
  `docs/WEBOTS_RBX_DRIVER_PLAN.md`, `docs/completed/GAZEBO_SIM_CONNECTOR_INTEGRATION.md`

---

## 1. Problem

Today, testing an autonomous-obstacle-course routine in simulation means someone
hand-authors a Gazebo world/model (as was done for `obstacle_course/model.sdf` — hand-placed
boxes for chicane walls and a ramp). For an FRC/FTC team with a real practice field, that's
a real barrier. The ask: let them scan the real course with a phone (Stray Scanner app on
iPhone — LiDAR depth + IMU + RGB video + pose) and have that scan auto-convert into a
matching simulated environment, surfaced as a new "upload phone scan" action near the
existing Environment control.

## 2. What changed since this plan was first drafted (read this before §3+)

A large amount of unrelated work (76 commits) landed in this repo in the interim: a
generic multi-simulator "sim_connector" app, Webots RBX drivers (rover + quadcopter, fully
live-verified), an ArduPilot SITL quadcopter, camera-view rework, auto-launch tooling, and
more. Two corrections to this plan's original assumptions fell out of catching up on that
work:

### 2.1 There are two parallel obstacle-course mechanisms, not one live + one dead

The original draft of this plan found a "live" RBX-level mechanism and a "dead" app-level
one and treated the choice between them as open. Rereading `SIM_CONNECTOR_REMAINING_WORK.md`
corrects that:

- **RBX-driver-level (older, static)**: `rbx_sim_node.py`'s own `environment` Setting
  (`Discrete`, options `["FLAT_GROUND","OBSTACLE_COURSE"]`), dispatching to
  `sim_bridge_node.py`'s `setObstacleCourseAction`/`setObstacleCourse`. The option list is
  passed once into `RBXRobotIF.__init__` (`capSettings`) with no update path found anywhere
  in `device_if_rbx.py` — growing it means restarting the driver.
- **App-level (current, dynamic — NOT dead)**: `device_if_sim.py`'s
  `available_environment_options` / `setEnvironmentOptionFunction` / `has_environment_controls`,
  wired through `sim_connector_bridge_gazebo.py` to the *same* `obstacle_course/model.sdf`.
  This is genuinely live, generic-contract machinery, explicitly designed to be
  live-refreshed rather than frozen at construction (`refreshEnvironmentOptions()`) — and
  it's the **#1 item on `SIM_CONNECTOR_REMAINING_WORK.md`'s current punch list**: a recent
  fix corrected an enable/disable bug (the Gazebo bridge was silently dropping `enabled=False`),
  live verification is still pending. This is active, current work, not a dead path.
- **A real bug in that path worth not repeating**: the environment's on/off state is
  currently **client-side only** — no `SimStatus` field reports the server-side active
  state, so a page reload always shows "off" regardless of what's actually spawned. Any new
  scanned-environment feature must report which environment is active server-side, not
  repeat this gap.

This firms up §6's recommendation below from "open decision" to a clear call: build on the
app-level mechanism, since it's both the architecturally-intended dynamic-list path *and*
already mid-hardening for exactly this use case.

### 2.2 Webots has no obstacle-course model at all

Confirmed via the remaining-work doc: Webots' environment options report empty — there is
no obstacle-course equivalent ported there yet. This reinforces §4's original
Gazebo-first recommendation; for Webots, scan-to-sim would be new territory on both ends
(no existing model *and* no existing spawn pattern), not just a port.

### 2.3 No asset/world-upload mechanism exists anywhere, confirmed

`SIMULATOR_AUTO_LAUNCH_PLAN.md`'s own scope statement: it launches simulators/worlds
**already installed** on the dev VM and explicitly does **not** provision new content
("does not provision a VM from nothing"). Nothing else in the codebase does this either.
The upload/conversion pipeline in this plan is genuinely new territory — but the auto-launch
feature's existing SSH plumbing (`NEPI_SSH_KEY` env var, no committed credentials) is real,
working infrastructure worth reusing for getting the scan and its converted output onto the
dev VM, rather than inventing a second remote-access mechanism (see §7 Phase 1).

### 2.4 The current uncommitted WIP doesn't touch any of this

A camera-view six-topic expansion is currently uncommitted in this repo
(`sim_bridge_node.py`, `rbx_sim_node.py`, `sim_connector_app_node.py`, camera-rig scripts).
Confirmed by diff review: it does not touch the `environment` Setting, obstacle-course
spawn logic, or the port-9030 mechanism anywhere. Nothing here conflicts with this plan.

## 3. The scan data itself (unchanged — Stray Scanner export, confirmed by inspection)

Location today: `nepi_office_strayscan/779206be34/` (uncommitted, at the `nepi_drones` repo
root — see §8 for where it should move).

| File / folder | Format | Notes |
|---|---|---|
| `rgb.mp4` | HEVC video, 1920x1440, 30fps, 1444 frames (~48s) | Main color stream |
| `depth/NNNNNN.png` | 16-bit grayscale, 256x192, 1444 frames | LiDAR depth in millimeters (native ARKit resolution, much lower than RGB) |
| `confidence/NNNNNN.png` | 8-bit grayscale, 256x192, 1444 frames | Per-pixel depth confidence (0/1/2, low/med/high) |
| `odometry.csv` | CSV, 1444 rows | Per-frame `timestamp, frame, x, y, z, qx, qy, qz, qw, fx, fy, cx, cy` — authoritative pose + per-frame intrinsics (ARKit VIO, not ground truth) |
| `imu.csv` | CSV, 4779 rows (~100Hz) | Raw IMU; not needed for v1 (pose already comes fused from `odometry.csv`) |
| `camera_matrix.csv` | 3x3, single averaged matrix | Legacy/back-compat only — `odometry.csv`'s per-frame intrinsics are authoritative |

No pre-computed point cloud or mesh ships with the export — reconstruction has to be
derived from depth + pose + confidence.

## 4. Reconstruction pipeline

Reference implementation to adapt (not reinvent): **`kekeblom/StrayVisualizer`**
(`stray_visualize.py`), the closest thing to an established pipeline for this exact Stray
Scanner format — confirmed to already: parse `odometry.csv` into per-frame poses, scale
intrinsics to the 256x192 depth resolution, zero out low-confidence depth pixels, feed
RGB-D + pose into Open3D TSDF integration (`ScalableTSDFVolume`/`VoxelBlockGrid`), and
extract a mesh via marching cubes.

Making the mesh simulation-safe needs two separate outputs, per Gazebo's own documented
best practice (decouple visual and collision geometry):

- **Visual mesh**: cleaned (drop small disconnected components, fix normals), decimated to
  a fixed triangle budget via Open3D's `simplify_quadric_decimation`.
- **Collision mesh**: Gazebo's physics engines are unreliable against raw concave/
  non-manifold meshes, which is exactly what marching-cubes output will be. Fix: convex
  decomposition (V-HACD, or CoACD for fewer/tighter hulls) on the decimated mesh, producing
  N convex hulls. `gizatt/convex_decomp_to_sdf` is a real, working reference for the
  "mesh → V-HACD via trimesh → SDF with N convex `<collision>` elements" pattern.

**Accuracy caveats to surface to the user up front, not hidden in fine print**: iPhone
LiDAR depth is accurate to roughly ±1-3cm under 3m, degrading past that. ARKit odometry is
VIO **without loop closure** — a long single walkthrough will show cumulative drift by the
far end. Practical mitigation to recommend to the scanning team: walk a loop back near the
start point before stopping the recording. This plan assumes "accept best-effort geometry
for v1" rather than adding an offline bundle-adjustment pass, unless told otherwise.

## 5. Simulator choice: Gazebo only, for now

Evaluated Isaac Sim, Unity+ROS-TCP-Connector, Webots, and CoppeliaSim as alternatives:

- **Isaac Sim** has an actual smartphone-scan-to-USD workflow, but it's explicitly
  **render-only geometry with no inherent collision** — wrong tool for a rover that needs
  to physically hit obstacles — and it's on a stated ROS1-deprecation path.
- **Unity** and **Webots** hit the same concave-mesh-needs-convex-decomposition problem
  Gazebo has, with a less-native ROS1 story.
- **CoppeliaSim** has built-in automatic convex decomposition on mesh import — a genuinely
  nice feature Gazebo lacks natively — but not decisive enough to justify a second
  simulator investment.

**Gazebo first.** It's the one fully-closed sim_connector integration
(`docs/completed/GAZEBO_SIM_CONNECTOR_INTEGRATION.md`), and §4's decimate-then-convex-decompose
step routes around its lack of CoppeliaSim-style auto-decomposition. Per §2.2, Webots has
no obstacle-course precedent at all yet — revisit it only after that gap is closed
independently of this feature.

## 5.5 Update mid-implementation: a raw-SDF-upload mechanism landed concurrently

While implementing Phase 2, a concurrent session added a real, working
"editable dimensions" system for `generic_rover`/`obstacle_course`:
`sim_container/scripts/generate_model_sdf.py` renders `model.sdf` from a
curated `dimensions.yaml`, and `sim_connector_app_node.py` now has a
`sim/upload_environment_model_sdf` topic (`uploadModelSdfCb` -> `pushDirtyDimensions`
-> `simulator_launcher.py`'s `push_dimensions`) that lets a raw SDF string
bypass generation entirely -- explicitly built as "the escape hatch for
geometry the curated fields don't cover," which a scanned mesh environment is
a textbook case of. Mechanically: the device (where `sim_connector_app_node.py`
runs) is the authoritative store; `push_dimensions` SSHes a `cat > model.sdf`
into `sim_container/models/<model_name>/` on the VM ahead of the next Launch
(`_push_file_content` in `simulator_launcher.py`) -- the same SSH channel the
auto-launch feature already uses, exactly the reuse this plan called for in
§9 Phase 1.

This changes the integration recommendation in §6/§7 below: rather than
building a new dynamic-named-options mechanism from scratch, the better fit
is to **extend this already-working push mechanism to also carry the mesh
files** a scanned model needs (today it only pushes `model.sdf`/`dimensions.yaml`
text, since that's all `generic_rover`/`obstacle_course` ever needed). Two
narrower, more concrete changes fall out of that, in place of §6/§7's original
plan:

1. `push_dimensions`/`_push_file_content` only handles small text files over
   `cat`. A scanned model's mesh files (a few MB after decimation) need a
   binary-safe push (e.g. a new `_push_binary_file`/`scp` helper) alongside
   the existing text one -- not a replacement for it.
2. `generate_model_sdf.py`'s `<model_name>` is currently a closed set of two
   (`generic_rover`, `obstacle_course`). Rather than fighting that, run
   `scan_to_environment.py` **on the VM directly** (reusing
   `simulator_launcher.py`'s existing `_run_remote`, the same way it already
   remote-invokes `generate_model_sdf.py`) so the output lands straight in
   the VM's own `sim_container/models/<scan_name>/` -- no push-back needed for
   the generated model itself, only the raw scan upload needs to travel
   device -> VM.

This is a live-editing area right now (confirmed: `rbx_sim_node.py`,
`sim_bridge_node.py`, and `sim_connector_app_node.py` all changed within
hours of this plan being written, in this exact subsystem). The rest of this
plan (§6 on) is kept as the technical reference for the environment-toggle
mechanism as understood at the time it was written, but **the concrete next
implementation step is the two changes above, into `simulator_launcher.py` and
a new remote-conversion trigger** -- not the dynamic-options-list design
originally sketched below, which this newer mechanism supersedes for the
"get a new environment onto the VM" half of the problem.

## 5.6 Correction: rbx_sim_node.py cannot local-scan for models -- it runs on the device

`rbx_sim_node.py` runs on the NEPI **device** (the Pi); `sim_container/models/`
and `environment_models.py` live on the dev **VM** -- two separate machines,
bridged only by `sim_bridge_node.py`'s TCP socket. `self.cap_settings` is
snapshotted once at `__init__` (`self.cap_settings = self.getCapSettings()`,
`rbx_sim_node.py` line 474) from the class-level `CAP_SETTINGS` dict -- so
even fixing the "runs on the wrong machine" problem, this is still a
construction-time snapshot, not a live-refreshing list (consistent with
§2.1's earlier finding that `device_if_rbx.py` has no capSettings refresh
path at all).

The correct shape of this piece, not yet implemented: at driver startup,
`rbx_sim_node.py` asks the VM (a new message over the existing bridge socket,
e.g. `{"type":"get_environment_options"}` in / `{"type":"environment_options","options":[...]}`
out -- `sim_bridge_node.py` answers using `environment_models.list_environment_models()`)
for the current model list, with a short timeout and a safe fallback to
`["FLAT_GROUND","OBSTACLE_COURSE"]` if the VM isn't reachable yet -- the same
"best-effort, don't block startup" instinct every other cross-machine call in
this codebase already follows (see `pushDirtyDimensions`'s own comment).
Dispatch also needs to change from `setObstacleCourseAction(enabled: bool)`
(one hardcoded model, on/off) to a by-name send (`{"type":"environment","model_name":...}`),
with `sim_bridge_node.py`'s handler switched from its dedicated
`setObstacleCourse` method to `environment_models.EnvironmentModelSpawner`.

**Implemented and live-tested** (coordinated across both files, per a
concurrent-session sync -- see message log; adopted a push-on-connect design
instead of a request/response, per that session's suggestion, avoiding the
blocking-call hazard entirely rather than just bounding it):

- `sim_bridge_node.py` now sends `{"type":"environment_options","options":[...]}`
  (from `environment_models.list_environment_models()`) immediately on every
  client connect, and handles incoming `{"type":"environment","model_name":...}`
  via `environment_models.EnvironmentModelSpawner.set_active_model()` (a new
  simpler by-name API, replacing the old boolean-only `setObstacleCourse`).
  The old hardcoded `OBSTACLE_COURSE_MODEL_NAME`/SDF-read/`obstacle_course_spawned`
  state is gone, folded into the shared helper.
- `rbx_sim_node.py` still keeps a bounded (`ENVIRONMENT_OPTIONS_WAIT_SEC = 5.0`)
  `threading.Event.wait()` in `__init__` before `RBXRobotIF` is constructed --
  not to block on a request it sends, but simply to give the already-async
  bridge thread a moment to receive the VM's connect-time push before
  capabilities are baked in; falls back to the original
  `["FLAT_GROUND","OBSTACLE_COURSE"]` unchanged if the VM doesn't answer in
  time. `setObstacleCourseAction(enabled: bool)` became
  `setEnvironmentAction(environment_value: str)`, looking up the real model
  name via `ENVIRONMENT_VALUE_TO_MODEL` (rebuilt by the new
  `processEnvironmentOptionsLine` on every announcement).
- `environment_models.py`'s directory-scan gained an opt-in
  `.environment_option` marker-file requirement (per a collision heads-up
  from the concurrent session's own dimensions.yaml/generate_model_sdf.py
  work, which can produce other model.sdf-having folders under
  `sim_container/models/` that should NOT be offered as environment
  options). `obstacle_course/` was given the marker; `scan_to_environment.py`
  writes it automatically for every model it generates.

**Live-verified end-to-end** against a real headless Gazebo instance: fresh
connect correctly announced `["obstacle_course"]`; after generating a second
scanned model, a fresh connect announced both; `{"type":"environment","model_name":"obstacle_course"}`
spawned it (confirmed via `get_world_properties`), switching to the scanned
model's name correctly deleted `obstacle_course` first and spawned the new
one, and switching to `model_name: null` (FLAT_GROUND) correctly deleted it
with nothing left spawned but `ground_plane`. One real bug caught by this
testing and fixed: `environment_models.py`'s `MODELS_ROOT` had an extra `".."`
(pointing one directory too high, silently returning an empty options list
until corrected).

A caveat this design still carries, stated plainly rather than glossed over:
a scan converted while a driver process is already running still won't
appear as a selectable option until that driver's *next* bridge reconnect or
restart -- `RBXRobotIF` capabilities have no live-refresh path once
constructed (section 2.1), so "the announcement arrived" and "the RUI can
select it this session" are only the same moment on a fresh connect, not on
every later announcement after that.

### Full-stack live verification, no physical device required

Beyond the VM-side wire-protocol test above, `rbx_sim_node.py` itself (the
actual device-side driver code, not just its wire format) was run standalone
against a bare `roscore` + `gzserver` + `sim_bridge_node.py`, using
`~/sim_connector_test_ws` (a pre-existing disposable scratch catkin
workspace, per `test_device_if_sim_harness.py`'s own docstring) for
`nepi_interfaces`' compiled message bindings, with the device-wide
`debug_mode`/`user_folders` params pre-seeded per
`docs/completed/SIM_CONNECTOR_NAVPOSE_HANG_BUG.md`'s documented fix (else a
bare-roscore `wait_for_param(timeout=1000)` genuinely blocks ~16.7 minutes).
Two real environment-setup issues were found and fixed along the way, both
in the disposable test workspace, not in any shared/committed code:
`sim_connector_test_ws`'s own `nepi_api/device_if_rbx.py` and `nepi_interfaces`
message bindings were stale relative to the current sandbox (missing
`teleopControlsReadyFunction`/`teleop_control_mode_ready`, unrelated to this
feature) -- fixed by copying the current file in and rebuilding that one
package (`catkin_make --pkg nepi_interfaces`).

With that environment up, confirmed via real ROS calls (not just log
inspection):

- `rbx/settings/capabilities_query` reported the `environment` Discrete
  setting's `options_list` as `[FLAT_GROUND, OBSTACLE_COURSE, OFFICE_SCAN_RBX_TEST]`
  -- the scanned model, converted moments earlier, showing up correctly
  because the driver was started *after* conversion (the documented
  restart-to-see-it caveat above, working as intended).
- Publishing a real `nepi_interfaces/Setting` update
  (`{type: Discrete, name: environment, value: OFFICE_SCAN_RBX_TEST}`) to
  `rbx/settings/update_setting` -- the exact mechanism a real RUI uses --
  was received, applied, and confirmed via `/gazebo/get_world_properties` to
  have actually spawned `office_scan_rbx_test` in the live Gazebo world.

This is full end-to-end proof, from the real NEPI RBX Settings API down to
Gazebo, with no physical/Docker device involved at any point. One
unrelated, benign artifact observed during this test and root-caused (not a
bug): the bridge connection cycled every ~5 seconds throughout, because no
rover model was ever spawned into the bare `empty.world`, so
`sim_bridge_node.py`'s `telemetryPushLoop` had nothing to send
(`self.latest_telemetry` is only set by `odomCb`, which needs a real
`/rover/odom` publisher) -- `rbx_sim_node.py`'s bridge client treats 5
seconds of silence as a dead connection and reconnects. Confirmed harmless
here (each reconnect correctly re-ran the environment-options handshake) and
is pre-existing behavior, unrelated to this feature -- would not occur
against `generic_rover.world`, which always has a real rover publishing
odometry.

## 6. Integration point: extend the app-level dynamic mechanism, not the RBX static one

Per §2.1: build this on `device_if_sim.py`'s `available_environment_options` /
`setEnvironmentOptionFunction` path, wired through `sim_connector_bridge_gazebo.py` —
**not** `rbx_sim_node.py`'s static `environment` CAP_SETTINGS list. Rationale:

1. It's the mechanism explicitly designed to be live-refreshed without a driver restart —
   exactly what "upload a scan, use it a minute later" needs.
2. It's already the actively-being-hardened path for this exact class of bug
   (enable/disable, live-state reporting) per the remaining-work punch list — building here
   means inheriting fixes already in flight instead of duplicating them on a second path.
3. It matches this codebase's own stated rule for extending the sim architecture ("adding a
   new simulator/capability should only mean writing/extending a bridge... never require
   changing `device_if_sim.py`'s core contract, `sim_connector_app_node.py`, or the RUI
   components" beyond the generic mechanisms already there).
4. It avoids a driver restart per upload, which appending to the RBX-level static list
   would require.

**Leave `rbx_sim_node.py`'s `environment` Setting untouched.** Don't extend it — that would
create two divergent, separately-maintained environment lists for the same underlying
concept. This plan flags that pre-existing duplication rather than fixing it, matching this
project's own convention of noting out-of-scope dead/duplicate code rather than silently
folding a fix for it into an unrelated feature (see `SIM_CONNECTOR_CONFIG_CONTROLS_PLAN.md`'s
own "explicitly not doing" section for the same pattern).

## 7. Avoid a third copy of the spawn/delete logic

The obstacle-course spawn/delete-by-name logic is already duplicated twice
(`sim_bridge_node.py` and `sim_connector_bridge_gazebo.py`, the latter's own comment calling
itself a "reused verbatim pattern" from the former). Adding scanned-environment spawning
as a third hand-copied block would make this worse. Proposed instead: extract the
spawn-if-not-spawned/delete-if-spawned-by-name logic (currently hardcoded to
`OBSTACLE_COURSE_MODEL_NAME`/`OBSTACLE_COURSE_SDF_PATH`) into one small shared helper
(e.g. `sim_container/bridges/gazebo/environment_models.py`), generalized to take a model
name + SDF path, imported by both existing call sites and the new one. This is a
refactor-while-extending move, not a separate cleanup pass — same file touched either way.

## 8. Folder reorg: `nepi_office_strayscan/`

Currently sits at the `nepi_drones` repo root (`nepi_office_strayscan/779206be34/`,
uncommitted, 107MB) — clutter at the top level, and not near the sim machinery it feeds.
Recommended destination, mirroring `sim_container/`'s existing `models/`/`worlds/`/
`scripts/`/`bridges/` layout:

```
sim_container/scan_data/
├── raw/office_779206be34/       <- nepi_office_strayscan/779206be34/ moves here as-is
├── processed/                   <- intermediate point clouds/meshes (pipeline output, §4)
└── (generated models land directly in sim_container/models/<scan_name>/, not here)
```

Not executed yet — this plan just proposes the destination for review. Worth deciding
whether `sim_container/scan_data/raw/` should be `.gitignore`'d, like the existing SITL
artifacts (`eeprom.bin`/`logs/`/`mav.*`), rather than committed, since it's binary and grows
with every future scan.

## 9. Phased development plan

**Phase 0 — Housekeeping.** Move `nepi_office_strayscan/779206be34/` per §8. Decide the
`.gitignore` question. No code.

**Phase 1 — Getting the scan (and the conversion job) onto the dev VM.**
Open3D TSDF fusion + decimation + convex decomposition over ~1400 frames is real compute —
run it on the dev VM (which already runs Gazebo), not the Raspberry Pi NEPI device. Reuse
`SIMULATOR_AUTO_LAUNCH_PLAN.md`'s existing SSH plumbing (`NEPI_SSH_KEY`, no committed
credentials) rather than building a second remote-access path. New pieces needed:
- A new Flask multipart upload route on the RUI backend (`nepi_rui`/`nepi_drones` mirrors)
  — no existing route handles binary/large-file upload; the "Upload Robot Config" precedent
  (`Nepi_IF_Sim.js`, String-over-rosbridge) is the wrong shape for 100MB+ binary data.
  Land the raw upload under `sim_container/scan_data/raw/<new_scan_id>/` on the device,
  then relay to the dev VM over the existing SSH channel (or have the browser target the
  dev VM directly if network topology allows — decide during implementation).
- **Watch for** (from `SIM_CONNECTOR_REMAINING_WORK.md`/postmortems): heartbeat checks must
  read a literal `ALIVE` reply, not just rely on `connect()` succeeding — a reverse SSH
  tunnel can make a dead far end look connected; `apps_mgr` doesn't auto-relaunch a killed
  app node, so a crashed upload/conversion job needs its own restart handling, not an
  assumption of self-healing.

**Phase 2 — Conversion script.** New `sim_container/scripts/scan_to_environment.py`
(Open3D + trimesh + V-HACD/CoACD), run as an offline/background job on the dev VM (too slow
for an inline HTTP request). Needs a job-status mechanism (poll or callback) so the RUI can
show "converting..." rather than blocking. Output: a new `sim_container/models/<scan_name>/model.sdf`
(visual mesh + N convex collision hulls, mirroring `obstacle_course/model.sdf`'s structure).
- **Watch for**: `rospy.init_node()` silently sets a 60-second global socket timeout that
  affects any later plain `socket.socket()` in the same process — this script likely runs
  standalone (no rospy), but if it's ever imported into a node, set explicit shorter
  timeouts. Also: `open3d` needs `LD_PRELOAD=libgomp.so.1` for a manual on-device relaunch
  (static-TLS crash otherwise) — confirm whether that applies on the dev VM too before
  assuming a clean run.

**Phase 3 — Wiring into the environment mechanism (§6/§7).** Extract the shared
spawn/delete-by-name helper (§7). Wire a new `getAvailableEnvironmentOptionsFunction`-style
directory scan of `sim_container/models/` (or a manifest file) into
`sim_connector_bridge_gazebo.py` so newly-converted scans appear as environment options
without a driver restart. Explicitly fix (for the *new* options only — not attempting to
fix it for the pre-existing `obstacle_course` option in the same pass) the server-side
active-state reporting gap from §2.1, so a reload doesn't lose track of which scanned
environment is spawned.
- **Watch for**: Gazebo Classic's `<include>` can't override per-instance plugin params —
  multi-robot/multi-instance worlds need inlined duplicate `<model>` blocks, the same
  constraint `obstacle_course/model.sdf` already works within; a generated scan model needs
  the same treatment if it's ever spawned into a multi-rover world.

**Phase 4 — RUI.** New "Upload Phone Scan" button (near the existing Environment control),
upload-progress/conversion-status display, and the environment dropdown extended to list
scanned environments alongside `FLAT_GROUND`/`OBSTACLE_COURSE`.

**Phase 5 — Verification.** Follow this project's own established pattern: VM-side
verification first (spawn/delete a converted scan model live in Gazebo via `rosservice
call`), then full RUI-driven verification, then on-device confirmation — mirroring exactly
how Gazebo/Webots/PyBullet were each brought up per `MULTI_SIMULATOR_INTEGRATION_PLAN.md`.

## 10. Explicitly not doing (yet)

- Not extending `rbx_sim_node.py`'s static `environment` Setting — see §6.
- Not building Webots (or any other simulator) support for this feature — see §2.2/§5.
- Not fixing the pre-existing client-side-only active-state bug for `obstacle_course`
  itself — only ensuring new scanned options don't inherit it (§9 Phase 3).
- Not re-fusing IMU (`imu.csv`) into the pose estimate — `odometry.csv`'s ARKit-fused pose
  is used as-is for v1; a bundle-adjustment/loop-closure pass is future scope if drift turns
  out to be a real problem in practice.
- Not solving pre-deploy config-preset editing — unrelated, out of scope (see
  `SIM_CONNECTOR_CONFIG_CONTROLS_PLAN.md`'s own non-goal of the same shape).

## 11. References

- `src/nepi_drivers/rbx_drivers/rbx_sim_node.py` (RBX-level `environment` Setting, static)
- `sim_container/scripts/sim_bridge_node.py` (legacy obstacle-course spawn pattern)
- `src/nepi_apps/nepi_app_sim_connector/api/device_if_sim.py` (`available_environment_options`,
  `refreshEnvironmentOptions`, `setEnvironmentOptionFunction`)
- `sim_container/bridges/gazebo/sim_connector_bridge_gazebo.py` (current spawn pattern,
  the mechanism this plan extends)
- `sim_container/models/obstacle_course/model.sdf` (model structure to mirror)
- `src/nepi_apps/nepi_app_sim_connector/rui/Nepi_IF_Sim.js` (existing upload precedent, and
  why it doesn't fit this data size)
- `docs/SIM_DEVICE_IF_CONTRACT.md`, `docs/SIM_CONNECTOR_REMAINING_WORK.md`,
  `docs/MULTI_SIMULATOR_INTEGRATION_PLAN.md`, `docs/SIMULATOR_AUTO_LAUNCH_PLAN.md`
- External: `strayrobots/scanner` (`docs/format.md`), `kekeblom/StrayVisualizer`
  (`stray_visualize.py`), `gizatt/convex_decomp_to_sdf`, Open3D (`ScalableTSDFVolume`/
  `VoxelBlockGrid`, `simplify_quadric_decimation`), V-HACD / CoACD
