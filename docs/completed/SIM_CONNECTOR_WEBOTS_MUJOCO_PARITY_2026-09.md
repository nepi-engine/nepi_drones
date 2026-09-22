# Webots/MuJoCo feature parity with Gazebo, and the bugs that blocked it

**Status:** Done. Webots and MuJoCo now match Gazebo for robot-dimension editing,
environment/obstacle-course switching, camera offset + FOV + yaw/tilt editing, and the
"Lock Scene Camera To Robot" feature. All changes are committed on `main` and deployed live
on the device as of 2026-09-22.

**Context:** requested live, in order: *"for everything we've got working so far, get mujoco
to that same point too, with all the same features i asked for with webots"* → *"go ahead and
start devving the next big part of it for webots... robot configs work, obstacle and robot
changing dimensions work, and environment/fov can be changed on the spot... do the same with
mujoco"* → (bug reports) *"for webots rover, making the motors negative doesnt make it go
backwards. obstacle course spawning in webots also doesnt work... make sure its almost
infinite space"* → *"i still dont think the lock scene camera to robot feature works for any
of the three sims - get it working for gazebo and then the rest."*

This file is the single reference for everything that was built and fixed across that whole
arc. It supersedes scattered commit messages as the place to look first.

## What was built

### 1. Robot-dimensions generation (Webots + MuJoCo)

Both simulators previously used a hand-authored, fixed-geometry rover — editing "Robot
Dimensions" in the RUI had no effect. Both now regenerate their rover file from
`sim_container/models/generic_rover/dimensions.yaml` — the same file Gazebo's own
`generate_model_sdf.py` already read — at every fresh launch:

- **Webots:** `sim_container/bridges/webots/scripts/generate_rover_wbt.py`, invoked as a step
  in `webots_rover`'s `launch_command` (`simulator_launch_targets.yaml`) before Webots itself
  starts. Produces `rbx_rover.wbt`.
- **MuJoCo:** `sim_container/bridges/mujoco/generate_rover_xml.py`, called from
  `mujoco_rbx_bridge.py`'s own `__init__` (MuJoCo has no separate launcher process to hook a
  generation step into — the bridge *is* the simulator). Produces `rbx_rover.xml`.

Both mirror `generate_model_sdf.py`'s own physical-relationship formulas (wheel axle height =
`wheel_radius_m`, chassis center height, front-mounted robot camera, wheel positions from
`wheelbase_m`/`track_width_m`). `wheel_independence_enabled` (Gazebo's crab-steer mode) has no
equivalent in either sim and is a documented, deliberate scope gap — both always use a
left/right-grouped drive model.

### 2. Environment / obstacle-course switching (Webots + MuJoCo)

The RBX driver's `environment` Setting (`FLAT_GROUND` / `OBSTACLE_COURSE`) now actually
does something on both sims, matching Gazebo:

- **Webots:** `sim_container/bridges/webots/scripts/generate_environment_wbt.py` builds the
  obstacle-course geometry (two boundary walls, a two-baffle chicane, a ramp) as a VRML
  fragment from `sim_container/models/obstacle_course/dimensions.yaml`, mirroring Gazebo's
  own `buildObstacleCourseSdf` formulas. `webots_rbx_bridge.py`'s `setObstacleCourseEnabled`
  spawns/despawns it live via Supervisor `importMFNodeFromString`/`node.remove()` — Webots
  has no live "add a body to a running world" primitive short of this.
- **MuJoCo:** `sim_container/bridges/mujoco/generate_environment_xml.py` builds the identical
  geometry as MJCF `<geom>` elements, always compiled into `rbx_rover.xml` but disabled
  (invisible, non-colliding) by default. `mujoco_rbx_bridge.py`'s `setObstacleCourseEnabled`
  toggles each geom's `rgba` alpha + `contype`/`conaffinity` live — a compiled `MjModel`
  cannot have bodies added/removed at runtime the way Webots' Supervisor can, so this is the
  MuJoCo-idiomatic equivalent.

### 3. Camera offset editing, including yaw/tilt (Webots + MuJoCo)

`camera_offset_x/y/z`, `scene_offset_x/y/z`, and `camera_fov_deg` were already live-editable
on both sims from earlier work this arc. This pass added the missing **`camera_offset_yaw`,
`camera_offset_tilt`, `scene_offset_yaw`, `scene_offset_tilt`** Settings, matching Gazebo's
`rbx_sim_node.py` exactly (absolute degrees, not deltas):

- **Webots:** `webots_rbx_bridge.py`'s `applyCameraSettings` now also writes each camera's
  `rotation` field, via a new `yawTiltToAxisAngle(yaw_deg, tilt_deg)` helper that composes
  `R = Rz(yaw) * Ry(tilt)` (the same fixed-axis convention Gazebo's own SDF `<pose>` uses)
  into the single axis-angle Webots' `SFRotation` needs.
- **MuJoCo:** `mujoco_rbx_bridge.py`'s `applyCameraSettings` now also writes each camera's
  `model.cam_quat`, via a new `yawTiltToQuat(yaw_deg, tilt_deg)` helper doing the same
  rotation composition, converted to MuJoCo's quaternion convention.

Both helpers were verified to reproduce each camera's own **factory** orientation exactly at
their factory yaw/tilt values (Webots: matches `SCENE_CAM`'s committed `rotation 0 1 0
0.5834`; MuJoCo: matches the compiled model's own `cam_quat`, read directly from a loaded
`MjModel`, to floating-point precision).

### 4. "Lock Scene Camera To Robot"

This RUI toggle (`Nepi_IF_Sim-Controls.js`) computes yaw/tilt via `atan2` so the scene camera
keeps facing the robot's origin as its position changes, then sends the result as
`scene_offset_yaw`/`scene_offset_tilt` Settings. It now works on all three sims — see the bug
writeup below for what was actually broken (it was never the lock math itself).

## Bugs found and fixed

Each of these was found by **actually running the thing**, not by reading the code — several
looked completely correct on inspection and were only caught by live testing (through the real
production launch pipeline and, for the last one, the real `rosbridge` wire protocol, not a
raw `rospy` publisher). If a future change in this area "looks right" but a report says it
isn't, distrust the read and go run it.

### Webots dimensions generator silently used defaults, always

`generate_rover_wbt.py`'s `MODELS_DIR` was computed as two directories up from `scripts/`
(`sim_container/bridges/models`, which doesn't exist) instead of three
(`sim_container/models`). `loadDimensions()`'s own `os.path.exists(path)` guard silently fell
through to hardcoded defaults on every run, so **every** dimensions edit was a no-op — this
had never worked, ever, since the generator was written. The identical off-by-one existed in
`webots_rbx_bridge.py`'s own `DIMENSIONS_PATH` (used to scale commanded velocity to the real
wheel size), invisible only because its fallback constants happened to match the generator's
own defaults. Fixed both; verified by pushing non-default dimensions through the real
production `launch_command` and confirming the generated `.wbt`'s wheel/chassis geometry, FOV,
and mass all matched.

### Webots rover motors ignored every negative command

`rbx_webots_node.py`'s `setMotorControlRatio` clamped `speed_ratio` to `[0.0, 1.0]` — the
clamp meant for ArduPilot's motor *test* action (a one-directional prop spin-up check).
Gazebo's `rbx_sim_node.py` and MuJoCo's `rbx_mujoco_node.py` both already use the correct
`[-1.0, 1.0]` range for a genuinely-reversible wheeled rover; only the Webots driver had the
wrong one. Confirmed live: after the fix, commanding a negative motor ratio moved the rover to
negative `x_m` in its own reported NavPose.

### Webots obstacle-course toggle did nothing (root cause was one directory deeper than it looked)

Reported as "obstacle course spawning in webots doesn't work." The spawn/despawn code itself
(Supervisor import/remove) was correct and worked when driven directly. The real blocker was
one level up: `rbx_webots_node.py`'s `self.settings_dict` was a bare `copy.deepcopy(self.
FACTORY_SETTINGS)` — missing `options`/`bounds`/`default`, the fields `nepi_controls.
get_clean_value()` actually requires for a `Discrete`/`Selection`-typed Setting. Toggling
`environment` raised an **uncaught `KeyError: 'options'`** inside `get_clean_value` on every
attempt, so `setEnvironmentAction` was never reached at all. `rbx_ardupilot_node.py` (the
production driver) already had the fix — its own `initSettingsDict()` docstring documents the
identical bug, found there first as an empty RUI Settings panel — it had just never been
ported to the three simulator drivers. Ported `initSettingsDict()` to `rbx_webots_node.py`,
`rbx_mujoco_node.py`, and `rbx_webots_quadcopter_node.py`. `rbx_sim_node.py` (Gazebo) already
had it, so Gazebo's own obstacle-course toggle was never affected by this specific bug.

### Webots default floor was 6x6 meters

Grown to 1000x1000 (`Floor`'s own `tileSize` defaults to 0.5m regardless of overall size, so
grass-texture density scales automatically with no stretching). Comfortably larger than the
obstacle course's own footprint (~24m along its long axis).

### Missing `TeleopKeymap.js` blocked every RUI build, for everyone, always

Discovered while rebuilding the RUI to deploy the fixes above. `NepiDeviceRBX-Controls.js`
imports `buildTeleopKeyMap`/`loadTeleopBindings` from `./TeleopKeymap` — a file that was never
actually committed (confirmed via `git log --all`, no trace of it ever existing). This broke
the production RUI build outright (`Module not found: Error: Can't resolve './TeleopKeymap'`)
— **every** `nepibld` on this device, and presumably any other checkout, would have hit this
identical failure. Implemented the file for real (named WASD/QE/RF key bindings,
localStorage-backed, matching the exact API surface the importing file already expected).

### Root cause of "lock scene camera to robot doesn't work for any of the three sims"

The scene-camera-lock backend chain (RUI → `UpdateControl` Setting → driver →
`sendCameraSettings` → sim-specific bridge → actual camera pose) looked, and tested, correct
for Gazebo when driven with a direct `rospy.Publisher`. The user's report was still right —
the bug was in a layer that raw `rospy` testing doesn't exercise at all: **`rosbridge`**, the
WebSocket bridge the real browser RUI actually talks through.

`Nepi_IF_Sim-Controls.js`'s own `sendControlUpdate()` built its outgoing message with a `type`
field:
```js
const data = { name, display_name: "", description: "", type: type, value: [], ... }
```
`nepi_interfaces/UpdateControl.msg` has no `type` field (`name`, `display_name`, `description`,
`value`, `index`, `min_bound`, `max_bound`, `options` only — the function's own comment
already correctly documented this field list; the code just didn't match it).
`rosbridge_websocket` validates every published message against the real message definition
and **silently drops anything with an unrecognized field** — logging only to its own node's
ROS log, never back to the browser:
```
[Client 0] publish: Message type nepi_interfaces/UpdateControl does not have a field type
```
This affected **every** Setting this function sends: camera/scene offsets (including the lock
feature), the environment toggle, `camera_controls_enabled`/`autonomous_movement_enabled`, and
`enabled_image_sources`. It's isolated to this one function — the shared, generic
`sendUpdateControlValue()` in `Store.js` (used by the plain RBX device panel) never had the
extra field and was never affected.

**How this was found:** publishing the exact same message via a raw `rospy.Publisher`
succeeded (rospy's own subscriber machinery doesn't validate against the message definition
the way `rosbridge` does); publishing it via the actual `rosbridge` WebSocket protocol
(`ws://<device>:9090`, `{"op":"publish","topic":...,"msg":{...}}`) silently did nothing —
confirmed via an independent `rostopic echo` on the same topic seeing nothing at all. The
rejection reason was only visible in `rosbridge_websocket`'s own ROS log file
(`~/.ros/log/<run-id>/nepi-device1-rosbridge_websocket-*.log`), not anywhere the RUI or a
`rostopic`-level check would surface it.

**Takeaway for future "the RUI shows nothing happened" reports on this app:** a raw `rospy`
test proves the *backend* works; it does **not** prove the real browser path works. If a
`rospy`-level test passes but the actual RUI still doesn't do anything, check
`rosbridge_websocket`'s own log for a silent rejection before assuming the bug is elsewhere.

## Deploying changes to the device (and its own gotchas)

`nepi_drones` is a standalone dev repo; the physical device runs a separately-built NEPI
image. Getting a source change from this repo onto the actual running device is:

1. `NEPI_REMOTE_SETUP=1 NEPI_IP=<device ip> bash deploy_nepi_source.sh` (from `nepi_drones`) —
   rsyncs (`--delete`) this repo's `src/` tree to the device's
   `/mnt/nepi_config/system_cfg/src` config overlay. Matches the `nepidpl` alias.
2. On the device (`nepihost@<ip>`, then inside the NEPI container): `nepibld` — this is **not**
   a quick restart. It stops NEPI, wipes `${NEPI_BASE}/etc`, `docker_cfg`, `docker`, then runs
   `build_nepi_complete.sh`: applies the config overlay onto
   `/mnt/nepi_storage/nepi_src/nepi_engine_ws`, then does a **full catkin build** of the ROS
   workspace and a **full npm/React build** of the RUI. Budget real time for this, and expect
   NEPI to be down for the duration.
3. `nepistart` to actually run the freshly-built code.

Both `nepihost` (host OS user, port 22) and `nepi` (container user, port 2222) use the
publicly-documented default password `nepi` for `sudo`.

**Known, pre-existing, unrelated bug hit repeatedly during this work:** `drivers_mgr.py`
sometimes starts before `config_mgr` has published `/nepi/device1/system_folders`, and crashes
outright (`TypeError: 'NoneType' object is not subscriptable` reading
`system_folders['drivers_param']`) instead of waiting/retrying. This is a startup-order race,
not caused by anything in this session's changes, and was **not fixed** (out of scope). When
it happens, just restart the one node once `config_mgr` has settled:
```
rosrun nepi_managers drivers_mgr.py __name:=drivers_mgr
```
(with `ROS_NAMESPACE=/nepi/device1` set first). It has hit on every `nepistart` performed
during this session — expect it on the next one too.

**Also noticed, not fixed:** the `enabled_image_sources` Setting's `default`/`value` in a live
`settings/status` read shows a stringified nested dict (`"{'name': 'enabled_image_sources',
'type': 'String', 'value': ''}"`) instead of a plain empty string — looks like a real, separate
bug somewhere in how that one Setting's default gets constructed, but it didn't block anything
this pass touched and wasn't investigated further.

## Verification techniques worth reusing

- **Standalone visual verification before touching the device:** for both Webots (via a
  `--stdout --stderr` batch launch + a small fake TCP "device" listener standing in for the
  real driver) and MuJoCo (via `mujoco.Renderer` loaded directly, no ROS at all), render a
  frame to PNG/JPG and actually look at it. Caught real bugs that a topic-value check alone
  would have missed.
- **Test the real wire protocol, not just `rospy`:** `rospy.Publisher` bypasses `rosbridge`'s
  message-definition validation entirely. Any bug that only manifests through `rosbridge` (like
  the camera-lock one above) is invisible to a plain `rospy` test. Use a raw WebSocket client
  against `ws://<device>:9090` with the exact `{"op":"publish",...}` shape when a "works for me
  in a script, not in the browser" mismatch is suspected.
- **`rostopic echo` as an independent witness:** subscribing to a topic from a completely
  separate process (not the one you're debugging) proves whether a message actually reached
  ROS at all, independent of whatever the intended subscriber's callback does with it.
