# Simulator System Handoff — 2026-09-25

*Written at the end of a development session, for whoever picks this up next. Covers the
NEPI simulator stack end to end: what exists, what was fixed in this session, what's known
to be broken or unfinished, and how to actually operate the dev VM + device pair this all
runs on. Where an older plan doc already covers something in depth (referenced inline),
this doc gives the current-state summary and points there for detail rather than repeating
it.*

## 0. TL;DR for a new developer

- **MuJoCo is now the primary/default simulator**, not Gazebo. It supports the same crab-
  steer ("wheel independence") swerve-drive feature Gazebo has, plus all four environment
  types (Flat Ground, Obstacle Course, Custom Obstacles, Aerial Obstacle Course).
- The **Sim Connector app's own on-screen controls** (motor sliders, goto buttons, Reset
  Sim, etc.) did not work for any ground rover before this session — only Devices → Robots
  did. That's fixed now; both panels work.
- **Phone-scan-to-simulated-environment ("real2sim") is partially built and not usable
  end-to-end.** The hard part (scan → 3D mesh → collidable model) works if you get a scan
  onto the machine by hand. There is no upload button, no UI, and MuJoCo doesn't support
  scanned environments at all (Gazebo only). See §5.
- **The dev VM's `/opt/nepi` (installed NEPI code) does not survive a device reboot, or
  even a plain `nepibld`/`nepistart` cycle.** Every time either happens, you must redeploy
  and rebuild from scratch. See §6 — this will bite you immediately if you don't know it.

## 1. What this system is

`nepi_app_sim_connector` (`src/nepi_apps/nepi_app_sim_connector/`) is a NEPI device app that
launches and controls a simulator running on a separate Linux VM (today: a WSL2 Ubuntu VM
reachable from the device over a reverse SSH tunnel / shared-storage mailbox — see
`docs/SIM_VM_CONNECTION_SETUP.md`). The device side and VM side are two different processes
talking a small JSON-over-TCP wire protocol; there is no ROS between them.

Three simulators are wired in: **Gazebo**, **Webots**, and **MuJoCo** (PyBullet and WPILib
were evaluated and removed — see `docs/SIM_CONNECTOR_REMAINING_WORK.md` item 5). Architecture
docs worth reading before changing any of this:

- `docs/SIM_DEVICE_IF_CONTRACT.md` — the generic device ↔ app contract
  (`device_if_sim.py`), which every simulator's bridge script talks to identically.
- `docs/MULTI_SIMULATOR_INTEGRATION_PLAN.md` — how a new simulator gets added (a new
  bridge script + a new `launch_targets` entry — never a change to the shared contract).
- `docs/SIMULATOR_AUTO_LAUNCH_PLAN.md` — the one-click deploy/install mechanism
  (`simulator_launcher.py`, `simulator_launch_targets.yaml`, `vm_command_watcher.py`).

For each simulator, there are actually **two separate control surfaces** into the same
running rover:

1. **Devices → Robots** (`NepiDeviceRBX.js`/`NepiDeviceRBX-Controls.js`) — talks straight to
   the RBX device's own ROS topics/settings. This has always worked.
2. **The Sim Connector app's own panel** (`Nepi_IF_Sim.js`/`Nepi_IF_Sim-Controls.js`) — meant
   to be the "one app to control whichever simulator is selected" surface, including
   environment/camera/launch controls Devices → Robots doesn't have. **Its manual/goto
   controls did not work for any rover before this session** — see §2.1.

## 2. What changed this session

### 2.1 The Sim Connector panel's manual/goto controls were dead for every rover

`sim_connector_app_node.py`'s `setMotorControlRatio`/`gotoPosition`/`gotoPose`/`goHome`/
`goStop`/`setSetupActionInd` all sent over `device_if_sim.py`'s generic bridge protocol
(TCP port 9030) — a protocol only the ArduPilot-quadcopter bridge script ever actually
listens on. For a rover (Gazebo, Webots, or MuJoCo), nothing was on the other end, so every
command from this panel was silently dropped. `isBridgeConnected`/`autonomousControlsReady`
were also permanently `False` for a rover, since they checked that same dead connection.

**Fixed**: when a real RBX-device simulator is selected (`self.selected_simulator`, set by
`simDiscoveryCb`), these functions now forward the command as a direct ROS publish to that
device's own topic (`goto_position`, `set_motor_control`, etc.) instead of the 9030 bridge,
and readiness is read from that device's own `DeviceRBXStatus.manual_control_mode_ready`/
`autonomous_control_mode_ready` fields (now cached per-device in `simDeviceStatusCb`). See
`forwardToRbx`/`getSelectedSimDeviceInfo` in `sim_connector_app_node.py`. The old 9030 path
is left in place as a fallback (still used by the quadcopter target) — nothing about
ArduPilot SITL was touched.

**A real caveat found while fixing this, not yet resolved**: `gotoPose` requires the
*robot config's own profile* to declare `has_goto_pose: true`. Neither of the built-in ground
rover configs (`ground_robot_4_wheel`, `crabrover`) does — so the Pose/yaw control on this
panel is currently unreachable for any rover regardless of the forwarding fix, purely a
config flag. It shows fine on Devices → Robots (gated on the driver's own capabilities, not
the profile). Flip `has_goto_pose: true` on a rover's robot-config entry if you want it on
this panel too.

**Also fixed in the same area**:
- A real crash: `sim_connector_app_node.py` imported `GeoPoint` from `geometry_msgs.msg`
  (wrong package — it's `geographic_msgs.msg.GeoPoint`), which took the whole app down the
  moment `setHome` was wired up. Confirmed via running the node directly and reading the
  traceback — this is why "did the app even come up" is always worth checking after any
  deploy (`rosnode list | grep app_sim_connector`).
- `stopSimulatorCb`/`runRedeploy` did nothing if this app's own process had restarted while
  a simulator kept running underneath it (its in-memory `active_launch_target` was blank, so
  there was "nothing to stop"). Both now fall back to asking the VM's shared-storage watcher
  what's actually running (`SimulatorLauncher.find_running_target`) before giving up.
- A launch's "already running, refuse to launch" guard was only recognized for Gazebo's own
  wording (`isGazeboConflictError`/`GAZEBO_ALREADY_RUNNING_ERROR_SIGNATURE`); it's now
  `ALREADY_RUNNING_ERROR_SIGNATURES`, matching MuJoCo's guard text too. `Kill All Gazebo`
  now also clears a stray `mujoco_rbx_bridge.py`.
- A genuine watcher bug: redeploying a target whose previous run had died read that dead
  run's leftover status and reported a false "launch failed" while the new launch was
  actually coming up fine. `vm_command_watcher.py`'s status now echoes back which request
  (`control.last_updated`) it's answering (`handled_update`), so `simulator_launcher.py`
  never mistakes an old answer for a fresh one. If you touch the shared-storage
  launch/stop/redeploy protocol, keep this token flowing through — it's the only thing
  telling a stale status apart from a current one.

### 2.2 Wheel independence ("crab steer") — now on both Gazebo and MuJoCo

This is a swerve-drive mode: every wheel steers independently and the chassis holds its own
heading unless explicitly told to rotate (an explicit yaw/pose command still rotates it; a
position command, or a manual differential-motor "turn" gesture, never does). Toggled by the
`wheel_independence_enabled` RBX Setting, live, no relaunch needed on either simulator. A
robot config's own `dimensions.yaml` can also set the *starting* mode (`crabrover`'s config
has `wheel_independence_enabled: '1'`), which is what the RUI should offer as the "crab
rover" preset.

**Gazebo** (`sim_container/nepi_gazebo_plugins/src/crab_steer_plugin.cpp`): the chassis is a
**kinematic** body (`SetKinematic(true)`) driven by `SetLinearVel`/`SetAngularVel` — it is
immune to gravity/contact forces and **passes straight through obstacles**, an explicit,
requested tradeoff. Do not add a second, separate pose-integration step here — ODE already
advances a kinematic body from the velocity you set on it; adding one produced a real bug
(doubled every commanded speed) that was found and removed this session. Each wheel gets its
own per-wheel tangential velocity (`v = v_center + omega × r`) so a real rotation doesn't
under-turn or tumble the rover — see the file's own extensive header comments for the exact
incident history (severe damping, tumbling, a `Model` vs `Link` velocity bug) if you need to
touch this again.

**MuJoCo** (`sim_container/bridges/mujoco/`): a genuinely different, *dynamic* design — every
wheel is a real swerve module (a steer hinge + a spin hinge, both real joints with real
actuators; see `generate_rover_xml.py`'s wheel-body generation). The chassis is an ordinary
free-floating dynamic body; it moves **only because the wheels push it**, so unlike Gazebo it
still physically collides with and reacts to obstacles even in crab-steer mode. The bridge
(`mujoco_rbx_bridge.py`) computes each wheel's required body-frame velocity the same way
Gazebo's plugin does, then picks the *nearest equivalent steer angle* to the wheel's current
one (a wheel can point `theta` or `theta ± 180°` with the spin direction reversed — the
solver always picks whichever is closer, so a wheel never swings the long way around and a
reversal is a reverse, not a half-turn) and rate-limits the steer command
(`STEER_SLEW_RADPS`) so snapping all four wheels doesn't kick the chassis via reaction
torque. Same active yaw-hold as Gazebo (servo back to the heading it was at when rotation
stopped, rate-capped).

If you need to retune the steer actuator: `STEER_KP=1000`/`STEER_DAMPING=15`/
`STEER_ARMATURE=0.1` in `generate_rover_xml.py` were reached by a live sweep (lower values
were either too weak to hold against ground contact, or numerically unstable at MuJoCo's
2ms timestep) — don't drop the armature back near 0 without re-sweeping.

### 2.3 MuJoCo environment parity

MuJoCo cannot add/remove geoms from a compiled `MjModel` at runtime (Gazebo/Webots can
spawn/delete a model; MuJoCo cannot), so every environment's geoms are compiled into
`rbx_rover.xml` from the start, all superimposed in the same world, and only one group's
`contype`/`conaffinity`/visibility is switched on at a time. Four options now exist, matching
Gazebo/Webots' own list:

- `FLAT_GROUND` — nothing enabled.
- `OBSTACLE_COURSE` — walls/baffles/ramp from `models/obstacle_course/dimensions.yaml`
  (`generate_environment_xml.buildObstacleCourseGeomsXml`).
- `CUSTOM_OBSTACLES` — user-defined wall/circle/triangle primitives from
  `models/custom_obstacles/dimensions.yaml` (`buildCustomObstaclesGeomsXml`, new this
  session).
- `AERIAL_OBSTACLE_COURSE` — the same static 4-gate flight-racing layout Gazebo/Webots use,
  transcribed geometrically from `models/aerial_obstacle_course/model.sdf` (new this
  session, `buildAerialObstacleCourseGeomsXml`). Most of its bars sit 0.85m+ above the
  ground since it's built for a flying vehicle — a short ground rover will roll underneath
  most of it. That's the course's own real geometry, not a bug; there is no MuJoCo
  quadcopter target to actually fly this properly (see §4).

**A real, non-obvious MuJoCo gotcha, found the hard way**: a geom compiled with
`contype="0" conaffinity="0"` is left out of MuJoCo's collision structures *at compile time*
— flipping it to `1` later at runtime does **not** make it start colliding. Every
environment geom is now compiled `contype="1" conaffinity="1"` (colliding) and the *bridge*
switches collision off for the inactive ones at load — the inverse of what you'd naturally
write. If a future environment addition seems to render fine but the rover drives straight
through it, this is almost certainly why.

**New: `REFRESH_ENVIRONMENT` setup action.** Editing an environment's dimensions (adding a
wall, changing the obstacle course's ramp) previously only took effect on the *next full
relaunch* of MuJoCo, unlike Gazebo/Webots which can respawn live. This action rebuilds
`rbx_rover.xml` and swaps in a fresh `MjModel`/`MjData` in place
(`mujoco_rbx_bridge.py`'s `_bindModel`/`refreshEnvironment`), while explicitly preserving:
pose, velocity, commanded velocity, per-wheel steer state, the currently-selected
environment, and camera offsets/FOV (this last one was a real bug found and fixed this
session — a refresh used to silently snap both cameras back to their factory position/FOV).
If you add more per-model state to the bridge in the future, check whether
`refreshEnvironment` needs to be taught to preserve it too — it is very easy to introduce a
"looks like a crash/reset" side effect here without realizing it.

**Settings/environment resync is now periodic, not just on-change.** `rbx_mujoco_node.py`
runs a 2-second timer (`settingsResyncCb`) that re-sends camera settings and the current
environment unconditionally. This exists because a setting change that lands **during** a
bridge reconnect (which does happen — see §6) was previously silently dropped with no retry.
Both `mujoco_rbx_bridge.py`'s `setEnvironment` and the settings-forward path are written to
be idempotent/cheap when nothing actually changed, specifically so this periodic resend
doesn't spam logs or do real work every 2 seconds in the steady state — keep that property if
you touch either.

### 2.4 MuJoCo is now the default simulator

`simulator_launch_targets.yaml` was reordered so `mujoco_rover` is listed first (the RUI's
simulator dropdown and `getAvailableSimulators()`/`get_target` both follow this file's own
key order — there is no separate "default" flag anywhere). Picking a flight robot config
(`flight_robot_4_motor`) while MuJoCo is selected now redirects to `gazebo_quadcopter` via a
new `launch_target_overrides` entry, the same mechanism `gazebo_rover`/`webots_rover` already
use to redirect to *their* quadcopter target — MuJoCo has no quadcopter model of its own.

Also added for parity with Gazebo: `move_with_manual_enabled` Setting (manual motor sliders
do/don't drive the rover), and a live "Wheel Independence" toggle directly on the Sim
Connector panel (previously reachable only via Devices → Robots' generic Settings panel).

### 2.5 Cosmetic

"Upload Raw model.sdf" is now hidden on the dimensions editor when a non-Gazebo target is
selected (`selectedTargetUsesModelSdf()`, `Nepi_IF_Sim.js`) — MuJoCo/Webots never read that
file, so the button was a dead end there.

## 3. Testing performed this session

Everything above was verified live against the real device + VM pair, including after a
genuine device reboot followed by a full redeploy/rebuild/restart (see §6) — not just
read-through. Specifically confirmed working: crab-steer on both simulators (straight,
diagonal, rotate-then-translate, sideways, mode-switch mid-run), the swerve nearest-angle
solver under a hard rotation, per-wheel collision under both Obstacle Course and Custom
Obstacles (rover stops at the correct wall face, doesn't pass through), Aerial Obstacle
Course spawning/toggling, `REFRESH_ENVIRONMENT` preserving pose/velocity/camera settings,
all four MuJoCo camera streams (robot/scene × color/depth), the "Lock Scene Camera To Robot"
wire path (position + computed look-at yaw/tilt reaching the rendered image), the Sim
Connector panel's motor/goto forwarding, robot-config switching re-deriving readiness/
capabilities correctly, and the full crash → redeploy → fresh-launch cycle for both Gazebo
and MuJoCo (6-step lifecycle: deploy while running / deploy again / close window / redeploy
/ hard-kill / redeploy after crash — all pass).

**Not verified this session** (flag before relying on them):
- Two simulator instances running at once ("New Sim" / secondary launch) specifically
  against MuJoCo — the underlying reuse-detection fix is simulator-agnostic and was proven
  against Gazebo's own crash/relaunch cycle, but the secondary-slot path itself wasn't
  re-exercised against MuJoCo this session. A second MuJoCo instance cannot work today
  regardless (fixed ports 9051/9056, one pgid file, fixed `DEVICE_ID='rover1'` in
  `rbx_mujoco_discovery.py`) — only MuJoCo-as-primary + something-else-as-secondary is
  even plausible, and that combination is untested.
- `robot_link` (physical-robot mirroring) against MuJoCo specifically.
- `set_camera_view_mode`/`set_active_image_topic` sent through the *app's* own
  `sim/set_camera_view_mode` topic (as opposed to the RBX device's own environment/camera
  Settings, which are confirmed working) — per an earlier research pass, this app-level path
  also goes over the dead 9030 protocol for a rover and was not part of this session's
  forwarding fix. If it's still needed, it wants the same `forwardToRbx` treatment §2.1 gave
  the other controls.

## 4. Known gaps, not fixed this session

- **MuJoCo has no quadcopter model.** Any flight-vehicle work stays on Gazebo (real
  ArduPilot SITL) or Webots (Supervisor-injected flight, no real aerodynamics).
- **`gotoPose` unreachable from the Sim Connector panel for any built-in rover config** —
  see §2.1's caveat. A config/data fix (`has_goto_pose: true`), not a code fix.
- **A second MuJoCo instance is architecturally impossible today** — fixed ports, one pgid
  file, fixed device ID. Would need per-instance port allocation the same way Gazebo's
  rover/quadcopter targets already have separate `GAZEBO_MASTER_URI`s, if ever needed.
- **No Aerial Obstacle Course collision-tuning pass for MuJoCo** — the geometry was
  transcribed correctly from the Gazebo SDF, but nobody has flown/driven a vehicle capable
  of actually threading these gates in MuJoCo (there's no quad model to do it with). If a
  MuJoCo quadcopter is ever added, expect to revisit gate sizing/placement.
- **`SIM_CONNECTOR_REMAINING_WORK.md` is stale** — it still tracks PyBullet/WPILib, both
  removed 2026-08-26. Worth a cleanup pass or a note pointing here instead.

## 5. Real2sim (phone scan → simulated environment) — the big unfinished piece

Full design history: `docs/SCAN_TO_SIM_ENVIRONMENT_PLAN.md` (long — read §2 and §5.5/5.6 for
the actual current mechanism, the rest is earlier-draft context). Short version of where it
actually stands, not touched this session:

**What works**: scan a real obstacle course with an iPhone (Stray Scanner app — LiDAR depth +
IMU + RGB + pose), and `sim_container/scripts/scan_to_environment.py` (Open3D TSDF fusion →
decimated visual mesh → V-HACD convex-decomposed collision mesh) converts it into a real,
collidable Gazebo `model.sdf`, dropped alongside `obstacle_course` under
`sim_container/models/<scan_name>/`. This conversion step is genuinely implemented and was
live-verified end-to-end at the time (real RBX Settings API call → spawned in Gazebo,
confirmed via `get_world_properties`) — see the plan doc's own §5.6 for the exact test
transcript.

**What does not work — this is the part that was "never properly fixed"**:

1. **There is no upload path.** `convertPhoneScanCb`'s own comment says it plainly: "the
   browser → device upload step that would populate it isn't built yet." The conversion
   function assumes the raw scan folder already exists at
   `SCAN_UPLOADS_STORAGE_DIR/<name>/` on the device — today the only way to get it there is
   to manually copy files over yourself. There is no RUI button, no HTTP upload route, no
   drag-and-drop. Someone has to build a real upload mechanism (the plan doc's §9 Phase 1
   sketches a Flask multipart route plus reusing the existing SSH plumbing to relay the scan
   to the VM — never built).
2. **A converted scan doesn't appear as a selectable option until the driver restarts.**
   `rbx_sim_node.py`'s `environment` Setting option list is a construction-time snapshot;
   there's no live-refresh path in `RBXRobotIF`/`device_if_rbx.py` for a capability's option
   list to grow after the driver is already running. Converting a scan mid-session gets you
   a real model on disk that simply isn't offered anywhere until you restart the rover
   driver or relaunch the sim.
3. **Gazebo only.** Webots has no obstacle-course precedent at all (confirmed empty options
   list), and **MuJoCo — built entirely after this plan was written — has no scanned-mesh
   support whatsoever.** MuJoCo's environment mechanism (§2.3 above) is a fixed, closed set
   of four compiled-in options; it has no analogue of "drop a new model.sdf on disk and pick
   it up." Adding real2sim support to MuJoCo would mean converting the scan's mesh into MJCF
   geoms (or an MJCF mesh asset) and — per the "everything is compiled in, nothing spawns
   at runtime" constraint that already shapes every other MuJoCo environment — deciding
   whether to compile every ever-converted scan into every future `rbx_rover.xml`
   (unbounded model-file growth) or accept that a MuJoCo relaunch is required after every
   new scan, same as Gazebo's own driver-restart requirement above, just for a structural
   reason instead of a capability-snapshot one.
4. **No UI at all.** No "converting..." progress indicator, no way to trigger a conversion
   from the RUI, no environment-dropdown integration for scanned names.
5. There is a leftover **107MB uncommitted raw scan** (`nepi_office_strayscan/779206be34/`)
   sitting at the `nepi_drones` repo root, per the plan doc's own §8 — it was never moved to
   `sim_container/scan_data/raw/` as recommended, and the `.gitignore` question for future
   scans (should raw scan data even be committed?) was never decided. Worth resolving before
   this repo's root gets a second one of these dropped into it.

**If you pick this up**: the conversion pipeline (the genuinely hard 3D-vision part) is
done and proven. What's missing is almost entirely product/plumbing work — an upload
mechanism, a way to refresh a running driver's option list (or accept the restart
requirement and just surface it clearly in the RUI), and a decision on whether MuJoCo needs
this at all given its fundamentally different "everything compiled in" environment model.

## 6. Operational gotchas — read this before you touch the device again

These cost significant time this session and will cost you the same if you don't know them
going in.

- **`/opt/nepi` (the installed/running NEPI code) does not survive a device reboot.**
  Confirmed repeatedly: after a reboot, the installed code reverts to an old baked image —
  none of your recent work is there until you redeploy + rebuild. The recovery sequence,
  every time, is: `NEPI_REMOTE_SETUP=1 NEPI_IP=<device> bash deploy_nepi_source.sh` (from
  `nepi_drones`) → SSH in, clear `__pycache__`, run `nepibld` then `nepistart` (both must be
  wrapped as `nohup bash -c "source ~/.nepi_system_aliases; nepibld" ...` — `nohup` cannot
  invoke a bash *function* directly, and a non-interactive SSH session doesn't source that
  aliases file on its own).
- **`nepibld`/`nepistart`'s own `nepistop` step can recreate the device's Docker container
  from a baked image**, not just stop services — this happens even when you didn't reboot
  anything, and has the exact same effect as a reboot (`/opt/nepi` reverts, `/tmp` is wiped,
  the container's clock resets to the Unix epoch). Expect this on *every* build cycle, not
  just after a physical reboot.
- **The device's clock has no working time source** (no RTC, and its configured NTP server
  doesn't respond) — after any container recreation it's back at Jan 1 1970. This isn't
  cosmetic: a build running under a bogus clock can silently fail to pick up source changes
  (make/catkin's staleness checks compare mtimes). Always `sudo date -s "$(date '+%Y-%m-%d
  %H:%M:%S')"` on the device right after it comes back, before rebuilding. A real fix (an
  NTP server at the configured address, or a battery-backed RTC) was never done.
- **`sim_container/` is not covered by `deploy_nepi_source.sh`.** It only syncs `src/`.
  `simulator_launch_targets.yaml` needs a separate manual `scp` to
  `/mnt/nepi_config/simulator_launch_targets.yaml`. The MuJoCo/Gazebo/Webots bridge scripts
  under `sim_container/bridges/`/`nepi_gazebo_plugins/` live and run **directly from this VM
  filesystem** — editing them takes effect on the *next fresh process launch*, no device
  deploy needed at all, but also **no hot-reload**: a long-running bridge process keeps
  executing whatever code was loaded when it started. If you edit a bridge script and don't
  see your change take effect, check whether you're actually looking at a fresh process
  (`pgrep -af mujoco_rbx_bridge`) or a stale one from before your edit — kill it and let it
  relaunch.
- **One-shot `rostopic pub -1` is unreliable against a freshly-created subscriber.** It
  silently fails to deliver often enough that it produced multiple false "this doesn't work"
  diagnoses this session. For anything that has to land, write a small script that
  publishes 5x over ~1.5s instead of trusting a single one-shot publish.
- **Test hygiene**: always clear leftover motor ratios (`set_motor_control` all zero) and
  any active goto target (`go_stop`) before starting a new manual test. Several apparent
  "bugs" this session (and probably in the past) were actually a *previous* test's command
  still being obeyed. `RESET_SIM` is the fast, reliable way to get back to a known-clean
  state for either simulator.
- **`vm_command_watcher.py` can desync from reality** if its tracked launch process is
  killed directly (bypassing it) rather than through its own stop path — it can end up
  believing a target is running (or not) when the opposite is true, and silently no-op a
  request that names the same target it already (wrongly) believes is active. If a
  redeploy/launch/stop stops making sense, check for stray duplicate `vm_command_watcher.py`
  processes and stray orphaned launch-script processes; kill everything and restart the
  watcher cleanly rather than trying to reason your way back to a consistent state.

## 7. Where the code lives

Everything in this session's work is in the **`nepi_drones`** repo (this one), and the
sim-VM-side pieces are also mirrored into the separate **`nepi_simulation`** repo
(`github.com/nepi-engine/nepi_simulation`), which exists as a synced copy of just the
VM-side simulator harness (`sim_container/` → `nepi_sim_app/` there) — it does **not**
contain the NEPI device-side driver code (`src/nepi_drivers/`, `src/nepi_apps/`), which has
no home outside `nepi_drones`. If you change anything under `sim_container/`, sync the
matching files into `nepi_simulation` too (see that repo's own git log for the established
"Sync X from nepi_drones" commit convention) — this handoff doc is duplicated there as well.
