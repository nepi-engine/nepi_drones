# Sim Connector: Deploy & Live-Update Reference

Written 2026-09-19 after a long debugging pass that touched almost every layer of
the Gazebo sim integration. This is a "how it actually works, and why" reference,
not a plan — read it first if any of this breaks again before re-deriving
everything from scratch.

## 1. The topology (read this first)

There are **three** machines involved, and confusing them wastes enormous amounts
of debugging time:

- **This Claude Code session's own shell** runs directly on `SunnyZG14`, the
  user's WSL2 Ubuntu VM. This is the SAME machine that runs Gazebo/gzserver/
  gzclient and every `sim_container/scripts/*.py` VM-side script. No SSH needed
  to inspect or affect Gazebo — `ps aux`, `rosservice call`, editing
  `sim_container/` files, all act directly on the live simulator.
- **The NEPI device** (`nepi@nepi:2222`, key `~/.ssh/nepi_default_ssh_key`) is a
  genuinely separate machine (hostname `device1`) running the actual NEPI engine
  (`drivers_mgr`, `apps_mgr`, `sim_connector_app_node.py`, the RBX driver nodes,
  the RUI). This is reached only over SSH.
- **`nepi_drones`** (this repo) is the user's own dev repo, checked out on the
  VM. It is what the VM directly executes (`sim_container/scripts/*.py`,
  `sim_container/worlds/*.world`, `simulator_launch_targets.yaml`) — no deploy
  step needed for VM-side files, they're just live on disk. Device-side files
  (drivers, the app node, RUI) need to be copied over — see §8.

## 2. Deploy architecture: shared storage, not SSH

The device and the VM never open a live SSH/reverse-tunnel connection for
deploy commands. Everything goes through a shared-storage mailbox:
`vm_command_watcher.py` (a systemd service, `nepi-vm-command-watcher.service`,
running on the VM) polls
`/mnt/nepi_share_storage/databases/nepi_app_sim_connector/vm_commands/<instance_id>/`
for command files the device writes, executes them, and writes status files
back. `simulator_launcher.py` (device-side, in `nepi_app_sim_connector/api/`) is
the client of this mailbox.

Two separate protocols live in that same mailbox, and mixing them up causes
real bugs (see §7.6):

- **`deploy_state.yaml`** — a single persistent file, one `desired_target`
  field. This is `launch()`/`stop()`'s own primary transport
  (`_launch_via_deploy_state`). Only ONE target can ever be tracked this way at
  a time — setting a new `desired_target` makes the watcher stop whatever was
  running first. This is what the **primary slot** (Use Open Sim/Deploy/Kill)
  uses.
- **One-shot `cmd_<id>.json` / `status_<id>.json` request-response pairs** — used
  by `install`/`check_installed`/`ready_check`/`push_dimensions`, AND by the
  `'launch'` action's own `_handleLaunch`/`self.launch_procs` dict on the
  watcher side, which is **already keyed per target_key** and therefore safe
  for more than one target running at once. `launch()` never used this for
  itself (deploy_state.yaml superseded it), which is exactly why it was
  available, unused, for the **secondary slot** ("New Sim") to reuse — see §3.

## 3. Primary vs. secondary launch slots ("New Sim")

`sim_connector_app_node.py` tracks the primary deployment
(`active_launch_target`/`selected_launch_target`/`launcher_state`) exactly as it
always did — this is what "Deploy"/"Use Open Sim"/"Kill" operate on, and it was
deliberately left untouched by the secondary-slot work below (zero regression
risk to the already-verified primary path).

**"New Sim"** launches a second, genuinely concurrent target via
`launch_new_simulator` → `runLaunchSecondary` → `simulator_launcher.py`'s
`launch_secondary`/`wait_until_ready_secondary`/`stop_secondary` — these go
through the **one-shot mailbox path** (`_dispatch_shared_storage`, action
`"launch"`/`"stop"`/`"ready_check"`), NOT `deploy_state.yaml`, so a secondary
launch cannot stop whatever the primary slot has running.

For this to actually produce a second window, `gazebo_rover` and
`gazebo_quadcopter` in `simulator_launch_targets.yaml` each got their own:

- **`GAZEBO_MASTER_URI`** (rover: default `11345`; quadcopter: `11346`) — two
  gzserver processes cannot share the default port.
- **`ROS_MASTER_URI`** (rover: default `11311`; quadcopter: `11312`, its own
  `roscore -p 11312`) — this is the one people forget. `libgazebo_ros_api_plugin.so`
  advertises plain, un-namespaced names like `/gazebo/spawn_sdf_model`, and
  `camera_rig_controller_ardupilot.py`/`ai_targeting_controller_ardupilot.py`
  call those exact same absolute names. Two gzservers sharing ONE roscore would
  silently steal each other's `/gazebo/...` service registration (whichever
  advertises last wins) even with distinct Gazebo transport ports. Each target
  needs its OWN roscore, not just its own Gazebo port.
- Port-specific refuse-guards (`if timeout 1 bash -c "cat < /dev/null >
  /dev/tcp/127.0.0.1/<port>"` instead of a blanket `pgrep -x gzserver`) so
  launching one target never refuses because the OTHER one is already up.
- `stop_command`'s unscoped `pkill -x gzclient` was removed from both targets —
  it used to kill ANY gzclient window on the host, including the other
  target's, when stopping one of them. The pgid-based kill just above it
  already reaches the right one's own gzclient.

Known, disclosed limitation: `kill_all_gazebo` is still host-wide (kills EVERY
gzserver/gzclient regardless of target) — it exists specifically for "a
gzserver this app never started is blocking me," and narrowing it to "just the
one in conflict" would need to know which port the conflict was on, which the
caller doesn't currently report.

"New Sim" on a target that's already open (primary or secondary) now warns
("You already have this robot config open...") and, if confirmed, redeploys
that slot in place — it does NOT create a genuine third window (no third port
pair exists).

## 4. Live updates: FOV, camera offsets, environment

**The key fact**: dimensions/geometry pushed via `pushDirtyDimensions` (writing
`dimensions.yaml` + running `generate_model_sdf.py` on the VM) only take effect
on the next FRESH gzserver launch — `generate_model_sdf.py` only runs when a
world is actually starting. Editing dimensions while a sim is already running
does NOT, by itself, change anything live.

There is a SEPARATE, older, already-working live-update channel:
`rbx_sim_node.py`/`rbx_ardupilot_node.py` expose `camera_offset_x/y/z/yaw/tilt`,
`scene_offset_*`, and `camera_fov_deg` as ordinary RBX **Settings**. Any update
to one of these calls `sendCameraSettings()`, which sends `{'type':
'camera_settings', ...}` to `sim_bridge_node.py` over its own TCP line
protocol, which does `respawnRoverWithCameraOffsets` — a live delete+respawn of
just the robot model with the new offsets/FOV baked in, no full redeploy.

**Fix (2026-09-18): wire the RUI's FOV box into this live channel.**
`sim_connector_app_node.py`'s `setDimensionsCb` now calls
`pushLiveDimensionsIfConnected('robot')` after every robot-dimensions edit,
which pushes `camera_fov_deg` as a live Setting update to whichever RBX device
is `self.selected_simulator` (see `pushLiveCameraFov`). FOV now updates
instantly, matching camera offsets.

**Environment is not a Setting-driven respawn by default.** `EnvironmentModelSpawner.set_active_model`
no-ops if `model_name == self.spawned_name` — re-selecting the SAME model
(e.g. switching between two custom dimensions configs that both map to
`obstacle_course`) sends an unchanged `"environment"` Setting value, which
`SettingsIF.update_setting_value` short-circuits on ("Setting allready set")
before `setEnvironmentAction` is ever called. Fix: `set_active_model` grew a
`force=True` path (invalidates the cached SDF text, deletes+respawns even when
the name matches), reachable via a new `REFRESH_ENVIRONMENT` RBX setup action
on `rbx_sim_node.py`. `deployEnvironmentCb` calls this **only when
`model_name == self.deployed_environment_model_name`** (i.e. re-deploying onto
the model that's already live) — NOT unconditionally. See §7.9 for why
unconditional was itself a bug.

**The "Environment Config" deploy dropdown used to go through a fragile ref.**
`Nepi_IF_Sim.js`'s `onDeployEnvironmentConfig` used to reach
`Nepi_IF_Sim-Controls.js`'s `setEnvironmentSetting` via
`this.simControlsRef.current.wrappedInstance` — a real but fragile mobx-react
5.4.2 Injector hop (see that ref's own long comment in the file). This worked
for the FOV box (`pushCameraFovLive`, same ref) but not reliably for the
environment dropdown in practice. Fixed by moving the whole thing server-side:
the RUI now just publishes the config's display name to a new
`sim/deploy_environment` topic; `sim_connector_app_node.py`'s
`deployEnvironmentCb` does the name→Setting-value translation and push
directly, using a cached, long-lived publisher (see §7.8 for why that caching
matters).

**Deploying a NAMED config (built-in or custom) resolves through its own saved
file, never by guessing from the name.** Every entry in the deploy
dropdown — "Flat", "Obstacle Course", "Aerial Obstacle Course", or a custom
name like `complexcourse` — is a real file in
`environment_configs/<name>.yaml`, each with its own `_environment_model` key
naming which real model directory it maps to. `deployEnvironmentCb` reads that
file, resolves the real model name from it, pushes THAT config's own geometry
to the VM, and only then flips the `"environment"` Setting. The old
RUI-side `setEnvironmentSetting` guessed the model by uppercasing the
config's own display name — which only ever worked for the built-ins, whose
names happen to match their model directory by construction.

## 5. Discovery & detection

- **Heartbeat-miss debounce was sized for a transport that no longer exists.**
  `rbx_sim_discovery.py`'s `HEARTBEAT_MISS_THRESHOLD` (6) and
  `HEARTBEAT_LISTEN_TIMEOUT_SEC` (6s) were both raised in an earlier session to
  absorb reverse-SSH-tunnel jitter during a reboot/settling burst. That tunnel
  is gone (shared storage only, see §2) — lowered to 2 and 4s respectively.
  Detecting a killed sim went from 20-30s to single digits.
- **ArduPilot SITL was never detected because the driver's own discovery
  "connection" option was stuck on `SERIAL`.** `rbx_ardupilot_discovery.py`
  only probes the SITL/MAVLink TCP endpoint when its `connection` Discrete
  Setting is `SITL`; the default is `SERIAL` (physical-hardware probing).
  `sim_connector_app_node.py` now pushes `connection=SITL` automatically
  (`ensureArdupilotSitlConnection`) whenever it launches `gazebo_quadcopter`
  (primary or secondary), so this never needs a manual driver-settings flip.
  Never reverted back to `SERIAL` automatically — this app can't know whether a
  physical ArduPilot also needs to keep working on the same device.

## 6. The dimensions editor

- **`checkRobotDimensionsViable`** (mirrored client-side in `Nepi_IF_Sim.js`
  and server-side in `sim_connector_app_node.py`, the server copy being the
  authoritative gate everything actually writes through) checks two AABB
  overlaps: side-to-side (`track_width_m <= wheel_width_m`) and, since
  2026-09-18, front-to-back (`wheelbase_m <= 2 * wheel_radius_m` — the front
  and rear wheels are circles of that radius, `wheelbase_m` apart
  center-to-center). The front-to-back case was missing entirely before.
- **Custom obstacles (circle/wall/triangle) saved onto an `obstacle_course`-
  based config didn't render.** `generate_model_sdf.py`'s `obstacles` list
  handling lived ONLY in `buildCustomObstaclesSdf` (the separate
  `custom_obstacles` model); `buildObstacleCourseSdf` never read the field at
  all, even though the RUI's Add Wall/Circle/Triangle buttons write it onto
  ANY environment config regardless of which model it maps to. Fixed by
  extracting the obstacle-rendering loop into a shared `_renderExtraObstacles(dims)`
  and calling it from both builders.
- **Side (elevation) views** were added for robot (`renderRobotDimensionsSideDiagram`
  — chassis height / ground clearance, neither representable top-down) and the
  ground obstacle course (`renderEnvironmentDimensionsSideDiagram` — wall/baffle
  height and the ramp's actual rise/angle; the ramp math there matches
  `buildObstacleCourseSdf`'s own `run`/`ramp_z`/`plateau_z` exactly). No new sync
  plumbing was needed — both views read/write the same
  `<role>_dimensions_fields`/`preview_fields` state via the existing
  `startDimensionDrag` mechanism, so a drag on either view already redraws
  both.
- **The environment config Delete button was wired to the wrong dropdown's
  state.** It's rendered directly below the read-only "Environment YAMLs"
  viewer dropdown (`viewingName`), but read
  `this.state.environment_dimensions_selected_config` — the EDIT dropdown,
  elsewhere on the page. Picking a name in the viewer and clicking Delete
  right below it silently deleted whatever the (unrelated, possibly stale)
  edit selection was instead. Fixed: `onDeleteDimensionConfigClicked` now
  takes the name explicitly, and the button passes `viewingName`.
- **Deleting the config that's currently deployed** is now blocked, both
  client-side (an immediate alert) and server-side
  (`deployed_environment_config_name`, authoritative regardless of client) —
  "deploy something else first."

## 7. Bug catalogue (root cause → fix), for quick lookup

Ordered roughly as debugged, most instructive first.

1. **`nepi_controls.py`'s empty-list sentinel** (`value != ['']` /
   `options != ['']`) treated an ordinary Settings update's empty `options`
   field as "please wipe the options list," silently zeroing a Selection/
   Discrete setting's valid-options on its first ordinary value update.
   Fixed to `len(value) > 0` / `len(options) > 0`. This one bug explained most
   of "settings don't stick" across the whole session.
2. **Class-attribute-vs-instance-attribute leaks** in `SimDiscovery`
   (`active_devices_dict`/`launch_time_dict`/`dont_retry_list`) and
   `ArdupilotDiscovery` (same three, plus a log-name field) — declared at
   class-body level with no `self.x = ...` in `__init__`, so state leaked
   across every instance/restart. Fixed by assigning fresh copies in
   `__init__`.
3. **`rbx_sim_node.py` read `~drv_dict` (private-namespace) instead of the
   fully-qualified param path** drivers_mgr actually wrote it to — caused an
   endless launch-crash-relaunch loop whenever `ROS_NAMESPACE` didn't happen to
   resolve the relative name the same way.
4. **`READY_CHECK_ATTEMPTS` (60s) was too short for a cold boot**, and worse,
   the timeout path called `stop()` before reporting failure — a slow-but-
   genuinely-succeeding deploy got reported as failed AND killed out from
   under itself. Raised to 40 attempts (120s).
5. **The deployed RUI JS bundle was stale relative to source** — a device
   reboot resets `/opt/nepi` (see §8) to whatever `system_cfg` last had, which
   can lag several commits behind `nepi_drones`. Symptom: fixes that were
   correct in source and even confirmed working via direct topic publishes did
   nothing when clicked in the actual browser. No code fix — just: rebuild
   (`npm run build` in `nepi_rui/src/rui_webserver/rui-app`, Node 10 works fine
   despite the target being 8.11.1) and redeploy BOTH `/opt/nepi` (live now)
   and `system_cfg` (survives the next reboot).
6. **Reverse SSH remnants** — an earlier pass removed a leftover reverse-SSH
   fallback that the shared-storage architecture (§2) was supposed to have
   fully replaced.
7. **A camera-offset live-push helper (`getOrCreateDynamicPublisher`) that
   created a fresh `rospy.Publisher` and published immediately** lost its
   first message almost every time — the classic ROS "no subscriber has
   connected yet" gap. Fixed by caching publishers per-topic on `self` and
   giving a brand-new one up to 1s to get its first connection before the
   first publish.
8. **`environment_models.py`'s `_delete` never waited for the deletion to
   actually finish** before the caller moved on to `_spawn` — `DeleteModel`
   returning success means Gazebo ACCEPTED the request, not that the model is
   gone. Racing a spawn against an in-flight deletion is exactly the class of
   bug `sim_bridge_node.py`'s own camera-respawn code already documents at
   length for the ROVER model; `environment_models.py` never had the
   equivalent wait at all. This was the actual cause of "aerial/obstacle
   course flashes in then disappears" (2026-09-19) — added
   `_waitForModelGone`, a `get_world_properties` poll with a 3s timeout,
   between `_delete` and the caller's next `_spawn`.
9. A **contributing cause of the same flash-then-disappear symptom**: an
   earlier version of `deployEnvironmentCb` called the `force=True` refresh
   (§4) UNCONDITIONALLY on every deploy, not just when re-deploying the same
   already-live model. For a genuine switch to a different model, this fired
   a redundant SECOND delete+respawn moments after the first (correct) one —
   which, combined with bug 8 above, could fail silently and leave the model
   deleted with no successful respawn. Fixed by tracking
   `deployed_environment_model_name` and only forcing when the resolved model
   name matches it.

## 8. Persistence: `/opt/nepi` is ephemeral

**A device reboot wipes `/opt/nepi` back to whatever `/mnt/nepi_config/system_cfg`
holds.** Every fix deployed only to `/opt/nepi/...` during a session is lost on
the next reboot unless it is ALSO copied to its `system_cfg` mirror:

| Live path (ephemeral) | Persistent mirror |
|---|---|
| `/opt/nepi/nepi_engine/lib/nepi_app_sim_connector/sim_connector_app_node.py` | `/mnt/nepi_config/system_cfg/src/nepi_apps/nepi_app_sim_connector/scripts/sim_connector_app_node.py` |
| `/opt/nepi/nepi_engine/lib/python3/dist-packages/nepi_api/simulator_launcher.py` | `/mnt/nepi_config/system_cfg/src/nepi_apps/nepi_app_sim_connector/api/simulator_launcher.py` |
| `/opt/nepi/nepi_engine/lib/nepi_drivers/rbx_*.py` | `/mnt/nepi_config/system_cfg/src/nepi_drivers/rbx_drivers/rbx_*.py` |
| `/opt/nepi/nepi_engine/lib/python3/dist-packages/nepi_sdk/nepi_controls.py` | `/mnt/nepi_config/system_cfg/src/nepi_engine/nepi_sdk/src/nepi_sdk/nepi_controls.py` |
| `/opt/nepi/nepi_rui/.../rui-app/src/Nepi_IF_Sim*.js` | `/mnt/nepi_config/system_cfg/src/nepi_apps/nepi_app_sim_connector/rui/Nepi_IF_Sim*.js` |
| `/opt/nepi/nepi_rui/.../rui-app/src/NepiDeviceRBX.js` | `/mnt/nepi_config/system_cfg/src/nepi_rui/NepiDeviceRBX.js` |
| `/mnt/nepi_config/simulator_launch_targets.yaml` | already the live config path, not ephemeral |

`sim_container/` (VM side) needs no deploy step at all — it's read live off
this VM's own `nepi_drones` checkout.

The compiled RUI **build output** (`build/static/js/main.*.js`) is not known to
have its own persistent mirror distinct from rebuilding from `system_cfg`'s
source on next boot — if a reboot doesn't rebuild automatically, the compiled
bundle may still need a manual rebuild+redeploy afterward. Not fully verified
this session; treat a post-reboot "my fix isn't showing up" as "check whether
the RUI needs rebuilding" first (§7.5).

**Manually restarting a manager (`drivers_mgr`) or app
(`sim_connector_app_node.py`) is NOT the same as a reboot** and needs care:

- Apps (anything under `apps_mgr`) restart cleanly via
  `apps_mgr/update_state` (`nepi_interfaces/UpdateBool`, `name` = the
  `pkg_name` key like `nepi_app_sim_connector`, NOT the ROS node name) —
  publish `value: false` then `value: true` a few seconds apart. Takes ~15-20s
  each way; apps_mgr's own check-and-update cycle runs every 5s.
- Managers (`drivers_mgr`, etc.) have no such toggle. Killing one
  (`sudo kill <pid>`) and relaunching it manually needs the EXACT boot
  environment or it fails in confusing ways (missing `PYTHONPATH`,
  `LD_LIBRARY_PATH`, `ROS_NAMESPACE`, or `PATH` each produce a different
  half-working failure). The reliable way: `sudo -E /opt/nepi/nepi_engine/env.sh
  nohup python /opt/nepi/nepi_engine/lib/nepi_managers/<name>.py __name:=<name>`,
  with `ROS_NAMESPACE=/nepi/device1` exported first. `env.sh` sources the
  correct `setup.sh`/`ROS_PACKAGE_PATH` — reconstructing the environment by
  hand from a sibling process's `/proc/<pid>/environ` is NOT equivalent (it's
  missing the catkin devel-space wiring `rosrun`/`roslaunch` need to resolve
  package-relative paths correctly).

## 9. Known limitations (not fixed, deliberately)

- `kill_all_gazebo` is host-wide, will take out a "New Sim" secondary instance
  too. See §3.
- No genuine THIRD concurrent instance — only rover+quadcopter have their own
  port pair.
- `nepi_sdk.logger`'s `log_name_list=[]` mutable-default-argument bug (glued
  log-prefix chains like `rbx_sim_discovery: : rbx_ardupilot_discovery: :`) is
  cosmetic only, left unfixed by explicit time tradeoff.
- The RUI build-output persistence question in §8 is unresolved.
- **`sim_control_relay_vm.py`'s device→VM control-signal relays (reset/
  teardown/start-trigger) take a few seconds to establish after a fresh
  `gazebo_quadcopter` deploy** (retrying every 2s against "Connection
  refused" until `rbx_ardupilot_node.py`'s own device-side listeners come
  up). Starting a follow-mission script within that window can get "Sim
  target start not reachable... timed out" even though everything is
  otherwise healthy — not fixed, just noted; retrying the script (or
  waiting ~5-10s after Deploy before launching it) works.

## 10. A confirmed reboot-recovery procedure (2026-09-21)

The device (`nepi@nepi:2222`) runs its NEPI engine inside an overlayfs
container (confirmed via `/proc/1/cgroup` + `mount`, containerd-managed).
**A device reboot resets `/opt/nepi` to whatever was baked into the container
image**, not to whatever `/mnt/nepi_config/system_cfg` currently holds — §8's
own "reboot wipes `/opt/nepi` back to `system_cfg`" framing undersells this:
`system_cfg` is the correct, intact reference copy, but nothing copies it
into `/opt/nepi` automatically. Every fix in §8's table, including the
compiled RUI bundle, must be **manually re-applied after every reboot**:

1. Compare file sizes/dates between each `/opt/nepi/...` path and its
   `system_cfg` mirror (§8's table) — a reverted file is smaller/older.
2. `cp` every reverted file from its `system_cfg` mirror back to its live
   `/opt/nepi` path.
3. Rebuild the RUI: `export NVM_DIR=$HOME/.nvm && . "$NVM_DIR/nvm.sh"` (node
   14.1.0 via nvm on this device), then `npm run build` in
   `/opt/nepi/nepi_rui/src/rui_webserver/rui-app`. Confirm
   `build/index.html` references the new hashed bundle filename — no
   restart of the RUI webserver process is needed, it serves static files
   directly off disk.
4. Restart `nepi_app_sim_connector` via `apps_mgr/update_state`
   (`nepi_interfaces/UpdateBool`, `value: false` then `value: true` ~15-20s
   apart) so the running app process picks up the restored Python source
   (it only loads once, at process start).
5. Restart `drivers_mgr` (no update_state toggle exists for managers): kill
   its PID, then `sudo -E env ROS_NAMESPACE=/nepi/device1
   /opt/nepi/nepi_engine/env.sh nohup python
   /opt/nepi/nepi_engine/lib/nepi_managers/drivers_mgr.py __name:=drivers_mgr
   < /dev/null > /tmp/drivers_mgr_restart.log 2>&1 &`. It re-discovers and
   relaunches every RBX driver node fresh a short while later — confirm via
   `ps aux` that `rbx_sim_node.py`/`rbx_ardupilot_node.py` have a start time
   AFTER the file restore, not before.

## 11. `drone_follow_object_mission_script.py` / sim target chair bugs (2026-09-21)

Two more `nepi_interfaces/Control` field-name bugs, same root cause as §7.1's
`nepi_controls.py` one but in this mission script's own inlined RBX-settings
code (not shared SDK code, so not covered by that fix):

- **`rbx_settings_callback` referenced nonexistent `Control` fields**
  (`set_int`/`set_float`/`set_strings`/`set_bool`/`set_index`/
  `string_options`) — `Control.msg` has none of these, only a plain
  `string[] value` (see `nepi_controls.py`'s `update_status_msg`, which is
  what actually builds these messages). Every real invocation raised
  `AttributeError: 'Control' object has no attribute 'set_float'`, which
  meant `self.rbx_settings` never got set and the script hung forever on
  "Waiting for current rbx settings to publish" — it never even reached
  `rospy.on_shutdown(self.cleanup_actions)`. The identical bug existed in
  the takeoff-height override's `UpdateControl` builder a few lines earlier
  (`setting_msg.set_float = ...`/`setting_msg.type = ...`, also invalid).
  Fixed: both now just read/write `.value` (a `string[]`), matching how
  every other current-API code in this codebase (`nepi_controls.py`,
  `sim_connector_app_node.py`'s `UpdateControl` pushes) already does it.
- **`ai_targeting_controller_ardupilot.py`'s start/teardown triggers were
  each a one-shot thread**, and teardown called `rospy.signal_shutdown()`
  afterward, on the assumption a "launch trigger" mechanism
  (`sim_launch_listener.py`, tied to the older manual
  `nepi_sitl_dev_env.sh` dev workflow) would relaunch a fresh instance for
  the next run. That relaunch path is dead in the current Deploy-button
  flow — confirmed live: stopping the follow script correctly despawned
  the chair and exited the whole node, but starting the follow script a
  second time (without redeploying `gazebo_quadcopter`) got a successful-
  looking local reply but no chair, since nothing was listening on the VM
  side to spawn one anymore. Fixed by replacing the two one-shot threads
  with one persistent `triggerLifecycleLoop` that cycles
  start→spawn→teardown→despawn→repeat for the node's whole lifetime, and by
  actually clearing `self.target_spawned` in `despawnTargetModel` (never
  reset before, harmless only while this was a single-cycle-then-exit
  design).

Verified live end-to-end after both fixes: deploy `gazebo_quadcopter`,
launch `sim_ai_targeting_bridge_script.py` then
`drone_follow_object_mission_script.py`, chair spawns, drone takes off and
tracks it (range converging, live `follow_debug.log` entries), cancel the
script (chair despawns, drone disarms/teleports home via `RESET_SIM`), then
launch the follow script again with NO redeploy in between — chair spawns
again, tracking resumes. Repeated twice with the same clean result each time.
