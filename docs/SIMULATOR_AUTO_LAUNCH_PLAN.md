# Simulator Auto-Launch Plan

**Status:** Draft, not yet built. Written before implementation per the user's
request; check items off in place as each is done.

## 0. What this adds, and why it's additive

Today `nepi_app_sim_connector` is deliberately **passive**: per
`SIM_DEVICE_IF_CONTRACT.md`'s stated convention (restated in
`MULTI_SIMULATOR_INTEGRATION_PLAN.md`'s deferred-scope section), a new
simulator is only ever a new bridge script + a new `robot_configs` entry —
never a change to `device_if_sim.py`'s core contract. You start the
simulator and its bridge script yourself (on a VM, wherever), and the app
just detects the connection once it happens.

This plan adds a **launch button**, not a replacement for that model: select
a simulator + robot config in the RUI, and something SSHes into the right
VM and runs that simulator's own launch command for you, using the exact
same bridge scripts and `robot_configs` entries that already exist. It does
this **without touching `device_if_sim.py` or its existing selector
contract** — new topics, a new small status message, and a new RUI section,
all additive. If this plan ever finds itself needing to change the existing
contract to work, that's a stop-and-write-up moment (same rule the other
plan already follows), not something to patch around.

## 1. Where configuration lives, and why (locked in per user decision)

**Most secure, still discoverable — reusing what already exists rather than
inventing a new secret-handling mechanism:**

- **No SSH credentials in any committed file, ever.** The SSH private key
  path comes from the `NEPI_SSH_KEY` environment variable, exactly the
  convention every `deploy_*.sh` script in this repo already uses
  (`NEPI_SSH_KEY=${NEPI_SSH_KEY:-~/.ssh/nepi_default_ssh_key}`). Nothing new
  to remember, nothing new to leak — the same key already trusted to deploy
  code is what launches a simulator.
- **Per-simulator launch targets live in `nepi_drones`, not `nepi_apps`.**
  `nepi_drones` is the dev/test-tooling repo; a real deployed device never
  needs to know how to SSH into a dev VM to launch Gazebo, so this config
  has no business shipping with the app. Concretely:
  `nepi_drones/sim_container/simulator_launch_targets.yaml` — plain text,
  host/port/launch-command only, safe to commit and read.
- **`sim_connector_app_params.yaml` (in `nepi_apps`, ships with the app)
  stays exactly as it is** — `robot_configs` describes robot *capabilities*,
  which is legitimately part of the app's own contract and already correct.
  Launch targets describe *dev infrastructure*, a different concern, hence
  the different file and different repo.

### `simulator_launch_targets.yaml` shape

```yaml
# No credentials here -- SSH key path comes from the NEPI_SSH_KEY env var
# (same convention as every deploy_*.sh in this repo), read at launch time,
# never stored. host/user/port only.
launch_targets:
  gazebo_rover:
    display_name: "Gazebo (generic rover)"
    host: "192.168.x.x"
    ssh_user: "suraj"
    ssh_port: 22
    # Run on the target exactly as a human would from an interactive shell --
    # sourcing .bashrc picks up sim_rover_dev_env.sh's function definitions.
    launch_command: "bash -lc 'sim_rover_gazebo'"
    stop_command: "bash -lc 'pkill -x gzclient; pkill -x gzserver; pkill -f sim_heartbeat_listener.py; pkill -f sim_bridge_node.py'"
    # Matches this target's own bridge script defaults -- used to confirm
    # launch actually succeeded (heartbeat probe) and to auto-select the
    # right sim once it's up.
    heartbeat_host: "192.168.x.x"
    heartbeat_port: 9022
    bridge_port: 9023
    # Which sim_connector_app_params.yaml robot_configs entry this target's
    # bridge expects -- auto-selected on successful launch.
    default_robot_config: ground_robot_2_wheel
  webots_rover: {}   # filled in once Phase 2 below is reached
  stage_rover: {}
  pybullet_rover: {}
```

## 2. New, additive surface (nothing existing changes)

- [x] New topics on `sim_connector_app_node.py` (not `device_if_sim.py` —
      this logic is app-specific, not part of the reusable `SimDeviceIF`
      contract):
  - `sim/launch_simulator` (`std_msgs/String`) — a key from
    `launch_targets`.
  - `sim/stop_simulator` (`std_msgs/Empty`).
- [x] New status message + latched topic, published by the app node
  directly (not `SimStatus` — a separate concern, separate message):
  `sim/launcher_status` (new `SimLauncherStatus.msg`):
  ```
  string available_launch_targets[]
  string available_launch_target_names[]
  string selected_launch_target
  string launcher_state      # idle | launching | running | stopping | failed
  string last_error
  ```
- [x] New helper, `nepi_apps/nepi_app_sim_connector/api/simulator_launcher.py`
  (app-owned, imported by the node script, not part of `SimDeviceIF`):
  reads `simulator_launch_targets.yaml`, opens an SSH connection using
  `NEPI_SSH_KEY`, runs `launch_command`, and — since the launch target
  already knows its own `default_robot_config` — calls the *existing*,
  unmodified `select_robot_config` topic once confirmed, so the rest of the
  app behaves exactly as it already does today. The launcher is a
  convenience trigger for the existing passive flow, not a parallel path.
  Two corrections made during implementation, not in the original design
  above:
  - No separate `heartbeat_host:heartbeat_port` probe. Readiness is a
    per-target `ready_check_command` run over the *same* SSH connection
    used to launch — one mechanism instead of two, and no second port to
    configure per target.
  - Does **not** call `select_simulator`. That selector picks among other
    simulator-*capable NEPI devices* discovered on the ROS graph
    (`simDiscoveryCb`'s `DeviceRBXStatus` scan) — a different axis entirely
    from which simulator *software* a launch target starts on a dev VM.
    Only `select_robot_config` applies here.
  - Also had to become session-lifetime-aware: closing the ssh exec channel
    ends the remote login session, and on a systemd-managed host that tears
    down every process still in that session's cgroup regardless of
    `nohup`. `launch()` now holds the connection open with `Popen` for as
    long as the simulator should run (`launch_command` itself ends in
    `wait <pids>`), mirroring how `sim_rover_gazebo()` keeps a human's
    terminal open for the same reason.
- [x] New RUI component, `Nepi_IF_SimLauncher.js` — deliberately a **separate
  file** from `Nepi_IF_Sim.js` (which is already built, tested, and owned in
  spirit by the multi-simulator integration work) rather than editing it in
  place: a launch-target dropdown, Launch/Stop buttons, and a
  `launcher_state` indicator. Mounted alongside `NepiIFSim` in
  `NepiAppSimConnector.js`.

## 3. Build order

1. [x] `SimLauncherStatus.msg` + wire it into `nepi_app_sim_connector`'s
   `CMakeLists.txt`/`package.xml` message list.
2. [x] `simulator_launch_targets.yaml` with the Gazebo rover entry filled in.
   Ended up meaningfully different from the sketch in section 1 below --
   see the corrections noted in section 2 above (no heartbeat port, an
   explicit `roscore` bootstrap, `DISPLAY`/`XAUTHORITY` exports, and a
   trailing `wait` instead of background-and-forget) -- all found by
   actually running it against the real VM, not reused as originally
   assumed from `sim_rover_gazebo`.
3. [x] `simulator_launcher.py` helper — built and unit-tested standalone
   (no ROS) against the real dev VM: launch, wait_until_ready, and stop all
   verified working end-to-end, including real Gazebo + the new bridge
   script coming up and `bridge_connected` flipping `true` on the device.
4. [x] Wire the two new topics + status publisher into
   `sim_connector_app_node.py`.
5. [x] **Test Gazebo end to end** — done against an isolated, uniquely-named
   test instance of the real node (separate listen port, separate ROS
   params) rather than the live production instance, to avoid disrupting
   the device's other running apps. `sim/launch_simulator` → `running` →
   `sim/status`'s `bridge_connected: true` and `selected_robot_config:
   ground_robot_2_wheel`, all confirmed live. `sim/stop_simulator` → `idle`
   and the VM-side processes actually exiting, also confirmed.
6. [x] `Nepi_IF_SimLauncher.js` written, wired into `NepiAppSimConnector.js`,
   registered in `sim_connector_app_params.yaml`'s `rui_files`, and built
   via a real `npm run build` on the device (production build, not dev
   server) — succeeded with no errors attributable to the new file.
   **Not yet visually verified in a browser** — that part is on the user.
7. [ ] Repeat steps 2-5 for `webots_rover`, `stage_rover`, `pybullet_rover`
   — each is a config entry, not new code, once the mechanism itself works
   for Gazebo. This mirrors `MULTI_SIMULATOR_INTEGRATION_PLAN.md`'s own
   phase structure on purpose.

## 3a. Follow-up fixes found while verifying on the real device

All four were real defects found by running against the live device and VM,
not by review. Recorded here because each one's failure mode was invisible
from the outside.

- [x] **`SIM_VEHICLE_DICT` never reached the app at all** — a pre-existing
  bug that predates this plan and broke the *existing* robot-config selector,
  not just auto-launch. `nepi_sdk/nepi_apps.py`'s `getAppsDict` extracts only
  `APP_DICT` (plus `RUI_DICT`, nested into it) from each app's params yaml and
  discards every other top-level key; `apps_mgr` then `set_param`s just that
  one `app_dict`. So this app -- the only one in the repo shipping a third
  top-level key -- silently ran with nothing but the capability-empty factory
  profile (`available_robot_configs: ['default']`).
  Fixed *inside the app* (`loadVehicleDictFromParamsFile`), reading its own
  installed params yaml via `system_folders['apps_param']` when the param is
  absent, rather than adding new generic behavior to `apps_mgr` -- core,
  shared by every app, and a stop-and-write-up change per this repo's rules.
  The param still wins when set, so a future `apps_mgr` that does propagate
  these keys needs no change here. Now reports all six configs.
- [x] **Launch-target config could never be found by the real app** — the
  original env-var-only opt-in (`NEPI_SIM_LAUNCH_TARGETS_CONFIG`) worked only
  for a hand-launched node, because `apps_mgr` spawns app nodes with no
  per-app env vars. Added a discoverable default path,
  `$NEPI_CONFIG/simulator_launch_targets.yaml` (`NEPI_CONFIG` is already in
  every app node's environment); env var still overrides. Absence of the file
  is still what safely disables auto-launch on a real device.
- [x] **SSH key resolution was wrong on-device** — `NEPI_SSH_KEY` is a bare
  *filename* in the platform's own environment (`NEPI_SSH_KEY_PATH` holds the
  path), and `apps_mgr` runs app nodes as **root**, so `~/.ssh` expands to
  `/root/.ssh`, which has no key. Now tries an ordered candidate list and
  picks the first that exists, so the launcher works unchanged both as an
  apps_mgr-spawned root process and standalone as a normal user.
- [x] **Stop was dangerously broad, and readiness false-positived** — both
  found against a live ArduPilot SITL Gazebo session sharing the VM:
  - `stop_command` ran `pkill -x gzserver; pkill -x gzclient`, which would
    have destroyed that unrelated simulation. Now scoped to the launch's own
    **process group** via a pidfile, so it reaches its own gazebo wrapper,
    gzserver, gzclient and bridge and nothing else.
  - Two Gazebos cannot share gzserver's port 11345: the second one's gzserver
    silently dies while its gzclient attaches to the *first* one's world. The
    launcher reported a fully successful launch (`running`,
    `bridge_connected: true`) while `generic_rover.world` had never loaded and
    `/rover/odom` had `Publishers: None` -- the bridge having merely
    registered itself as a subscriber. `launch_command` now refuses to start
    when any gzserver is already running, surfacing the reason in
    `last_error`, and `ready_check_command` now requires this target's own
    world in gzserver's command line *and* a real publisher on `/rover/odom`.
- [ ] **Editing `simulator_launch_targets.yaml` needs an app restart.**
  `SimulatorLauncher` reads the yaml once at construction (as its own
  docstring says), so a live config edit has no effect until the app is
  toggled off/on -- confirmed the hard way when a freshly-deployed
  refuse-to-launch guard didn't fire because the node still held the previous
  config in memory. Not yet addressed; the honest options are to re-read the
  file per launch request, or to surface the loaded config's mtime in
  `SimLauncherStatus` so a stale config is at least visible.

## 4. Deferred / explicitly out of scope for this plan

- Any change to `device_if_sim.py`'s existing constructor, selector
  contract, or `SimStatus`/`SimInfo` messages — matching the same rule the
  multi-simulator plan already committed to.
- Auto-*installing* a simulator that isn't already present on the target VM
  — this launches what `MULTI_SIMULATOR_INTEGRATION_PLAN.md`'s phases
  already installed and proved working by hand; it does not provision a VM
  from nothing.
- Multi-robot / multiple simultaneous launch targets running at once.
- Windows/macOS launch targets — the existing deploy scripts and this
  plan's `launch_command`/`stop_command` shape assume a POSIX shell on the
  target, matching every other tool in this repo.
