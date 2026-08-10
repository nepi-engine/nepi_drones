# Simulator Auto-Launch Plan

**Status:** Draft, not yet built. Written before implementation per the user's
request; check items off in place as each is done.

## 0. What this adds, and why it's additive

Today `nepi_app_sim_connector` is deliberately **passive**: per
`SIMULATION_INTERFACE_SPEC.md`'s stated convention (restated in
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

- [ ] New topics on `sim_connector_app_node.py` (not `device_if_sim.py` —
      this logic is app-specific, not part of the reusable `SimDeviceIF`
      contract):
  - `sim/launch_simulator` (`std_msgs/String`) — a key from
    `launch_targets`.
  - `sim/stop_simulator` (`std_msgs/Empty`).
- [ ] New status message + latched topic, published by the app node
  directly (not `SimStatus` — a separate concern, separate message):
  `sim/launcher_status` (new `SimLauncherStatus.msg`):
  ```
  string available_launch_targets[]
  string available_launch_target_names[]
  string selected_launch_target
  string launcher_state      # idle | launching | running | stopping | failed
  string last_error
  ```
- [ ] New helper, `nepi_apps/nepi_app_sim_connector/api/simulator_launcher.py`
  (app-owned, imported by the node script, not part of `SimDeviceIF`):
  reads `simulator_launch_targets.yaml`, opens an SSH subprocess using
  `NEPI_SSH_KEY`, runs `launch_command` in the background, probes
  `heartbeat_host:heartbeat_port` a few times to confirm the launch actually
  came up before flipping `launcher_state` to `running`, and — since the
  launch target already knows its own `default_robot_config` — calls the
  *existing*, unmodified `select_simulator`/`select_robot_config` topics
  once confirmed, so the rest of the app behaves exactly as it already does
  today. The launcher is a convenience trigger for the existing passive
  flow, not a parallel path.
- [ ] New RUI component, `Nepi_IF_SimLauncher.js` — deliberately a **separate
  file** from `Nepi_IF_Sim.js` (which is already built, tested, and owned in
  spirit by the multi-simulator integration work) rather than editing it in
  place: a launch-target dropdown, Launch/Stop buttons, and a
  `launcher_state` indicator. Mounted alongside `NepiIFSim` in
  `NepiAppSimConnector.js`.

## 3. Build order

1. [ ] `SimLauncherStatus.msg` + wire it into `nepi_app_sim_connector`'s
   `CMakeLists.txt`/`package.xml` message list.
2. [ ] `simulator_launch_targets.yaml` with the Gazebo rover entry filled in
   (already have `sim_rover_gazebo` working end-to-end this session — reuse
   it, don't reinvent).
3. [ ] `simulator_launcher.py` helper — build and unit-test the SSH
   launch/probe/stop logic in isolation (a plain Python script, no ROS)
   before wiring it into the node.
4. [ ] Wire the two new topics + status publisher into
   `sim_connector_app_node.py`.
5. [ ] **Test Gazebo end to end** before touching any other simulator:
   RUI → select `gazebo_rover` → Launch → confirm Gazebo actually comes up
   on the VM, `launcher_state` reaches `running`, and — because the
   launcher calls the existing selectors on success — `sim/status`'s
   existing `bridge_connected` flips `true` exactly as it does today when
   you start `sim_rover_gazebo` by hand.
6. [ ] `Nepi_IF_SimLauncher.js`, wired into the RUI build (same
   `Nepi_IF_Apps.js`/`rui-app/src/apps/` mechanism already documented in
   `NEPI_APP_BUILD_AND_TEST_CHECKLIST.md`), verified live in a browser.
7. [ ] Repeat steps 2-5 for `webots_rover`, `stage_rover`, `pybullet_rover`
   — each is a config entry, not new code, once the mechanism itself works
   for Gazebo. This mirrors `MULTI_SIMULATOR_INTEGRATION_PLAN.md`'s own
   phase structure on purpose.

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
