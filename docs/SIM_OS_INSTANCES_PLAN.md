# Multi-OS-Instance Deploy Targets

**Status:** Built, not yet exercised against a real second machine. Everything
below is additive over `SIMULATOR_AUTO_LAUNCH_PLAN.md`'s existing single-VM
auto-launch mechanism -- nothing there changed. Not yet registered/tested on
any real machine (deliberately, per the user's own request while building
this) -- do that by hand against the actual code before considering this
closed, same "VM-side done, real confirmation still open" bar every other
plan doc in this repo uses.

## 0. What this adds, and why it's additive

Every target in `sim_container/simulator_launch_targets.yaml` hardcodes
`host: "127.0.0.1"`, `ssh_user: "suraj"`, `ssh_port: 12222` -- one specific
developer's one VM. `simulator_launcher.py`'s `launch()`/`stop()`/
`is_installed()`/`install()`/`_ssh_cmd()` all read those three fields straight
off each target's own dict already, so nothing about the existing
launch/install/ready-check machinery needs to change to support more than one
machine -- only *where those three fields come from* needs to become
selectable instead of fixed.

This plan adds exactly that: a small, separately-persisted registry of
additional "OS instances" (`api/os_instance_registry.py`), a header picker in
the Sim Connector RUI panel to choose which one is active, and a guided
setup flow that generates the exact copy-paste commands for registering a new
one -- replacing `SIM_VM_CONNECTION_SETUP.md`'s manual walkthrough with an
in-RUI wizard for anyone who doesn't want to hand-edit yaml/env vars. Nothing
in `device_if_sim.py`, `SimLauncherStatus.msg`, or any existing target's
`launch_command`/`install_command`/`check_installed_command` changed.

## 1. The one integration point

`os_instance_registry.py`'s `select(instance_id, launcher)` walks
`launcher.config["launch_targets"]` (a plain in-memory dict) and overwrites
every target's `host`/`ssh_user`/`ssh_port` with the selected instance's
values, in place. That's the entire generalization -- every existing code
path in `simulator_launcher.py` is already instance-aware the moment this
runs, since it already reads those three fields per-target on every call.

Only the SSH-control-leg port needs to be unique per instance, auto-allocated
starting at 12223 (one above the existing hardcoded default) so a new
instance can never collide with it. The sim-utility ports (9021-9029,
9041/9042/9046/9047, 9051) stay the fixed numbers every target already
hardcodes -- only one simulator ever runs at a time regardless of which
instance is selected (the existing, unchanged resource constraint), so there
is no per-instance port math needed for those.

## 2. New, additive surface

- `msg/SimOsInstancesStatus.msg` -- kept separate from `SimLauncherStatus` for
  the same reason that message was kept separate from `SimStatus`: a
  different concern (which machine, not which simulator/lifecycle state).
- `api/os_instance_registry.py` -- `register`/`verify`/`select`/`remove`/
  `list_instances`, plus `build_setup_commands` (the copy-paste block: SSH
  key reminder, both a systemd-unit and a plain-`autossh` reverse-tunnel
  variant with the real allocated port substituted in, and the verify
  one-liner). Persists one YAML file per instance under
  `/mnt/nepi_storage/databases/nepi_app_sim_connector/os_instances/` --
  mirrors `sim_connector_app_node.py`'s own `ROBOT_CONFIGS_STORAGE_DIR`
  convention (RUI-created entries live in the read/write database tree, not
  the hand-authored `$NEPI_CONFIG` tree `simulator_launch_targets.yaml`
  itself uses), not imported by `simulator_launcher.py` and does not import
  it either -- only `LauncherError` is shared, so this stays usable even on a
  deployment with no launch-targets config at all.
- New topics on `sim_connector_app_node.py` (app-specific, same layer
  `sim/launch_simulator` already lives at):
  - `sim/os_instances/register` (`std_msgs/String`, display name in)
  - `sim/os_instances/verify` (`std_msgs/String`, JSON
    `{"instance_id", "ssh_user", "host"?}` in)
  - `sim/os_instances/select` (`std_msgs/String`, instance_id in) -- calls
    `select()`, then republishes both `SimOsInstancesStatus` and (unchanged)
    `SimLauncherStatus` so the existing Simulator dropdown's install-check
    state re-evaluates against the newly-selected machine
  - `sim/os_instances/remove` (`std_msgs/String`, instance_id in)
  - `sim/os_instances/status` (latched `SimOsInstancesStatus`)
- `rui/Nepi_IF_SimOsInstances.js` -- mounted in `NepiAppSimConnector.js`
  **above** `<NepiIFSim>` (a new sibling, not an edit to `Nepi_IF_Sim.js` or
  `Nepi_IF_SimLauncher.js`): the "OS" header selector (verified instances +
  "+ Add New OS Instance"), the register/verify wizard, and a removable list
  of every registered instance regardless of status.

## 3. Build order

1. [x] `SimOsInstancesStatus.msg`, wired into `CMakeLists.txt`.
2. [x] `api/os_instance_registry.py` -- port allocation, persistence,
   SSH-key resolution (duplicated from `simulator_launcher.py`'s own
   `_ssh_key_candidates`/`_ssh_cmd` shape, not imported -- see that module's
   own docstring for why), `build_setup_commands`.
3. [x] Four new topics + `SimOsInstancesStatus` publisher wired into
   `sim_connector_app_node.py`; `select` also republishes
   `SimLauncherStatus`; a previously-selected instance re-applies onto a
   freshly-constructed `SimulatorLauncher` on node startup.
4. [x] `rui/Nepi_IF_SimOsInstances.js`, mounted above `<NepiIFSim>` in
   `NepiAppSimConnector.js`.
5. [x] Registered in `sim_connector_app_params.yaml`'s `rui_files`.
6. [x] This doc + a pointer from `docs/README.md` and a short note in
   `docs/SIM_VM_CONNECTION_SETUP.md`.
7. [ ] **Real verification, not done yet** -- register a genuine second
   machine through the RUI, confirm the generated setup commands actually
   work end to end (SSH key, reverse tunnel, Test Connection flipping to
   verified), select it, and confirm a Deploy click from
   `Nepi_IF_SimLauncher` actually reaches THAT machine rather than the
   default VM. Explicitly deferred by request -- this VM is meant to be
   registered as that second machine by hand afterward, not by this pass.

## 3a. Follow-up fixes from live feedback (2026-09-02)

Two real gaps reported after the first deploy, both fixed in place -- not a
redesign:

- **The picker showed a generic "Default (this app's own config)" entry
  whenever nothing had ever been explicitly selected.** Reported live: "there
  shouldn't be any 'default' -- it should name the name of the one it's
  currently connected to." Fixed by having `ensure_baseline()` register a
  real, always-present, already-`verified` pseudo-instance (id `baseline`)
  representing whatever connection `simulator_launch_targets.yaml` itself
  hardcodes -- named from a new optional `connection_display_name` yaml key,
  falling back to `"{ssh_user}@{host}:{port}"` (a real identity, never a
  placeholder) when that key is absent. Called once at startup from
  `sim_connector_app_node.py`, right after the launcher loads its config, so
  a genuinely first boot (or any boot where nothing was ever explicitly
  selected) defaults `selected_instance_id` to `baseline` instead of leaving
  it empty. `baseline` is otherwise a completely ordinary instance record --
  `select('baseline', launcher)` needs no special-casing (it just re-applies
  its own captured host/ssh_user/ssh_port, which is a no-op the first time
  and a genuine "go back to the hardcoded default" the next time a real
  instance was previously selected) -- only `remove()` special-cases it, by
  refusing (nothing to fall back to if it were gone). The RUI drops its old
  empty-selection branch entirely and just relies on `baseline` showing up
  in the normal instance loop; `renderInstanceList` hides the Remove button
  for it specifically.
- **The generated setup commands were hard to follow.** Reported live:
  "make it easier to follow and give the right places to go properly." The
  original shape read as one continuous shell script with `#`-comments
  explaining each line, when it's actually two machines (the new one and
  the NEPI device) and two alternative tunnel paths (systemd vs plain
  `autossh`) interleaved. `build_setup_commands()` now renders three
  explicitly numbered steps, each with its own `Where: on the NEW machine
  (<name>)` line, blank-line-separated command blocks (not comment-prefixed
  shell), and every value the operator must fill in marked
  `<REPLACE with ...>` rather than a bare angle-bracket placeholder that
  could be mistaken for a literal token. The RUI's own field labels next to
  the block were reworded to match (`"Setup Commands -- Run On The New
  Machine, In Order"`, etc.).

## 3b. SSH-keygen step removed entirely (2026-09-02)

Reported live: "most people will already have the normal os to nepi
connection working, so theres no reason to have the ssh pubpriv thng
there. make it so just hte reverse ssh is set up." The original 3-step
wizard's Step 1 (`ssh-keygen -f ~/.ssh/nepi_default_ssh_key`) assumed the
machine being registered had never touched that path before -- true for a
genuinely new machine, but **false and actively destructive** for the
overwhelming common case: a machine that already completed NEPI's own
Remote Setup (`NEPI_REMOTE_SETUP.md`) and already has a working,
device-trusted `nepi_default_ssh_key`. Confirmed the hard way this session:
running Step 1 against exactly that kind of machine (this project's own dev
VM) overwrote the working private key with a fresh, never-authorized one,
locking out both the NEPI device's host and its running container until
physical console access could restore it (`docker exec` from the host was
the eventual fix for the container side, once the host was reachable
again).

`build_setup_commands()` now generates a 2-step wizard (tunnel, then
verify) that never touches `~/.ssh/nepi_default_ssh_key` at all -- it
documents up front that the new machine is assumed to already have Remote
Setup done, and points at `NEPI_REMOTE_SETUP.md` for the rare case where it
genuinely doesn't. No code elsewhere needed to change: `verify()`/`select()`
never cared how the tunnel got set up, only that it eventually works.

## 3c. Concrete example instead of a bare placeholder, and a real port bug fix

Reported live: "what would this be in my case? give the example and also
put that as the example in the RUI" (about the leftover
`<REPLACE with your NEPI device's user>@<REPLACE with its IP or hostname>`
placeholder). Fixed two ways:

- `_guess_device_ip()` (new, zero-ROS-dependency) fills `DEVICE_SSH_HOST`/
  the `autossh` target with this device's own real IP, falling back to a
  placeholder only if that genuinely can't be determined. **Reads
  `/opt/nepi/etc/nepi_system_config.yaml`'s own `NEPI_STATIC_IP` first, not
  a network-routing guess** -- found live that this device is multi-homed
  (a static `eth0` at 192.168.179.103, the address every other machine in
  this whole session actually used, alongside a DHCP `wlan0` used only for
  the device's own internet uplink) and a naive "connect to 8.8.8.8, see
  which interface answers" guess picked the wrong one (`wlan0`, since
  that's the default route). The socket-routing guess is now only a
  fallback for a deployment with no system config file to read.
- **Found and fixed a real bug while verifying this against the actual
  device**: the generated commands (and `SIM_VM_CONNECTION_SETUP.md`'s own
  manual Step 2 examples, copied from the same source) used
  `DEVICE_SSH_PORT=22`, but the device's host-level `nepi` OS account has a
  `/sbin/nologin` shell and can't SSH in at all -- confirmed directly.
  Port 2222 (the container's own sshd, where `nepi` has a real shell) is
  the only combination that actually works, matching what
  `nepi_tunnel()`/`nepi-tunnel.service` already default to when unset. Both
  the wizard and the manual doc now hardcode 2222, documented as this
  platform's fixed convention (not a per-deployment placeholder) rather
  than something to "replace."

## 4. Explicitly not doing

- ArduPilot SITL auto-install -- unchanged, still manual-fallback-only (no
  honest single install command for a from-source build).
- Running more than one simulator across instances at once -- matches the
  existing, resource-driven "one simulator at a time" constraint. Multiple
  instances can be *registered/reachable* simultaneously; only one is ever
  the active deploy target.
- A global NEPI top-nav element -- there's no existing top-level picker in
  the base `nepi_rui` to extend, and adding one would mean touching
  `nepi_engine_ws`/`nepi_apps`/`nepi_rui`, outside this repo's own
  push/approval boundary. Scoped instead to this app's own panel.
- Restoring a target's original hardcoded host/ssh_user/ssh_port after a
  different instance has been selected -- there is no stored "original"
  copy to restore to; restart the app (which reloads
  `simulator_launch_targets.yaml` fresh) if that's ever actually needed.
