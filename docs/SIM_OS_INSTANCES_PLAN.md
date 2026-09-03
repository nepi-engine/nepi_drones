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

## 3d. Missing `autossh` install step

Reported live: a fresh WSL machine following the wizard hit
`Command 'autossh' not found`. Both options need it (Option A's
`nepi-tunnel.service` execs `autossh` under the hood same as Option B does
directly), so `build_setup_commands()` and `SIM_VM_CONNECTION_SETUP.md`'s
own Step 2 now both open with a one-line install-if-missing check
(`command -v autossh >/dev/null || sudo apt-get install -y autossh`) before
either option, rather than assuming it's already present.

## 3e. Verify step ran on the wrong machine, and a bracket-placeholder shell bug

Reported live, from an actual attempt: `ssh -p 12223 <REPLACE with your
username on this machine>@localhost echo ok` pasted verbatim gave
`-bash: REPLACE: No such file or directory` (`<` is shell redirection
syntax -- pasting a bracketed placeholder literally doesn't fail
obviously, it breaks the shell parse), and after fixing that by hand,
`ssh -p 12223 suraj@localhost` / `nepi@localhost` both got "Connection
refused" -- because **the command was run on the wrong machine**. The
`-R 12223:127.0.0.1:22` tunnel makes the *NEPI device* listen on 12223 and
forward back to *this (new) machine's* own sshd -- so `localhost:12223`
only means anything from the device's side. `SIM_VM_CONNECTION_SETUP.md`'s
own original Step 2 said this correctly ("run ON THE DEVICE"); the
wizard's Step 2 regressed from that when it was restructured for clarity
earlier this session, losing the WHERE distinction it had needed most.

Fixed by making the RUI's own "Test Connection" button (which already runs
the equivalent check from the device's own side, correctly) the *only*
step described in the normal case -- Step 2 now just says "enter your
username, click Test Connection," with the manual command demoted to an
explicitly-labeled fallback ("has to run ON THE NEPI DEVICE ITSELF... never
on the new machine"). Every remaining placeholder in this module
(`YOUR_USERNAME_ON_THE_NEW_MACHINE`, `YOUR_NEPI_DEVICE_IP_OR_HOSTNAME`) and
in `SIM_VM_CONNECTION_SETUP.md`'s own manual doc now avoids angle brackets
entirely, so pasting one literally produces an obvious "no such user"/"no
such host" failure instead of a cryptic shell parse error.

## 3f. First real end-to-end test (2026-09-02) -- two genuine gaps found and fixed

Ran the actual flow against the real device for the first time this
session (register → tunnel → verify → select → remove), using this repo's
own dev VM as the "new machine," rather than just reading code. Found two
real, previously-undocumented gaps -- neither was a bug in existing code,
both were missing prerequisites the wizard never mentioned:

- **No SSH server on the new machine.** This VM had an SSH *client* only
  (used all session to reach the device) -- no `sshd`, no
  `openssh-server` package. The reverse tunnel's whole point is forwarding
  the device's port back to *this* machine's own sshd; without one
  listening, `autossh` still reports success (an `-R` forward isn't
  validated against its target at connect time), and the failure only
  surfaces later, confusingly, at Test Connection. Installed
  `openssh-server` and started it to complete the test.
- **The device's own public key was never trusted on the new machine.**
  A working tunnel only gets the device's SSH client TO the new machine's
  sshd -- it still needs a key that machine trusts to actually log in.
  Confirmed directly: the tunnel itself worked immediately (`ssh -p 12223
  -i /home/nepi/.ssh/nepi_default_ssh_key suraj@localhost` succeeded) once
  the device's own public key (`nepi@numurus`, read from
  `/home/nepi/.ssh/nepi_default_ssh_key.pub` inside the container) was
  appended to the new machine's `authorized_keys`. This is exactly the
  second direction `SIM_VM_CONNECTION_SETUP.md`'s original Step 1 already
  required ("On the device... whose PUBLIC half is in the VM user's
  authorized_keys") -- the 2-step wizard dropped it when the keygen step
  was removed, since it read like the same concern.

Both fixed by adding a new **STEP 1 of 3** ("One-time prerequisites") ahead
of the tunnel step: an install-if-missing check for `openssh-server` (same
pattern as the existing `autossh` check), and a real, auto-read line to
append to `authorized_keys` -- `_device_public_key()` (new, mirrors
`_ssh_key_candidates()`'s own priority order, reading the `.pub` half of
whatever `_ssh_key()` would use) fills in the actual key content, not a
vague instruction. Steps renumbered 1-3; verify moved to Step 3, unchanged
otherwise.

**Also found (RUI, not the wizard text) while diagnosing the earlier
`suraj`/`nepi` mix-up**: the verify form's two fields were genuinely
ambiguous about *whose* username/host each one wants -- an operator (this
session) put the device's own `nepi` account in the username field and
their own `suraj` in the host field, exactly backwards. Both labels now
name the two machines explicitly ("YOUR OWN login username, on the NEW
machine (not the device's 'nepi' user)" / "Leave this BLANK -- only fill in
if you skipped the reverse tunnel entirely"), and the username field shows
a live placeholder example (the existing `baseline` instance's own real
ssh_user) rather than an abstract one.

Confirmed working end to end after these fixes: register → both new
Step-1 prerequisites → tunnel (autossh) → verify (real `ssh_user`,
`host` left blank) → `verified` → select → `simulator_launch_targets.yaml`
targets correctly repointed at the new instance → reselect `baseline` →
remove test instance, leaving a clean slate.

## 3g. Install button failed everywhere: `sudo: a terminal is required` (2026-09-02)

Reported live: `Install command exited 1: sudo: a terminal is required to
read the password`, on a target that had nothing specifically wrong with
it -- a foundational bug, not a per-target one. `_ssh_cmd()` never allocates
a pty (no `-t`), so *any* `install_command` that shells out to `sudo` has no
terminal for `sudo` to prompt on. This affects every target with an
`install_command` (`gazebo_rover`, `webots_rover`, `webots_quadcopter`,
`mujoco_rover`, and now `gazebo_quadcopter` -- see 3h below), not just the
one that happened to be tried first.

Piping a password over a non-interactive SSH session would mean storing or
transmitting a sudo password -- against this codebase's own "no
credentials, ever" design for SSH keys, extended here to sudo. Fixed
instead by requiring passwordless sudo on the target machine
(`/etc/sudoers.d/nepi-sim-connector`, `NOPASSWD:ALL`), detected rather than
assumed: `simulator_launcher.py`'s `install()` now checks `stderr` for the
telltale "a terminal is required to read the password" / "no tty present"
strings and raises a `LauncherError` with a new
`SUDO_NOPASSWD_FALLBACK_COMMANDS` as `manual_fallback_commands` -- which
surfaces automatically through the existing generic
`setLauncherError`/`publishLauncherStatus` mechanism, no changes needed to
`sim_connector_app_node.py` itself. Added as a new wizard prerequisite
(now **STEP 1c**, alongside the SSH-server and device-pubkey-trust steps
from 3f) and to `SIM_VM_CONNECTION_SETUP.md`. Deployed, verified
error-free, committed (`60c2e62`), and persisted via `nepicommit`.

## 3h. ArduPilot + Gazebo auto-install (2026-09-02)

`gazebo_quadcopter` had no `install_command` at all (a deliberate choice
until now -- ArduPilot SITL setup was judged a from-source build with no
honest single install command, see §4's own prior wording) -- its
`manual_fallback_commands`
was the only path, and it was itself incomplete: it built ArduPilot off
`master`, which doesn't build against Ubuntu 20.04's stock Python 3.8.10
(reported live: "these gazebo comamnds are also missing making it useable
with pyhon 3.8.10, since thats the max ubuntu 20.04 can go to"), and it
never installed Gazebo11 or built the `ardupilot_gazebo` bridge plugin at
all -- both are hard requirements of this target's own `launch_command`.

Fixed using the user's own validated manual recipe as the reference
sequence: pin ArduPilot to the `Copter-4.5` branch (the same
version-pinning pattern this project has hit before -- WPILib/robotpy
needed `2022.4.8` for a GCC10/C++20 issue, Webots needed R2023a for a glibc
issue; ArduPilot's Python-3.8 issue is the same recurring class of problem),
add the Gazebo11 OSRF apt repo/key/package steps, and clone+cmake-build+
`sudo make install` the `ardupilot_gazebo` bridge plugin. Every stage is
skip-if-already-done, matching `gazebo_rover`'s own `install_command`
convention, so re-running this on a partially-set-up box is harmless.

This also required reconstructing `iris_arducopter_cmac.world`
(`check_installed_command`/`launch_command`/`ready_check_command` all
hard-require it, but it existed nowhere in this repo -- a hand-customized
file that lived only on the original dev machine). Reconstructed as the
stock `iris_arducopter_runway.world` from `khancyr/ardupilot_gazebo`
(fetched from GitHub) plus one added `<include>model://camera_rig</include>`
block, matching the syntax `generic_rover_multi.world` uses for the same
model -- but as a single, non-inlined instance, since ArduPilot SITL is
single-vehicle by construction and the multi-rover world's inlined
duplicates exist specifically to work around a *multi*-instance topic
collision that doesn't apply here. `GAZEBO_MODEL_PATH` already includes
this repo's `sim_container/models` (set by this target's existing
`launch_command`), so `model://camera_rig` resolves with no launch_command
changes. `INSTALL_TIMEOUT_SEC` raised 600s -> 3600s in
`simulator_launcher.py` alongside this -- the previous value was sized for
package-manager one-liners, not a from-source ArduPilot build.

**Not live-tested against real Gazebo/compute hardware** -- no such
environment is available in this session. Code-reviewed against the user's
own validated recipe and the stock upstream world file; ships as
VM-side-done, on-device-confirmation-still-open, the same convention this
doc's own 3a-3f entries already followed before their own live tests
happened.

## 4. Explicitly not doing

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
