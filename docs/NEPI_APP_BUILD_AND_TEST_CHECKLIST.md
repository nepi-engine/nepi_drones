# NEPI App Build & Test Checklist

A practical, step-by-step checklist for building a NEPI app from
`nepi_templates/nepi_app_template` (or `nepi_connect_templates` for a
"connect to another node's IF" app) through to seeing it actually run in the
RUI on a real device — with a debug/test step at each stage, so a failure
gets caught where it happens instead of surfacing three stages later as a
confusing symptom.

Written after building out `nepi_app_sim_connector` end to end and hitting
(and fixing, or learning were red herrings) real problems at almost every
stage below. Each step names the actual failure that stage catches.

See `src/nepi_templates/` (a submodule of this workspace) for the template
source itself — `nepi_app_template/APP_ARCHITECTURE.md` and
`GETTING_STARTED.md` are the canonical reference this checklist summarizes
into a linear path. Read those in full before building something non-trivial;
this doc is the fast-reference version, not a replacement.

## Stage 0 — Before you write any code

- [ ] Pick `nepi_app_template` (full app: own node, config, RUI page, custom
      msgs) or `nepi_connect_templates/nepi_app_<x>_connect` (thin app that
      just consumes another node's `Connect<X>IF`) — don't build a full app
      if a connect example already does 90% of what you need.
- [ ] Copy the template folder **into your own repo** (this workspace: that
      means `nepi_drones`, not `nepi_apps` directly — see
      `feedback-nepi-drones-repo` if you have that memory note, or just: new
      app work is drafted in `nepi_drones`, promoted to `nepi_apps` only once
      it's the team's real deliverable, matching how `nepi_app_sim_connector`
      itself got here).
- [ ] Run `setup_new_app.sh` (edit its EDIT-THESE block first) rather than
      hand-renaming. The rename touches ~8 files and the three names that
      must agree (`pkg_name`, `app_file`, `rui_main_class`) are exactly the
      ones a typo breaks silently (blank RUI page, not an error).

**Debug/test this stage:** none yet — nothing runs. Just don't skip the rename
script; a hand-rename mismatch here is the single most common root cause of
"the app installs fine but its RUI page is blank" three stages from now, and
it's much cheaper to catch by running `setup_new_app.sh --dry-run` and
reading the plan than by debugging a blank page later.

## Stage 1 — Node logic, in isolation

- [ ] Write the node's real logic (params, pubs, subs, worker thread/timer
      callbacks). Keep the standard `MsgIF` + 1Hz latched status pattern from
      the template.
- [ ] Syntax-check before ever running it:
      `python3 -m py_compile scripts/*_node.py api/connect_app_*.py`
- [ ] Sanity-check the params yaml parses and has the required top-level keys:
      `python3 -c "import yaml; d=yaml.safe_load(open('params/<name>_app_params.yaml')); assert 'APP_DICT' in d and 'RUI_DICT' in d"`

**Debug/test this stage:** run the node directly against a scratch roscore
before touching the real device — see "Testing against a bare roscore"
below for the two gotchas that make a bare roscore behave differently from a
real device.

## Stage 2 — Deploy to the source tree, build

- [ ] Deploy with the template's own `deploy_app.sh` (`NEPI_REMOTE_SETUP=0`
      running on the target, `=1` from a dev host) or `rsync` by hand into
      `.../nepi_engine_ws/src/nepi_apps/<app_folder>/` (on the real device
      this is usually a symlink target under
      `/mnt/nepi_storage/nepi_src/nepi_engine_ws/...` — resolve the symlink
      and rsync into the real path, not through it, or nothing changes).
- [ ] Build. On a real device, prefer a **scoped** build over the full
      `build_nepi_code.sh` (which starts with `nepistop` and takes down the
      whole live stack):
      ```bash
      cd /mnt/nepi_storage/nepi_src/nepi_engine_ws
      source /opt/ros/noetic/setup.bash
      export SETUPTOOLS_USE_DISTUTILS=stdlib
      catkin build --profile=release --env-cache -j$(nproc) <pkg_name>
      ```
      The `release` profile has `install: true` pointed at `/opt/nepi/...`
      already configured, so this installs straight to production paths
      while only touching the one package + its message deps.

**Debug/test this stage:**
- Build failure mentioning a specific missing file during the `install` step
  (e.g. `error: can't copy 'src/nepi_api/some_file.py': doesn't exist`) —
  check for a **dangling symlink** at that exact path before assuming the
  file is genuinely missing. `rsync -a` copies symlinks *as symlinks*,
  preserving the original absolute target path — if that symlink pointed
  to another machine's filesystem (a dev-host live-patch trick, for
  example), it silently becomes a broken link the moment it lands anywhere
  else. `find <pkg_src_dir> -type l -exec ls -la {} \;` before rsyncing
  anything into a source tree you don't want to break.
- After a scoped build, confirm you rebuilt what you think you rebuilt:
  `catkin build <pkg>` only rebuilds that package and its declared deps —
  it will **not** pick up a change to a *different* package (e.g. a
  shared `nepi_api` fix) unless that package is also named in the build
  command or is an explicit dependency.

## Stage 3 — Run the node for real, watch it start

- [ ] Launch exactly the way `apps_mgr` would (name/namespace remap matters
      less than you'd think, but match it anyway):
      ```bash
      source /opt/nepi/nepi_engine/setup.bash
      export ROS_MASTER_URI=http://localhost:11311
      export PYTHONUNBUFFERED=1
      rosrun <pkg_name> <app_file> __name:=<node_name> __ns:=/nepi/device1
      ```

**Debug/test this stage — the most important lesson in this whole doc:**
- **Always set `PYTHONUNBUFFERED=1` (or run `python3 -u`) whenever you
  redirect a node's output to a file.** ROS's console log handler
  (`rosgraph/roslogging.py`) never calls `.flush()`. On a TTY this is
  invisible (line-buffering makes every `\n` flush anyway); redirected to a
  file, Python switches to full block buffering and log lines sit unflushed
  for as long as several KB of buffer takes to fill. A node that looks
  "stuck" after redirecting to a log file may have already finished starting
  — you just can't see it yet. This produced a multi-hour false debugging
  trail chasing a NavPoseIF "hang" that turned out not to exist; see
  `completed/SIM_CONNECTOR_NAVPOSE_HANG_BUG.md` for the full writeup.
- **Never `kill -9` a process just because its log looks stalled.** `-9`
  bypasses normal interpreter shutdown, which is what would otherwise flush
  the stdio buffer — so the kill itself destroys the evidence that would
  have told you the process was fine. If you must kill a test process, send
  plain `SIGTERM` (`kill <pid>`, no `-9`) and give it a moment.
- **Verify via direct ROS state, not by grepping a log for a "ready"
  string:** `rosservice call .../<app>/status_query`, `rostopic list |
  grep <app>`, `rostopic echo -n1 <topic>`, `rosnode info <node>`, `ss
  -tlnp` for a bridge socket. All of these reflect current process state
  regardless of what the log file happened to have flushed.
- If a node *genuinely* never completes startup (confirmed via the above,
  not just an unresponsive log), a live stdlib stack dump is a `py-spy`/`gdb`
  substitute when neither is installed on-device:
  ```python
  import sys, traceback
  for tid, frame in sys._current_frames().items():
      traceback.print_stack(frame)
  ```
  Wire this to a signal handler or a debug topic subscriber temporarily; it
  needs no extra dependencies.

### Testing against a bare roscore (no `config_mgr`, no real device)

A scratch/throwaway roscore with nothing else registered is genuinely
different from a real device in one specific way that looks exactly like a
hang: several `nepi_api`/`nepi_sdk` calls (`MsgIF.updaterCb` reading
`debug_mode`, `SaveDataIF.__init__` reading `user_folders`, and likely other
`nepi_system.get_*` calls) go through `nepi_sdk.wait_for_param(...,
timeout=1000)`. On a real device, `config_mgr` provisions these params before
any app node starts, so the first 1-second poll succeeds and this is
invisible. On a bare roscore, nobody ever sets them, so the call blocks for
the full 1000 seconds (~16.7 minutes) — a real block this time, not a
buffering illusion.

- [ ] Before launching anything that constructs a device IF
      (`NPXDeviceIF`/`SimDeviceIF`/etc.) against a bare roscore, pre-seed at
      least:
      ```bash
      rosparam set /nepi/device1/debug_mode false
      rosparam set /nepi/device1/user_folders "{data: /tmp/nepi_test_data}"
      ```
      (adjust the namespace to match your test node). This is the difference
      between a 30-second test and a 16-minute one.

## Stage 4 — Register with `apps_mgr`

- [ ] Confirm `apps_mgr` sees the app at all:
      `rostopic echo -n1 /nepi/device1/apps_mgr/status | grep -A20 '"<pkg_name>"'`
      — check `running`, `group_name`, `rui_main_file`, `rui_main_class`,
      `rui_files_list` all match what's in your params yaml.
- [ ] If a newly-deployed app doesn't show up within `apps_mgr`'s poll
      interval (~5s), don't restart the `apps_mgr` *process* — that resets
      its subprocess-tracking dict and can duplicate-launch every
      already-running app. Nudge it instead:
      `rostopic pub /nepi/device1/apps_mgr/reset_config std_msgs/Empty "{}"`
- [ ] To actually start/stop it under `apps_mgr`'s own management (rather
      than a manually-launched `rosrun` you're tracking yourself), toggle
      `apps_mgr/update_state` (`nepi_interfaces/UpdateBool`, by
      `pkg_name`), then verify via `ps aux`/`rosnode info` that only *this*
      app's PID changed — unrelated already-running apps must keep their
      exact original PIDs.

**Debug/test this stage:** if `apps_mgr/status` never mentions your app at
all (not even with `running: false`), check `nepi_sdk/nepi_apps.py`'s app
discovery scan is looking for `.yaml` files in the params folder, not `.py`
— this exact bug existed and was fixed once already; if you're on an older
device build, it may have regressed or never received the fix.

## Stage 5 — Wire the RUI page in

This is the step most likely to be silently skipped, because everything
*else* about the app can be completely correct and running, and the RUI will
still show nothing.

- [ ] Copy the app's `rui/*.js` files **flat** into
      `nepi_rui/src/rui_webserver/rui-app/src/` (not into a subfolder — the
      app's own files use relative imports like `./Columns` that assume
      they're siblings of the RUI's own shared components).
- [ ] Copy the app's params yaml into
      `nepi_rui/src/rui_webserver/rui-app/src/apps/` (create that directory
      if it doesn't exist yet — on a fresh nepi_rui checkout, it usually
      doesn't).
- [ ] Run `build_nepi_rui.sh`, or by hand: add an
      `import <rui_main_class> from "./<rui_main_file minus .js>"` and a
      `["<rui_main_class>", <rui_main_class>]` entry to `appsClassMap` in
      `Nepi_IF_Apps.js` (look for the `//ADD APP FILE IMPORTS` /
      `//ADD APP FILE MAPPINGS` comment markers), then `npm run build`.
- [ ] Redeploy the built `build/` directory to
      `/opt/nepi/nepi_rui/src/rui_webserver/rui-app/src/build/` (or
      wherever `RUI_HOME` resolves `APP_BUILD_PATH` to) and restart nothing
      — Flask serves the build directory fresh per request, no service
      restart needed.

**Debug/test this stage:**
- Before assuming this step was ever done, check: does `appsClassMap` in
  `Nepi_IF_Apps.js` actually have an entry, or is it still the empty
  placeholder? `grep -n "appsClassMap" Nepi_IF_Apps.js` — an empty
  `new Map([\n\n])` means **no app has ever been wired through this
  mechanism on this checkout**, regardless of how correct the app's own
  packaging is.
- `rui_main_class` must exactly match the React class's own `export
  default` name — a mismatch gives a blank page with no error, since the
  `appsClassMap.get(...)` lookup just returns `undefined`.
- The RUI's app selector only lists an app while `running: true` in
  `apps_mgr/status` (Stage 4) — an enabled-but-crashed app disappears from
  the menu instead of showing an error. If the app vanished from the RUI,
  check whether the node process is still alive before debugging the RUI
  side.
- Confirm the deployed bundle actually contains your change before opening
  a browser: `curl -s http://localhost:5003/ | grep -o 'main\.[a-z0-9]*\.js'`
  to get the current bundle hash, then
  `curl -s http://localhost:5003/static/js/main.<hash>.js | grep -c
  YourComponentName`.
- If the RUI shows a *stale* build after you were sure you deployed a new
  one: the RUI's own build output (`/opt/nepi/nepi_rui/...`) is **not**
  persisted across a device reboot/container recreation — back up
  (`cp -a build build.bak-$(date +%s)`) before every deploy so a bad push
  is a one-line revert, and expect to redeploy again after any reboot.

## Stage 6 — Full loop, real browser

- [ ] Open the RUI, select the app's `group_name` menu, confirm the app
      appears and its page renders (not blank).
- [ ] Exercise the actual feature end to end, not just "it renders" — for a
      connect-style app, select a real source and confirm the reusable
      `Nepi_IF_Connect<X>` component reflects it; for a full app, drive its
      real controls and confirm the underlying node reacts.

At this point you have verified every layer: node logic, build/install,
process startup, `apps_mgr` registration, and RUI wiring — each with its own
failure mode caught at the stage it actually happens, rather than discovered
three stages later as an unrelated-looking symptom.
