# RESOLVED: Sim Connector RUI fixes kept "reverting" — two independent causes, both on the device, neither in git

**Status:** Closed on this device (`nepihost@192.168.179.103`, container committed as
`nepi_fs_a:nepi-0p0p0-rpi-latest-20260923-sim_connector_rui_fix_and_camera_lock`). Neither
cause is a code/git problem — read this before assuming a Sim Connector RUI change "isn't
really fixed" again.

**Reported live (2026-09-23):** *"the sim connector rui got reverted to an older version
again - the second robot config and environment config section needs to be called robot YAMLs
and Env. yamls... the robot yamls section still has it as buttons and not dropdowns, i cant
change rotate obstacles or edit the default one, etc."*

Every one of those features (the "Robot YAMLs"/"Environment YAMLs" dropdowns, obstacle
rotation, editing built-in configs) was **already correct in `nepi_drones`' committed source**
(`src/nepi_apps/nepi_app_sim_connector/rui/Nepi_IF_Sim.js`, most recently touched 2026-09-18).
`git log`, `git status`, and `git branch -a` all confirmed nothing was reverted, missing, or
sitting uncommitted anywhere in this repo. The bug was entirely in how (and whether) that
already-correct source ever reached the browser.

## Cause 1: `nepi_rui`'s own on-disk tree shadows app-owned RUI files, every single build

`build_nepi_complete.sh` runs two build steps in order: `build_nepi_code.sh` (catkin — this is
what installs each app's own `rui/*.js` files, e.g. `nepi_app_sim_connector`'s, into
`/opt/nepi/nepi_rui/src/rui_webserver/rui-app/src/`), then `build_nepi_rui.sh` (the npm/React
build). But `build_nepi_rui.sh` starts with this, **unconditionally, every time**:
```bash
sudo rsync -arp ${SCRIPT_FOLDER}/src/nepi_rui/ ${NEPI_BASE}/nepi_rui/
```
`${SCRIPT_FOLDER}/src/nepi_rui/` is a **static, non-git-tracked directory** on this device
(`/mnt/nepi_storage/nepi_src/nepi_engine_ws/src/nepi_rui/` — confirmed via `git log` there:
`fatal: not a git repository`), apparently frozen since initial device provisioning. It had its
own leftover copies of `NepiAppSimConnector.js`, `Nepi_IF_Sim-Controls.js`, `Nepi_IF_Sim.js`,
and `Nepi_IF_SimLauncher.js` — all dated **2026-09-02/03** — plus a fifth orphan,
`Nepi_IF_SimOsInstances.js`, left over from a multi-machine-deploy-target feature that was
built and then **removed** from `nepi_drones`' own git history (commit `d48205d`) — yet its
stale file kept getting served regardless.

Since this rsync has no `--delete` but does overwrite files that exist in both trees, it
silently clobbered every one of those 5 files back to their Sep 2/3 state on **every single
`nepibld`**, no matter how correct the freshly-built, freshly-installed versions were one step
earlier. No other app's RUI files were affected (checked via a 3-way basename comparison
against every `nepi_apps/*/rui/*.js` in `nepi_drones` — Sim Connector was the only match).

**This means any and every previous "fix" to this app's RUI — the 2026-09-14/09-17/09-18
dropdown/rename/rotate-handle work, and the previous session's own `rosbridge` stray-field fix
— may never have actually reached a real browser on this device**, regardless of how correct
the underlying source was or how many times it was rebuilt.

**Fix:** deleted the 5 stale files directly from
`/mnt/nepi_storage/nepi_src/nepi_engine_ws/src/nepi_rui/src/rui_webserver/rui-app/src/` (not a
git repo, so a plain `rm` is the correct and complete fix — nothing to commit there). After
that, `nepibld`'s catkin-installed files survive the later rsync untouched, since the rsync no
longer has anything under those specific relative paths to overwrite them with. Verified: the
final built bundle (`build/static/js/main.<hash>.js`) now actually contains `"Robot YAMLs"` and
is free of the previous session's `type: type` bug.

**If this app's RUI ever looks stale again**, check first whether
`/mnt/nepi_storage/nepi_src/nepi_engine_ws/src/nepi_rui/src/rui_webserver/rui-app/src/`
has grown a new stale copy of any `nepi_app_sim_connector` (or any other app's) `rui/*.js`
file before assuming the source itself regressed.

## Cause 2: the running container was never `nepicommit`'d, so nothing above matters after a restart

Independent of Cause 1: `docker images` on the host (`nepihost@192.168.179.103`) showed the
**last-ever `nepicommit`** was `nepi-0p0p0-rpi-latest-20260916-...` — three images, all dated
2026-09-16. The running container (`docker ps`) was based on one of those. **Nothing done
after 2026-09-16 — not the 09-17/09-18 Sim Connector RUI work, not the previous session's
driver/camera-lock fixes — was ever persisted into a new image.** A `nepibld`+`nepistart`
cycle only updates the *running* container's writable layer; it does not survive that
container being recreated (a device power-cycle, `docker stop`+`run`, etc.) unless someone
runs `nepicommit` afterward. Nobody had, going back at least a week.

**Fix:** after confirming the rebuilt RUI bundle was correct (Cause 1's fix + a clean
`nepibld`/`nepistart`), ran (from the **host**, `nepihost@<device-ip>`, not the container):
```bash
nepicommit sim_connector_rui_fix_and_camera_lock
```
This snapshots the current container into a new image
(`nepi_fs_a:nepi-0p0p0-rpi-latest-20260923-sim_connector_rui_fix_and_camera_lock`), stops the
old container, and starts a fresh one from the new image — verified all 20 ROS nodes came up
healthy on that fresh boot, including `app_sim_connector`, and the RUI bundle inside it still
has the fix.

**Going forward: any `nepibld` session that's meant to stick needs to end with `nepicommit
<some-tag>` on the host, not just `nepistart` inside the container.** Skipping this step is
functionally invisible until the next restart, which is exactly why this kept looking like a
recurring regression instead of a one-time missed step.

## Also recurring, unrelated, not fixed here

Cleaning stale root-owned `__pycache__` directories under `/opt/nepi/nepi_engine` before every
`nepibld` is still required (see the parity doc,
`SIM_CONNECTOR_WEBOTS_MUJOCO_PARITY_2026-09.md`) — hit again during this pass, cleaned again
the same way. The pre-existing `drivers_mgr`/`config_mgr` startup race documented there did
**not** recur on this particular restart, but don't assume it's gone for good.
