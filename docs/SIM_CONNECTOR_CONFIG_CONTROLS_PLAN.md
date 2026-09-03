# Sim Connector: Missing Configuration Controls

## Problem

Two distinct gaps, found by auditing `Nepi_IF_Sim-Controls.js` against what every RBX driver
in this project (`rbx_sim_node.py`, `rbx_webots_node.py`, `rbx_webots_quadcopter_node.py`)
actually reports via Settings, and what `NepiDeviceRBX.js` (Devices -> Robots) already
exposes for those same Settings.

**1. Configuration controls that already exist are invisible.** `NepiIFSimControls`
(`Nepi_IF_Sim-Controls.js`) already implements `renderRobotCapabilityControls()`
(`autonomous_movement_enabled`/`teleop_movement_enabled`/`camera_controls_enabled` toggles)
and `renderImageSourceCuration()` (`enabled_image_sources` checkboxes) -- exactly the
"what should be showcased in Devices -> Robots" controls this app is supposed to own. But
`Nepi_IF_Sim.js` only mounts `<NepiIFSimControls>` at all when `show_controls === true`
(`NepiAppSimConnector.js` passes `show_controls={false}`, a deliberate decision -- see
`NepiAppSimConnector.js`'s own comment -- to hide LIVE control, e.g. motor sliders and goto
buttons, which belongs in Devices -> Robots). Because live control and configuration were
bundled into one component gated as a single unit, hiding live control silently took the
configuration controls down with it too.

**2. Several Settings every driver reports have no editable control anywhere at all** --
not in Sim Connector, not even in Devices -> Robots:
- `environment` (FLAT_GROUND/OBSTACLE_COURSE) -- the RBX driver's own Setting. (A
  differently-named, differently-mechanism'd `renderEnvironmentControls()` already exists in
  `Nepi_IF_Sim-Controls.js`, but it is driven by `SimCapabilitiesQuery.has_environment_controls`,
  which is `sim_connector_app_node.py`'s OWN generic bridge-protocol capability (port 9030) --
  confirmed dead for every currently-deployable target: `gazebo_rover`/`webots_rover` both
  launch the simple-protocol RBX stack instead of anything that dials into port 9030 (see
  `gazebo_rover`'s own `launch_command` comment), and `pybullet_rover`/`wpilib_rover` have no
  `launch_command` at all yet. Left in place, not deleted -- it is plausible near-future code
  for whenever those two targets get wired up, not abandoned dead code -- but it does not
  answer this gap.)
- `max_linear_speed_mps`, `max_angular_rate_dps` (rover + quadcopter), `max_vertical_speed_mps`,
  `takeoff_height_m` (quadcopter only) -- no control anywhere.

`camera_view_mode`/`camera_offset_x/y/z`/`scene_offset_x/y/z` DO already have controls, but
only in `NepiDeviceRBX.js` (moved there deliberately per an earlier decision -- "the rbx
driver should have every feature available no matter the robot"). The user's ask is for
these to ALSO be editable from Sim Connector, not exclusively from Devices -> Robots.

## What "editable before and after deploying" actually means here

After a robot is deployed and its RBX driver is live, every Setting listed above is a real,
two-way `nepi_interfaces/Settings` value -- editing it from Sim Connector updates the SAME
live driver Settings `NepiDeviceRBX.js` reads, so a change made from either panel is visible
in both immediately (same `settings/status` topic, same `update_setting` service both
already use).

Before a robot is deployed, there is no live driver to read Settings from at all -- the only
existing pre-deploy configuration mechanism is the Robot Config selector (a preset baked into
`sim_connector_app_params.yaml`'s `robot_configs`). This plan does not add a second,
parallel way to edit preset defaults pre-deploy (that would mean building config-file editing
UI, a materially larger and different feature than what's missing here) -- it makes every
Setting control render as soon as a robot IS live (the same "no device yet" empty state
`NepiDeviceRBX.js` itself already has), which covers "after deploying" fully and is the
existing, consistent pattern this whole codebase already uses.

## Plan

1. **Un-hide configuration controls.** `Nepi_IF_Sim.js` always mounts `<NepiIFSimControls>`
   (previously conditional on `show_controls`), passing a new `show_live_controls={show_controls}`
   prop instead. `NepiIFSimControls` splits `renderControls()` into:
   - `renderLiveControls()` -- motor sliders, goto SEND buttons, home/stop/setup actions, the
     live camera image viewer + active-topic selector. Rendered only when
     `show_live_controls !== false`, preserving today's deliberate hide.
   - `renderConfigControls()` -- capability toggles, image-source curation, camera view-mode
     buttons, camera/scene offset inputs, the `environment` Setting toggle, and movement-limit
     inputs (all new, see below). Always rendered.

2. **No `rbxSettingsListener` changes needed** -- every new control below is either a
   hardcoded-option Discrete (environment, camera_view_mode -- matching `NepiDeviceRBX.js`'s
   own hardcoded-label convention exactly, e.g. "Flat Ground"/"Obstacle Course") or a plain
   editable Float input (offsets, movement limits) -- the already-tracked names/values are
   sufficient for both; no need to also fetch `setting_caps_list`.

3. **New controls in the always-visible configuration group**, each gated on the Setting's
   presence in `rbxSettingsNamesList` (so an older driver without a given Setting simply
   shows no control for it, same convention as everything else in this file):
   - `renderCameraViewModeControls()` -- Robot View / Scene View buttons for
     `camera_view_mode`, gated additionally on `camera_controls_enabled` (mirrors
     `NepiDeviceRBX.js`'s own gating exactly).
   - `renderCameraOffsetControls(namePrefix, titlePrefix)` -- editable X/Y/Z Float inputs,
     ported from `NepiDeviceRBX.js`'s method of the same name (same editable-input pattern:
     `setElementStyleModified`/`clearElementStyleModified`, Enter-to-apply), called once for
     `camera_offset` and once for `scene_offset`, each gated on that triple's own presence
     plus `camera_controls_enabled`.
   - `renderEnvironmentSetting()` -- new. Toggle/buttons for the RBX driver's `environment`
     Setting (FLAT_GROUND/OBSTACLE_COURSE), reading `CAP_SETTINGS`' own `environment` options
     list via the newly-captured `setting_caps_list` rather than hardcoding the two option
     strings.
   - `renderMovementLimits()` -- new. Editable Float inputs for whichever of
     `max_linear_speed_mps`/`max_angular_rate_dps`/`max_vertical_speed_mps`/`takeoff_height_m`
     the connected driver reports -- naturally shows 3 inputs for the rover (no
     `max_vertical_speed_mps`) and all 4 for the quadcopter (adds `takeoff_height_m`), with no
     per-robot-type branching in the JSX itself.

4. **Mirror into `nepi_rui` and `nepi_drones`** (both flat copies of this app's RUI source,
   same convention every RUI change in this project already follows), rebuild the RUI bundle,
   redeploy, and `nepicommit` if the fix needs to survive a container restart.

## Separate finding, not fixed in this pass

`renderCameraControls()` (the live image viewer + active-topic selector + a SECOND,
`SimCapabilitiesQuery`-based camera-view-mode dropdown) is ALSO dead code for every
currently-deployable target: `has_camera`/`available_image_topics` and
`has_camera_view_control`/`available_camera_view_modes` are all derived in
`device_if_sim.py` from `getAvailableSensorTopicsFunction`, which
`sim_connector_app_node.py` only ever populates from a `{"type":"sensor_topics",...}`
message arriving over the same dead generic bridge-protocol connection (port 9030) as
`renderEnvironmentControls()`. The REAL, live image data for a deployed rover/quadcopter
lives on the RBX driver's own device namespace (e.g. `webots_robot/color_2d_image`), not on
`app_sim_connector`'s own separate image pipeline -- confirmed by checking
`refreshSensorTopics`'s only caller. This means Sim Connector's live camera preview has
never actually worked for a real deployment, independent of the `show_controls` gating bug
this plan fixes. Left alone here: fixing it means wiring
`getAvailableSensorTopicsFunction` to also read the connected RBX driver's own reported
sensor topics, a genuinely separate, bigger change than what's asked for, not implemented
in this pass. `renderCameraControls()` stays classified as "live control" (gated by
`show_live_controls`) since none of it is real configuration.

## Incident: `renderEnvironmentSetting()` briefly deleted, then restored + unified (2026-09-03)

While reorganizing `Nepi_IF_Sim.js`'s own "Environment Config" dimensions
selector (see `SIM_OS_INSTANCES_PLAN.md`'s dated entries for that
reorganization), `renderEnvironmentSetting()` (this doc's own "Flat
Ground"/"Obstacle Course" dropdown for the RBX driver's `environment`
Setting) was mistakenly deleted under the belief that it duplicated that
other, unrelated dropdown. It does not: `environment_dimensions_selected_config`
(Flat/Obstacle Course/Aerial Obstacle Course/Custom Obstacles) only edits a
model's *geometry fields* (`pushDirtyDimensions` explicitly does nothing at
all for the Flat/`ENVIRONMENT_MODEL_NONE` case); `renderEnvironmentSetting()`'s
own `environment` Setting is the ONLY thing that actually spawns/despawns
obstacles in a running Gazebo world (`rbx_sim_node.py` ->
`sim_bridge_node.py` -> `environment_models.py`'s real
`/gazebo/spawn_sdf_model`/`delete_model` calls). Deleting it broke real
spawn state entirely -- reported live: "the environment config on the top
is selected as flat, but it still shows the obstacle course in the
gazebo... i think you confused environment yaml with the dropdown."

Restored verbatim, then additionally wired the two together:
`Nepi_IF_Sim.js`'s `onSelectDimensionConfig('environment', name)` now also
calls `NepiIFSimControls.setEnvironmentSetting()` (a new public method,
factored out of `renderEnvironmentSetting()`'s own `onChange`, reached via
a `React.createRef()` from the parent) whenever `name` is exactly "Flat" or
"Obstacle Course" -- the only two names with a direct `environment` Setting
equivalent. Picking either dropdown now keeps both in sync; "Aerial
Obstacle Course"/"Custom Obstacles"/any other saved name has no Setting
equivalent to guess at and is left alone on purpose.

## Explicitly not doing

- Not building pre-deploy preset-editing UI (editing `robot_configs` entries themselves) --
  a materially larger, different feature; see "before and after deploying" section above.
- Not deleting the existing SimCapability-based `renderEnvironmentControls()` -- plausible
  future-relevant code for `pybullet_rover`/`wpilib_rover` once those get `launch_command`s,
  not abandoned dead code.
- Not building a fully generic "render any Setting automatically" engine. The four new
  render methods above are enumerated and named, matching this file's own existing
  convention (one method per logical control group, gated by capability/Setting presence) --
  a future Setting not covered by one of these still shows nothing, same as today, rather
  than guessing at a generic UI for an unknown type.
