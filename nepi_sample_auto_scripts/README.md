# nepi_sample_auto_scripts
Sample automation scripts for NEPI Engine AI and automation software

## What's in here

- **Top-level `*_script.py` files** — the actual automation scripts. Each one is a
  standalone ROS node (plain `rospy`/`nepi_sdk`, no catkin package here) meant to run
  continuously on a NEPI device, reacting to sensor/AI/robot topics.
- **[`tools/`](tools/README.md)** — one-off command-line utilities (AI training-data
  helpers, calibration backup/restore, batch find-and-replace across scripts) — not
  automation nodes, run once from a terminal and exit. See `tools/README.md` for what each
  one does and the exact command to run it.
- **[`tests/`](tests/README.md)** — automated unit tests (`test_*.py`, safe to run anytime,
  no ROS needed) plus manual `live_smoke_test_*` scripts (run by hand against a real device
  over SSH — none have been run live yet, see their own caveats). See `tests/README.md` for
  how each kind works and what every individual test checks.

## How to run any automation script

**Settings are edited in the file, not in the RUI.** The RUI's Scripts panel (confirmed by
reading its source, `nepi_rui/.../NepiMgrScripts.js`) can discover, start, stop, and show the
live log of a script — it does not include a code editor. So: open the script's
**`USER SETTINGS - Edit as Necessary`** block near the top in a normal text editor, change
whatever constants you need (topic names, thresholds, target locations, etc.), then deploy
that edited file — there's nothing to configure after it's running except by editing the
file again and re-deploying.

### 1. Deploy the script to the device

Copy the file into `/mnt/nepi_storage/nepi_scripts/` on the device (that folder is what the
RUI's Scripts list and `scripts_mgr` both scan — usually a bind-mounted host path, so a plain
`scp`/`rsync` onto the device works with no rebuild or container restart needed). It shows up
in the RUI within about a second — `scripts_mgr` watches that folder continuously.

### 2. Run it from the RUI, step by step

1. Open the RUI and go to the **Scripts** page (top-level menu item, backed by
   `scripts_mgr` — labeled `"Scripts"` in the menu).
2. Find your script by filename in the **left-hand "Scripts" list** (this is every `.py`
   file currently sitting in `/mnt/nepi_storage/nepi_scripts/`) and click it to select it.
3. The **Control and Status** panel below shows the selected file's name and a
   **Start**/**Stop** button pair.
4. Click **Start**. The script now also appears in the **"Running Scripts"** list on the
   right.
5. Watch it initialize in the **live message pane** underneath (subscribes to that script's
   own `MsgIF` output in real time) — every script in this folder logs each step
   (`Waiting for topic: ...`, `Found topic: ...`, `Initialization Complete`, warnings for
   anything it's still blocked on) so you can see exactly what it's doing or stuck on without
   needing SSH/log-file access.
6. Click **Stop** when you're done (or leave it running — that's the normal mode for most of
   these). There's also an **auto-start** checkbox next to the controls if you want this
   script to launch automatically every time the device boots.

**Prerequisites matter.** Every script below lists what has to already be running/attached
*before* you click Start — most of them call `wait_for_topic`/`wait_for_node` with a timeout
and will just log a warning and idle (not crash) if that prerequisite never shows up. That's
expected, not a bug — re-check the "Requires" line below and start the right things first.

### 3. Running it directly instead (for dev-machine/SITL testing, no RUI)

Make sure a ROS master and the NEPI SDK are reachable (source the workspace; see
`src/nepi_engine/nepi_sdk/CLAUDE.md`/this repo's own sim docs if `nepi_sdk`-based nodes hang
on startup — they need to run in a properly namespaced ROS environment), then run it like
any Python ROS node: `python3 <script_name>.py`. Same live log-line behavior as above, just
printed to your terminal instead of the RUI's message pane.

---

## Automation scripts

### `ai_detector_config_script.py`
**What it does:** enables an AI model via `ai_models_mgr`, then points the resulting
detection node at a specific camera image topic and starts detection on that stream.
**Requires:** nothing else running first — this is meant to be the *first* script that
turns AI detection on. `DETECTION_MODEL` must be the display name of a model actually
installed on the device (check the RUI's AI Models panel, or
`rosservice call .../ai_models_mgr/model_status_query`) — there's no default model shipped
with the engine, so the factory placeholder value will not match anything real until you
change it.
**Key settings:** `IMAGE_INPUT_TOPIC_NAME` (which camera topic to detect on),
`AI_FRAMEWORK_NAME` (e.g. `"yolov8"`), `DETECTION_MODEL`, `DETECTION_THRESHOLD`.
**Status: fixed 2026-08-06.** Previously non-functional — `ai_detector_mgr`, the manager
this script originally talked to, was replaced by `ai_models_mgr`, a structurally different
architecture. Re-ported against the real current mechanism (`update_framework_state` +
`update_model_state` to launch the model, then that model's own `set_img_topic`/
`set_threshold`/`enable` topics to point it at your camera) — see the file's own header
comment for the full detail. Smoke-tested live: launches cleanly, correctly times out and
logs a clear warning when the camera topic or model name doesn't exist yet, idles safely.
**In the RUI:** select `ai_detector_config_script.py` and Start it. Watch the message pane
for `Enabling AI framework: ...` → `Enabling AI model: ...` → `Waiting for detection node:
...` → either `Detector ... enabled on ...` (success) or a warning telling you exactly which
of `AI_FRAMEWORK_NAME`/`DETECTION_MODEL` didn't match anything real on this device.

### `drone_follow_object_mission_script.py`
**What it does:** watches for a named AI-detected target (e.g. `"chair"`) and flies the
drone toward it, using RBX goto commands, once it appears.
**Requires:** an ArduPilot RBX driver already running and publishing detected
`Target`/`Targets` messages on `app_ai_targeting/target_localizations` (an AI targeting app
that isn't part of this workspace yet — for SITL testing, use
`sim_container/scripts/ai_targeting_controller_ardupilot.py` (dev-VM side) plus
`tools/sim_ai_targeting_bridge_script.py` (device side) as stand-ins; see both files' own
docstrings).
**Key settings:** `RBX_ROBOT_NAME`, `TARGET_TO_FOLLOW`, `TARGET_OFFSET_GOAL_M`,
`TAKEOFF_HEIGHT_M`, `HOME_LOCATION`, the `GOTO_*_ERROR`/`CMD_*_TIMEOUT` tuning values.
**Status:** live-tested against the SITL stand-ins above — the follow logic itself works
correctly (detects the target, computes and issues the right goto command). A separate,
pre-existing ArduPilot SITL takeoff issue can prevent the drone from visibly closing the
distance in Gazebo; that's not a bug in this script.
**In the RUI:** start the ArduPilot RBX driver (and the two SITL stand-in scripts if
testing without real hardware/`app_ai_targeting`) first, *then* select
`drone_follow_object_mission_script.py` and Start it. Watch the message pane for
`Waiting for namespace containing: ardupilot` (confirms it found the RBX driver) followed by
target-detection/goto log lines once `TARGET_TO_FOLLOW` appears in view.

### `drone_inspection_demo_mission_script.py`
**What it does:** a scripted inspection flight — takes off, flies to a location (or a list
of waypoint "corners"), runs any configured pre/post-mission actions, then returns.
**Requires:** an ArduPilot RBX driver already running (the script waits for a node whose
name contains `RBX_ROBOT_NAME`).
**Key settings:** `RBX_ROBOT_NAME`, `GOTO_LOCATION` / `GOTO_LOCATION_CORNERS`,
`TAKEOFF_HEIGHT_M`, `HOME_LOCATION`, `ENABLE_FAKE_GPS` (turn on if flying SITL with no real
GPS), the `GOTO_*_ERROR`/`CMD_*_TIMEOUT` tuning values.
**Status:** works against the ArduPilot SITL setup — the standard demo mission script to
launch alongside the RBX driver.
**In the RUI:** start the ArduPilot RBX driver first, then select
`drone_inspection_demo_mission_script.py` and Start it. Watch the message pane for
`Waiting for namespace containing: ardupilot` → `Waiting for status message` → the
mission's own takeoff/goto/return log lines.

### `led_adjust_on_object_detect_action_script.py`
**What it does:** turns an LED on/adjusts its brightness while a named object stays
detected in view, and drops back down when it's lost for a few checks in a row.
**Requires:** an LSX-driver-based light (`lsx/turn_on_off`/`lsx/set_intensity`/
`lsx/blink_on_off` topics) already running, **and** an AI model already enabled and
detecting on a camera topic (see `ai_detector_config_script.py` above) so
`<base_namespace>/bounding_boxes` actually has something publishing to it.
**Key settings:** `OBJECT_LABEL_OF_INTEREST`, `LOST_COUNT_THRESHOLD`, `LED_LEVEL_MAX`,
`LED_BLINK_RATE`/`LED_BLINK_THRESHOLD`, `WATCHDOG_TIME`.
**Status: fixed 2026-08-06.** Previously non-functional — depended on `ai_detector_mgr`'s
old bounding-box output (`darknet_ros_msgs`), which doesn't exist anywhere in this
workspace. Re-ported to subscribe to the current engine's aggregated
`<base_namespace>/bounding_boxes` (`nepi_interfaces/AiBoundingBoxes`) instead — every
enabled AI model publishes there regardless of framework, so this script now reacts to
*any* currently-running detector, not one hardcoded manager. Also fixed a real pre-existing
bug while re-porting: the old per-box loop incremented its "lost" counter once per
non-matching box in a frame instead of once per frame, so a frame with several other
objects in it could falsely trigger the "object lost" LED action. Smoke-tested live: imports
and launches cleanly, correctly idles waiting for the LSX topic with no LSX hardware
attached.
**In the RUI:** get an LSX light driver and an AI model both running first (see
`ai_detector_config_script.py`), then select
`led_adjust_on_object_detect_action_script.py` and Start it. Watch the message pane for
`Waiting for topic name: lsx/turn_on_off` → `Waiting for topic: .../bounding_boxes` →
intensity/blink log lines once `OBJECT_LABEL_OF_INTEREST` is detected.

### `led_alerts_action_script.py`
**What it does:** watches for a named AI-detected object and drives an LED to one look
while it's in view and a different look otherwise (color/intensity/blink, whichever the
connected light actually supports).
**Requires:** an LSX-driver-based light, **and** an AI model already enabled and detecting
on a camera topic (see `ai_detector_config_script.py`) — same detection source as
`led_adjust_on_object_detect_action_script.py` above.
**Key settings:** `LED_STATUS_TOPIC_NAME`, `OBJECT_LABEL_OF_INTEREST`,
`ALERT_LOST_COUNT_THRESHOLD`, `START_STATE`, `ALERT_TRUE_ACTIONS`, `ALERT_FALSE_ACTIONS`
(each is `[on_off, intensity, blink, blink_interval_sec, color]`, use `-999` for "don't
touch this one").
**Status: fixed 2026-08-06.** Previously non-functional — depended on an `app_ai_alerts`
app publishing a `base_namespace/app_ai_alerts/alert_state` boolean topic that never
existed anywhere in this workspace's `nepi_apps`, so it blocked forever waiting for it.
Rather than invent a fake stand-in app, this now derives the exact same True/False "alert"
signal directly from the real current AI detection output (same
`<base_namespace>/bounding_boxes` topic and debounce approach as
`led_adjust_on_object_detect_action_script.py`'s own fix) — "alert" is simply "is
`OBJECT_LABEL_OF_INTEREST` currently detected." Smoke-tested live: imports and launches
cleanly, correctly idles waiting for the LSX status topic with no LSX hardware attached.
**In the RUI:** get an LSX light driver and an AI model both running first, then select
`led_alerts_action_script.py` and Start it. Watch the message pane for
`Waiting for status topic name: lsx/status` → `Waiting for bounding boxes topic: ...` →
`Alert State updated to: True/False` as `OBJECT_LABEL_OF_INTEREST` comes in and out of view.

### `led_auto_level_process_script.py`
**What it does:** watches a camera feed, estimates how bright the scene is with OpenCV, and
continuously adjusts an LED's brightness to compensate.
**Requires:** an LSX-driver-based light with an intensity control, plus a live camera image
topic.
**Key settings:** `IMAGE_INPUT_TOPIC_NAME`, `LED_LEVEL_MAX`, `SENSITIVITY_RATIO`,
`AVG_LENGTH` (smoothing window).
**Status:** works standalone against any LSX light + camera — no known gaps.
**In the RUI:** start the camera driver and LSX light first, then select
`led_auto_level_process_script.py` and Start it. Watch the message pane for
`Waiting for topic name: lsx/set_intensity_ratio` → `Waiting for topic: color_2d_image` →
steady operation with no further log lines (it just keeps adjusting; nothing to see beyond
the LED itself responding).

### `led_step_adjust_process_script.py`
**What it does:** the simplest one here — just steps an LED's brightness up by a fixed
amount on a timer, wrapping back to 0 once it hits the max. Good for confirming an LSX
light's intensity control actually responds before wiring up anything smarter.
**Requires:** an LSX-driver-based light with an intensity control.
**Key settings:** `LED_LEVEL_MAX`, `LED_LEVEL_STEP`, `LED_STEP_SEC`.
**Status:** works standalone — no known gaps.
**In the RUI:** start the LSX light driver first, then select
`led_step_adjust_process_script.py` and Start it. Watch the message pane for
`Waiting for topic name: lsx/set_intensity` → `Setting LED level to: 0.05`, `0.10`, ...
ticking up every `LED_STEP_SEC` — the easiest script here to visually confirm is working.

### `navpose_config_script.py`
**What it does:** points NEPI's navigation-position system at your own driver's GPS/
odometry/heading topics, so the rest of NEPI has a live position/orientation solution to
work with.
**Requires:** whatever driver publishes your GPS fix / odometry / heading topics already
running.
**Key settings:** `NEPI_NAVPOSE_SOURCE_GPS_TOPIC`, `NEPI_NAVPOSE_SOURCE_ODOM_TOPIC`,
`NEPI_NAVPOSE_SOURCE_HEADING_TOPIC` (set any of these to `""` to skip it),
`NEPI_NAVPOSE_FRAME_NAME` (leave as `"base_frame"` unless you've created a custom frame).
**Status:** works against the current `navpose_mgr` — no known gaps, but double-check your
heading topic is actually a `NavPoseHeading` message (see the file's own docstring); if it's
some other message type, `navpose_mgr` silently ignores it with no error.
**In the RUI:** start your GPS/odometry/heading-publishing driver first, then select
`navpose_config_script.py` and Start it. Watch the message pane for its
`set_frame_comp_topic` publish confirmations — then check the RUI's NavPose page to confirm
`base_frame` is actually tracking your source topics.

### `navpose_set_fixed_config_script.py`
**What it does:** the opposite case from the script above — if your system has **no**
GPS/IMU/compass at all, this sets one fixed lat/long/altitude/heading/roll/pitch/yaw value
and keeps NEPI reporting that same position forever (useful for a stationary test rig or
early bring-up before real sensors are wired in).
**Requires:** nothing else running.
**Key settings:** `START_GEOPOINT`, `START_HEADING_DEG`, `START_ORIENTATION_DEGS`,
`NAVPOSE_TARGET_FRAME` (leave as `"base_frame"` unless targeting a custom frame).
**Status:** works — no known gaps.
**In the RUI:** select `navpose_set_fixed_config_script.py` and Start it — no
prerequisites, it should reach `Initialization Complete` immediately. Check the RUI's
NavPose page to confirm `base_frame` now shows your fixed values.

### `opencv_image_contours_process_script.py`
**What it does:** the simplest camera-processing example — subscribes to a camera topic,
draws detected contours as an overlay with OpenCV, and republishes the result as a new
image topic (`<namespace>/image_contours`) you can view in the RUI. Good starting template
for writing your own image-processing automation script.
**Requires:** a live camera image topic.
**Key settings:** `IMAGE_INPUT_TOPIC_NAME`.
**Status:** works — no known gaps.
**In the RUI:** start the camera driver first, then select
`opencv_image_contours_process_script.py` and Start it. Watch the message pane for
`Waiting for topic: color_2d_image` → `Initialization Complete`, then open the RUI's Image
Viewer app and add the `image_contours` topic to see the live overlay.

---

## Tools & helper scripts, and tests

Moved to their own READMEs so each is easier to go through on its own:

- **[`tools/README.md`](tools/README.md)** — every command-line utility in `tools/`
  (AI training-data helpers, calibration backup/restore, batch find-and-replace across
  scripts, the two exceptions that actually deploy/run like real automation scripts), what
  each one does, and the exact command to run it.
- **[`tests/README.md`](tests/README.md)** — the automated unit tests and the manual live
  smoke tests in `tests/`, how each kind works, and a table of exactly what every individual
  test checks.
