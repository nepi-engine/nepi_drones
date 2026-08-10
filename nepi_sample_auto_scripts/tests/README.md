# tests/

Two different kinds of test live here — read this before running either kind, since they
work completely differently and check different things.

| | `test_*.py` (unit tests) | `live_smoke_test_*` (manual smoke tests) |
|---|---|---|
| Runs against | Nothing real — stubbed-out fake ROS/NEPI modules | A real NEPI device, over SSH |
| Run automatically? | Yes — normal `pytest` discovery picks these up | No — never collected by pytest, run by hand |
| Needs a device/ROS? | No | Yes |
| Checks | The script's own Python logic still runs correctly against today's API | Whether the *live topics/services* the script depends on actually still exist, with the expected message type |

## Automated unit tests (`test_*.py`)

**Why these exist:** this dev sandbox has no built catkin workspace — no `roscore`, and
`nepi_sdk`/`nepi_api`/`nepi_interfaces` exist here only as unbuilt source trees, not
installed Python packages with generated message classes. So each test **stubs out**
every unavailable module (`rospy`, `nepi_sdk.nepi_ros`, `nepi_api.messages_if`,
`nepi_interfaces.msg`/`.srv`, etc.) with a fake version that has the *exact* current
call signatures and field names, then actually imports and runs the real script's
`__init__` against those fakes. The point is to catch exactly one class of bug: a renamed
topic, attribute, or message field silently slipping through — if the real API drifts
again, these tests fail on the mismatch instead of staying silently green.

**How to run them all:**
```bash
pip install pytest   # if you don't already have it
cd nepi_sample_auto_scripts
python3 -m pytest tests/
```
Or run just one: `python3 -m pytest tests/test_led_auto_level_process_script.py -v`

**No ROS, no device, no catkin build needed** — these run anywhere Python 3 + pytest are
installed. Safe to run as often as you like.

| Test file | Verifies (for the matching script) |
|---|---|
| `test_ai_detector_config_script.py` | The real (2026-08-06 fix) `ai_models_mgr` enable sequence — `update_framework_state`/`update_model_state` publishers fire with the right `UpdateBool` fields, `wait_for_node` gets called for the launched model, the per-detector `set_img_topic`/`set_threshold`/`enable` publishers get created and used once the node comes up, and both the "node comes up" and "node never comes up" paths idle/clean up correctly. |
| `test_drone_follow_object_mission_script.py` | `__init__` runs through node/namespace setup, RBX settings wiring, and target-follow logic correctly against stand-ins for `nepi_sdk`, `nepi_api`, `nepi_interfaces`, and `geographic_msgs`. |
| `test_drone_inspection_demo_mission_script.py` | Same idea for the inspection mission's takeoff/goto/return-home flow. |
| `test_led_adjust_on_object_detect_action_script.py` | The LED-control wiring (on/off, intensity, blink) works correctly standalone; the real (2026-08-06 fix) `AiBoundingBoxes`-derived detection logic correctly matches/ignores labels and debounces lost-count over `LOST_COUNT_THRESHOLD` consecutive misses (also guards the real double-counting bug fixed in that same pass). |
| `test_led_alerts_action_script.py` | The LSX status-subscribe/capabilities-query/action-dispatch logic is correct against the current `DeviceLSXStatus`/`LSXCapabilitiesQuery` shapes; the real (2026-08-06 fix) `AiBoundingBoxes`-derived alert state correctly flips True/False and debounces over `ALERT_LOST_COUNT_THRESHOLD`. |
| `test_led_auto_level_process_script.py` | The brightness-estimate → LED-intensity control loop is correct, using the *real* `cv_bridge`/`cv2`/`numpy` (genuinely importable here) with only `rospy`/`nepi_sdk`/`nepi_api` stubbed. |
| `test_led_step_adjust_process_script.py` | The step-up/wrap-to-zero timer logic and shutdown-to-zero cleanup behavior are correct. |
| `test_navpose_config_script.py` | The `set_frame_comp_topic` wiring (GPS/odom/heading → `base_frame` components) matches the current `navpose_mgr` API. |
| `test_navpose_set_fixed_config_script.py` | The fixed-NavPose publish (`set_frame_fixed_navpose` + `set_frame_comp_topic` set to `'Fixed'`) matches the current `navpose_mgr` API. |
| `test_opencv_image_contours_process_script.py` | The contour-overlay image pipeline is correct, using the real `cv_bridge`/`cv2` with only `rospy`/`nepi_sdk`/`nepi_api` stubbed. |

## Manual live smoke tests (`live_smoke_test_*`)

**Why these exist:** the unit tests above prove the script's *logic* still runs against a
faked API — they can't prove the *real* topics/services that logic depends on still exist
on an actual device with the names and message types the script expects. These smoke tests
check exactly that, against a real live device, over SSH. **None of them assert the script
itself works end-to-end** — they check "does the dependency this script needs actually
exist, with the right type," which is the specific thing that breaks silently when NEPI's
own API changes underneath a script like this.

**⚠ None of these have actually been run against a live device yet** — they were written
against the confirmed-correct current topic/service names and types, but the device wasn't
reachable from the sandbox they were authored in. Run them by hand once you have SSH access
to close that gap.

**Prerequisites:**
- SSH reachability to the device: `ssh -p 2222 -i ~/.ssh/nepi_default_ssh_key nepi@nepi`
  (every script here shells out to this exact command internally — if it can't reach the
  device, the script reports that clearly and stops).
- The device's NEPI environment sourced remotely — each script/command already does this
  itself (`source /opt/nepi/nepi_engine/setup.bash`) as part of the SSH call; you don't need
  to do anything extra locally besides having SSH access.

**How to run one:**
```bash
python3 tests/live_smoke_test_<name>.py
# or, for the one shell-script exception:
bash tests/live_smoke_test_led_intensity_topic.sh
```

**Reading the output:** each one prints a `FAIL` for something that's asserted as always
required and missing (a real regression), vs. an `INFO`/`SKIP` for something that's only
expected to exist when specific hardware/drivers/sims happen to be attached — read each
one's own header comment (right at the top of the file) for exactly which of its checks
fall into which category before treating a result as a failure.

| Test file | Checks against the live device |
|---|---|
| `live_smoke_test_ai_detector_config.py` | `ai_models_mgr`'s current control topics exist with the right types, **and** confirms the *old* `ai_detector_mgr/start_classifier`/`stop_classifier` topics this script originally used genuinely don't exist (proving the documented gap is real, not a misreading). |
| `live_smoke_test_drone_follow_object_mission.py` | `app_fake_gps/enable` (required — a base app, always present); an RBX-capable robot namespace's topics/services (only if a robot driver is attached — checked, not required). |
| `live_smoke_test_drone_inspection_demo_mission.py` | `app_fake_gps/enable` + the system-wide `snapshot_trigger` (both required); RBX driver topics (only if attached). |
| `live_smoke_test_led_adjust_on_object_detect.py` | The four `lsx/*` LED-control topics (checked if an LSX light is attached) and, info-only, whether the old `ai_detector_mgr` bounding-box topics exist (expected absent). |
| `live_smoke_test_led_alerts_action.py` | `lsx/status` + `lsx/capabilities_query` (checked if an LSX light is attached); info-only check that `app_ai_alerts/alert_state` is (expectedly) absent. |
| `live_smoke_test_led_auto_level.py` | `lsx/set_intensity` and a `color_2d_image`-matching topic (both only if the relevant hardware/driver is attached) — if found, asserts their message type is still correct. |
| `live_smoke_test_led_intensity_topic.sh` | Same idea as the Python ones but shell-scripted: looks for any `lsx/*set_intensity` topic and checks its type is `std_msgs/Float32`. Named for the topic it checks, not the script — it's the smoke test for `led_step_adjust_process_script.py`. |
| `live_smoke_test_navpose_config.py` | `navpose_mgr`'s `set_frame_comp_topic` topic (required — always present regardless of hardware); the three RBX source topics it points at (only if a robot driver is attached). |
| `live_smoke_test_navpose_set_fixed_config.py` | `navpose_mgr`'s `set_frame_fixed_navpose` + `set_frame_comp_topic` topics (both required — `base_frame` always exists, no hardware-dependent surface here). |
| `live_smoke_test_opencv_image_contours.py` | A `color_2d_image`-matching input topic (only if a camera is attached) and the script's own `image_contours` output topic (only while the script itself is running) — if found, asserts message types. |
