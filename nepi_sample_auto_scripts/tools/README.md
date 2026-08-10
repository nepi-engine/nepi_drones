# tools/

One-shot command-line utilities. Unlike the automation scripts one level up, these are
**not** long-running ROS nodes launched through the RUI's Automation Scripts panel — you run
them directly from a terminal, they do their job, print a summary, and exit. Two exceptions
are called out explicitly below (`start_stop_scripts_helper_script.py`, which does run as a
ROS node, and `sim_ai_targeting_bridge_script.py`, which deploys like a real automation
script despite living in this folder).

Every script has its own `SETUP - Edit as Necessary` block near the top — open the file and
edit those constants before running.

## AI training-data helpers (no ROS needed)

These four all work the same way: they look for **sub-folders sitting next to the script
itself**, scan each one, and write their output back into that same sub-folder. Run them
with a plain `python3 <script>.py` from inside `tools/` — no ROS, no NEPI device, no
`nepi_sdk` needed at all.

### `partition_ai_training_data_script.py`
**What it does:** scans each data sub-folder for `<image>.<ext>` / `<image>.txt` label
pairs, then writes `data_train.txt` and `data_test.txt` (lists of file paths) split
randomly according to `TEST_DATA_PERCENTAGE`. Images with no matching label file go into
`data_unlabeled.txt` instead, with a warning.
**Edit first:** `TEST_DATA_PERCENTAGE` (default 20).
**Run:** `python3 partition_ai_training_data_script.py`

### `create_txt_from_xml_labels_script.py`
**What it does:** converts Pascal-VOC-style `.xml` bounding-box label files into
Darknet-style `.txt` label files, one folder at a time.
**Requires:** each data sub-folder needs its own `classes.txt` file (one class name per
line) — folders without one are skipped with a message, not an error. Needs the
`declxml` Python package installed.
**Edit first:** nothing required; `TEST_DATA_PERCENTAGE` is defined but unused by this
script.
**Run:** `python3 create_txt_from_xml_labels_script.py`

### `remove_currupt_image_files_script.py`
**What it does:** opens every image file it finds in each data sub-folder; any image that
fails to open is deleted, along with its matching `.xml` label file if one exists. A
cleanup pass to run before training on a data set you didn't fully curate yourself.
**Edit first:** nothing required.
**Run:** `python3 remove_currupt_image_files_script.py`

### `rename_xml_label_script.py`
**What it does:** batch-renames one class label to another across every `.xml` file in
each data sub-folder (e.g. fixing an inconsistently-typed label like `"Hard Hat"` →
`"HardHat"`).
**Edit first:** `Orig_Label`, `New_Label`.
**Run:** `python3 rename_xml_label_script.py`

## NEPI-connected utilities (need a ROS/NEPI environment)

These import `nepi_sdk`/`rospy` and only work when run where a NEPI Python environment is
reachable (on the device itself, or a dev VM with the workspace built and sourced) —
running them from a plain sandbox without that will fail on import.

### `navpose_convert_yaml_files.py`
**What it does:** finds NavPose `.yaml` data files (as written by `navpose_mgr`) and
converts them into a human-readable printout.
**Edit first:** nothing on the command line — it's hardcoded to read from
`/mnt/nepi_storage/data` (edit that path in the `if __name__ == '__main__':` block at the
bottom of the file if your data lives elsewhere).
**Run (on the device, or anywhere the NEPI Python env is sourced):**
`python3 navpose_convert_yaml_files.py`
**⚠ Known gap:** built against an older, nested NavPose message shape that predates the
current flat `NavPose.msg` — see the file's own header comment before trusting its output
against current save-data files.

### `zed_cal_file_backup_restore.py`
**What it does:** backs up a ZED camera's calibration files from `/usr/local/zed/settings`
to `/mnt/nepi_storage/user_cfg/zed_cals` (and can restore them back the other way — see the
bottom of the file for the restore call, commented differently than the backup call).
**Edit first:** nothing required; the source/backup paths are set as constants in the
`if __name__ == '__main__':` block if you need to point them elsewhere.
**Run (on the device):** `python3 zed_cal_file_backup_restore.py`

### `start_stop_scripts_helper_script.py`
**What it does:** the one script here that's actually a ROS node, not a one-shot CLI tool.
On startup it calls `scripts_mgr`'s services to launch every script named in `SCRIPT_LIST`
that isn't already running; on shutdown it stops them again, except any that were *already*
running before this helper started (those get left alone/relaunched as they were).
Effectively a scripted "start this batch of automation scripts together" instead of
clicking each one on in the RUI.
**Edit first:** `SCRIPT_LIST` (filenames of the automation scripts to manage together).
**Run:** deploy and launch it exactly like a normal automation script (see the top-level
`README.md`'s "How to run any automation script" section) — it needs `scripts_mgr`'s
services, which only exist on a running NEPI device.

### `sim_ai_targeting_bridge_script.py`
**Not a plain CLI tool — despite living in `tools/`, deploy and run this exactly like a
real automation script** (top-level `README.md`'s "How to run" section), alongside
`drone_follow_object_mission_script.py`. It's SITL/Gazebo-only test scaffolding: it connects
to the dev VM's `ai_targeting_controller_ardupilot.py` bridge (`127.0.0.1:9027`, over the
same reverse SSH tunnel the ArduPilot SITL setup already uses), relays the simulated
target's position onto `app_ai_targeting/target_localizations`, and republishes the RBX
driver's live camera feed onto `app_ai_targeting/targeting_image` — standing in for the
`app_ai_targeting` app that doesn't exist in this workspace yet, so
`drone_follow_object_mission_script.py`'s follow logic has something real to react to
during SITL testing.
**Edit first:** `BRIDGE_HOST`/`BRIDGE_PORT` (only if your tunnel uses a different port than
9027), `RBX_ROBOT_NAME` (must match the mission script's own setting).

## Script-maintenance helpers (batch edit other scripts — use with care)

Both of these rewrite files in place. **Work on a copy of your scripts in a temp folder
first** (their own docstrings say the same thing) — there's no undo.

### `replace_strings_helper_script.py`
**What it does:** scans every `.py` file in `SCRIPT_FOLDER` (or just the files named in
`SCRIPT_LIST`, if you fill that in) and replaces every occurrence of each `[find, replace]`
pair in `FIND_REPLACE_LIST`, anywhere it appears on a line.
**Edit first:** `SCRIPT_FOLDER`, `SCRIPT_LIST` (optional filter), `FIND_REPLACE_LIST`.
**Run:** `sudo python3 replace_strings_helper_script.py` (needs `sudo` if `SCRIPT_FOLDER`
is a system path like `/mnt/nepi_storage/...` that your user can't write to directly).

### `replace_key_value_helper_script.py`
**What it does:** similar idea, but line-oriented rather than substring-oriented — for
every file in `SCRIPT_FOLDER`, any line that *starts with* `KEY_WORD_STRING` gets replaced
entirely with `KEY_WORD_STRING = "<KEY_WORD_VALUE_OR_STRING>"`. Useful for batch-updating a
`USER SETTINGS` constant (like a topic name) across a whole folder of deployed scripts at
once, without opening each one by hand.
**Edit first:** `SCRIPT_FOLDER`, `SCRIPT_LIST` (optional filter), `KEY_WORD_STRING`,
`KEY_WORD_VALUE_OR_STRING`.
**Run:** `sudo python3 replace_key_value_helper_script.py` (same `sudo` caveat as above).
