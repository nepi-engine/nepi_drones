#!/usr/bin/env python3
#
# MANUAL / INTEGRATION SMOKE TEST -- NOT part of the automated unittest suite.
#
# led_adjust_on_object_detect_action_script.py's ROS dependencies are:
#   1. a topic matching "lsx/turn_on_off"       (expects std_msgs/Bool)
#   2. a topic matching "lsx/set_intensity"     (expects std_msgs/Float32)
#   3. a topic matching "lsx/blink_on_off"      (expects std_msgs/Bool)
#   4. a topic matching "lsx/set_blink_interval" (expects std_msgs/Float32)
#   5. (KNOWN GAP, checked here as an INFO-only, non-failing observation)
#      <base_namespace>/ai_detector_mgr/bounding_boxes,
#      <base_namespace>/ai_detector_mgr/detection_image, and
#      <base_namespace>/ai_detector_mgr/found_object -- ai_detector_mgr does
#      not exist in this workspace (replaced by ai_models_mgr, a completely
#      different framework/model-enable architecture -- see
#      src/nepi_engine/nepi_managers/scripts/ai_models_mgr.py), and the
#      darknet_ros_msgs package (BoundingBoxes/ObjectCount message types)
#      does not exist anywhere in this workspace either. This script's
#      AI-detector wiring is therefore expected to be unreachable on a real
#      device regardless of attached hardware; its absence is not a
#      regression to flag here.
#
# Like led_alerts_action_script's and led_auto_level's live smoke tests,
# none of the lsx/* topics are guaranteed to exist regardless of attached
# hardware -- they only appear if an LSX (LED) driver is currently running
# on the device. So this script does NOT assert those topics must exist; it
# looks for them and, if found, asserts their message TYPE is still what
# this script's plumbing expects (that's the actual API-drift risk this
# session was about).
#
# Run manually against the real device with:
#   python3 tests/live_smoke_test_led_adjust_on_object_detect.py
#
# Requires SSH reachability to the device:
#   ssh -p 2222 -i ~/.ssh/nepi_default_ssh_key -o ConnectTimeout=5 nepi@nepi \
#       'source /opt/nepi/nepi_engine/setup.bash; <cmd>'
# This was NOT reachable from the sandbox this test was authored in
# (connection timed out) -- it has not been run against a live device yet.

import subprocess
import sys

SSH_CMD = [
    "ssh", "-p", "2222",
    "-i", "/home/suraj/.ssh/nepi_default_ssh_key",
    "-o", "ConnectTimeout=5",
    "-o", "BatchMode=yes",
    "nepi@nepi",
]

REMOTE_SOURCE = "source /opt/nepi/nepi_engine/setup.bash"

LSX_TOPIC_TYPES = [
    ("lsx/turn_on_off", "std_msgs/Bool"),
    ("lsx/set_intensity", "std_msgs/Float32"),
    ("lsx/blink_on_off", "std_msgs/Bool"),
    ("lsx/set_blink_interval", "std_msgs/Float32"),
]

KNOWN_GAP_TOPIC_FRAGMENTS = [
    "ai_detector_mgr/bounding_boxes",
    "ai_detector_mgr/detection_image",
    "ai_detector_mgr/found_object",
]


def run_remote(cmd):
    """Run `cmd` on the device after sourcing the NEPI setup.bash. Returns
    (returncode, stdout, stderr)."""
    full_cmd = SSH_CMD + [f"{REMOTE_SOURCE}; {cmd}"]
    proc = subprocess.run(full_cmd, capture_output=True, text=True, timeout=30)
    return proc.returncode, proc.stdout.strip(), proc.stderr.strip()


def check_topic_type(name_fragment, expected_type):
    rc, out, err = run_remote("rostopic list")
    if rc != 0:
        print(f"  SKIP: could not list topics (rc={rc}): {err}")
        return None

    matches = [t for t in out.splitlines() if name_fragment in t]
    if not matches:
        print(f"  SKIP: no topic matching '{name_fragment}' currently published "
              f"(requires a matching LSX driver attached) -- not a failure.")
        return None

    all_ok = True
    for topic in matches:
        rc, type_out, err = run_remote(f"rostopic type {topic}")
        if rc != 0:
            print(f"  FAIL: could not get type for {topic}: {err}")
            all_ok = False
            continue
        if type_out != expected_type:
            print(f"  FAIL: {topic} has type '{type_out}', expected '{expected_type}'")
            all_ok = False
        else:
            print(f"  PASS: {topic} is {expected_type}")
    return all_ok


def check_known_gap_ai_detector_topics_absent():
    """Documents (does not fail on) the confirmed missing-node gap: no
    ai_detector_mgr/* topics should exist since ai_detector_mgr was replaced
    by ai_models_mgr in this workspace. If any ARE found, that's useful
    information (the gap may have been closed, or a legacy node is
    somehow still running) but still not treated as a failure of this
    smoke test.
    """
    rc, out, err = run_remote("rostopic list")
    if rc != 0:
        print(f"  SKIP: could not list topics (rc={rc}): {err}")
        return
    for fragment in KNOWN_GAP_TOPIC_FRAGMENTS:
        matches = [t for t in out.splitlines() if fragment in t]
        if not matches:
            print(f"  INFO: no topic matching '{fragment}' found, as expected "
                  "-- this confirms the documented KNOWN GAP is still current "
                  "(led_adjust_on_object_detect_action_script.py will block "
                  "forever at wait_for_topic(AI_DETECTION_IMAGE_TOPIC), and "
                  "its darknet_ros_msgs import would ImportError first "
                  "anyway, on this device).")
        else:
            print(f"  INFO: found {matches} -- an ai_detector_mgr-like topic "
                  "now exists on this device. This script's KNOWN GAP note "
                  "may be stale; re-verify against "
                  "src/nepi_engine/nepi_managers before relying on it.")


def main():
    print("Checking device reachability...")
    rc, out, err = run_remote("echo connected")
    if rc != 0:
        print(f"UNREACHABLE: could not reach device over SSH (rc={rc}): {err}")
        print("This smoke test requires network access to the NEPI device; "
              "it has not been executed successfully yet.")
        return 2

    print("Device reachable. Base namespace should be /nepi/device1/.")

    overall_ok = True
    for fragment, expected_type in LSX_TOPIC_TYPES:
        print(f"\nChecking for a '{fragment}' topic ({expected_type})...")
        result = check_topic_type(fragment, expected_type)
        if result is False:
            overall_ok = False

    print("\nChecking KNOWN GAP: ai_detector_mgr/* topics (expected absent)...")
    check_known_gap_ai_detector_topics_absent()

    if not overall_ok:
        print("\nRESULT: FAIL -- one or more found lsx/* topics had an "
              "unexpected type.")
        return 1

    print("\nRESULT: PASS (or SKIP where no matching LSX hardware/driver was attached).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
