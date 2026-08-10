#!/usr/bin/env python3
#
# MANUAL / INTEGRATION SMOKE TEST -- NOT part of the automated unittest suite.
#
# ai_detector_config_script.py's original purpose was to point
# ai_detector_mgr at an image topic and start/stop a named classifier via
# ai_detector_mgr/start_classifier + ai_detector_mgr/stop_classifier
# (ClassifierSelection / Empty). Per this session's confirmed API-change
# notes and the script's own KNOWN GAP docstring, ai_detector_mgr was
# replaced by ai_models_mgr, which has a structurally different, per-caller
# incompatible architecture -- there is no "start this model on this image
# topic" call anymore.
#
# This smoke test does NOT (and cannot) verify the script's original
# mechanism still works, because that mechanism no longer exists anywhere to
# point at. Instead it verifies the KNOWN GAP claim itself against the live
# device, i.e. it checks that:
#   1. ai_models_mgr is running and exposes the current control topics this
#      session's source reading found (refresh_frameworks,
#      update_framework_state, enable_all_models, disable_all_models,
#      update_model_state) with their documented types.
#   2. The OLD ai_detector_mgr/start_classifier and
#      ai_detector_mgr/stop_classifier topics genuinely do NOT exist on the
#      device -- confirming the gap is real and not a misreading of the
#      source, and catching the (unlikely but worth guarding) case where a
#      future firmware still runs both managers side by side.
#
# Run manually against the real device with:
#   python3 tests/live_smoke_test_ai_detector_config.py
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

BASE_NS = "/nepi/device1"

# Current ai_models_mgr control topics (confirmed via source read of
# src/nepi_engine/nepi_managers/scripts/ai_models_mgr.py SUBS_DICT/PUBS_DICT)
# and their expected rostopic-reported types.
CURRENT_AI_MODELS_MGR_TOPICS = {
    f"{BASE_NS}/ai_models_mgr/refresh_frameworks": "std_msgs/Empty",
    f"{BASE_NS}/ai_models_mgr/update_framework_state": "nepi_interfaces/UpdateBool",
    f"{BASE_NS}/ai_models_mgr/enable_all_models": "nepi_interfaces/UpdateBool",
    f"{BASE_NS}/ai_models_mgr/disable_all_models": "nepi_interfaces/UpdateBool",
    f"{BASE_NS}/ai_models_mgr/update_model_state": "nepi_interfaces/UpdateBool",
    f"{BASE_NS}/ai_models_mgr/status": "nepi_interfaces/MgrAiModelsStatus",
}

# Old ai_detector_mgr topics this script originally published to. These must
# NOT exist -- their absence is the actual regression this test guards.
OLD_AI_DETECTOR_MGR_TOPICS = [
    f"{BASE_NS}/ai_detector_mgr/start_classifier",
    f"{BASE_NS}/ai_detector_mgr/stop_classifier",
]


def run_remote(cmd, timeout=30):
    """Run `cmd` on the device after sourcing the NEPI setup.bash. Returns
    (returncode, stdout, stderr)."""
    full_cmd = SSH_CMD + [f"{REMOTE_SOURCE}; {cmd}"]
    proc = subprocess.run(full_cmd, capture_output=True, text=True, timeout=timeout)
    return proc.returncode, proc.stdout.strip(), proc.stderr.strip()


def get_topic_list():
    rc, out, err = run_remote("rostopic list")
    if rc != 0:
        return None, err
    return out.splitlines(), None


def get_topic_type(topic):
    rc, out, err = run_remote(f"rostopic type {topic}")
    if rc != 0:
        return None
    return out.strip()


def main():
    print("Checking SSH reachability to nepi device...")
    topics, err = get_topic_list()
    if topics is None:
        print(f"SKIP: could not reach device or list topics ({err}). "
              "This smoke test requires the live NEPI device; it is not run "
              "as part of the automated suite for exactly this reason.")
        return 0

    failures = []

    print("\n-- Checking current ai_models_mgr topics exist with expected types --")
    for topic, expected_type in CURRENT_AI_MODELS_MGR_TOPICS.items():
        if topic not in topics:
            failures.append(f"MISSING expected ai_models_mgr topic: {topic}")
            print(f"  FAIL: {topic} not found in rostopic list")
            continue
        actual_type = get_topic_type(topic)
        if actual_type != expected_type:
            failures.append(
                f"TYPE MISMATCH on {topic}: expected {expected_type}, got {actual_type}"
            )
            print(f"  FAIL: {topic} type={actual_type!r}, expected {expected_type!r}")
        else:
            print(f"  OK: {topic} ({actual_type})")

    print("\n-- Confirming OLD ai_detector_mgr topics no longer exist --")
    for topic in OLD_AI_DETECTOR_MGR_TOPICS:
        if topic in topics:
            failures.append(
                f"UNEXPECTED: old ai_detector_mgr topic still present: {topic} "
                "(KNOWN GAP docstring in ai_detector_config_script.py may need revisiting)"
            )
            print(f"  FAIL: {topic} unexpectedly present")
        else:
            print(f"  OK: {topic} absent (confirms the documented KNOWN GAP)")

    if failures:
        print(f"\n{len(failures)} FAILURE(S):")
        for f in failures:
            print(f"  - {f}")
        return 1

    print("\nAll checks passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
