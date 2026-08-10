#!/usr/bin/env python3
#
# MANUAL / INTEGRATION SMOKE TEST -- NOT part of the automated unittest suite.
#
# led_auto_level_process_script.py's two ROS dependencies are:
#   1. a topic matching "lsx/set_intensity" (expects std_msgs/Float32)
#   2. a topic matching "color_2d_image" (expects sensor_msgs/Image)
#
# Unlike ai_models_mgr / navpose_mgr / drivers_mgr, neither of these topics is
# guaranteed to exist regardless of attached hardware -- they only appear if
# an LSX (LED) driver and a camera/image-publishing driver or app are
# currently running on the device. So this script does NOT assert those
# topics must exist; it looks for them and, if found, asserts their message
# TYPE is still what this script's plumbing expects (that's the actual
# API-drift risk -- a topic renamed or its type changed out from under a
# script that never subscribes to a nepi_interfaces message here at all).
#
# Run manually against the real device with:
#   python3 tests/live_smoke_test_led_auto_level.py
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
              f"(requires matching hardware/driver attached) -- not a failure.")
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


def main():
    print("Checking device reachability...")
    rc, out, err = run_remote("echo connected")
    if rc != 0:
        print(f"UNREACHABLE: could not reach device over SSH (rc={rc}): {err}")
        print("This smoke test requires network access to the NEPI device; "
              "it has not been executed successfully yet.")
        return 2

    print("Device reachable. Base namespace should be /nepi/device1/.")

    print("\nChecking for an lsx/set_intensity topic (std_msgs/Float32)...")
    lsx_ok = check_topic_type("lsx/set_intensity", "std_msgs/Float32")

    print("\nChecking for a color_2d_image topic (sensor_msgs/Image)...")
    img_ok = check_topic_type("color_2d_image", "sensor_msgs/Image")

    failed = (lsx_ok is False) or (img_ok is False)
    if failed:
        print("\nRESULT: FAIL -- one or more found topics had an unexpected type.")
        return 1

    print("\nRESULT: PASS (or SKIP where no matching hardware/driver was attached).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
