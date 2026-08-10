#!/usr/bin/env python3
#
# MANUAL / INTEGRATION SMOKE TEST -- NOT part of the automated unittest suite.
#
# navpose_config_script.py's core dependency is the navpose_mgr node's
# set_frame_comp_topic topic (nepi_interfaces/UpdateString). Unlike an
# RBX/LSX driver topic, navpose_mgr runs on the device regardless of
# whether any drone/rover/LED hardware or sim is attached (per this
# session's confirmed architecture notes), so this smoke test DOES assert
# that topic must exist with the expected type -- that's the actual
# API-drift risk this port depends on.
#
# The script's three *source* topics (rbx/gps_fix, rbx/odom, rbx/heading)
# are RBX-driver-dependent and will only exist if a robot driver (e.g.
# ardupilot_rbx_driver_script.py) is currently attached/running, so those
# are checked opportunistically (SKIP, not FAIL, if absent) exactly like
# the sibling LED smoke test's pattern for hardware-dependent topics.
#
# Run manually against the real device with:
#   python3 tests/live_smoke_test_navpose_config.py
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


def check_topic_type(name_fragment, expected_type, required):
    rc, out, err = run_remote("rostopic list")
    if rc != 0:
        print(f"  SKIP: could not list topics (rc={rc}): {err}")
        return None

    matches = [t for t in out.splitlines() if name_fragment in t]
    if not matches:
        if required:
            print(f"  FAIL: no topic matching '{name_fragment}' found -- "
                  f"navpose_mgr should publish this regardless of attached "
                  f"hardware.")
            return False
        print(f"  SKIP: no topic matching '{name_fragment}' currently published "
              f"(requires an RBX driver attached) -- not a failure.")
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

    print("\nChecking for navpose_mgr/set_frame_comp_topic (nepi_interfaces/UpdateString, "
          "required -- navpose_mgr runs regardless of attached hardware)...")
    comp_topic_ok = check_topic_type(
        "navpose_mgr/set_frame_comp_topic", "nepi_interfaces/UpdateString", required=True)

    print("\nChecking for rbx/gps_fix (sensor_msgs/NavSatFix, optional -- requires an "
          "RBX driver attached)...")
    gps_ok = check_topic_type("rbx/gps_fix", "sensor_msgs/NavSatFix", required=False)

    print("\nChecking for rbx/odom (nav_msgs/Odometry, optional -- requires an RBX "
          "driver attached)...")
    odom_ok = check_topic_type("rbx/odom", "nav_msgs/Odometry", required=False)

    results = [comp_topic_ok, gps_ok, odom_ok]
    if any(r is False for r in results):
        print("\nRESULT: FAIL -- one or more checked topics were missing (when "
              "required) or had an unexpected type.")
        return 1

    print("\nRESULT: PASS (or SKIP where no matching hardware/driver was attached).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
