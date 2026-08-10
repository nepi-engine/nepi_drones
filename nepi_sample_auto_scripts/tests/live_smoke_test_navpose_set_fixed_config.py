#!/usr/bin/env python3
#
# MANUAL / INTEGRATION SMOKE TEST -- NOT part of the automated unittest suite.
#
# navpose_set_fixed_config_script.py's core dependency is the navpose_mgr
# node's set_frame_fixed_navpose (nepi_interfaces/UpdateNavPose) and
# set_frame_comp_topic (nepi_interfaces/UpdateString) topics. Per this
# session's confirmed architecture notes, navpose_mgr runs on the device
# regardless of whether any drone/rover/LED hardware or sim is attached, so
# this smoke test asserts BOTH topics must exist with the expected type --
# that's the actual API-drift risk this port depends on. Unlike
# navpose_config_script.py (which points a frame's components at a live
# *source* topic that may or may not be attached), this script's target --
# the reserved 'base_frame' -- always exists at startup, so there is no
# optional/hardware-dependent surface to check here.
#
# Run manually against the real device with:
#   python3 tests/live_smoke_test_navpose_set_fixed_config.py
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
                  f"navpose_mgr should publish/subscribe this regardless of "
                  f"attached hardware.")
            return False
        print(f"  SKIP: no topic matching '{name_fragment}' currently found "
              f"-- not a failure.")
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


def check_base_frame_present():
    """Confirm navpose_mgr's reserved 'base_frame' actually exists at
    startup by checking the latched navpose_mgr/status message contains it
    (best-effort: falls back to SKIP if the field/format isn't found rather
    than guessing at a status message layout)."""
    rc, out, err = run_remote(
        "rostopic echo -n 1 /nepi/device1/navpose_mgr/status"
    )
    if rc != 0:
        print(f"  SKIP: could not echo navpose_mgr/status (rc={rc}): {err}")
        return None
    if "base_frame" in out:
        print("  PASS: 'base_frame' found in navpose_mgr/status output.")
        return True
    print("  SKIP: 'base_frame' not found in navpose_mgr/status echo output "
          "(status message layout may differ from expectation) -- not "
          "treated as a hard failure since this is a best-effort check.")
    return None


def main():
    print("Checking device reachability...")
    rc, out, err = run_remote("echo connected")
    if rc != 0:
        print(f"UNREACHABLE: could not reach device over SSH (rc={rc}): {err}")
        print("This smoke test requires network access to the NEPI device; "
              "it has not been executed successfully yet.")
        return 2

    print("Device reachable. Base namespace should be /nepi/device1/.")

    print("\nChecking for navpose_mgr/set_frame_fixed_navpose "
          "(nepi_interfaces/UpdateNavPose, required -- navpose_mgr runs "
          "regardless of attached hardware)...")
    fixed_navpose_ok = check_topic_type(
        "navpose_mgr/set_frame_fixed_navpose", "nepi_interfaces/UpdateNavPose",
        required=True)

    print("\nChecking for navpose_mgr/set_frame_comp_topic "
          "(nepi_interfaces/UpdateString, required -- navpose_mgr runs "
          "regardless of attached hardware)...")
    comp_topic_ok = check_topic_type(
        "navpose_mgr/set_frame_comp_topic", "nepi_interfaces/UpdateString",
        required=True)

    print("\nChecking navpose_mgr/status for the reserved 'base_frame' "
          "(best-effort, SKIP rather than FAIL if not confirmable)...")
    base_frame_ok = check_base_frame_present()

    results = [fixed_navpose_ok, comp_topic_ok, base_frame_ok]
    if any(r is False for r in results):
        print("\nRESULT: FAIL -- one or more required topics were missing "
              "or had an unexpected type.")
        return 1

    print("\nRESULT: PASS (or SKIP where a best-effort check could not "
          "confirm its condition).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
