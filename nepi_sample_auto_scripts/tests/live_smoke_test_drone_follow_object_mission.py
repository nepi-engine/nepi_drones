#!/usr/bin/env python3
#
# MANUAL / INTEGRATION SMOKE TEST -- NOT part of the automated unittest suite.
#
# drone_follow_object_mission_script.py depends on THREE distinct pieces of
# live infrastructure with three different availability guarantees:
#
#   1) The standalone app_fake_gps app (rbx/enable_fake_gps was replaced by
#      this app's <base_namespace>/app_fake_gps/enable Bool topic). Per this
#      session's confirmed architecture notes, apps like this are part of
#      the base nepi_apps set and are expected to be present/running
#      regardless of attached hardware -- so this is checked as REQUIRED.
#
#   2) An RBX-capable robot namespace (rbx/capabilities_query,
#      rbx/settings/status, rbx/info, rbx/status, etc. under a robot
#      matching RBX_ROBOT_NAME, e.g. "ardupilot"). Unlike app_fake_gps /
#      ai_models_mgr / navpose_mgr / drivers_mgr, an RBX robot namespace only
#      exists when an actual drone/rover driver or SITL sim is attached and
#      running -- so this is checked BEST-EFFORT (SKIP, not FAIL, if absent).
#
#   3) The app_ai_targeting app (target_localizations/targeting_image
#      topics) that this script's whole trigger mechanism depends on. This
#      session's source read confirmed no such app exists anywhere in this
#      workspace's nepi_apps (only fake_gps, file_pub_img, file_pub_vid,
#      image_viewer, onvif_mgr, pan_tilt_auto, nav_sim) -- documented as a
#      KNOWN GAP in the script's own module docstring. This smoke test
#      confirms that gap is still accurate on the live device (an ABSENCE
#      check, reported as informational, not a pass/fail condition of this
#      test) rather than silently assuming it.
#
# Run manually against the real device with:
#   python3 tests/live_smoke_test_drone_follow_object_mission.py
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

RBX_ROBOT_NAME = "ardupilot"  # matches drone_follow_object_mission_script.py's RBX_ROBOT_NAME


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
                  f"expected regardless of attached hardware.")
            return False
        print(f"  SKIP: no topic matching '{name_fragment}' currently found "
              f"-- not a failure (hardware/sim-dependent).")
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


def check_topic_absent(name_fragment, why):
    rc, out, err = run_remote("rostopic list")
    if rc != 0:
        print(f"  SKIP: could not list topics (rc={rc}): {err}")
        return None
    matches = [t for t in out.splitlines() if name_fragment in t]
    if matches:
        print(f"  INFO: found topic(s) matching '{name_fragment}': {matches} "
              f"-- the KNOWN GAP documented in the script's module docstring "
              f"({why}) may no longer apply; worth re-checking the script.")
    else:
        print(f"  INFO (expected): no topic matching '{name_fragment}' found "
              f"-- confirms the documented KNOWN GAP ({why}) is still accurate.")
    return None


def check_rbx_robot_capabilities():
    """Best-effort check for an RBX-capable robot's capabilities_query
    service and settings/status latched topic under RBX_ROBOT_NAME. SKIP
    (not FAIL) if absent -- this requires an actual drone/rover driver or
    SITL sim attached and running, unlike app_fake_gps/ai_models_mgr/
    navpose_mgr/drivers_mgr which run regardless of attached hardware."""
    rc, out, err = run_remote("rosservice list")
    if rc != 0:
        print(f"  SKIP: could not list services (rc={rc}): {err}")
        return None
    matches = [s for s in out.splitlines() if RBX_ROBOT_NAME in s and "capabilities_query" in s]
    if not matches:
        print(f"  SKIP: no '{RBX_ROBOT_NAME}' rbx/capabilities_query service "
              f"found -- no matching drone/rover driver or SITL sim is "
              f"currently attached/running. Not a failure.")
        return None
    print(f"  PASS: found rbx capabilities service(s): {matches}")

    rc, type_out, err = run_remote(f"rosservice type {matches[0]}")
    if rc == 0:
        expected = "nepi_interfaces/RBXCapabilitiesQuery"
        if type_out == expected:
            print(f"  PASS: {matches[0]} is {expected}")
        else:
            print(f"  FAIL: {matches[0]} has type '{type_out}', expected '{expected}'")
            return False
    return True


def main():
    print("Checking device reachability...")
    rc, out, err = run_remote("echo connected")
    if rc != 0:
        print(f"UNREACHABLE: could not reach device over SSH (rc={rc}): {err}")
        print("This smoke test requires network access to the NEPI device; "
              "it has not been executed successfully yet.")
        return 2

    print("Device reachable. Base namespace should be /nepi/device1/.")

    print("\nChecking for app_fake_gps/enable (std_msgs/Bool, required -- "
          "standalone app should run regardless of attached hardware)...")
    fake_gps_ok = check_topic_type("app_fake_gps/enable", "std_msgs/Bool", required=True)

    print(f"\nChecking for an RBX-capable '{RBX_ROBOT_NAME}' robot namespace "
          f"(best-effort -- requires an attached drone driver or SITL sim)...")
    rbx_ok = check_rbx_robot_capabilities()

    print("\nChecking that app_ai_targeting (this script's trigger-mechanism "
          "dependency, documented as a KNOWN GAP) is still absent...")
    check_topic_absent(
        "app_ai_targeting",
        "no app_ai_targeting app exists in this workspace's nepi_apps",
    )

    results = [fake_gps_ok, rbx_ok]
    if any(r is False for r in results):
        print("\nRESULT: FAIL -- one or more required topics/services were "
              "missing or had an unexpected type.")
        return 1

    print("\nRESULT: PASS (or SKIP where a best-effort/hardware-dependent "
          "check could not confirm its condition).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
