#!/usr/bin/env python3
#
# MANUAL / INTEGRATION SMOKE TEST -- NOT part of the automated unittest suite.
#
# led_alerts_action_script.py's ROS dependencies are:
#   1. a topic matching "lsx/status" (expects nepi_interfaces/DeviceLSXStatus)
#   2. a service matching "lsx/capabilities_query"
#      (expects nepi_interfaces/LSXCapabilitiesQuery)
#   3. (KNOWN GAP, not checked here as a pass/fail condition) a topic at
#      <base_namespace>/app_ai_alerts/alert_state (expects std_msgs/Bool) --
#      no app_ai_alerts app exists in this workspace's nepi_apps today, so
#      this topic is expected to be ABSENT on a real device running this
#      workspace's software; its absence is not a regression to flag here.
#
# Like led_auto_level's live smoke test, neither the lsx/status topic nor
# the lsx/capabilities_query service is guaranteed to exist regardless of
# attached hardware -- they only appear if an LSX (LED) driver is currently
# running on the device. So this script does NOT assert those must exist;
# it looks for them and, if found, asserts their message/service TYPE is
# still what this script's plumbing expects (that's the actual API-drift
# risk this session was about -- DeviceLSXStatus / LSXCapabilitiesQuery
# renamed or restructured out from under the script).
#
# Run manually against the real device with:
#   python3 tests/live_smoke_test_led_alerts_action.py
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

EXPECTED_STATUS_TYPE = "nepi_interfaces/DeviceLSXStatus"
EXPECTED_CAPS_SERVICE_TYPE = "nepi_interfaces/LSXCapabilitiesQuery"


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


def check_service_type(name_fragment, expected_type):
    rc, out, err = run_remote("rosservice list")
    if rc != 0:
        print(f"  SKIP: could not list services (rc={rc}): {err}")
        return None

    matches = [s for s in out.splitlines() if name_fragment in s]
    if not matches:
        print(f"  SKIP: no service matching '{name_fragment}' currently advertised "
              f"(requires a matching LSX driver attached) -- not a failure.")
        return None

    all_ok = True
    for service in matches:
        rc, type_out, err = run_remote(f"rosservice type {service}")
        if rc != 0:
            print(f"  FAIL: could not get type for {service}: {err}")
            all_ok = False
            continue
        if type_out != expected_type:
            print(f"  FAIL: {service} has type '{type_out}', expected '{expected_type}'")
            all_ok = False
        else:
            print(f"  PASS: {service} is {expected_type}")
    return all_ok


def check_known_gap_alert_topic_absent():
    """Documents (does not fail on) the confirmed missing-app gap: no
    app_ai_alerts/alert_state topic should exist since app_ai_alerts is not
    part of this workspace's nepi_apps. If one IS found, that's useful
    information (the gap may have been closed) but still not treated as a
    failure of this smoke test.
    """
    rc, out, err = run_remote("rostopic list")
    if rc != 0:
        print(f"  SKIP: could not list topics (rc={rc}): {err}")
        return
    matches = [t for t in out.splitlines() if "app_ai_alerts/alert_state" in t]
    if not matches:
        print("  INFO: no app_ai_alerts/alert_state topic found, as expected "
              "-- this confirms the documented KNOWN GAP is still current "
              "(led_alerts_action_script.py will block forever at "
              "wait_for_topic(alert_state_topic) on this device).")
    else:
        print(f"  INFO: found {matches} -- an app_ai_alerts-like topic now "
              "exists on this device. This script's KNOWN GAP note may be "
              "stale; re-verify against src/nepi_apps before relying on it.")


def main():
    print("Checking device reachability...")
    rc, out, err = run_remote("echo connected")
    if rc != 0:
        print(f"UNREACHABLE: could not reach device over SSH (rc={rc}): {err}")
        print("This smoke test requires network access to the NEPI device; "
              "it has not been executed successfully yet.")
        return 2

    print("Device reachable. Base namespace should be /nepi/device1/.")

    print("\nChecking for an lsx/status topic (nepi_interfaces/DeviceLSXStatus)...")
    status_ok = check_topic_type("lsx/status", EXPECTED_STATUS_TYPE)

    print("\nChecking for an lsx/capabilities_query service "
          "(nepi_interfaces/LSXCapabilitiesQuery)...")
    caps_ok = check_service_type("lsx/capabilities_query", EXPECTED_CAPS_SERVICE_TYPE)

    print("\nChecking KNOWN GAP: app_ai_alerts/alert_state topic (expected absent)...")
    check_known_gap_alert_topic_absent()

    failed = (status_ok is False) or (caps_ok is False)
    if failed:
        print("\nRESULT: FAIL -- one or more found topics/services had an "
              "unexpected type.")
        return 1

    print("\nRESULT: PASS (or SKIP where no matching LSX hardware/driver was attached).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
