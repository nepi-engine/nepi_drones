#!/usr/bin/env python3
#
# MANUAL / INTEGRATION SMOKE TEST -- NOT part of the automated unittest suite.
#
# drone_inspection_demo_mission_script.py's ROS dependencies fall into two
# tiers:
#
#   1. Always-present regardless of attached hardware/sim:
#        - app_fake_gps/enable (std_msgs/Bool) -- standalone app
#        - snapshot_trigger at the base namespace (std_msgs/Empty) --
#          system-wide, per nepi_api/connect_system_if.py's snapshot_all pub
#      These are asserted REQUIRED (FAIL, not SKIP, if absent/wrong-typed).
#
#   2. Only present once an RBX driver (e.g. ardupilot_rbx_driver_script.py)
#      is attached and running:
#        - rbx/capabilities_query (nepi_interfaces/RBXCapabilitiesQuery service)
#        - rbx/info (nepi_interfaces/DeviceRBXInfo)
#        - rbx/status (nepi_interfaces/DeviceRBXStatus)
#        - rbx/settings/status (nepi_interfaces/SettingsStatus)
#        - rbx/goto_location (nepi_interfaces/GotoLocation)
#        - rbx/set_goto_timeout (std_msgs/UInt32) -- renamed from
#          rbx/set_cmd_timeout this session; this check is the actual
#          API-drift risk this script's fix depends on.
#      These are checked opportunistically (SKIP, not FAIL, if no driver is
#      currently attached), exactly like the sibling LED/navpose smoke
#      tests' pattern for hardware/driver-dependent topics.
#
# Run manually against the real device with:
#   python3 tests/live_smoke_test_drone_inspection_demo_mission.py
#
# Requires SSH reachability to the device:
#   ssh -p 2222 -i ~/.ssh/nepi_default_ssh_key -o ConnectTimeout=5 nepi@nepi \
#       'source /opt/nepi/nepi_engine/setup.bash; <cmd>'
# This was NOT reachable from the sandbox this test was authored in
# ("ssh: connect to host nepi port 2222: Connection timed out") -- it has
# not been run against a live device yet.

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
            print(f"  FAIL: no topic matching '{name_fragment}' found -- this "
                  f"should exist regardless of attached RBX hardware.")
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


def check_service_type(name_fragment, expected_type, required):
    rc, out, err = run_remote("rosservice list")
    if rc != 0:
        print(f"  SKIP: could not list services (rc={rc}): {err}")
        return None

    matches = [s for s in out.splitlines() if name_fragment in s]
    if not matches:
        if required:
            print(f"  FAIL: no service matching '{name_fragment}' found.")
            return False
        print(f"  SKIP: no service matching '{name_fragment}' currently advertised "
              f"(requires an RBX driver attached) -- not a failure.")
        return None

    all_ok = True
    for svc in matches:
        rc, type_out, err = run_remote(f"rosservice type {svc}")
        if rc != 0:
            print(f"  FAIL: could not get type for {svc}: {err}")
            all_ok = False
            continue
        if type_out != expected_type:
            print(f"  FAIL: {svc} has type '{type_out}', expected '{expected_type}'")
            all_ok = False
        else:
            print(f"  PASS: {svc} is {expected_type}")
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

    print("\nChecking for app_fake_gps/enable (std_msgs/Bool, required -- "
          "standalone app, runs regardless of attached hardware)...")
    fake_gps_ok = check_topic_type("app_fake_gps/enable", "std_msgs/Bool", required=True)

    print("\nChecking for snapshot_trigger (std_msgs/Empty, required -- "
          "system-wide snapshot_all mechanism)...")
    snapshot_ok = check_topic_type("snapshot_trigger", "std_msgs/Empty", required=True)

    print("\nChecking for rbx/capabilities_query (nepi_interfaces/RBXCapabilitiesQuery, "
          "optional -- requires an RBX driver attached)...")
    caps_ok = check_service_type(
        "rbx/capabilities_query", "nepi_interfaces/RBXCapabilitiesQuery", required=False)

    print("\nChecking for rbx/info (nepi_interfaces/DeviceRBXInfo, optional)...")
    info_ok = check_topic_type("rbx/info", "nepi_interfaces/DeviceRBXInfo", required=False)

    print("\nChecking for rbx/status (nepi_interfaces/DeviceRBXStatus, optional)...")
    status_ok = check_topic_type("rbx/status", "nepi_interfaces/DeviceRBXStatus", required=False)

    print("\nChecking for rbx/settings/status (nepi_interfaces/SettingsStatus, "
          "optional -- confirms settings moved under the rbx/settings/ "
          "sub-namespace)...")
    settings_ok = check_topic_type(
        "rbx/settings/status", "nepi_interfaces/SettingsStatus", required=False)

    print("\nChecking for rbx/goto_location (nepi_interfaces/GotoLocation, optional)...")
    goto_ok = check_topic_type("rbx/goto_location", "nepi_interfaces/GotoLocation", required=False)

    print("\nChecking for rbx/set_goto_timeout (std_msgs/UInt32, optional -- "
          "renamed this session from rbx/set_cmd_timeout)...")
    timeout_ok = check_topic_type("rbx/set_goto_timeout", "std_msgs/UInt32", required=False)

    print("\nConfirming the OLD rbx/set_cmd_timeout name is gone (should be SKIP/absent)...")
    old_timeout_ok = check_topic_type("rbx/set_cmd_timeout", "std_msgs/UInt32", required=False)
    if old_timeout_ok is not None:
        print("  NOTE: an rbx/set_cmd_timeout topic was found -- unexpected; the "
              "current driver should only expose rbx/set_goto_timeout.")

    results = [fake_gps_ok, snapshot_ok, caps_ok, info_ok, status_ok, settings_ok, goto_ok, timeout_ok]
    if any(r is False for r in results):
        print("\nRESULT: FAIL -- one or more checked topics/services were missing "
              "(when required) or had an unexpected type.")
        return 1

    print("\nRESULT: PASS (or SKIP where no RBX driver was attached).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
