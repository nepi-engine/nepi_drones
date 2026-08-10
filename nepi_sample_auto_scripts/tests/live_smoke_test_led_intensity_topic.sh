#!/usr/bin/env bash
#
# MANUAL / INTEGRATION SMOKE TEST -- NOT part of the automated test suite.
#
# This is intentionally named so pytest's default discovery (test_*.py /
# *_test.py) never collects it, and it is not invoked by any CI or `pytest`
# run. Execute it by hand against the real NEPI device when you want to
# confirm the live topic surface led_step_adjust_process_script.py depends
# on still matches what this session's mock-stub unit test assumes.
#
# Unlike navpose_mgr / ai_models_mgr / drivers_mgr status topics, the LED
# intensity topic this script waits on (lsx/set_intensity) is published by
# a per-device LSX driver, which only exists if LSX-capable light hardware
# is actually attached and configured on the device. So a "not found" result
# here is expected and NOT a failure if no LED/light hardware is attached --
# it just means this particular live check can't run further; it does not
# indicate a broken script. If any lsx/* topics exist, this checks that
# set_intensity carries the expected std_msgs/Float32 type.
#
# Usage:
#   bash live_smoke_test_led_intensity_topic.sh
#
# Requires: ssh access configured exactly as used elsewhere this session:
#   ssh -p 2222 -i ~/.ssh/nepi_default_ssh_key nepi@nepi
#
# NOTE: this device was NOT reachable from the sandbox this test was
# authored in (ssh connect timed out on port 2222) -- this script has been
# written to the confirmed current API/topic naming but has not actually
# been executed against a live device this session. Run it by hand once the
# device is reachable to close that gap.

set -u

SSH_HOST="nepi@nepi"
SSH_PORT=2222
SSH_KEY="${HOME}/.ssh/nepi_default_ssh_key"
REMOTE_SETUP="source /opt/nepi/nepi_engine/setup.bash"

ssh_cmd() {
  ssh -p "${SSH_PORT}" -i "${SSH_KEY}" -o ConnectTimeout=5 "${SSH_HOST}" \
    "${REMOTE_SETUP}; $1"
}

echo "== 1. Checking device reachability =="
if ! ssh_cmd "echo reachable" >/tmp/live_smoke_reach.$$ 2>&1; then
  echo "FAIL: device not reachable via ssh -p ${SSH_PORT} ${SSH_HOST}"
  cat /tmp/live_smoke_reach.$$
  rm -f /tmp/live_smoke_reach.$$
  exit 1
fi
rm -f /tmp/live_smoke_reach.$$
echo "OK: device reachable"

echo
echo "== 2. Listing lsx/* topics under base namespace (/nepi/device1/) =="
LSX_TOPICS="$(ssh_cmd "rostopic list 2>/dev/null | grep -E 'lsx/' || true")"
if [ -z "${LSX_TOPICS}" ]; then
  echo "INFO: no lsx/* topics found. This is expected if no LSX-capable"
  echo "      light hardware/driver is currently attached/configured on"
  echo "      this device. Nothing further to check -- not a failure of"
  echo "      led_step_adjust_process_script.py itself."
  exit 0
fi
echo "${LSX_TOPICS}"

echo
echo "== 3. Locating a set_intensity topic among lsx/* topics =="
INTENSITY_TOPIC="$(echo "${LSX_TOPICS}" | grep 'set_intensity' | head -n1)"
if [ -z "${INTENSITY_TOPIC}" ]; then
  echo "WARN: lsx/* topics exist but none matched 'set_intensity'."
  echo "      led_step_adjust_process_script.py's LED_CONTROL_TOPIC_NAME"
  echo "      ('lsx/set_intensity') may need to be re-checked against the"
  echo "      attached driver's actual topic name."
  exit 1
fi
echo "Found: ${INTENSITY_TOPIC}"

echo
echo "== 4. Confirming topic type is std_msgs/Float32 =="
TOPIC_TYPE="$(ssh_cmd "rostopic type ${INTENSITY_TOPIC} 2>/dev/null")"
echo "Type reported: ${TOPIC_TYPE}"
if [ "${TOPIC_TYPE}" != "std_msgs/Float32" ]; then
  echo "FAIL: expected std_msgs/Float32, got '${TOPIC_TYPE}'"
  exit 1
fi

echo
echo "PASS: ${INTENSITY_TOPIC} exists with type std_msgs/Float32,"
echo "      matching what led_step_adjust_process_script.py publishes."
