# RESOLVED: `nepi_app_sim_connector` "hang" was a logging artifact, not a real hang

**Status:** Closed. There is no functional hang. The node starts up correctly every time.
**Found:** 2026-08-06. **Root-caused:** 2026-08-07.

## TL;DR

`nepi_app_sim_connector` never actually hangs. It was always finishing its full startup
(NavPoseIF included) and reaching `nepi_sdk.spin()` normally. What looked like a permanent
hang was **stdout buffering**, combined with test scripts that `kill -9`'d the process the
moment it seemed stuck — which discarded the still-buffered-but-real log lines before they
ever reached disk. The process was healthy the whole time; only the *evidence* of that health
(the log file) was unreliable.

If you're testing any NEPI ROS node by redirecting its output to a file
(`rosrun ... > file.log 2>&1 &`), read "How to test NEPI nodes correctly" below — this bites
any node, not just this one.

## Root cause

ROS's console log handler, `RosStreamHandler` in
`/opt/ros/noetic/lib/python3/dist-packages/rosgraph/roslogging.py`, writes log lines like this
(`emit()` → `_write()`):

```python
def _write(self, fd, msg, color):
    if self._colorize and color and hasattr(fd, 'isatty') and fd.isatty():
        msg = color + msg + _color_reset
    fd.write(msg)
```

There is **no `.flush()` call anywhere in this path**. This is stock ROS Noetic behavior, not
a NEPI bug.

- When stdout is a terminal (TTY), the C stdio layer is **line-buffered**, so each `\n`
  effectively flushes immediately — you see output right away when running interactively.
- When stdout is redirected to a regular file (as in `> /tmp/foo.log 2>&1 &`, exactly what
  every test command in this investigation used), Python switches to **full block buffering**
  (several KB). Log lines accumulate in that buffer and are only pushed to disk once the
  buffer fills, or the process exits cleanly (interpreter shutdown flushes streams).

Every time this investigation's test harness decided the node was "hung" (because the log
hadn't shown "Sim connector listening..." within some timeout) and ran `kill -9` on it, that
`kill -9` bypassed normal interpreter shutdown — so whatever was sitting in the stdout buffer,
including possibly the very lines proving the node had finished starting, was silently lost.
The log file's *apparent* stopping point was just wherever the buffer happened to be, not
where execution actually stopped.

## How this was confirmed

With a still-running "stuck" process (PID 325933, log stalled at
`NavPoseIF: ... Starting Node IF Initialization Processes` for 100+ seconds):

1. A live Python thread-stack dump (via `sys._current_frames()`, a stdlib substitute for
   `py-spy`/`gdb`, neither of which is available on-device) showed `MainThread` already
   sitting in `nepi_sdk.spin()` → `rospy.spin()` → `wallsleep(0.5)` — the very last statement
   in `NepiSimConnectorApp.__init__()`, reached only *after* both
   `self.msg_if.pub_info("Sim connector listening on 0.0.0.0:9030")` and
   `self.msg_if.pub_info("Initialization Complete")` had already executed. An exception
   anywhere earlier in `__init__` would have killed the thread, not left it happily parked in
   `spin()` — so this is unambiguous proof the entire constructor, NavPoseIF included, had
   already completed successfully.
2. Direct ROS introspection against the same live, "stuck" process confirmed it end to end:
   - `rosservice call .../app_sim_connector/sim/device_info_query` and the status service
     responded normally.
   - `rostopic list` showed `.../app_sim_connector/npx/navpose`,
     `.../npx/navpose/status`, `.../npx/navpose_frame_transform/*` — proof `NavPoseIF` fully
     registered.
   - `rostopic echo .../npx/navpose -n1` returned a real, well-formed `NavPose` message.
   - `ss -tlnp` showed the sim bridge TCP socket genuinely `LISTEN`ing on `0.0.0.0:9030`.
3. The log file, checked again several seconds later with the process still alive and
   unchanged, still had **not** gained the "Sim connector listening" line — confirming the
   gap wasn't a delayed flush, but a permanently lost write (later killed without ever being
   flushed in prior runs; in this particular check the process was simply left running past
   where any test harness would normally have given up and killed it).

## What was fixed vs. what wasn't a real bug

- **No fix needed in `nepi_app_sim_connector` or `NPXDeviceIF`/`NavPoseIF`/`NodeServicesIF`.**
  All of that code was working correctly the entire time. The original theory in this doc's
  first version (that `NodeServicesIF`/`nepi_sdk.create_service()` was hanging while
  registering the NavPose `capabilities_query` service) is **disproven** — it was never
  actually stuck there; the log just never caught up.
- **Kept as a real, worthwhile improvement (independent of this investigation):**
  `nepi_app_sim_connector/api/messages_if.py` overrides the shared `nepi_api.messages_if`
  module (via this app's `CMakeLists.txt` `api/` → `nepi_api` install rule) to fix
  `MsgIF.updaterCb()`, which called `nepi_system.get_debug_mode()` —
  `nepi_sdk.wait_for_param(..., timeout=1000)`, a genuinely blocking poll for up to 1000
  seconds if the device-wide `debug_mode` param is never set. That's a real anti-pattern for
  a 1 Hz self-rescheduling timer (it should just check non-blockingly and try again next
  tick), and every `MsgIF` instance in the process starts one of these timers — a
  deeply-nested construction like this app's can spin up a dozen+ of them. This fix reduces
  real thread pile-up risk under a slow/missing `config_mgr`; it just turned out **not** to
  be the cause of the "hang" symptom, since that symptom didn't exist.
- **Known, same-shaped, not fixed:** `nepi_api/system_if.py`'s `SaveDataIF.updaterCb()` has
  the identical anti-pattern calling `nepi_system.get_timezone()` (also
  `wait_for_param(..., timeout=1000)`). Left alone for now — fixing it would mean overriding
  the entire (large, multi-class) `system_if.py` via this app's `api/` mechanism for one
  line, which isn't worth the maintenance overhead unless it's ever shown to cause real
  problems. Worth another look if a similar investigation turns up thread pile-up again.
- **Removed:** the temporary `_watchdogDumpStacks()` diagnostic thread that was added to
  `sim_connector_app_node.py` for this investigation (stdlib-only stand-in for `py-spy`/`gdb`,
  neither available on-device). No longer needed now that the mystery is resolved.
- **Unrelated, already fixed, still valid:** importing `cv2` before `nepi_api.device_if_sim`
  (which transitively imports `open3d` via `NPXDeviceIF` → `nepi_api.system_if`) crashes on
  this device's aarch64 build with `ImportError: libgomp.so.1: cannot allocate memory in
  static TLS block`. Fixed by import reordering in `sim_connector_app_node.py`. This is a
  real, separate, 100%-reproducible-before-the-fix bug and is unrelated to the logging issue
  above.

## How to test NEPI nodes correctly (the actual lesson here)

Because `RosStreamHandler` never flushes, **any** NEPI ROS node's console output becomes
unreliable the moment you redirect it to a file for testing, and outright lossy if you ever
`kill -9` it instead of letting it shut down cleanly. To test a node this way without chasing
phantom hangs:

- Run it unbuffered: `python3 -u sim_connector_app_node.py ...` (or launch via
  `rosrun`/apps_mgr with `PYTHONUNBUFFERED=1` in the environment), so every `write()` reaches
  the file immediately.
- Prefer checking actual ROS state (`rosservice call .../status_query`, `rostopic echo`,
  `rosnode info`, `ss -tlnp` for this app's bridge socket) over grepping a redirected log for
  a specific "ready" string — the log can lag or lose lines even when unbuffered output isn't
  the culprit (e.g. slow disk flush timing).
- If you do kill a test process, send `SIGTERM` (plain `kill`, not `kill -9`) so Python's
  normal interpreter shutdown gets a chance to flush stdio buffers, and give it a moment
  before concluding anything from the log.

## Verification

With the process still live (no restart needed — it had been running correctly the entire
time), confirmed via direct ROS calls: `sim/device_info_query`, `npx/navpose`,
`npx/navpose/status`, and `npx/navpose_frame_transform/*` topics/services all present and
responding; sim bridge TCP listener up on `0.0.0.0:9030`. A follow-up clean run with
`python3 -u` is recommended (see above) to visually confirm the log now streams
"Sim connector listening on 0.0.0.0:9030" promptly instead of appearing to stall, but this is
a testing-methodology confirmation, not a functional one — functionally, nothing was ever
broken.

## Addendum 2026-08-07: the same `wait_for_param` pattern genuinely blocks in a bare/mock test
environment — a real, separate gotcha from the buffering issue above, found independently
while doing Phase 0 of `MULTI_SIMULATOR_INTEGRATION_PLAN.md`

This does **not** contradict the resolution above for a real device — it's a different test
environment surfacing a related, real behavior of the same underlying code path, worth
recording so it doesn't get mistaken for a recurrence of the "hang."

**Setup:** a from-scratch isolated `roscore` on a throwaway port, nothing else registered —
deliberately no `config_mgr`, no `apps_mgr`, none of the real device's usual node population.
Running `test_device_if_sim_harness.py rover` (which does supply a real, non-`None`
`getNavPoseCb`, exercising the same `NPXDeviceIF`/`NavPoseIF` construction path as the real app)
against this bare roscore genuinely never completed — not a buffering illusion this time. A
live stack-dump watchdog (`sys._current_frames()`, same stdlib technique used above) caught the
exact same call, on every one of 15+ dumps spanning 100+ seconds, sitting at:

```
nepi_api/system_if.py:1670, in __init__
  user_folders = nepi_system.get_user_folders(...)
nepi_sdk/nepi_system.py:115, in get_user_folders
  data = nepi_sdk.wait_for_param(param_namespace, timeout=timeout, ...)
nepi_sdk/nepi_sdk.py:655, in wait_for_param
  time.sleep(1)
```

`get_user_folders(timeout=1000)` waits on `<base_namespace>/user_folders`, a param nothing in
this bare environment ever sets — this is the same anti-pattern already identified and fixed
for `debug_mode` in this app's own `messages_if.py` override (see above), just hit at a
different call site (`SaveDataIF.__init__` reading `user_folders`, not `MsgIF.updaterCb` reading
`debug_mode`), and this one is a **one-shot blocking call during construction**, not a
background timer — so unlike the `debug_mode` case, this one really does stall the whole
constructor, not just leak a stray thread.

**Confirmed the fix:** pre-seeding both params directly (`rosparam set .../debug_mode false` and
`rosparam set .../user_folders "{data: /tmp/nepi_test_data}"`) before launching let the harness
complete its *entire* startup in under 30 seconds, `NavPoseIF`'s `NodeServicesIF` registration
included — consistent with the finding above that nothing is actually broken in
`NodeServicesIF`/`NavPoseIF` itself.

**Why this matters for standalone bridge testing specifically:** a real device's `config_mgr`
provisions `user_folders` (and presumably other `nepi_system.get_*`-style params) for a
namespace before any app node using them gets launched, so `wait_for_param`'s first 1-second
poll succeeds and this is invisible there — matching the "no real hang" finding above. A bare
test roscore with no `config_mgr` has nobody to ever set that param, so the same call
legitimately blocks for the full 1000-second timeout. Every phase of
`MULTI_SIMULATOR_INTEGRATION_PLAN.md` develops bridges against exactly this kind of minimal
test environment before deploying to a real device, so: **when standing up a bare test roscore
for bridge development, pre-seed at least `<namespace>/debug_mode` and
`<namespace>/user_folders`** (see the two `rosparam set` commands above) before launching
anything that constructs a `SimDeviceIF`/`NPXDeviceIF`, or budget for a ~15-16 minute wait
instead (`user_folders` alone is up to 1000s = ~16.7 min, and it's a hard 1000s ceiling; if
`config_mgr` never provisions it, this is unblocking, not slow). Worth `nepi_api` core
considering a non-blocking default (matching the `debug_mode` fix's shape) at every
`nepi_sdk.wait_for_param`-based `nepi_system.get_*` call site outside `config_mgr`'s own
bootstrap, but that's a nice-to-have here, not a blocker — the workaround above is sufficient
for this plan's bridge-development testing.
