# How NEPI Driver Discovery Works (ArduPilot RBX)

Companion to `SITL_IMPLEMENTATION_PLAN.md`. This explains the mechanism that
finds a flight controller, launches `mavros`, and launches the RBX node — both
the generic NEPI drivers framework and the ArduPilot-specific discovery class.
Read this to understand *why* the SITL change (a new `connection` branch) is all
that's needed, and to debug discovery when a device does not appear.

All paths are under `/home/production/nepi_engine_ws/src/`.

---

## 1. Mental model

"Discovery" is NEPI's answer to a simple question asked repeatedly: **"Is there
a device I should be running a driver for, and is the driver I already started
still healthy?"** It is a polling loop, not an event system. Roughly once per
second the drivers manager asks every enabled driver to look for its hardware;
each driver either launches what's missing or reaps what has died.

Every NEPI driver is **three files** (the "3-file pattern"):

| File | Role |
|---|---|
| `*_params.yaml` | Static descriptor: package name, node/discovery class names, and the user-facing **OPTIONS** shown in the RUI |
| `*_discovery.py` | The discovery logic: probe for the device, launch the transport + device node, monitor/teardown |
| `*_node.py` | The actual device node that does the work once connected |

For ArduPilot those are `rbx_ardupilot_params.yaml`,
`rbx_ardupilot_discovery.py`, and `rbx_ardupilot_node.py`.

There are **two ways** a discovery can run, declared by
`DISCOVERY_DICT.process` in the params YAML:

- **`CALL`** (what ArduPilot uses): the manager imports the discovery class into
  its own process and **calls a method on it** every tick. State lives in the
  class instance the manager holds.
- **`LAUNCH`**: the manager runs the discovery file as its **own ROS node**.
  (Not used here; mentioned so you recognize the branch in `drivers_mgr.py`.)

---

## 2. The framework loop (drivers manager)

File: `nepi_engine/nepi_managers/scripts/drivers_mgr.py`.

### 2.1 The heartbeat

At startup the manager arms a self-rescheduling one-shot timer:

```python
nepi_sdk.start_timer_process(1.0, self.updaterDriversCb, oneshot=True)   # line 364
```

`updaterDriversCb` (line 519) does one full pass and, at the end, reschedules
itself one second later (line 790). So discovery runs **~1 Hz**, paced so each
pass finishes before the next is queued (a slow probe just slows the cadence; it
never overlaps itself).

### 2.2 What one pass does

Each pass (starting line 519):

1. **Enumerate device paths.** `available_paths_list = self.getAvailableDevPaths()`
   (line 547) which is literally `glob.glob('/dev/*')` (line 1301). This is the
   "does the port still physically exist" reference list — note it is **not** the
   serial-scan list (see the nuance in 3.2).
2. **Purge disabled drivers.** Any driver that is no longer in the active list
   but still has a running discovery gets torn down. For `CALL` drivers this
   calls the class's `killAllDevices(...)` and then **`del sys.modules[module]`**
   (lines 587-610) — this is how a code edit to the discovery script gets
   reloaded: disable the driver (module deleted) then re-enable it (re-imported).
3. **Run discovery for each active driver** (loop at line 619):
   - **First time seen** (`CALL`): import the discovery file and instantiate the
     class once, caching it in `self.discovery_classes_dict[driver_name]`
     (lines 685-694). The instance is created **once** and reused — which is why
     the class can hold state across passes (see §5).
   - **Every subsequent pass**: call the cached instance's `discoveryFunction`
     (line 710, the single most important line in the framework):
     ```python
     self.active_paths_list = discovery_class.discoveryFunction(
         available_paths_list, self.active_paths_list, self.base_namespace, drv_dict, retry)
     ```
   - If `discoveryFunction` raises, the manager disables the driver and deletes
     its module (lines 714-724) — a hard failure takes the driver offline rather
     than spamming exceptions.
4. **Publish the options interface** so the RUI can read/change OPTIONS
   (`createDriverOptionsIf`, line 733).

### 2.3 The argument contract of `discoveryFunction`

```python
def discoveryFunction(self, available_paths_list, active_paths_list, base_namespace, drv_dict, retry_enabled):
```

| Arg | Meaning | Who owns it |
|---|---|---|
| `available_paths_list` | Every `/dev/*` path right now | Manager (fresh each pass) |
| `active_paths_list` | Paths this driver has already claimed and launched | **Manager holds it; discovery returns the updated copy** |
| `base_namespace` | e.g. `/nepi/device1` | Manager |
| `drv_dict` | The parsed params YAML **plus** injected runtime keys (`path`, `node_name`, `retry_enabled`, `user_cfg_path`, and later `DEVICE_DICT`) | Manager builds; discovery reads OPTIONS and adds `DEVICE_DICT` |
| `retry_enabled` | Global "keep retrying failed launches?" flag | Manager (`self.retry_enabled`) |

The critical design point: **`active_paths_list` is threaded in and out**. The
manager passes the current list in, the discovery mutates it (appends newly
launched paths, removes dead ones), and the return value becomes the manager's
new list. That single shared list is how the manager and the driver agree on
"what is already running" without a database.

### 2.4 How OPTIONS get changed (RUI -> discovery)

The `connection` and `fake_gps` values in `drv_dict['DISCOVERY_DICT']['OPTIONS']`
are what steer discovery. They are changed at runtime, not just from the YAML:

- The RUI publishes a `Setting` message; `updateSettingCb` (line 883) routes it
  to `updateSetting` (line 900).
- `updateSetting` builds a caps dict from the option's declared **`type`** and
  **`options`** list and calls `nepi_settings.check_valid_setting(...)`. **A new
  value is rejected unless it is one of the declared `options`.** This is exactly
  why the SITL plan must add `SITL` to `connection.options` in the YAML — without
  it, selecting SITL fails validation and silently reverts to the default.
- On a valid change it writes
  `drvs_dict[driver_name]['DISCOVERY_DICT']['OPTIONS'][name]['value']` and calls
  `save_config()` (persists across reboots). The next `discoveryFunction` pass
  reads the new value at the top and branches accordingly.

---

## 3. Inside `ArdupilotDiscovery.discoveryFunction`

File: `rbx_drivers/rbx_ardupilot_discovery.py` (class `ArdupilotDiscovery`,
method at line 77). One pass does four things in order:

### 3.1 Read options (lines 84-96)
```python
connection_type = drv_dict['DISCOVERY_DICT']['OPTIONS']['connection']['value']   # SERIAL / TCP / UDP / (SITL)
fake_gps_val    = drv_dict['DISCOVERY_DICT']['OPTIONS']['fake_gps']['value']
self.enable_fake_gps = (str(fake_gps_val).lower() == 'true')
self.retry = retry_enabled
if self.retry:                 # when retry is on, forget past failures and re-probe everything
    self.dont_retry_list = []
```

### 3.2 Purge dead connections (lines 99-109)
For every path this instance believes is active, call `checkOnDevice(path)`; if
it is no longer healthy, drop it from `active_devices_dict` and from
`active_paths_list`. This is the "reap what died" half of the loop — it is why
unplugging the FCU makes the driver clean itself up within a second or two.

`checkOnDevice` (line 163) declares a device dead if either:
- its `mavros` subprocess exited (`mavlink_subproc.poll() is not None`), or
- for a serial path, the port vanished
  (`path_str.startswith('/dev/') and path_str not in self.available_paths_list`).

> **Nuance — two different "lists":** the SERIAL scan enumerates candidate ports
> with `nepi_serial.get_serial_ports_list()` (pyserial `comports()` plus globs of
> `/dev/ttyTHS*` and `/dev/ttyTCU*`). But **liveness** is checked against
> `available_paths_list` (all of `/dev/*`). They usually agree for USB serial,
> but they are computed differently — worth knowing when a port is present in one
> and not the other.

### 3.3 Branch on connection type (lines 113-155)
```python
if connection_type == 'SERIAL':        # scan serial ports, probe for heartbeat
    ...
elif connection_type == 'TCP' or connection_type == 'UDP':
    ...                                 # iterate ip/port lists, probe, launch
# (SITL branch would be added here per the SITL plan)
```

**SERIAL path in detail** (lines 113-130), the production path today:
```python
self.path_list = nepi_serial.get_serial_ports_list()
for path_str in self.path_list:
    valid_path = True
    if path_str in self.active_paths_list or path_str in self.dont_retry_list:
        valid_path = False                       # already running, or known-bad and retry off
    if valid_path:
        for exclude_device in self.excludedDevices:   # excludedDevices = ['ttyACM']
            if path_str.find(exclude_device) != -1:
                valid_path = False
    if valid_path:
        [found_device, path_str, comp_id, sys_id, baud_str] = self.checkForSerialDevice(path_str)
        if found_device:
            success = self.launchSerialDeviceNode(path_str, comp_id, sys_id, baud_str)
            if success:
                self.active_paths_list.append(path_str)
```
So each pass: skip ports already claimed / on the don't-retry list / matching an
excluded prefix, probe the rest for a real MAVLink heartbeat, and launch on a
hit. `ttyACM` is excluded because that is typically the Pixhawk's USB-console
interface, not the telemetry serial link.

### 3.4 Return the updated active list (line 157)
`return self.active_paths_list` — handed straight back to the manager (§2.3).

---

## 4. The probe and the launch

### 4.1 `checkForSerialDevice` — sniffing a MAVLink heartbeat (lines 298-355)
This is the part that decides "is a flight controller actually on this port."
For each baud in `baudrate_list` (only `['57600']` today) it:
1. Opens the serial port with pyserial (1 s timeout).
2. Reads up to 500 packets, scanning for the **MAVLink 2** start magic `0xFD`,
   then reads the 9-byte header.
3. Identifies a **HEARTBEAT** by: packet length 9, message id `0x000000`, and a
   plausible system id (`0 < sys_id < 240`).
4. On a hit, records `mav_sys_id` / `mav_comp_id` and returns `found_device=True`.

It is deliberately protocol-level (not a full MAVLink parser) so it can cheaply
confirm a live autopilot without a heavyweight dependency. **For TCP/UDP/SITL
there is no heartbeat sniff** — `checkForTcpDevice` (line 370) does a real socket
`connect_ex` (so it only "finds" the device if the port is open), while
`checkForUdpDevice` (line 400) always returns True (connectionless). This
difference is the reason the SITL plan prefers the TCP-based probe: it gives
correct start-ordering for free.

### 4.2 `launchSerialDeviceNode` -> `launchDeviceNode` (lines 358-365, 212-293)
`launchSerialDeviceNode` just builds the connection string and delegates:
```python
device_id_str = path_str.split('/')[-1]   # '/dev/ttyUSB0' -> 'ttyUSB0'
fcu_url = path_str + ':' + baud_str        # '/dev/ttyUSB0:57600'
gcs_url = ""
self.launchDeviceNode(path_str, device_id_str, mav_comp_id, mav_sys_id, fcu_url, gcs_url)
```

`launchDeviceNode` is the shared workhorse for **all** connection types. In order:

1. **Relaunch backoff** (lines 216-223): if this path was launched less than
   `NODE_LOAD_TIME_SEC` (10 s) ago, skip. Prevents a crash-loop from respawning
   `mavros` many times per second.
2. **Name the mavros node** (line 226): `mav_node_name = "mavlink_" + device_id_str`
   -> `mavlink_ttyUSB0`. (For SITL we set `device_id_str = "sitl"` -> `mavlink_sitl`.)
3. **Load the APM configs** into the mavros node's private namespace
   (lines 230-231): `apm_pluginlists.yaml` + `apm_config.yaml` from
   `/opt/nepi/nepi_engine/share/mavros/launch/`. Then it lowers `timesync_rate`
   to cut log noise and un-blacklists the `hil` plugin (lines 233-238).
4. **Launch mavros** (lines 242-245) as a subprocess with `_fcu_url:=<...>`. This
   is the only place the connection string reaches mavros.
5. **Build `DEVICE_DICT` and hand it to the node** (lines 256-267):
   ```python
   self.drv_dict['DEVICE_DICT'] = {
       'device_name': ardu_device_name, 'device_path': path_str,
       'mavlink_node_name': mav_node_name, 'fcu_url': fcu_url, 'gcs_url': gcs_url,
       'mav_sys_id': mav_sys_id, 'mav_comp_id': mav_comp_id,
       'fake_gps': self.enable_fake_gps }
   nepi_sdk.set_param(<ardu_node>/drv_dict, self.drv_dict)
   ```
   **This param is the entire contract between discovery and the RBX node.** The
   node reads it at startup (`rbx_ardupilot_node.py` lines 185-201) to learn which
   mavros namespace to attach to and whether fake GPS is active. This is why the
   node is connection-agnostic and needs no change for SITL.
6. **Launch the RBX node** (line 270): `nepi_drvs.launchDriverNode(file_name, ardu_node_name)`.
7. **Record or roll back** (lines 274-293): on success, store a `device_entry`
   with all three subprocess handles (mavlink / ardupilot / fake_gps) in
   `active_devices_dict`. On failure, **kill the mavros process it just started**
   (so a half-up device doesn't linger) and, if retry is off, add the path to
   `dont_retry_list`.

---

## 5. State and lifecycle

Because the manager keeps **one** `ArdupilotDiscovery` instance, these
class/instance attributes persist across passes and carry the loop's memory:

| State | Purpose |
|---|---|
| `active_devices_dict` | path -> `{mavlink_subproc, ardu_subproc, fgps_subproc, node names, sys/comp id}`. The authoritative record of what is running. |
| `active_paths_list` | (Held by the manager, threaded in/out.) Paths already claimed — skipped by the scan so a device isn't launched twice. |
| `launch_time_dict` | path -> last launch time, for the 10 s relaunch backoff. |
| `dont_retry_list` | Paths whose launch failed while retry was off; skipped until retry is re-enabled (which clears the list, §3.1). |
| `enable_fake_gps` | Set from the option each pass; copied into `DEVICE_DICT`. |

**Teardown paths:**
- Per-device, automatically: `checkOnDevice` -> `killDeviceProcesses`
  (lines 192-209) kills the ardupilot, fake_gps, and mavlink subprocesses in that
  order when a device goes unhealthy.
- Whole-driver, on disable: the manager calls `killAllDevices(active_paths_list)`
  (line 418) which purges every `device_entry`, then deletes the discovery module
  so a re-enable re-imports fresh code.

### Lifecycle timeline (SERIAL, happy path)
```
  t0   manager instantiates ArdupilotDiscovery once
  t0+  pass 1: read options -> SERIAL; scan ports; /dev/ttyUSB0 not yet claimed
                 checkForSerialDevice -> heartbeat found (sys_id=1)
                 launchDeviceNode -> mavros(mavlink_ttyUSB0) + rbx node(ardupilot_ttyUSB0)
                 append /dev/ttyUSB0 to active_paths_list
  t0+1 pass 2: purge -> mavros alive, port present -> keep
                 scan -> /dev/ttyUSB0 already in active_paths_list -> skip
  ...  steady state: each pass just confirms liveness
  tX   FCU unplugged: pass -> checkOnDevice sees port gone -> killDeviceProcesses
                 remove /dev/ttyUSB0 from active list; next pass is free to rediscover
```

---

## 6. Where each connection type diverges (map for the SITL work)

| Concern | SERIAL | TCP | UDP | SITL (proposed) |
|---|---|---|---|---|
| Candidate enumeration | `get_serial_ports_list()` | `ip_addr_list` x `ip_tcp_port_list` | `ip_addr_list` x `ip_udp_port_list` | `sitl_addr_list` x `sitl_tcp_port_list` |
| Reachability probe | MAVLink heartbeat sniff (`checkForSerialDevice`) | real `connect_ex` (`checkForTcpDevice`) | none, always True (`checkForUdpDevice`) | reuse `checkForTcpDevice` |
| `fcu_url` built in | `launchSerialDeviceNode` (`path:baud`) | `launchTcpDeviceNode` (`tcp://ip:port`) | `launchUdpDeviceNode` (hardcoded) | `launchSitlDeviceNode` (`tcp://127.0.0.1:5760`) |
| Node id (`device_id_str`) | port basename (`ttyUSB0`) | IP+port flattened (`127001_5760`) | IP+port flattened | `sitl` -> `mavlink_sitl` |
| Everything after | `launchDeviceNode` (shared) | same | same | same |

The table makes the SITL plan's claim concrete: only the left three rows differ
per connection type, and the entire bottom row (`launchDeviceNode` + the RBX
node) is shared and unchanged.

---

## 7. Debugging discovery (quick reference)

Run inside the container (name changes each restart):
```bash
C=$(docker ps --format '{{.Names}}' | head -1)
RX() { docker exec -i "$C" bash -c "export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=127.0.0.1; source /opt/ros/noetic/setup.bash; $1"; }
```

- **Is the driver even active / what does discovery think?** watch the drivers
  manager log — it prints "Calling discovery function", "Found mavlink ... device
  at ...", "Launching mavlink node", and warnings on failure.
- **No device found on serial:** confirm the port appears in
  `get_serial_ports_list()` (USB serial or `ttyTHS*`/`ttyTCU*`), that it is not a
  `ttyACM*` (excluded), that the baud is `57600`, and that the FCU is emitting a
  MAVLink 2 heartbeat. The probe opens the port exclusively, so nothing else
  (a stray `mavproxy`) may hold it.
- **Device found but keeps relaunching:** the 10 s `launch_time_dict` backoff and
  the `dont_retry_list` (when `retry_enabled` is false) govern this; check the
  manager's `retry_enabled` param.
- **Option won't change from the RUI:** the new value must be in the option's
  `options` list (§2.4) or `check_valid_setting` rejects it. This is the classic
  "I selected SITL but it reverted" symptom -> add `SITL` to `connection.options`.
- **Code edit not taking effect:** the `CALL` discovery class is imported once and
  cached; disable then re-enable the driver (or restart the drivers manager) so
  the module is deleted and re-imported.

---

## 8. Reference file/line index

- Framework loop: `nepi_managers/scripts/drivers_mgr.py`
  - timer arm/reschedule: lines 364, 790
  - one pass: `updaterDriversCb` line 519
  - device path list: `getAvailableDevPaths` line 1301 (`glob('/dev/*')`)
  - discovery call: line 710
  - option validation/persist: `updateSetting` lines 900-935
  - disable/reload (`del sys.modules`): lines 587-610
- ArduPilot discovery: `rbx_drivers/rbx_ardupilot_discovery.py`
  - `discoveryFunction` line 77; SERIAL branch 113-130; TCP/UDP 132-155
  - `checkOnDevice` 163; `killDeviceProcesses` 192; `launchDeviceNode` 212
  - `checkForSerialDevice` 298; `launchSerialDeviceNode` 358
  - `checkForTcpDevice` 370; `launchTcpDeviceNode` 386
  - `checkForUdpDevice` 400; `launchUdpDeviceNode` 406; `killAllDevices` 418
  - class-level state/defaults: lines 42-62
- RBX node consumes `DEVICE_DICT`: `rbx_drivers/rbx_ardupilot_node.py` lines 185-201
- Serial enumeration: `nepi_sdk/src/nepi_sdk/nepi_serial.py` `get_serial_ports_list` line 56
- Params/OPTIONS: `rbx_drivers/rbx_ardupilot_params.yaml`
