# nepi_drivers — Developer Reference

## Purpose

`nepi_drivers` provides hardware abstraction for all physical devices supported by NEPI. Drivers are organized by device category (camera, navigation, pan-tilt, lighting, robot) and follow a three-tier pattern: a discovery script that detects hardware and launches nodes, a node script that registers the device with its NEPI interface class and handles ROS communication, and an optional driver script that handles raw hardware I/O. The `drivers_mgr` node in `nepi_engine` calls discovery functions periodically; everything else is self-contained here.

## Architecture

```
nepi_drivers/
├── idx_drivers/        # Image/camera drivers (GenICam, V4L2, ONVIF, ZED)
├── npx_drivers/        # Navigation/positioning drivers (NMEA, HNav, Microstrain)
├── ptx_drivers/        # Pan-tilt/actuator drivers (Sidus SS109, IQR, ONVIF PTZ)
├── lsx_drivers/        # Lighting drivers (Deepsea Sealite, Sidus SS182, AfTower)
├── rbx_drivers/        # Robot drivers (ArduPilot autopilot via mavros)
├── scripts/
│   └── fake_gps_node.py   # Legacy per-device GPS simulator (superseded by the nepi_app_fake_gps app)
├── CMakeLists.txt
├── package.xml
└── setup.py
```

Each driver category directory holds one set of files per supported device, following this naming pattern:
- `{cat}_{device}_discovery.py` — hardware detection and node lifecycle
- `{cat}_{device}_node.py` — ROS node, registers with device interface class
- `{cat}_{device}_driver.py` — (optional) raw hardware I/O abstraction
- `{cat}_{device}_params.yaml` — driver metadata and configurable options

Where `{cat}` is the three-letter category prefix (idx, npx, ptx, lsx, rbx).

## How It Works

`drivers_mgr` (in `nepi_managers`) calls each discovery function on a 1-3 second polling interval, passing:
- `available_paths_list` — serial ports, USB paths, or network endpoints found on the system
- `active_paths_list` — paths that already have a running node
- `base_namespace` — the device's ROS namespace
- `drv_dict` — the parsed YAML from the driver's params file
- `retry_enabled` — whether to attempt relaunching failed nodes

**Discovery** probes hardware (serial handshake, TCP socket test, USB enumeration), then calls `nepi_sdk.launch_node()` to start a driver node when hardware is found. The driver path and device config are passed to the node via the ROS param server at `~drv_dict`. Discovery also monitors running nodes: if a node dies or hardware disconnects, it removes it from `active_paths_list` and allows re-discovery on the next cycle.

**Nodes** call `nepi_sdk.init_node()`, read `~drv_dict` from the param server, then instantiate the appropriate device interface class from `nepi_api`:

| Category | Interface class |
|---|---|
| IDX | `IDXDeviceIF` |
| NPX | `NPXDeviceIF` |
| PTX | `PTXActuatorIF` |
| LSX | `LSXDeviceIF` |
| RBX | `RBXRobotIF` |

The interface class handles all ROS publisher/subscriber/service creation. The node provides callbacks for hardware reads and commands. Nodes run until hardware disconnects or `drivers_mgr` terminates them.

**Retry and backoff:** Discovery tracks the last launch time per device path in `launch_time_dict`. A configurable `NODE_LOAD_TIME_SEC` backoff (typically 10 seconds) prevents rapid relaunch loops. A `dont_retry_list` permanently blacklists devices that fail in a non-recoverable way.

### Simulator support

Two driver categories include built-in simulators for development without hardware:
- **NPX NMEA UDP:** A `threading.Thread` TCP server that generates NMEA sentences (GGA, RMC, VTG, HDG) at a configurable rate. Enabled via `simulate_gps: true` in the driver options.
- **NPX HNav TCU:** A packet-level simulator that builds binary HNav wire-format packets (with CRC-16) and listens on a local TCP port. Enabled via the `simulate_*` options in the YAML.

When a simulator is running, the driver node connects to `localhost` on the configured port and behaves identically to the hardware case.

## Current Driver Inventory

**IDX — Image/Camera (6 drivers):**
- `idx_genicam` — GenICam/GigE Vision cameras via Baumer Harvester library
- `idx_v4l2` — V4L2 USB cameras (scans `/dev/video*`, excludes ZED and virtual devices)
- `idx_onvif_generic` — Generic ONVIF IP cameras
- `idx_onvif_econroutecam` — EconRoute-specific ONVIF cameras
- `idx_zed` — Stereoscopic ZED cameras
- `idx_rpi_cam3` — Raspberry Pi Camera Module 3 (IMX708) via MIPI CSI-2 on Raspberry Pi 5; uses libcamera/picamera2 stack

**NPX — Navigation/Positioning (3 drivers):**
- `npx_nmea_udp` — NMEA sentences over TCP/UDP (configurable host:port, built-in simulator)
- `npx_hnav_tcu` — HNav binary protocol over TCP (built-in packet simulator)
- `npx_microstrain` — Microstrain IMU/AHRS via `/dev/microstrain*` serial

**PTX — Pan-Tilt (3 drivers):**
- `ptx_sidus_ss109_serial` — Sidus SS109 serial pan-tilt, ±175° pan / ±75° tilt, modular addressing A-Z
- `ptx_iqr` — IQR pan-tilt via `/dev/iqr_pan_tilt` serial
- `ptx_onvif_generic` — ONVIF PTZ cameras

**LSX — Lighting (3 drivers):**
- `lsx_deepsea_sealite` — Deepsea Sealite LED lights, serial, address range 1–255
- `lsx_sidus_SS182` — Sidus SS182 strobe, serial
- `lsx_aftowerlight` — AfTower tower light system

**RBX — Robots (1 driver):**
- `rbx_ardupilot` — ArduPilot autopilot systems (e.g. ArduCopter) via `mavros`. Unlike the other categories, the discovery script launches a `mavros_node` (as a `subprocess.Popen`, no `.launch` file) to carry the MAVLink link, then launches the `ArdupilotNode` RBX node, which attaches to that mavros namespace and registers with `RBXRobotIF`. The connection is selected by the `connection` discovery option: `SERIAL` (probes serial ports for a MAVLink heartbeat) is the production path; the code also implements `TCP`/`UDP` branches. See `rbx_drivers/SITL_IMPLEMENTATION_PLAN.md` (adding an ArduPilot SITL connection) and `rbx_drivers/DISCOVERY_EXPLAINED.md` (full discovery walkthrough).

## ROS Interface

Driver nodes do not publish directly to fixed topic names. All topics are determined by the device interface class (`IDXDeviceIF`, `NPXDeviceIF`, etc.) and are rooted at the device's namespace as assigned by `drivers_mgr`. The interface class follows conventions established in `nepi_api`; refer to `nepi_api` source for the exact topic and service names published per device type.

The `drv_dict` parameter (passed at `~drv_dict`) contains:
- `DEVICE_DICT` — `device_name`, `device_path`, `serial_number`, `model`
- `DISCOVERY_DICT` — discovery options (baud rate, addresses, TCP endpoint, simulator flags)
- `SAVE_DATA` — data logging configuration

## Build and Dependencies

Built as part of the `nepi_engine_ws` catkin workspace. No standalone build.

Hardware-specific runtime dependencies:

| Category | Dependency |
|---|---|
| IDX GenICam | Baumer Harvester library (`harvesters`, `genicam`), Baumer GenTL producers (`.cti` files) |
| IDX ZED | ZED SDK |
| IDX ONVIF | `requests`, XML/HTTP libraries |
| NPX Microstrain | Device present at `/dev/microstrain*` |
| PTX Sidus | Serial port present, correct baud rate |
| LSX Sealite | Serial port present, device at configured address |
| All serial | `pyserial`, `nepi_serial` SDK module |
| IDX RPi cam3 | `picamera2`, libcamera stack (`sudo apt install python3-picamera2`); Raspberry Pi 5 only |

## Naming Conventions

Follows the NEPI convention established in `nepi_api`:
- **Public methods:** `snake_case` with docstrings
- **Private/internal methods:** `_camelCase`, no docstrings
- **`Cb` suffix:** ROS callback; rename requires auditing all external call sites

Discovery function signature is standardized:
```python
def discoveryFunction(available_paths_list, active_paths_list, base_namespace, drv_dict, retry_enabled):
    ...
    return active_paths_list
```

## Driver Settings Pattern

All driver nodes expose device settings through four private methods and register
two of them with their device interface class. `idx_drivers/idx_v4l2_node.py` is
the reference implementation — copy its shape, adapted to the device's hardware.

```python
def initSettingsDict(self):        # build the settings once, at startup
def refreshSettingsDict(self):     # read live values (and bounds/options) back
def getSettingsFunction(self):     # return the current settings dict
def setSettingFunction(self, setting_name, setting_value):
```

Wire the last two into the device IF constructor:

```python
self.idx_if = IDXDeviceIF(device_info = self.device_info_dict,
                          getSettingsFunction = self.getSettingsFunction,
                          setSettingFunction = self.setSettingFunction,
                          ...)
```

All six device IF classes (`IDXDeviceIF`, `LSXDeviceIF`, `NPXDeviceIF`,
`PTXActuatorIF`, `RBXRobotIF`, `SVXActuatorIF`) accept exactly these two
arguments and build a `SettingsIF` only when both are non-`None`. A device with
no settings passes neither — that is the correct state, not an omission
(`npx_*`, `ptx_onvif_generic`, `lsx_aftowerlight`).

**Contract.** `getSettingsFunction()` takes no arguments and returns a
`nepi_controls` controls dict. `setSettingFunction(name, value)` returns
`[success, msg, settings_dict]` — all three, since `SettingsIF` replaces its own
dict with the third element. It is also called as a bare statement during
`SettingsIF.init()`, so it must tolerate its return being discarded.

**The settings dict is a controls dict.** Build the plain init dict, then hand it
to `nepi_controls.create_controls_dict()`. Each init entry is keyed by the
setting name and carries:

| key | applies to |
|---|---|
| `type` | required; must be in `nepi_controls.CONTROL_TYPES` |
| `default` | required; typed to match `type` |
| `bounds` | `Int`, `Float`, `FloatSlider(s)` |
| `options` | `Menu`, `Selection`, `Selections` |

Read and write values through `nepi_controls.get_control_value()` /
`set_control_value()` / `set_control_bounds()` / `set_control_options()` — never
by indexing the control dict directly. This applies to a node's own internal
reads too (see `rbx_ardupilot_node.py`, which reads `motor_count` and the
takeoff parameters out of its settings dict).

**Two traps when converting an old driver:**

- `'Discrete'` is *not* a `CONTROL_TYPE`. `create_controls_dict()` wraps every
  entry in a bare `except: pass`, so a `'Discrete'` setting is silently dropped
  and simply never appears in the RUI. A named option list is a `'Selection'`.
- The retired cap-settings form carried an `Int`/`Float` control's min and max in
  an `'options'` pair. Those are `'bounds'` now; `'options'` on a numeric control
  is ignored.

### Retired pattern

These no longer exist and no device IF accepts them. Any driver still using them
raises `TypeError` at construction:

```
getCapSettings()  getFactorySettings()  getSettings()  setSetting()  settingUpdateFunction()
capSettings=      factorySettings=      settingUpdateFunction=
nepi_controls.get_data_from_setting()   nepi_controls.check_valid_setting()
```

`device_if_ptx.py` still accepts a `getCapSettingsFunction` constructor argument,
but it is inert — it is never forwarded to `SettingsIF`, which has no such
parameter. Capability data (type, bounds, options) rides in the controls dict
itself. Do not pass it.

## Known Constraints and Fragile Areas

**GenICam requires Baumer libraries.** The `idx_genicam` driver loads `.cti` producer files (`libbgapi2_usb.cti`, `libbgapi2_gige.cti`). These must be present on the system at the paths expected by Harvester. Missing files cause silent failures during discovery.

**Serial device paths are not stable across reboots.** A device at `/dev/ttyUSB0` on one boot may be `/dev/ttyUSB1` on the next. Discovery handles this by re-probing all available ports, but configured addresses and baud rates must match. The `dont_retry_list` can permanently exclude a port after a bad probe sequence — check this list if a known device stops appearing.

**NMEA and HNav simulators run on localhost TCP.** If another process is already bound to the simulator port, the simulator thread will fail silently. The driver node will then fail to connect.

**RBX ArduPilot discovery launches `mavros` as a subprocess.** Unlike other categories, `rbx_ardupilot_discovery.py` does not only launch a node — it also spawns a `mavros_node` (via `subprocess.Popen` with `_fcu_url:=...`) to carry MAVLink, then launches `ArdupilotNode`. There are no `.launch` files; the `fcu_url` connection string is built in the per-type `launch*DeviceNode` helpers, and the shared `launchDeviceNode` writes a `DEVICE_DICT` param the node reads. The RBX node is connection-agnostic — it needs no change to switch transports. Only `SERIAL` is exposed in the params `connection` options today even though `TCP`/`UDP` branches exist; adding an ArduPilot SITL link is therefore mostly a new discovery branch plus a params-YAML option (see `rbx_drivers/SITL_IMPLEMENTATION_PLAN.md`). The APM configs it `rosparam load`s (`apm_pluginlists.yaml`, `apm_config.yaml`) live only at the installed path `/opt/nepi/nepi_engine/share/mavros/launch/`, not in this checkout.

**ONVIF drivers use HTTP/XML.** Network timeouts during ONVIF device probing can cause discovery to block. The `nepi_app_onvif_mgr` app (in `nepi_apps`) handles ONVIF device management at a higher level; the ONVIF drivers here are the low-level node implementations.

**RPi cam3 discovery requires libcamera-hello.** The `idx_rpi_cam3` discovery subprocess calls `libcamera-hello --list-cameras` to enumerate CSI cameras. This binary is only present on Raspberry Pi OS; the discovery node will find no cameras and remain idle on Jetson or x86 build hosts. The `device_path` for RPi cam3 is a camera index string (e.g. `'0'`), not a `/dev/` path — this is unique to this driver family.

**No hardware-in-the-loop CI.** Driver code is not covered by automated tests that require physical hardware. The built-in simulators partially address this for NPX, but IDX, PTX, and LSX drivers have no equivalent.

## Decision Log

- 2026-03 — CLAUDE.md created — Initial developer reference, Claude Code authoring pass.
- 2026-07 — Corrected stale RBX content — `rbx_drivers/` is no longer empty; documented the `rbx_ardupilot` driver (mavros subprocess launch, connection-agnostic node), fixed the RBX interface class name to `RBXRobotIF`, and noted `scripts/fake_gps_node.py` is superseded by the `nepi_app_fake_gps` app.
- 2026-09 — Converted all drivers to the new settings functions — Every driver family (idx, lsx, ptx, rbx, svx) now uses `initSettingsDict`/`refreshSettingsDict`/`getSettingsFunction`/`setSettingFunction`, with `idx_v4l2_node.py` as the reference. The old `capSettings`/`factorySettings`/`settingUpdateFunction` arguments had already been dropped from every `device_if_*` constructor, so every unconverted driver was raising `TypeError` at startup; `ptx_iqr` and `ptx_sidus_ss109` were additionally calling settings methods that did not exist on their own classes. See the Driver Settings Pattern section above.
