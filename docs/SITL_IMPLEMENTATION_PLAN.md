# ArduPilot SITL Support for the NEPI RBX Driver — Implementation Plan

Author handoff document. Goal: let the NEPI RBX ArduPilot driver talk to an
ArduPilot **SITL** (Software-In-The-Loop) simulated flight controller instead
of the physical Pixhawk, so autonomous control (GUIDED / ARM / TAKEOFF / GoTo /
Go Home) can be developed and regression-tested with no hardware.

---

## 0. TL;DR for the implementer

- **SITL is a normal MAVLink endpoint.** The whole NEPI stack already speaks
  MAVLink via `mavros`. We are not writing a new transport; we are just pointing
  `mavros` at a locally-running SITL binary instead of a serial port.
- **The RBX node does NOT change.** `rbx_ardupilot_node.py` is
  connection-agnostic: it attaches to whatever `mavros` namespace the discovery
  script hands it (via `~drv_dict['DEVICE_DICT']`). All SITL work lives in the
  **discovery script** and the **params YAML**.
- **NEPI runs with `--net=host`**, so anything SITL binds on `127.0.0.1`
  (TCP 5760, etc.) is directly reachable by `mavros` inside the container. No
  Docker port mapping is required.
- **Recommended wiring:** add a dedicated `SITL` connection option that makes
  `mavros` a **TCP client to `tcp://127.0.0.1:5760`** (SITL's primary MAVLink
  server), and **force fake-GPS off** (SITL simulates its own GPS + compass).
- Estimated NEPI-side change: ~25 lines across 2 files. The larger effort is
  standing up and validating the SITL instance itself (Step 1).

---

## 1. Background: how the driver connects today (read this first)

The ArduPilot RBX driver is the standard NEPI **3-file driver**, all under
`/home/production/nepi_engine_ws/src/nepi_drivers/rbx_drivers/`:

| File | Role |
|---|---|
| `rbx_ardupilot_params.yaml` | Driver descriptor + user-facing discovery OPTIONS |
| `rbx_ardupilot_discovery.py` | **Discovery**: probes for a device, launches `mavros`, launches the RBX node |
| `rbx_ardupilot_node.py` | The `ArdupilotNode` RBX device node (consumes the `mavros` namespace) |

The connection chain at runtime:

```
  ArduPilot FCU (Pixhawk over /dev/ttyUSB0)
        |  MAVLink @ 57600 baud (serial)
        v
  mavros_node   <-- launched by rbx_ardupilot_discovery.py with _fcu_url:=/dev/ttyUSB0:57600
        |  ROS topics/services under  /nepi/device1/mavlink_ttyUSB0/...
        v
  ArdupilotNode (rbx_ardupilot_node.py)  <-- attaches to that mavros namespace
        |  RBX device_if_rbx.py interface
        v
  Argus / RUI  (GUIDED, ARM, TAKEOFF, GoTo, Go Home buttons)
```

Key facts that make SITL easy (all verified in source):

1. **There are no `.launch` files.** `mavros` is started as a subprocess in
   `rbx_ardupilot_discovery.py:launchDeviceNode()` (lines 242-245):
   ```python
   node_run_cmd = ['rosrun', 'mavros', 'mavros_node', '__name:=' + mav_node_name,
                   '_fcu_url:=' + fcu_url, '_gcs_url:=' + gcs_url]
   mav_subproc = subprocess.Popen(node_run_cmd)
   ```
   **`fcu_url` is the single knob that decides what `mavros` connects to.**

2. **Discovery already branches on a `connection` option** (`discoveryFunction`,
   line 86 reads `drv_dict['DISCOVERY_DICT']['OPTIONS']['connection']['value']`).
   It implements three branches:
   - `SERIAL` -> scans serial ports, probes for a MAVLink heartbeat, then
     `fcu_url = "<port>:<baud>"` (e.g. `/dev/ttyUSB0:57600`).
   - `TCP` -> `fcu_url = "tcp://<ip>:<port>"` (lines 386-395).
   - `UDP` -> `fcu_url` is currently **hardcoded** to a bench IP
     `"udp://192.168.179.103:14555@192.168.179.5:14550"` (line 413).

   But `rbx_ardupilot_params.yaml` only exposes `SERIAL` in the selectable
   `connection.options`, so TCP/UDP are effectively dead paths from the UI today.

3. **The RBX node reads everything from `DEVICE_DICT`** that discovery pushes to
   the param server (`rbx_ardupilot_discovery.py` lines 256-267; consumed at
   `rbx_ardupilot_node.py` lines 185-201). As long as discovery launches a
   `mavros` node and populates `mavlink_node_name` / `device_path` / `fake_gps`,
   the node works **unchanged**. This is why no node edits are needed.

4. **fake_gps is a driver option** (`rbx_ardupilot_params.yaml` lines 24-29,
   default `True`). Discovery reads it (lines 87-88) and passes it to the node as
   `DEVICE_DICT['fake_gps']`. On real hardware the `nepi_app_fake_gps` app injects
   a simulated `GPS_INPUT` into `mavros`. **SITL provides its own simulated GPS
   and compass, so fake GPS must be OFF** or the two position sources will fight.

---

## 2. Design decision: how should `mavros` reach SITL?

ArduPilot SITL exposes MAVLink on several endpoints. The two realistic choices
for `mavros`:

| Option | `fcu_url` | Pros | Cons |
|---|---|---|---|
| **TCP to 5760 (recommended)** | `tcp://127.0.0.1:5760` | SITL's TCP server; the discovery `checkForTcpDevice()` does a real `connect_ex` probe, so `mavros` is only launched once SITL is actually up. Clean retry if SITL starts after NEPI. | SITL must expose 5760 to `mavros` (don't let MAVProxy grab it — see Step 1). |
| UDP to 14550 | `udp://127.0.0.1:14550@` | Matches `sim_vehicle.py` default MAVProxy out. | `checkForUdpDevice()` (line 400) **always returns True** with no probe, so `mavros` will launch against a dead endpoint if SITL isn't running yet, then sit disconnected. Ordering-fragile. |

**Recommendation: TCP to `127.0.0.1:5760`,** because the TCP reachability probe
gives us free, correct start-ordering (NEPI can boot before or after SITL and
self-heal on the 1 Hz discovery timer).

**Recommendation: add a dedicated `SITL` connection type** rather than reusing
the generic `TCP` branch. Reasons:
- Self-documenting in the RUI dropdown ("SITL" vs a bare "TCP").
- Doesn't collide with a future real networked-TCP flight controller.
- Lets us name the node cleanly (`mavlink_sitl`, `ardupilot_sitl`) instead of the
  IP-mangled `mavlink_127001_5760` the generic TCP path would produce
  (`launchTcpDeviceNode` builds `device_id_str` from the flattened IP+port,
  line 390).
- Lets us **force fake-GPS off** in that branch defensively.

A minimal-effort alternative (reuse the existing `TCP` branch, only edit the
YAML) is documented in Appendix A for anyone who wants zero new code paths.

---

## 3. Implementation steps

### Step 1 — Stand up an ArduPilot SITL instance (the non-NEPI part; largest effort)

This is plain ArduPilot, independent of NEPI. Because the NEPI container runs
with `--net=host` (`nepi_setup/resources/docker/nepi_docker_start.sh` lines
116-132), you may run SITL **on the host**, **in its own container with
`--net=host`**, or **inside the NEPI container** — in all three cases a
`127.0.0.1:5760` bind is reachable by `mavros`. **Running on the host is
recommended** (keeps the NEPI image clean; nothing to re-commit).

Build from source (one time):
```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
./waf configure --board sitl
./waf copter          # builds ArduCopter SITL
```

Run it so that **TCP 5760 is free for `mavros`** (this is the important part):
```bash
# --no-mavproxy: do NOT start the MAVProxy GCS, so it doesn't consume TCP 5760.
# mavros will be the sole MAVLink client on 5760.
Tools/autotest/sim_vehicle.py -v ArduCopter --no-mavproxy
```
or run the binary directly (also leaves 5760 free, no MAVProxy at all):
```bash
build/sitl/bin/arducopter -S -I0 --model + --speedup 1 \
    --defaults Tools/autotest/default_params/copter.parm
```

> **Why `--no-mavproxy` matters:** `sim_vehicle.py` by default launches MAVProxy,
> which connects to SITL's TCP 5760 and re-broadcasts on UDP 14550/14551. If
> MAVProxy holds 5760, `mavros` would have to use 5762/5763 or a MAVProxy `--out`
> UDP endpoint. Keeping MAVProxy out of the way lets `mavros` own 5760 cleanly.
> If you *want* a MAVProxy console for debugging, instead run with MAVProxy and
> add a dedicated output for mavros, e.g. `--out=tcpin:0.0.0.0:5762`, and set the
> SITL port option (Step 3) to `5762`.

Docker alternative (if you prefer not to build): use any prebuilt ArduPilot SITL
image run with `--net=host` so it binds `127.0.0.1:5760` on the shared stack.
Verify the image exposes the TCP server port and isn't hidden behind MAVProxy.

**Acceptance for Step 1:** from the host or inside the NEPI container,
`nc -z 127.0.0.1 5760 && echo open` reports the port open while SITL runs.

### Step 2 — Confirm NEPI can reach the SITL port (no code; just verify the assumption)

Because of `--net=host`, there is nothing to configure. Just verify from inside
the running NEPI container that the SITL port is reachable (container name
changes on restart; find it first):
```bash
C=$(docker ps --format '{{.Names}}' | head -1)
docker exec -i "$C" bash -c 'timeout 2 bash -c "cat < /dev/null > /dev/tcp/127.0.0.1/5760" && echo REACHABLE || echo NOT-REACHABLE'
```
If this prints `REACHABLE`, `mavros` will connect. (If NOT, SITL isn't running,
isn't on 127.0.0.1, or MAVProxy is holding the port — revisit Step 1.)

### Step 3 — Add the `SITL` connection option to the params YAML

File: `rbx_ardupilot_params.yaml`. Add `SITL` to the `connection` options so it
is selectable in the RUI driver-options panel:

```yaml
      connection:
        type: Discrete
        options:
          - SERIAL
          - SITL          # <-- add
        default: SERIAL
        value: SERIAL
```

> Leave `default: SERIAL` so a normal (hardware) appliance is unaffected. The
> user selects `SITL` at runtime from the RUI, which persists via the drivers
> manager (`drivers_mgr.py` `updateSetting`, save_config). For a
> SITL-dedicated dev image you may instead set `default`/`value` to `SITL`.

**Optional (nice-to-have):** expose the SITL host/port as editable options
instead of hardcoding them in the discovery class. The OPTIONS schema supports
string types; if you add them, read them in `discoveryFunction` next to the
existing `connection`/`fake_gps` reads (line 86-88). If you skip this, the
host/port live as class attributes in Step 4 (fine for a fixed `127.0.0.1:5760`).

### Step 4 — Add the `SITL` branch to the discovery script

File: `rbx_ardupilot_discovery.py`.

**(a)** Add SITL endpoint defaults to the class attributes (near lines 50-53):
```python
  sitl_addr_list = ['127.0.0.1']
  sitl_tcp_port_list = ['5760']
```

**(b)** Add a `SITL` branch in `discoveryFunction`, right after the
`elif connection_type == 'TCP' or connection_type == "UDP":` block (after line
155). It reuses the existing TCP reachability probe so `mavros` only launches
once SITL is actually accepting connections:
```python
    # RUN SITL PROCESS (ArduPilot Software-In-The-Loop over local TCP)
    elif connection_type == 'SITL':
      for ip_addr_str in self.sitl_addr_list:
        for ip_port_str in self.sitl_tcp_port_list:
          path_str = "SITL_" + ip_addr_str + "_" + ip_port_str
          if path_str not in self.active_paths_list and path_str not in self.dont_retry_list:
            # Reuse the TCP probe: only launch mavros once SITL's MAVLink TCP
            # server is up. checkForTcpDevice splits the "<type>_<ip>_<port>"
            # path and ignores the type field, so a "SITL_..." path works as-is.
            [found_device, path_str] = self.checkForTcpDevice(path_str)
            if found_device:
              success = self.launchSitlDeviceNode(path_str)
              if success:
                self.active_paths_list.append(path_str)
```

**(c)** Add the launch helper (alongside `launchTcpDeviceNode` /
`launchUdpDeviceNode`, after line 415):
```python
  def launchSitlDeviceNode(self, path_str):
    # path_str format: "SITL_<host>_<port>"
    [con_type, ip_addr_str, ip_port_str] = path_str.split("_")
    device_id_str = "sitl"          # -> mavros node "mavlink_sitl", rbx node "ardupilot_sitl"
    mav_comp_id = 1
    mav_sys_id = 1                   # SITL default SYSID_THISMAV = 1
    fcu_url = "tcp://" + ip_addr_str + ":" + ip_port_str   # e.g. tcp://127.0.0.1:5760
    gcs_url = ""
    # SITL simulates its own GPS + compass. Force fake GPS OFF so the injected
    # GPS_INPUT can't fight the simulated sensors, regardless of the option value.
    self.enable_fake_gps = False
    return self.launchDeviceNode(path_str, device_id_str, mav_comp_id, mav_sys_id, fcu_url, gcs_url)
```

> Why this is safe/minimal: `launchDeviceNode` (line 212) is unchanged and does
> all the real work (loads the APM plugin/config YAMLs, launches `mavros` with
> our `fcu_url`, builds `DEVICE_DICT`, launches the RBX node). We set
> `self.enable_fake_gps = False` **before** calling it, so the `DEVICE_DICT`
> written at line 264 carries `fake_gps: False` to the node.

### Step 5 — Ensure fake GPS is actually off

Two independent layers; for SITL both should be off (Step 4c handles the driver
layer, but confirm the app layer too):

1. **Driver layer (handled in Step 4c):** the `SITL` branch forces
   `enable_fake_gps = False`, so the node receives `fake_gps: False` and does not
   drive the fake-GPS app. You may also set the `fake_gps` option to `False` in
   the RUI for clarity.
2. **App layer:** the `nepi_app_fake_gps` app has its own enable flag
   (`fake_gps_app_node.py` line 67, `FACTORY_ENABLED = False`). It is OFF by
   default. If someone previously enabled it on this appliance, disable it (RUI
   toggle, or publish `False` to `<base>/app_fake_gps/enable`). Verify it is not
   publishing before an armed SITL test.

### Step 6 — SITL flight-controller params / arming expectations

**None of the real-hardware compass/GPS workarounds apply to SITL** (see the
project memory `project-fake-gps-arm-compass.md`: SITL simulates a healthy
compass and its own GPS). Concretely:

- Boot SITL with stock `copter.parm` defaults. It will arm in GUIDED without any
  `COMPASS_*` calibration restore, GPS-yaw hacks, or `GPS_INPUT` time injection.
- SITL needs ~20-40 s after boot to acquire a GPS 3D fix and settle the EKF /
  set the origin before it will arm. Wait for a healthy EKF before ARM.
- Autonomous sequence is unchanged and the same as hardware:
  **GUIDED -> ARM -> TAKEOFF -> (GoTo / Go Home).**
- SITL default `SYSID_THISMAV = 1` matches the `mav_sys_id = 1` we set in Step 4c.
- SITL default home is CMAC (Canberra). To simulate elsewhere, launch SITL with
  `-L <name>` or `--custom-location=lat,lon,alt,heading`. Not required for
  functional testing of the RBX flow.

### Step 7 — Deploy the code changes and restart

Follow the standing rule: **edit only in `/home/production/nepi_engine_ws/...`**
(canonical source), then rebuild/redeploy so the runtime copy under
`/opt/nepi/nepi_engine/lib/nepi_drivers/` is updated. (For a quick dev loop you
may hot-patch the deployed `/opt` copies of `rbx_ardupilot_discovery.py` and
`rbx_ardupilot_params.yaml` to match `/home`, but a proper build is the real
fix — the next build overwrites `/opt` from `/home`.)

Because the discovery script is imported by the **drivers manager**, code changes
require the drivers manager (or NEPI) to restart to reload it. The RBX node
itself is (re)launched by discovery, so it picks up new `DEVICE_DICT` values on
the next launch automatically.

Bring-up order that avoids fighting a real serial FCU:
1. Start the SITL instance (Step 1). Confirm port reachable (Step 2).
2. In the RUI drivers panel, set the ArduPilot driver `connection` option to
   `SITL` (and `fake_gps` to `False`).
3. The drivers manager's discovery timer probes `tcp://127.0.0.1:5760`, launches
   `mavros` as `mavlink_sitl`, and launches the RBX node as `ardupilot_sitl`.

---

## 4. Testing / acceptance checklist

Run ROS commands inside the container (name changes each restart):
```bash
C=$(docker ps --format '{{.Names}}' | head -1)
RX() { docker exec -i "$C" bash -c "export ROS_MASTER_URI=http://localhost:11311; export ROS_IP=127.0.0.1; source /opt/ros/noetic/setup.bash; $1"; }
```

1. **mavros connected to SITL:**
   ```bash
   RX 'rostopic echo -n1 /nepi/device1/mavlink_sitl/state'
   ```
   Expect `connected: True`. (Namespace is `mavlink_sitl` with the dedicated
   SITL option; it would be `mavlink_127001_5760` with the Appendix A shortcut.)

2. **RBX node up:** `ardupilot_sitl` RBX status topics are publishing; the device
   appears in Argus/RUI.

3. **Arming flow:** from the RUI, GUIDED -> ARM -> TAKEOFF. Expect
   `armed: True, guided: True, mode: "GUIDED"` and the simulated vehicle climbs.
   "Autonomous Ready" latches green.

4. **Motion commands** (the ones fixed for hardware this cycle): GoTo Position
   (e.g. "20 forward"), Go Home, Go Action. Expect no callback crashes and the
   SITL vehicle to actually move in the simulator (visible in a SITL map/console
   if you keep MAVProxy or a GCS connected on a secondary port).

5. **fake GPS is silent:** confirm `nepi_app_fake_gps` is not publishing
   `GPS_INPUT` / `gps_input` during the test.

6. **Regression:** switch `connection` back to `SERIAL`, confirm the hardware
   path still discovers `/dev/ttyUSB0` unaffected.

---

## 5. Notes, gotchas, and gotcha-avoidance

- **`mavros` is a TCP *client*; SITL is the TCP *server*.** `tcp://host:port`
  makes `mavros` dial out. Do not add a trailing `@` (that toggles server mode).
- **Only one MAVLink client can own SITL TCP 5760.** Keep MAVProxy off it
  (`--no-mavproxy`) or give `mavros` its own `--out` port. Two clients on 5760 =
  connection churn.
- **The APM config YAMLs** loaded by `launchDeviceNode` (lines 230-231) live only
  at the installed path `/opt/nepi/nepi_engine/share/mavros/launch/apm_config.yaml`
  and `apm_pluginlists.yaml`. They are not in the dev checkout. They apply
  unchanged to SITL (same MAVLink plugins) — just make sure they exist at runtime
  on the target (they do in the deployed image). No SITL-specific edit needed.
- **`UDP` branch is a trap for SITL:** `checkForUdpDevice` (line 400) never
  probes, so a UDP SITL option would launch `mavros` even when SITL is down, then
  show `connected: False`. Prefer the TCP-based SITL branch (Step 4).
- **fake GPS vs SITL GPS:** never run both. fake GPS injecting `GPS_INPUT` on top
  of SITL's simulated GPS produces EKF position fights and bad arming behavior.
- **Node naming:** the dedicated SITL option yields the clean namespace
  `/nepi/device1/mavlink_sitl`. If you take the Appendix A shortcut you get the
  ugly `/nepi/device1/mavlink_127001_5760` (IP+port flattened by
  `launchTcpDeviceNode`, line 390) — still functional, just noisy.
- **`device1`** in the namespace is the fixed appliance ID (`NEPI_DEVICE_ID`,
  from `nepi_base.launch`), NOT a per-device enumeration; it is unaffected by
  SITL.
- **Do not edit the `/mnt/nepi_storage/nepi_src` container source copy.** Canonical
  edits go in `/home/production/nepi_engine_ws`; the build populates `/opt` (and
  `/mnt`). See project memory `feedback-edit-home-nepi-engine-only`.

---

## 6. File-by-file change summary

| File | Change | Why |
|---|---|---|
| `rbx_ardupilot_params.yaml` (lines ~14-22) | Add `- SITL` to `connection.options` | Makes SITL selectable in the RUI |
| `rbx_ardupilot_discovery.py` (~line 50) | Add `sitl_addr_list = ['127.0.0.1']`, `sitl_tcp_port_list = ['5760']` | SITL endpoint defaults |
| `rbx_ardupilot_discovery.py` (after line 155) | Add `elif connection_type == 'SITL':` branch (Step 4b) | Route discovery to SITL, reusing the TCP reachability probe |
| `rbx_ardupilot_discovery.py` (after line 415) | Add `launchSitlDeviceNode()` (Step 4c) | Build `fcu_url = tcp://127.0.0.1:5760`, force fake GPS off, launch |
| `rbx_ardupilot_node.py` | **No change** | Node is connection-agnostic; reads `DEVICE_DICT` |
| SITL instance (external) | Build + run ArduCopter SITL, TCP 5760 free | The simulated flight controller itself |

---

## Appendix A — Minimal alternative (no new discovery code)

If you want to avoid adding a `SITL` branch, you can reuse the existing `TCP`
branch with YAML + two class-attribute edits only:

1. In `rbx_ardupilot_params.yaml`, add `- TCP` to `connection.options`.
2. In `rbx_ardupilot_discovery.py`, change the hardcoded defaults
   `ip_addr_list = ['127.0.0.1']` (line 51) and `ip_tcp_port_list = ['5760']`
   (line 53).
3. Set the driver `fake_gps` option to `False` (there is no forced-off safeguard
   on this path, so you must do it explicitly).
4. Select `connection = TCP` in the RUI.

Result: `fcu_url = tcp://127.0.0.1:5760` via `launchTcpDeviceNode`. Downsides vs
the dedicated option: repurposes "TCP" (can't also serve a real networked FC),
the node namespace becomes `mavlink_127001_5760`, and fake GPS has no safeguard.
Use only for a quick throwaway test.

---

## Appendix B — Reference file locations (absolute paths)

- Driver package: `/home/production/nepi_engine_ws/src/nepi_drivers/rbx_drivers/`
- Discovery: `.../rbx_drivers/rbx_ardupilot_discovery.py`
  (branch dispatch ~line 113-157; `launchDeviceNode` line 212; TCP helpers
  370-395; UDP helpers 400-415)
- Node: `.../rbx_drivers/rbx_ardupilot_node.py` (DEVICE_DICT read lines 185-201)
- Params: `.../rbx_drivers/rbx_ardupilot_params.yaml`
- fake GPS app: `/home/production/nepi_engine_ws/src/nepi_apps/nepi_app_fake_gps/scripts/fake_gps_app_node.py`
- Drivers manager: `/home/production/nepi_engine_ws/src/nepi_engine/nepi_managers/scripts/drivers_mgr.py`
- Container run script (host networking, `--net=host`): `/home/production/nepi_engine_ws/nepi_setup/resources/docker/nepi_docker_start.sh` (lines 116-132)
- Deployed runtime driver copy: `/opt/nepi/nepi_engine/lib/nepi_drivers/`
