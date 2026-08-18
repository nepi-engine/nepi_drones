# ROS Topics & Services — Simulation Stack

A complete inventory of every ROS topic, ROS service, and custom TCP bridge across the
five simulation driver variants in this repo: the generic rover (single and two-robot),
the Webots rover, the Webots quadcopter, and the ArduPilot SITL drone. For each entry:
who publishes/serves it, who subscribes/calls it, what the data looks like, and which
variant(s) it belongs to. (A sixth, RBX_GAZEBO -- an older, independently-built Gazebo
rover driver -- was removed 2026-08-18: superseded by RBX_SIM, never actually deployed
to any device, and its default ports collided with RBX_SIM's own. See its own section
below, kept for the removal record.)

This is a reference document, not a walkthrough — see `SIMULATION_OVERVIEW.md` for a
narrative file-by-file tour and concepts primer (RBX/driver/discovery/node vocabulary,
the two-machine setup, the "902x port block" convention) before this one if you're new
to the codebase.

Compiled from a full pass over `sim_container/`, `src/nepi_drivers/rbx_drivers/`,
`src/nepi_api/`, and `src/nepi_apps/nepi_app_sim_connector/` on 2026-08-18, followed by a
second pass the same day that resolved every open question the first pass had flagged
(driver internals for RBX_GAZEBO/RBX_WEBOTS/RBX_WEBOTS_QUADCOPTER, the core framework's
navpose/image/settings/save-data topics, fake-GPS's real mavros injection point, the
AI-targeting relay, and the Webots/PyBullet/WPILib generic-connector bridges). Two
corrections from that second pass, called out again in
[Since the first pass](#since-the-first-pass) at the bottom.

**Legend:** 🔵 topic · 🟣 service · 🟠 custom TCP/JSON bridge (not ROS) · 🟢 MAVLink (via
mavros) · ⚙️ Gazebo-internal (never crosses to the real device)

---

## The big picture

The dev VM (Gazebo/Webots/PyBullet) and the real NEPI device are two separate machines
running two separate ROS installations — different ROS masters, no shared network. A
topic published on one is invisible to the other. Everything that needs to cross that
gap does it over **one reverse SSH tunnel** (`autossh`), using one of three patterns: a
dedicated JSON-over-TCP bridge per driver, real MAVLink via mavros, or a shared generic
connector protocol one port serves to any simulator.

```
DEV VM (own Gazebo/Webots ROS master)             NEPI DEVICE (production ROS master + RUI)
───────────────────────────────                   ───────────────────────────────────────────

Rover ×1:
  generic_rover.world --ROS--> sim_bridge_node.py --TCP :9023 JSON (tunnel)--> rbx_sim_node.py --ROS--> RBXRobotIF → RUI

Rover ×2:
  generic_rover_multi.world --ROS--> sim_bridge_multi_node.py --TCP :9023 & :9025 (tunnel)--> rbx_sim_node.py ×2 --ROS--> RBXRobotIF ×2 → RUI

ArduPilot:
  ArduPilot SITL + Gazebo (iris world) --MAVLink TCP :5771 (tunnel)--> mavros → rbx_ardupilot_node.py --ROS--> RBXRobotIF → RUI
                                        \--TCP :9026 camera + :9021 reset (tunnel)--/   (mavros carries none of this)
```

RBX_WEBOTS and RBX_WEBOTS_QUADCOPTER follow the exact same top-row shape as the rovers,
just on their own ports (9041/9046 and 9042/9047) — see their own sections below.

Five RBX driver variants exist in this repo today, split across three connection
patterns. One uses a dedicated heartbeat+bridge port pair per instance, one uses real
MAVLink, and — separately from all of it — a newer shared "generic connector" protocol
on one fixed port serves any simulator's own small bridge script (Gazebo, Webots,
PyBullet, even WPILib).

| Driver | Simulator / vehicle | Pattern | Ports | Status |
|---|---|---|---|---|
| RBX_SIM (rover ×1) | Gazebo, differential-drive rover | 🟠 bridge | 9022 / 9023 | current |
| RBX_SIM (rover ×2) | Gazebo, two rovers side by side | 🟠 bridge | 9022–9025 | current |
| RBX_WEBOTS | Webots, 4-wheel tank-drive rover | 🟠 bridge | 9041 / 9046 | current, live-verified |
| RBX_WEBOTS_QUADCOPTER | Webots, Supervisor-velocity quadcopter | 🟠 bridge | 9042 / 9047 | current, live-verified |
| RBX_ARDUPILOT | ArduPilot SITL, quadrotor drone | 🟢 mavlink + 🟠 bridge for camera/reset | 5771, 9021, 9026 | current |

---

## Rover — single robot

`sim_bridge_node.py` runs on the dev VM and is the switchboard: it drives the rover in
Gazebo, reads back its position and camera feed, and relays all of it over one TCP
connection to the real NEPI device.

### Gazebo-side ROS topics (dev VM only)

| | Topic | Type | Publisher → Subscriber | What it carries |
|---|---|---|---|---|
| 🔵 | `/sim/heartbeat` | `std_msgs/Header` | `sim_bridge_node.py` | 1 Hz pulse — "the bridge process is alive." Answered on port 9022, not really a ROS subscription target. |
| 🔵 | `/nepi/sim/cmd_vel` | `geometry_msgs/Twist` | `sim_bridge_node.py` → `sim_bridge_node.py` | Internal relay hop: the JSON velocity command from the device lands here first, then gets re-published to `/rover/cmd_vel`. |
| 🔵 | `/rover/cmd_vel` | `geometry_msgs/Twist` | `sim_bridge_node.py` → Gazebo diff-drive plugin | Final drive command: `linear.x` m/s, `angular.z` rad/s. |
| 🔵 | `/rover/odom` | `nav_msgs/Odometry` | Gazebo diff-drive plugin → `sim_bridge_node.py` | Ground-truth pose straight from the physics engine (not simulated wheel encoders): x/y, yaw, linear/angular speed. 30 Hz. |
| 🔵 | `/rover/camera/image_raw` | `sensor_msgs/Image` | Gazebo camera plugin → `camera_rig_controller.py` | Raw first-person frame, rigidly attached to the rover. |
| 🔵 | `/rover/camera_chase/image_raw` | `sensor_msgs/Image` | Gazebo camera plugin → `camera_rig_controller.py` | Raw third-person "chase cam" frame. |
| 🔵 | `/camera_rig/image_compressed` | `sensor_msgs/CompressedImage` | `camera_rig_controller.py` → `sim_bridge_node.py` | JPEG (quality 65) of whichever view is selected, 10 Hz — the frame actually shipped to the device. |

### Inside the port-9023 bridge

One persistent TCP connection carries newline-delimited JSON in both directions:

```
device → VM, ~20 Hz
{"linear_x": 0.3, "angular_z": 0.1}

// camera view changes, resets, and obstacle-course toggles ride the same socket:
{"type": "camera_settings", "view_mode": "FIRST_PERSON", "offset_x": 0.2, "offset_y": 0.0, "offset_z": 0.65}
{"type": "reset"}
{"type": "obstacle_course", "enabled": true}

VM → device, ~10 Hz
{"x": 1.23, "y": 4.56, "yaw": 0.78, "linear_x": 0.3, "angular_z": 0.0, "stamp": 1723999999.1}
{"type": "image", "data": "<base64 jpeg>", "stamp": 1723999999.1}
```

Mapping: `/nepi/sim/cmd_vel` is fed by the velocity-cmd JSON; `/rover/odom` feeds the
telemetry JSON; `/camera_rig/image_compressed` feeds the image JSON. `{"type":"reset"}`
triggers `/gazebo/set_model_state`; `{"type":"obstacle_course"}` triggers
`/gazebo/spawn_sdf_model`/`delete_model`.

**Heartbeat, separately (port 9022).** Before any of the above starts, the device's
discovery script connects to port 9022 and requires the literal reply `ALIVE` before
deciding the simulator is up — a bare TCP `connect()` can succeed against the SSH
daemon on the other end even when nothing is actually listening, so a plain connection
test isn't good enough.

### Device-side ROS topic

| | Topic | Type | Publisher → Subscriber | What it carries |
|---|---|---|---|---|
| 🔵 | `<device_name>/color_2d_image` | `sensor_msgs/Image` | `rbx_sim_node.py` → `RBXRobotIF` | Decoded from the bridge's base64 JPEG. `RBXRobotIF` finds this topic by name automatically — the real image feed the rest of NEPI sees for this device. |

---

## Rover — two robots

Same everything, doubled. Two full copies of the rover run in one Gazebo world, each
with its own bridge port pair, so the two robots never share a topic or a socket.

> **Why the world file duplicates everything by hand:** Gazebo Classic 11 can't give two
> `<include>`d copies of the same model separately-namespaced plugins — both instances'
> plugins collide onto the same topic names. The fix was to write out two full,
> separately-named `<model>` blocks in `generic_rover_multi.world` instead of reusing
> one model twice.

| | Topic (rover1 / rover2) | Type | Publisher → Subscriber | Notes |
|---|---|---|---|---|
| 🔵 | `/rover1/cmd_vel` · `/rover2/cmd_vel` | `geometry_msgs/Twist` | `sim_bridge_multi_node.py` → diff-drive plugin | One `RobotBridge` instance per slot in the same process. |
| 🔵 | `/rover1/odom` · `/rover2/odom` | `nav_msgs/Odometry` | diff-drive plugin → `sim_bridge_multi_node.py` | Same fields as the single-robot version, per instance. |
| 🔵 | `/rover1/camera/image_raw` · `/rover2/…` | `sensor_msgs/Image` | Gazebo camera plugin → `camera_rig_controller_multi.py` | First-person view per rover. |
| 🔵 | `/camera_rig1/image_compressed` · `/camera_rig2/…` | `sensor_msgs/CompressedImage` | `camera_rig_controller_multi.py` → `sim_bridge_multi_node.py` | JPEG quality 60, 7 Hz. |
| 🔵 | `/sim/heartbeat` | `std_msgs/Header` | `sim_bridge_multi_node.py` | One shared heartbeat topic for both slots. |

Each robot's camera rig is a free-floating Gazebo model repositioned every frame via
`/gazebo/set_model_state` (20 Hz) rather than a rigid joint, so it can be swapped
between first-person and chase-cam live without respawning anything.

**Port assignments:**

| Port | Purpose |
|---|---|
| 9022 | rover1 heartbeat listener |
| 9023 | rover1 command/telemetry/image bridge |
| 9024 | rover2 heartbeat listener |
| 9025 | rover2 command/telemetry/image bridge |

---

## RBX_GAZEBO — removed (2026-08-18)

Deleted (`rbx_gazebo_node.py`/`rbx_gazebo_discovery.py`/`rbx_gazebo_params.yaml`, all
only ever in `nepi_drones`, never promoted to production) after confirming it was
superseded by RBX_SIM/the generic sim-connector path and never actually deployed to any
device. Its own default ports (9022 heartbeat / 9023 bridge) collided with RBX_SIM's
rover-single slot -- see `docs/SIM_CONNECTOR_REMAINING_WORK.md` item 4, now resolved by
removal rather than kept as an open decision.

---

## RBX_WEBOTS — rover in Webots

A 4-wheel tank-drive rover simulated in Webots instead of Gazebo, rebuilt from RBX_SIM's
pattern (not the since-removed RBX_GAZEBO one) specifically to pick up teleop control and the
camera-offset/capability settings the older driver lacks. Documented as "done and fully
live-verified end to end" (2026-08-17) — a real `goto_position` test moved it 0 → 2.02 m
with `cmd_success: true`.

| | Topic | Type | Publisher → Subscriber | Notes |
|---|---|---|---|---|
| 🔵 | `<device_name>/color_2d_image` | `sensor_msgs/Image` | `rbx_webots_node.py` → `RBXRobotIF` | This Webots world has only one physical camera device — the driver's "scene" and "robot" view options both honestly resolve to the same feed rather than faking a second one. |

Bridge protocol is the same bare-JSON shape as the rovers (`{"x","y","yaw","linear_x",
"angular_z"}`, no `"type"` key) — simpler than the richer generic-connector protocol
below. Role is reversed from the rover bridges though: `webots_rbx_bridge.py` runs as
the TCP **server** on the VM (heartbeat 9041, bridge 9046 — picked to stay clear of the
902x block), and `rbx_webots_node.py` dials in as the client. On the Webots side it
drives real controller devices: `wheel1`/`wheel3` (left motors), `wheel2`/`wheel4`
(right motors), plus `gps`, `imu`, and `camera`.

---

## RBX_WEBOTS_QUADCOPTER — a different vehicle class

Not ArduPilot SITL, and not rotor aerodynamics — this quadcopter flies as a
**Supervisor-velocity kinematic body**: Webots directly writes its 3D position and yaw
each tick rather than simulating thrust and gravity. It's the first genuinely 3D-flight
driver in the Webots family, and it's documented as "done, fully live-verified end to
end" (2026-08-17), with real TAKEOFF (0.3 m → 2.04 m), `goto_position` (0 → 2.03 m
holding altitude), and LAND (2.04 m → 1.02 m) telemetry.

| | Topic | Type | Publisher → Subscriber | Notes |
|---|---|---|---|---|
| 🔵 | `<device_name>/color_2d_image` | `sensor_msgs/Image` | `rbx_webots_quadcopter_node.py` → `RBXRobotIF` | Same convention as every other driver. |

Bridge ports: 9042 heartbeat / 9047 bridge (the next free slots after the rover's
9041/9046), served by `webots_rbx_bridge_quadcopter.py`. That bridge script uses
Webots' **Supervisor** API rather than plain `Robot`, which is what lets its setup
actions be real: `RESET_SIM` actually teleports the vehicle back to spawn — unlike the
rover bridge above, where a reset is an honest no-op because a plain `Robot` controller
has no permission to move itself in Webots.

**Setup actions:** `TAKEOFF`, `LAND`, `RESET_SIM`, `RETURN_HOME`. Manual per-motor
control is deliberately not exposed — a Supervisor-velocity body has no per-rotor speed
ratio that would mean anything — but teleop velocity control is wired.

---

## ArduPilot SITL drone

Here the cross-machine link is **real MAVLink**, not a custom protocol — mavros already
speaks the wire format ArduPilot's flight controller expects. The only things that
*don't* fit through mavros are the camera feed and the "teleport back to origin" reset,
so those get their own small TCP bridges.

### mavros topics (all under `<mavlink_node>/…`)

| | Topic | Type | Publisher → Subscriber | What it carries |
|---|---|---|---|---|
| 🟢 | `state` | `mavros_msgs/State` | mavros → `rbx_ardupilot_node.py` | `armed` and `mode` — drives the RBX status/mode display. |
| 🟢 | `battery` | `sensor_msgs/BatteryState` | mavros → `rbx_ardupilot_node.py` | Battery percentage. |
| 🟢 | `global_position/global` | `sensor_msgs/NavSatFix` | mavros → `rbx_ardupilot_node.py` | Lat/long/altitude (geoid-corrected). |
| 🟢 | `global_position/local` | `nav_msgs/Odometry` | mavros → `rbx_ardupilot_node.py` | Position/orientation, converted to ENU. |
| 🟢 | `global_position/compass_hdg` | `std_msgs/Float64` | mavros → `rbx_ardupilot_node.py` | True-north heading, degrees. |
| 🟢 | `statustext/recv` | `mavros_msgs/StatusText` | mavros → `rbx_ardupilot_node.py` | Free-text FCU messages: pre-arm failures, EKF warnings, failsafes — surfaced straight into RBX status. |
| 🟢 | `setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | `rbx_ardupilot_node.py` → mavros | Published at 50 Hz while a goto is in flight. |
| 🟢 | `setpoint_position/local` | `geometry_msgs/PoseStamped` | `rbx_ardupilot_node.py` → mavros | Local ENU position + yaw target for `goto_position`/`goto_pose`. |
| 🟢 | `setpoint_position/global` | `geographic_msgs/GeoPoseStamped` | `rbx_ardupilot_node.py` → mavros | Global lat/long/alt + yaw target for `goto_location`. |
| 🟢 | `setpoint_velocity/cmd_vel_unstamped` | `geometry_msgs/Twist` | `rbx_ardupilot_node.py` → mavros | Manual teleop only, 20 Hz, independent of the goto loop. |

### mavros services

| | Service | Type | Client | Purpose |
|---|---|---|---|---|
| 🟣 | `cmd/set_home` | `CommandHome` | `rbx_ardupilot_node.py` | Set the flight controller's home lat/long/alt. |
| 🟣 | `set_mode` | `SetMode` | `rbx_ardupilot_node.py` | STABILIZE / LAND / RTL / LOITER / GUIDED. |
| 🟣 | `cmd/arming` | `CommandBool` | `rbx_ardupilot_node.py` | Arm / disarm. |
| 🟣 | `cmd/takeoff` | `CommandTOL` | `rbx_ardupilot_node.py` | `min_pitch`, `altitude`. |
| 🟣 | `cmd/command` | `CommandLong` | `rbx_ardupilot_node.py` | Generic MAVLink passthrough — motor-test, and a *forced* disarm (magic param `21196.0`) needed to disarm mid-air for RESET_SIM. |
| 🟣 | `set_stream_rate` | `StreamRate` | `rbx_ardupilot_node.py` | Called once at startup, requesting `STREAM_ALL` @ 10 Hz. **Without this call, mavros reports `connected: true` but every telemetry topic above just sits silent forever.** |

### ArduPilot's own small TCP bridges

Unlike the rovers, ArduPilot doesn't need a bridge for flight data — mavros already
handles that. These two exist only for the two things MAVLink doesn't carry.

```
camera bridge — port 9026, device ↔ VM
{"type": "camera_settings", "view_mode": "FIRST_PERSON", "offset_x": 0.15, "offset_y": 0.0, "offset_z": -0.1}   // device -> VM
{"type": "image", "data": "<base64 jpeg>", "stamp": 1723999999.1}                                                // VM -> device, 7 Hz

reset bridge — port 9021, fires RESET_SIM
// rbx_ardupilot_node.py first force-disarms over mavros, then connects here.
// the listener runs:  gz world -w default -o     (pose reset ONLY)
// never:              gz world -w default -r     (this also resets sim TIME and crashes ArduPilot's SITL link)
// reply: "OK" or "ERR <message>"
```

### Real-camera fallback

| | Topic | Type | Publisher → Subscriber | Notes |
|---|---|---|---|---|
| 🔵 | `<device_name>/color_2d_image` | `sensor_msgs/Image` | `rbx_ardupilot_node.py` → `RBXRobotIF` | Fed by whichever source is live: the port-9026 sim bridge, *or* a real onboard camera driver's `idx/color_image` if one is running. |

### Fake-GPS app topics (published by `rbx_ardupilot_node.py`)

Feeds a separate GPS-spoofing app (`nepi_app_fake_gps`, part of the production
`nepi_apps` submodule) so ArduPilot can be tested as if it were somewhere else on Earth.

| | Topic | Type | Purpose |
|---|---|---|---|
| 🔵 | `app_fake_gps/enable` | `std_msgs/Bool` | Turn fake-GPS injection on/off. |
| 🔵 | `app_fake_gps/reset` | `geographic_msgs/GeoPoint` | Reset the fake position to a given location. |
| 🔵 | `app_fake_gps/go_stop` | `std_msgs/Empty` | Stop fake-GPS motion. |
| 🔵 | `app_fake_gps/goto_position` | `geometry_msgs/Point` | Relative ENU move. |
| 🔵 | `app_fake_gps/goto_location` | `geographic_msgs/GeoPoint` | Absolute WGS84 goto. |
| 🔵 | `app_fake_gps/select_mavros_node` | `std_msgs/String` | Which mavros node's GPS-input to inject into. |

**What the fake-GPS app does with those commands.** `fake_gps_app_node.py`
(`src/nepi_apps/nepi_app_fake_gps/scripts/`) finds mavros by scanning the whole ROS
graph for any topic of type `mavros_msgs/State` ending in `/state` — no hardcoded node
name.

| | Topic | Type | Direction | Notes |
|---|---|---|---|---|
| 🔵 | `<mavros_node>/gps_input/gps_input` | `mavros_msgs/GPSINPUT` | fake_gps_app_node.py → mavros | The actual injection point. Uses `GPSINPUT` rather than the also-imported `HilGPS`, specifically because `GPSINPUT` carries a `yaw` field that `HilGPS` lacks. |
| 🔵 | `gps_fix` | `sensor_msgs/NavSatFix` | fake_gps_app_node.py (local) | The app's own idea of where it's placed the vehicle. |
| 🔵 | `odom` | `nav_msgs/Odometry` | fake_gps_app_node.py (local) | Local-frame companion to `gps_fix`. |
| 🔵 | `navpose` | `nepi_interfaces/NavPose` | fake_gps_app_node.py, via the same `NavPoseIF` core class every device uses | See [Shared framework](#shared-nepi-framework-rbxrobotif) for the standard navpose convention. |
| 🔵 | `status` | `NepiAppFakeGpsStatus` | fake_gps_app_node.py (local, latched) | App status. |

### AI-targeting test scaffold (bonus, port 9027)

A separate test rig, not part of normal flight: `ai_targeting_controller_ardupilot.py`
circles a target chair around the drone in Gazebo and pushes range/bearing readings out
over TCP port 9027.

```
port 9027, VM → consumer, ~5 Hz
{"type": "target", "target_name": "chair", "range_m": 8.5, "azimuth_deg": 12.0, "elevation_deg": -3.0, "detected": true, "stamp": 1723999999.1}
```

| | Topic | Type | Publisher | Notes |
|---|---|---|---|---|
| 🔵 | `app_ai_targeting/target_localizations` | `nepi_interfaces/Targets` | `sim_ai_targeting_bridge_script.py` | Republishes the port-9027 JSON as a real NEPI targeting message, for any downstream AI-targeting app/pipeline to consume like a live detector. |
| 🔵 | `app_ai_targeting/targeting_image` | `sensor_msgs/Image` | `sim_ai_targeting_bridge_script.py` | Plain passthrough of whatever the device's own `color_2d_image` or `idx/color_image` is currently showing. |

---

## Shared NEPI framework (RBXRobotIF)

Every driver above plugs into the same `RBXRobotIF` class (`src/nepi_api/device_if_rbx.py`).
This is the layer that's identical no matter which simulator is running underneath, and
it's what the RUI and the rest of NEPI actually talk to.

### Publishes

| | Topic | Type | Notes |
|---|---|---|---|
| 🔵 | `info` | `DeviceRBXInfo` | Latched, device identity/capabilities snapshot. |
| 🔵 | `status` | `DeviceRBXStatus` | Latched. This exact message type is what the newer generic sim-connector app scans the whole ROS graph for to auto-discover "simulator" devices. |
| 🔵 | `status_str` | `std_msgs/String` | Human-readable status. |

### Commands it listens for

One subscriber per command — the full control surface every RBX driver gets for free:

| | Topic | Type | Fields / purpose |
|---|---|---|---|
| 🔵 | `goto_position` | `GotoPosition` | `x_meters, y_meters, z_meters, yaw_deg` — relative move. |
| 🔵 | `goto_pose` | `GotoPose` | `roll_deg, pitch_deg, yaw_deg` — attitude only. |
| 🔵 | `goto_location` | `GotoLocation` | `lat, long, altitude_meters, yaw_deg` — absolute goto. |
| 🔵 | `go_home` / `go_stop` | `Empty` | Return to home / halt in place. |
| 🔵 | `set_home` / `set_home_current` | `GeoPoint` / `GotoLocation` | Define the home point. |
| 🔵 | `set_state` / `set_mode` | `Int32` | Index into the driver's own state/mode list. |
| 🔵 | `setup_action` / `go_action` | `Int32` | Index into driver-defined action lists — e.g. ArduPilot's `RESET_SIM` and `LAUNCH`. |
| 🔵 | `set_motor_control` | `MotorControl` | `motor_ind, speed_ratio` (0–1) — direct per-motor control. |
| 🔵 | `set_teleop_velocity` | `geometry_msgs/Twist` | Only `linear.x/y/z` and `angular.z` are read. |
| 🔵 | `set_goto_timeout` / `set_goto_error_bounds` | `UInt32` / `ErrorBounds` | Tune how goto success/failure is judged. |
| 🔵 | `set_image_topic` / `enable_image_overlay` | `String` / `Bool` | Pick the active camera feed and toggle status overlay. |
| 🔵 | `publish_status` / `publish_info` | `Empty` | Force an immediate re-publish. |

### Services

| | Service | Type | Purpose |
|---|---|---|---|
| 🟣 | `device_info_query` | `DeviceInfoQuery` | RUI / other NEPI clients ask "what device is this?" |
| 🟣 | `capabilities_query` | `RBXCapabilitiesQuery` | Reports `has_goto_position`, `has_camera`, `motor_count`, etc — how the RUI knows which controls to show. |

### NavPose — the standard position/orientation feed

Every RBX driver supplies a `getNavPoseCb`; `RBXRobotIF` hands it to a core-engine
`NPXDeviceIF`, which is what actually publishes it. This is the one place any other NEPI
app reads "where is this device right now" from — not the RBX status topic.

| | Topic | Type | Notes |
|---|---|---|---|
| 🔵 | `<device_name>/npx/navpose` | `nepi_interfaces/NavPose` | Position + orientation, standard across every NEPI device type (not just RBX). |
| 🔵 | `<device_name>/npx/navpose/status` | `NavPoseStatus` | Latched companion. |

### Image — the "enhanced" status-overlay feed

> **Correcting a stale comment:** a line in `device_if_rbx.py` refers to viewing "the
> `enhanced_2D_image` topic" in a browser — but that exact string appears nowhere else
> in the codebase as a real topic name. It's an informal comment, not the actual topic.

| | Topic | Type | Notes |
|---|---|---|---|
| 🔵 | `<device_name>/image` | `sensor_msgs/Image` | The real topic name, published by `ImageIF` — a status/overlay-annotated version of whatever camera feed is active, viewable live through NEPI's `web_video_server` relay in a browser. |
| 🔵 | `<device_name>/image/status` | `ImageStatus` | Latched companion. |

### Settings & Save Data — the other two standard sub-interfaces

| | Topic / Service | Type | Notes |
|---|---|---|---|
| 🔵 | `<device_name>/rbx/settings/status` | `SettingsStatus` | Latched. Current value of every driver-defined setting. |
| 🔵 | `<device_name>/rbx/settings/update_setting` | `Setting` | Change one setting. |
| 🔵 | `<device_name>/rbx/settings/update_settings` | `Settings` | Change several at once. |
| 🔵 | `<device_name>/rbx/settings/reset_settings` | `Empty` | Back to factory defaults. |
| 🟣 | `<device_name>/rbx/settings/capabilities_query` | `SettingsCapabilitiesQuery` | What settings this driver exposes. |
| 🔵 | `<node_namespace>/save_data/status` | `SaveDataStatus` | Latched. |
| 🔵 | `<node_namespace>/save_data/{save_data_enable, save_data_prefix, save_data_subfolder, save_data_utc, save_data_rate}` | `Bool` / `String` / `SaveDataRate` | Standard recording controls, plus `snapshot_trigger` and `reset_save_data` (both `Empty`). The same set is mirrored under the shared `<base>/save_data/*` namespace for "apply to every device at once." |

---

## Gazebo-internal plumbing

A handful of Gazebo's own built-in topics and services do work under the hood that
never crosses to the real device — teleporting models, spawning obstacles, reading
ground-truth pose.

| | Name | Kind / type | Used by | What for |
|---|---|---|---|---|
| ⚙️ | `/gazebo/set_model_state` | topic, `ModelState` | `sim_bridge_node.py`, `sim_bridge_multi_node.py`, `camera_rig_controller*.py`, `ai_targeting_controller_ardupilot.py` | Fire-and-forget teleport — RESET_SIM, and moving the free-floating camera rig to follow the robot each frame. |
| ⚙️ | `/gazebo/set_model_state` | service, `SetModelState` | `sim_connector_bridge_gazebo.py` | Same name, blocking-call form — used only by the newer generic connector. |
| ⚙️ | `/gazebo/model_states` | topic, `ModelStates` | `camera_rig_controller_ardupilot.py`, `ai_targeting_controller_ardupilot.py` | All model poses in the world — how the ArduPilot side tracks the drone's position, since it has no `/odom` topic of its own on the VM. |
| ⚙️ | `/gazebo/spawn_sdf_model` | service, `SpawnModel` | `sim_bridge_node.py`, `sim_connector_bridge_gazebo.py`, `ai_targeting_controller_ardupilot.py` | Spawn a model into a running world — obstacle course, target chair, camera-offset respawn. |
| ⚙️ | `/gazebo/delete_model` | service, `DeleteModel` | `sim_bridge_node.py`, `sim_connector_bridge_gazebo.py` | Remove a spawned model (obstacle-course toggle, respawn). |

---

## A newer, separate system: the generic sim connector

> **Not a replacement for the above.** `nepi_app_sim_connector` is a newer,
> simulator-agnostic connector meant to work with Gazebo, Webots, PyBullet, or WPILib
> through one generic protocol on a single port. It is explicitly documented in-repo as
> not to be run against the same Gazebo instance at the same time as the RBX_SIM driver
> described above. See `SIM_DEVICE_IF_CONTRACT.md` and `SIM_CONNECTOR_TESTING_GUIDE.md`
> for the full spec.

One always-listening app on the NEPI device (`sim_connector_app_node.py`) accepts a
connection from whatever small bridge script the simulator side is running, on **port
9030**:

```
bridge → app (telemetry, goto results, images)
{"x_m":1.0, "y_m":2.0, "z_m":0.0, "roll_deg":0, "pitch_deg":0, "yaw_deg":45}   // bare NavPose line, all fields optional
{"type":"image", "topic_name":"gazebo_rover/robot_camera", "data":"<base64 jpeg>"}
{"type":"goto_result", "success":true}

app → bridge (commands)
{"type":"motor_control", "motor_ind":0, "speed_ratio":0.5}
{"type":"goto_position", "x_meters":1, "y_meters":2, "yaw_deg":90}
{"type":"go_home"}  /  {"type":"go_stop"}
```

On the Gazebo side, `sim_connector_bridge_gazebo.py` reads `/rover/odom` and the camera
topics to fill this protocol, and runs its own re-implemented goto controller to turn
commands back into `/rover/cmd_vel`.

> **Two different reset spellings — don't conflate them.** Every bridge on this
> port-9030 protocol checks for the literal action string `"RESET"` (matching a
> `ground_robot_2_wheel` setup-actions list). The separate RBX_SIM / RBX_WEBOTS /
> RBX_WEBOTS_QUADCOPTER / ArduPilot driver family uses `"RESET_SIM"` instead — an older,
> unrelated naming convention. They are two different systems that happen to do a
> similar thing.

### The other simulators speaking this protocol

| Bridge script | Simulator mechanism | Role on :9030 | Reset behavior |
|---|---|---|---|
| `sim_connector_bridge_gazebo.py` | Reads `/rover/odom` + camera topics; drives `/rover/cmd_vel` via its own goto controller. | client | `/gazebo/set_model_state` service call |
| `sim_connector_bridge_webots.py` | Webots **Robot** controller — reads `wheel1`–`4`, `gps`, `imu`, `camera` devices directly via the Webots controller API. | client | Honest no-op — a plain `Robot` node can't move itself in Webots. |
| `sim_connector_bridge_pybullet.py` | Pure-Python PyBullet, headless (`DIRECT` mode, no GUI/external process) — drives an `r2d2.urdf` robot by directly setting its base velocity each tick (force/torque was tried first and abandoned — the model's own wheel-joint friction fought it). | client | Real teleport via `resetBasePositionAndOrientation` — PyBullet has no Supervisor-style restriction. |
| `wpilib/robot.py` + `physics.py` | A real WPILib/`robotpy` `TimedRobot`, run through `pyfrc`'s own simulator — not ROS at all. See below. | client | Handled by the physics engine via `reset_requested`, a flag in the shared in-process state. |

### The odd one out: WPILib

Genuinely different from every other bridge here — no ROS, no Gazebo/Webots-style
simulator, and yet it dials into the exact same port-9030 protocol as the rest. It's a
real FRC robotics stack: `robot.py` is a `wpilib.TimedRobot` driving two real
`PWMVictorSPX` motor controllers (left on PWM channel 0, right on channel 1), run
headless via `pyfrc`'s simulator (`python3 robot.py sim`) with `DriverStationSim` forced
into an enabled autonomous state so periodic commands aren't safety-zeroed for lack of a
joystick.

`physics.py` (a `pyfrc.physics.PhysicsEngine`) reads the simulated PWM output every
tick, integrates a 2D pose with pyfrc's own two-motor drivetrain kinematics, and writes
the result into `shared_state.py` — a deliberately simple design: a module-level object
guarded by one `threading.Lock`, not NetworkTables or a queue, because a single-process
bridge doesn't need pyfrc's usual cross-process convention. `robot.py`'s own network
thread reads that shared state and is what actually dials out to :9030, implementing
`motor_control`/`goto_position`/`go_home`/`go_stop`/`setup_action` — by design, with no
camera or environment-control support. (Pinned to `robotpy 2022.4.8` specifically, due
to a C++20/GCC toolchain mismatch on the dev VM.)

---

## Port map

Every port here is forwarded end-to-end by one reverse SSH tunnel (`autossh`) between
the dev VM and the NEPI device — nothing on this list is reachable any other way.

| Port | Purpose |
|---|---|
| 9021 | `gz_reset_listener` — pose-only world reset for ArduPilot's RESET_SIM |
| 9022 | rover1 heartbeat listener |
| 9023 | rover1 command / telemetry / image bridge |
| 9024 | rover2 heartbeat listener (multi-robot only) |
| 9025 | rover2 command / telemetry / image bridge (multi-robot only) |
| 9026 | ArduPilot camera-only bridge |
| 9027 | AI-targeting test-scaffold bridge |
| 9028 | `sim_launch_listener` — triggers a full SITL/Gazebo stack launch |
| 9030 | generic sim-connector app (separate newer system, above) |
| 9041 | RBX_WEBOTS rover heartbeat listener |
| 9042 | RBX_WEBOTS_QUADCOPTER heartbeat listener |
| 9046 | RBX_WEBOTS rover command/telemetry/image bridge |
| 9047 | RBX_WEBOTS_QUADCOPTER command/telemetry/image bridge |

Webots' 9041/9046/9042/9047 were chosen deliberately to stay clear of the 902x block.

---

## Since the first pass

A second full pass answered every open question the first version of this page had
flagged as unconfirmed. Most of it is folded straight into the sections above — three
more driver variants, the framework's navpose/image/settings/save-data topics,
fake-GPS's real mavros injection point, and the AI-targeting relay. Two things are worth
calling out specifically, because the first version's guesses would have been wrong:

- **The RBX status-overlay image topic is `<device_name>/image`, not
  `enhanced_2D_image`** — that name only ever existed as an informal code comment, never
  as an actual topic. See [Shared framework](#shared-nepi-framework-rbxrobotif).
- **"Reset" is spelled two different ways depending on which system you're in** — the
  generic connector's bridges use `"RESET"`; the RBX_SIM/RBX_WEBOTS/
  RBX_WEBOTS_QUADCOPTER/ArduPilot driver family uses `"RESET_SIM"`. See
  [the generic connector section](#a-newer-separate-system-the-generic-sim-connector).
