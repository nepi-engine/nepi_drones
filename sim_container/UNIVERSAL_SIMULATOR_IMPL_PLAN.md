# UNIVERSAL_SIMULATOR_BRIDGE_IMPLEMENTATION_PLAN.md

## Document Overview

This document is a modular, step-by-step engineering blueprint for implementing a **Universal Gazebo Simulator Bridge & RBX Driver** for the NEPI Engine ecosystem.

The goal is to let any robot — ground rover, drone, robotic arm, servo assembly, or FIRST Robotics vehicle — be simulated in Gazebo and controlled seamlessly by NEPI with zero physical hardware attached. The plan is split into self-contained Phases and Steps so an AI assistant (or human developer) can execute and verify one component at a time without losing context.

**Environment note (added 2026-07-21):** the original draft's code samples, file paths, and network topology assumed a generic same-LAN, shared-Docker-network deployment. Section 1a below corrects that to match this project's actual dev setup (see `docs/SIMULATOR_DEV_GUIDE.md`, `scripts/nepi_sitl_dev_env.sh`) — read it before starting Phase 1. The rest of the document has been reformatted for readability but keeps the original's structure and intent.

## 1. System Architecture & Preliminary Setup

### High-Level Architecture

```
+-------------------------------------------------------+          +-------------------------------------------------------+
|                 NEPI ENGINE (remote device)            |          |             SIMULATOR (dev VM, this machine)           |
|                                                         |          |                                                         |
|  +---------------------------------------------------+ |          |  +---------------------------------------------------+ |
|  | rbx_sim_discovery.py                               | |          |  | sim_bridge_node.py                                 | |
|  | - Probes the TCP bridge port (not a shared ROS     | |          |  | - ROS node on the VM's own local roscore           | |
|  |   master -- see 1a)                                | |          |  | - Publishes /sim/heartbeat                         | |
|  | - Launches rbx_sim_node with DEVICE_DICT           | |          |  | - Relays motor/vel commands to Gazebo joints       | |
|  +---------------------------------------------------+ |          |  +---------------------------------------------------+ |
|                            |                            |          |                            |                            |
|                            v                            |          |                            v                            |
|  +---------------------------------------------------+ |  === 1. Motion Cmds ==>  |  +---------------------------------------------------+ |
|  | rbx_sim_node.py                                    | |  (bridge TCP port)       |  | Gazebo Physics Engine                              | |
|  | - Consumes DEVICE_DICT                             | |          |  | - Differential Drive / Joint Plugins               | |
|  | - Maps UI/App control topics to bridge messages    | |  <== 2. Telemetry ===    |  | - Camera / Sensor Plugins                          | |
|  | - Relays /odom, /tf, and camera streams to NEPI    | |  (bridge TCP port)       |  +---------------------------------------------------+ |
|  +---------------------------------------------------+ |                          |                                                         |
|                                                         |  <== 3. Camera Stream == |                                                         |
|                                                         |  (bridge TCP port)       |                                                         |
+-------------------------------------------------------+          +-------------------------------------------------------+
```

### 1a. Corrected network topology (read this before Phase 1)

The original draft assumed NEPI and the simulator share a single ROS master over the same LAN (`ROS_MASTER_URI=http://<SIMULATOR_IP>:11311`, Docker `--net=host`, "Cross-Container ROS Communication" via a shared `ROS_IP`/`ROS_MASTER_URI`). **That does not match this project's actual environment** and must not be built against:

- The real NEPI device and this dev VM are two separate machines with two separate ROS masters, reachable only through a reverse SSH tunnel that forwards raw TCP ports (`nepi_tunnel` in `scripts/nepi_sitl_dev_env.sh`). Neither machine can see the other's ROS graph directly, and there is no shared Docker network between them.
- This is exactly the constraint the existing ArduPilot SITL bridge already solves, and it's the pattern to reuse: MAVLink is a plain socket protocol, so it tunnels trivially over a simple `-R` port forward, and `mavros` runs *on the device side*, translating the tunneled MAVLink stream into ROS topics on the device's own master. There is no shared ROS graph anywhere in the existing working system.
- A generic Gazebo robot's `gazebo_ros_diff_drive` plugin speaks ROS topics natively, not a portable wire protocol, so it cannot be tunneled the way MAVLink is — ROS's dynamic XML-RPC port negotiation does not survive a simple SSH port forward. The correct approach is a small custom TCP bridge (same idea as `scripts/gz_reset_listener.py`, just bidirectional: motion commands one way, odometry/telemetry the other) rather than attempting to share a ROS master across machines.
- Consequence: this VM needs its own local `roscore` for `sim_bridge_node.py` and Gazebo's ROS-integrated plugins to run against. This is new — the existing ArduPilot Gazebo setup never needed one, since ArduPilot's Gazebo plugin speaks MAVLink/FDM directly, not ROS.

**Canonical source directories (corrected for this machine — replaces every `/home/production/nepi_engine_ws/...` reference below):**
- RBX driver work (sandbox, edit here first, never `src/nepi_drivers` directly — see the established sandbox-vs-production rule): `/home/suraj/nepi_engine_ws/nepi_drones/src/nepi_drivers/rbx_drivers/`
- Simulator-side assets (Gazebo models/worlds/bridge script, dev VM only): `/home/suraj/nepi_engine_ws/nepi_drones/sim_container/`
- "Running build/deploy populates runtime copies under `/opt/nepi/nepi_engine/lib/...`" (from Section "Understanding Things" below) is still correct in spirit, but the actual deployed path is flat: `/opt/nepi/nepi_engine/lib/nepi_drivers/*.py` on the remote device, not a nested `rbx_drivers/` subfolder — that subfolder layout is a sandbox/production-repo organizing convention only.

### Recommended execution order

Build and verify in phases, standalone before cross-machine:

1. **Phase 1** — Gazebo model/world + `sim_bridge_node.py`, verified entirely on this VM (`rostopic pub`/`echo` only). No remote device, no driver, no tunnel involved yet.
2. **Phase 2/3** — the actual `rbx_sim` driver (3-file pattern) on the remote device, plus the custom TCP bridge connecting it to Phase 1's `sim_bridge_node.py` across the existing reverse tunnel.
3. **Phase 4/5** — multi-robot namespacing and the regression checklist, once a single simulated robot works end to end.

Each phase should be its own implementation pass, not one continuous run — this mirrors how the ArduPilot SITL integration was actually built (Gazebo+SITL proven first, NEPI wired in afterward as a separate step), and keeps any one pass small enough to debug.

## Phase 1: Gazebo Simulation Bridge (dev VM only)

### Objective

Create a standalone ROS + Gazebo setup on this VM that loads a robot URDF/SDF model, exposes control/sensor topics, and publishes a heartbeat — verified in isolation before any NEPI/RBX wiring.

### Step 1.1: Define Gazebo Robot Model (URDF/SDF)

Create a flexible model representing a differential-drive rover.

- **File location:** `nepi_drones/sim_container/models/generic_rover/model.urdf` (or `.sdf` — match whatever the existing `~/ardupilot_gazebo/models` convention uses for this VM's installed Gazebo Classic 11 version; confirm in Phase 1's explore step rather than assuming)
- **Key requirements:**
  - Base link: `base_link`
  - Wheels / joints: `left_wheel_joint`, `right_wheel_joint`
  - Camera link: `camera_link`, placed at a relative offset (x, y, z) = (0.2, 0.0, 0.5)
  - Camera plugin: `libgazebo_ros_camera.so`, publishing to `/rover/camera/image_raw`
  - Drive plugin: `libgazebo_ros_diff_drive.so`, subscribing to `/rover/cmd_vel` and publishing `/rover/odom`

### Step 1.2: Implement sim_bridge_node.py

A plain ROS node (runs on this VM's own local `roscore` — see 1a) acting as the simulator-side entry point.

- **File location:** `nepi_drones/sim_container/scripts/sim_bridge_node.py`
- **Responsibilities:**
  1. Broadcast `/sim/heartbeat` at 1 Hz (`std_msgs/Header`).
  2. Subscribe to `/nepi/sim/cmd_vel` (`geometry_msgs/Twist`) and pass through to Gazebo's `/rover/cmd_vel`.
  3. Load robot-specific configuration parameters if provided by NEPI.

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Twist

class SimBridgeNode:
    def __init__(self):
        rospy.init_node("sim_bridge_node")

        self.heartbeat_pub = rospy.Publisher("/sim/heartbeat", Header, queue_size=1)
        self.cmd_sub = rospy.Subscriber("/nepi/sim/cmd_vel", Twist, self.cmdCb)
        self.gazebo_cmd_pub = rospy.Publisher("/rover/cmd_vel", Twist, queue_size=1)

        rospy.Timer(rospy.Duration(1.0), self.publishHeartbeatCb)
        rospy.loginfo("Simulator Bridge Node initialized.")

    def publishHeartbeatCb(self, event):
        hdr = Header()
        hdr.stamp = rospy.Time.now()
        hdr.frame_id = "gazebo_simulation"
        self.heartbeat_pub.publish(hdr)

    def cmdCb(self, msg):
        self.gazebo_cmd_pub.publish(msg)

if __name__ == "__main__":
    node = SimBridgeNode()
    rospy.spin()
```

(Naming note: callbacks use the `...Cb` convention used elsewhere in this codebase — see the naming rules in `nepi-prompt-suraj.md` / top-level `CLAUDE.md`.)

### Phase 1 Verification & Test Cases

- **Test Case 1.1 (Heartbeat check):**
  - Run `sim_bridge_node.py`.
  - Execute: `rostopic echo /sim/heartbeat`
  - *Pass criteria:* receives valid header timestamps at 1 Hz.
- **Test Case 1.2 (Motion forwarding):**
  - Publish a test command: `rostopic pub -1 /nepi/sim/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"`
  - *Pass criteria:* the Gazebo robot model moves forward in the simulation renderer.

## Phase 2: NEPI Driver Configuration & Discovery (rbx_sim)

### Objective

Implement the NEPI driver descriptor (`.yaml`) and discovery class (`.py`) following the existing 3-file pattern (see `rbx_ardupilot_params.yaml` / `rbx_ardupilot_discovery.py` as the reference).

### Step 2.1: Create rbx_sim_params.yaml

Register the driver descriptor and expose UI options.

- **File location:** `nepi_drones/src/nepi_drivers/rbx_drivers/rbx_sim_params.yaml`

```yaml
DRIVER_DICT:
  package_name: "rbx_drivers"
  discovery_class_name: "SimDiscovery"
  node_class_name: "SimNode"

DISCOVERY_DICT:
  process: "CALL"
  OPTIONS:
    connection:
      type: "string"
      options:
        - SERIAL
        - SIMULATOR
      value: "SIMULATOR"
    bridge_host:
      type: "string"
      value: "127.0.0.1"
    bridge_port:
      type: "string"
      value: "9022"
    robot_config:
      type: "string"
      value: "generic_rover.yaml"
```

**Crucial guardrail:** `"SIMULATOR"` MUST be explicitly listed under `connection.options`, or NEPI's setting-update mechanism (`check_valid_setting`) will reject the UI selection and silently revert it back to `SERIAL` (see the Phase 5 regression checklist).

**Corrected from the original draft:** the discovery options are `bridge_host`/`bridge_port` (the custom TCP bridge's endpoint on the dev VM, reached through the existing reverse tunnel) rather than `sim_ip` pointed at a shared ROS master port — see Section 1a.

### Step 2.2: Implement rbx_sim_discovery.py

Implement the `CALL` process class that polls for the bridge's TCP endpoint (mirrors `checkForTcpDevice` in `rbx_ardupilot_discovery.py` — reuse that pattern rather than writing a new probe from scratch).

- **File location:** `nepi_drones/src/nepi_drivers/rbx_drivers/rbx_sim_discovery.py`

```python
#!/usr/bin/env python3
import rospy
import socket

class SimDiscovery:
    def __init__(self):
        self.active_paths_list = []
        self.active_devices_dict = {}

    def checkBridgeReachable(self, ip, port):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.5)
        try:
            s.connect((ip, port))
            s.close()
            return True
        except Exception:
            return False

    def discoveryFunction(self, available_paths_list, active_paths_list, base_namespace, drv_dict, retry_enabled):
        options = drv_dict['DISCOVERY_DICT']['OPTIONS']
        conn_type = options['connection']['value']
        bridge_host = options['bridge_host']['value']
        bridge_port = int(options['bridge_port']['value'])
        config_name = options['robot_config']['value']

        if conn_type == "SIMULATOR":
            target_path = "SIM_" + bridge_host + "_" + str(bridge_port)

            if target_path not in self.active_paths_list:
                if self.checkBridgeReachable(bridge_host, bridge_port):
                    rospy.loginfo("[SimDiscovery] Bridge detected at " + bridge_host + ":" + str(bridge_port) + ". Populating DEVICE_DICT...")

                    drv_dict['DEVICE_DICT'] = {
                        'device_path': target_path,
                        'bridge_host': bridge_host,
                        'bridge_port': bridge_port,
                        'robot_config': config_name,
                    }
                    self.active_paths_list.append(target_path)

        return self.active_paths_list
```

(Corrected from the original draft: probes the custom TCP bridge port, not a shared ROS master port 11311 — see Section 1a. Path naming and the "not already active" guard mirror `rbx_ardupilot_discovery.py`'s `SITL_<ip>_<port>` convention and its `dont_retry_list` handling, which should be reused rather than reinvented — see that file for the full pattern including retry/backoff.)

### Phase 2 Verification & Test Cases

- **Test Case 2.1 (UI option validation):**
  - Load `rbx_sim_params.yaml` into the NEPI drivers manager.
  - Select `SIMULATOR` from the RUI dropdown.
  - *Pass criteria:* selection persists without reverting back to `SERIAL`.
- **Test Case 2.2 (Probe detection):**
  - Start the simulator bridge on the dev VM.
  - Monitor the drivers manager log.
  - *Pass criteria:* discovery logs `[SimDiscovery] Bridge detected at ...` and populates `DEVICE_DICT`.

## Phase 3: NEPI Device Node Logic (rbx_sim_node.py)

### Objective

Implement the RBX device node that attaches to the discovered bridge and exposes standard NEPI motion and sensor interfaces via `RBXRobotIF` — the same interface `rbx_ardupilot_node.py` implements. That file is the reference for the full callback surface required (states, modes, setup actions, goto functions, nav/pose callback, etc.) — a generic rover has no onboard autopilot, so `goto_position`-style autonomy needs a simple closed-loop controller implemented in this driver itself (publish velocity commands at a steady rate — 10-50 Hz — toward the target until within tolerance, the same "must be re-published continuously" pattern noted in `docs/SIMULATOR_DEV_GUIDE.md` for ArduPilot).

- **File location:** `nepi_drones/src/nepi_drivers/rbx_drivers/rbx_sim_node.py`

This step is intentionally left as a design task for the Phase 3 implementation pass rather than a fixed code sample here — it depends on decisions made during Phase 2 (exact bridge wire protocol) and should follow `rbx_ardupilot_node.py`'s actual structure (`RBX_STATES`, `RBX_MODES`, `RBX_SETUP_ACTIONS`, `RBX_GO_ACTIONS` paired with `..._FUNCTIONS` lists, and the full `RBXRobotIF(...)` constructor call) rather than the simplified single-topic-passthrough sketch in earlier drafts of this document.

### Phase 3 Verification & Test Cases

- **Test Case 3.1 (Command end-to-end):**
  - Publish to the NEPI RBX velocity/goto interface.
  - *Pass criteria:* motion is relayed across the bridge and the vehicle moves in Gazebo.
- **Test Case 3.2 (Telemetry echo):**
  - Check the RBX node's nav/pose output topic.
  - *Pass criteria:* real-time odometry coordinates (x, y, z) stream into NEPI.
- **Test Case 3.3 (Image stream pass-through):**
  - Open the NEPI RUI Image Viewer app.
  - *Pass criteria:* the Gazebo camera render appears in the web interface.

## Phase 4: Dynamic Configuration & Multi-Robot Deployment

### Objective

Allow loading arbitrary robot profiles from NEPI storage and support running multiple simulated robots side by side.

### Step 4.1: Define Robot Configuration Schema

- **File location:** `/mnt/nepi_storage/config/robots/generic_rover.yaml` (on the remote NEPI device)

```yaml
robot_name: "rover_01"
type: "differential_drive"
actuators:
  - name: "left_motor"
    topic: "/rover/left_wheel_controller/command"
  - name: "right_motor"
    topic: "/rover/right_wheel_controller/command"
sensors:
  camera:
    topic: "/rover/camera/image_raw"
    frame_id: "camera_link"
```

### Step 4.2: Support Multi-Robot Namespacing

To run two robots side by side (`robot_1`, `robot_2`), discovery and node topics must be prefixed by the `base_namespace` the drivers manager passes in.

- **Topic pattern:**
  - Robot 1: `/nepi/device1/...`
  - Robot 2: `/nepi/device2/...`
- On the simulator side, each robot needs its own bridge port (e.g. 9022, 9023, ...) since the bridge is per-robot, not shared — update the reverse tunnel's forwarded-port list accordingly for each additional robot.

### Phase 4 Verification & Test Cases

- **Test Case 4.1 (Side-by-side multi-robot test):**
  - Spawn `rover_01` and `rover_02` in Gazebo with separate namespaces and separate bridge ports.
  - Launch two `rbx_sim` instances in NEPI under `/nepi/device1` and `/nepi/device2`.
  - *Pass criteria:* commanding `/nepi/device1/...` moves Robot 1 only, without affecting Robot 2.

## Phase 5: Regression & Edge Case Checklist

| Scenario / edge case | Cause / mechanism | Solution / safeguard |
|---|---|---|
| Simulator boots after NEPI | NEPI driver starts before the Gazebo bridge is ready | Discovery's retry loop (see `rbx_ardupilot_discovery.py`'s `dont_retry_list` handling) polls continuously and self-heals once the bridge comes up — confirmed working for the ArduPilot case this same way |
| NEPI device restarts while the simulator is already running | The reverse SSH tunnel dies with the device's sshd; a plain `ssh -N` tunnel does not reconnect | Use `autossh` for the tunnel (already done in `scripts/nepi_sitl_dev_env.sh`'s `nepi_tunnel`), not a bare `ssh -N` |
| Deployed driver files revert after an on-device rebuild or power cycle | The device's stack runs from a committed Docker image; a live-container-only edit (`scpn` without `nepicommit`) is lost on the next restart | Always follow any on-device fix with `nepicommit` (which also restarts) and verify via `md5sum` that the fix survived — see the project memory on this recurring failure mode |
| Option reverts on UI selection | `SIMULATOR` missing from `rbx_sim_params.yaml`'s `connection.options` | Ensure `connection.options` includes `SIMULATOR` in the params YAML |
| High-latency camera stream | Uncompressed raw image stream over the bridge | Use `sensor_msgs/CompressedImage` for the bridged connection |
| Code changes not reloading | The discovery class (`process: "CALL"`) is imported and cached in memory by the drivers manager | Disable and re-enable the driver in the RUI (or via `update_driver_state`) to force a module reload (`del sys.modules` + re-import) |

## Summary Checklist for Implementer

- [ ] **Phase 1:** build the Gazebo model + `sim_bridge_node.py` on this dev VM and verify `/sim/heartbeat` + motion forwarding standalone (no NEPI/remote device involved).
- [ ] **Phase 2:** implement `rbx_sim_params.yaml` + `rbx_sim_discovery.py` in `nepi_drones/src/nepi_drivers/rbx_drivers/`, probing the custom TCP bridge (not a shared ROS master).
- [ ] **Phase 3:** implement `rbx_sim_node.py`, following `rbx_ardupilot_node.py`'s full `RBXRobotIF` integration pattern, including a self-implemented closed-loop position controller.
- [ ] **Phase 4:** add the robot config schema in NEPI storage and verify side-by-side multi-robot operation.
- [ ] **Phase 5:** validate self-healing reconnect and the full edge-case checklist.

Each phase should be verified before starting the next — see "Recommended execution order" above.

## Reference: Concepts and Conventions

Background detail supporting the phases above — read on demand rather than up front.

### 1. NEPI's 3-file driver pattern

- Every NEPI driver consists of a parameters file, a discovery file, and a device node file:
  - `rbx_[driver]_params.yaml` — static descriptor defining UI options, driver package names, and discovery class declarations.
  - `rbx_[driver]_discovery.py` — instantiated by the drivers manager to probe device reachability, launch the ROS node, and populate `DEVICE_DICT`.
  - `rbx_[driver]_node.py` — core device node handling runtime logic; stays connection-agnostic.
- **Connection agnosticism:** device nodes don't care whether they're talking to physical hardware or a simulation — they connect to whatever transport/namespace `DEVICE_DICT` provides.
- **Drivers manager polling loop:** `drivers_mgr.py` runs a discovery cycle (roughly 1-3s, not a strict 1 Hz) via its `discoveryFunction` call.
- **Discovery argument contract:** `discoveryFunction` receives `available_paths_list`, `active_paths_list`, `base_namespace`, `drv_dict`, `retry_enabled`; it mutates and returns `active_paths_list`.
- **Option validation safeguard:** any UI-selectable value (e.g. `SIMULATOR`, `SITL`) must be explicitly listed under `connection.options` in the params YAML, or the drivers manager's setting validation silently rejects and reverts the UI change.
- **Code caching / module reloading:** `process: "CALL"` discovery classes are imported and cached in memory. Code changes require disabling and re-enabling the driver (forces `del sys.modules` + re-import) — a plain drivers-manager restart also works but is heavier-handed.
- **Canonical source pathing:** see Section 1a above — this repo's sandbox path replaces the original draft's `/home/production/...` path.

### 2. Networking & environment architecture

See Section 1a for the authoritative version of this for our environment. The original draft's assumptions (Docker `--net=host`, shared `ROS_MASTER_URI`/`ROS_IP`) are kept here only as a record of what NOT to build against.

- **Self-healing probes:** using a plain TCP connect probe inside discovery (`checkForTcpDevice`-style) lets NEPI self-heal and connect regardless of whether NEPI or the simulator starts first — this part of the original design is correct and already proven working for ArduPilot SITL this session.

### 3. Simulator architecture & Gazebo fundamentals

- **Drone vs. non-drone simulation:** drones rely on a flight-controller binary (e.g. ArduPilot SITL) sitting between Gazebo and ROS to handle flight dynamics via MAVLink. Non-drones (rovers, servos, arms, FIRST-style robots) rely directly on Gazebo physics plugins (`gazebo_ros_control`, `diff_drive_controller`, etc.) with no such intermediary — this is why the bridge design differs from the ArduPilot case (see Section 1a).
- **Robot definition files (URDF/SDF):** describe link geometry, joint constraints, motor limits, wheel dimensions, and visual meshes.
- **Gazebo sensor plugins:** camera feeds are generated by `libgazebo_ros_camera.so` and published directly as ROS `sensor_msgs/Image` topics.
- **Offset/third-person cameras:** defined in URDF/SDF by attaching a camera frame to a fixed link at a specific (x, y, z) offset.

### 4. Universal interface specifications (NEPI <-> simulator)

- **Liveness/heartbeat:** `/sim/heartbeat` (`std_msgs/Header`), sent at 1 Hz by the simulator bridge to prove to NEPI discovery that the simulation is alive.
- **Control (NEPI -> simulator):**
  - Velocity: `/nepi/sim/cmd_vel` (`geometry_msgs/Twist` — linear x/y/z, angular roll/pitch/yaw).
  - Actuator/joint: `/nepi/sim/joint_states` (`sensor_msgs/JointState`, or individual servo-channel command topics for arms/custom mechanisms).
- **Navigation/position (simulator -> NEPI):**
  - Odometry: `/rover/odom` (`nav_msgs/Odometry` — 3D position, orientation quaternion, linear/angular velocity).
  - Transforms: `/tf` / `tf2_msgs/TFMessage` (relative positions between world, `base_link`, and sensor frames).
- **Vision (simulator -> NEPI):**
  - Image stream: `/rover/camera/image_raw` (`sensor_msgs/Image`, or `sensor_msgs/CompressedImage` for the bridged/remote case — see the Phase 5 checklist).
  - Camera info: `/rover/camera/camera_info` (`sensor_msgs/CameraInfo`).

Note: these topic names describe the *simulator-local* ROS graph on the dev VM (Phase 1). They do not cross the machine boundary directly — Phase 2/3's custom TCP bridge is what carries the equivalent data across to the remote device's own ROS graph (see Section 1a).

### 5. Configuration storage & multi-robot deployment

- Robot configuration files live in persistent NEPI storage (`/mnt/nepi_storage/config/robots/`).
- The RBX node parses the active robot configuration file to dynamically map motor publisher topics to joints/wheels.
- Multi-robot namespacing: all control/telemetry topics are prefixed with the discovery `base_namespace` (e.g. `/nepi/device1/` vs. `/nepi/device2/`).

### 6. Testing & acceptance milestones

1. **Standalone simulator:** run the Gazebo bridge independently and verify `/sim/heartbeat` publishes continuously.
2. **UI integration:** verify the `SIMULATOR` option appears in the NEPI driver settings UI and persists when selected.
3. **Discovery verification:** confirm `rbx_sim_discovery.py` detects the bridge and launches `rbx_sim_node.py` with `DEVICE_DICT` populated.
4. **Motion command loop:** send velocity commands from NEPI and verify the simulated robot moves inside Gazebo.
5. **Telemetry loop:** confirm NEPI's nav/pose output streams valid position coordinates back from Gazebo.
6. **Video stream verification:** open the NEPI RUI image viewer and confirm the rendered camera view streams cleanly.
