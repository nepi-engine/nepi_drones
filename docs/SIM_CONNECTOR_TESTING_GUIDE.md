# Testing Your Robot/Simulator with `nepi_app_sim_connector`

A practical guide for connecting a simulator to NEPI and testing it, aimed
at anyone with a Linux/Ubuntu box (a real machine or a VM) who wants to
either try the sim connector against the reference Gazebo setup that
already exists in this repo, or point it at their own simulator. Gazebo is
the default/reference case; the app itself has no idea what simulator is on
the other end — it only speaks a small JSON-lines protocol (see
`demo_bridge_client.py` for a from-scratch reference implementation with
zero ROS/NEPI dependencies).

## How the pieces fit together

```
Your simulator (Gazebo, or anything else)
        │  raw TCP, newline-delimited JSON
        ▼
sim_bridge_node.py  (or your own bridge script speaking the same protocol)
        │  ROS topics, on the simulator-side machine's own roscore
        ▼
        …TCP socket, port 9030 by default…
        ▼
nepi_app_sim_connector  (SimDeviceIF)  — runs on the NEPI device
        │  standard NEPI RBX-style ROS topics, under .../sim/
        ▼
RUI, scripts, other NEPI nodes
```

The NEPI device and the simulator do **not** share a ROS master — that's
the whole point of the bridge. If you're running the simulator on the same
machine as the NEPI device (or a VM you can reach), point the app at
`localhost`/that machine's IP and the bridge port; if it's on a separate dev
box, you need a way for the device to reach that port (a reverse SSH tunnel
is the pattern already used elsewhere in this repo for ArduPilot SITL — see
`sim_container/scripts/nepi_sitl_dev_env.sh`'s `nepi_tunnel` function).

## Quick start: the reference Gazebo rover, on your own Linux/Ubuntu box

This repo already ships a complete, working Gazebo rover world + bridge —
the same one used to verify `rbx_gazebo_node.py` and this app during
development. If you have (or are willing to install) Gazebo, this is the
fastest way to see the whole pipeline work with zero simulator-side coding.

**One-time setup on your Linux/Ubuntu machine:**

```bash
# Requires: gazebo11, ros-noetic-ros-base, ros-noetic-gazebo-ros-pkgs
sudo apt install gazebo11 ros-noetic-ros-base ros-noetic-gazebo-ros-pkgs

# Add to ~/.bashrc (adjust the path to wherever you cloned nepi_drones):
source /path/to/nepi_drones/sim_container/scripts/sim_rover_dev_env.sh
```

**Every time you want to test:**

```bash
sim_rover_gazebo
```

This one command starts a local `roscore` (if one isn't already running),
launches Gazebo with `generic_rover.world`, starts the heartbeat listener
(port 9022) and the command/telemetry bridge (port 9023, the JSON-lines
protocol `sim_bridge_node.py` speaks), and a camera-follow controller. Leave
it running in its own terminal; `Ctrl-C` tears everything down cleanly.

**Point the sim connector app at it:** if the NEPI device is on the same
machine, `localhost`; otherwise whatever address/tunnel reaches this
machine's port 9023 (see "How the pieces fit together" above; this rover
bridge intentionally uses a *different* port, 9023, than the sim connector
app's own default listen port, 9030 — the app's `listen_port` in
`sim_connector_app_params.yaml` is what the app itself listens on for a
bridge to *dial into*, so which side initiates the connection matters:
check your specific bridge script's own docstring for whether it dials out
or waits for a connection, and match the app's role accordingly).

**Reset the world or drive it directly, bypassing the app** (useful for
confirming the simulator itself works before involving NEPI at all):
```bash
move 10x              # drive forward 10m
move 45yaw             # turn in place 45 degrees
stop                   # zero velocity immediately
testcommands           # full command list with more examples
```

**Close it when done** — this is a heavy process (Gazebo GUI + physics),
and leaving it running eats CPU indefinitely on a shared machine:
```bash
# Ctrl-C in the sim_rover_gazebo terminal, or:
pkill -x gzclient; pkill -x gzserver; pkill -f sim_heartbeat_listener.py; pkill -f sim_bridge_node.py
```

## Selecting a robot config

The app doesn't know what kind of robot is connected until you tell it — a
`SimDeviceIF` starts in the capability-empty `default` config (every control
hidden) until you select one. This is deliberate: the RUI has nothing to
render incorrectly before a real robot config is chosen.

```bash
rostopic pub -1 /nepi/device1/app_sim_connector/sim/select_robot_config std_msgs/String "data: 'ground_robot_2_wheel'"
```

Available configs (from `sim_connector_app_params.yaml`'s `robot_configs`;
add your own there if none of these match your robot):

| Config | Wheels/Motors | Goto surfaces | Setup actions | Camera views | Environment control |
|---|---|---|---|---|---|
| `default` | 0/0 | none | none | none | no |
| `ground_robot_2_wheel` | 2/2 | position | RESET, RETURN_HOME | SCENE_CAMERA, ROBOT_CAMERA | yes |
| `ground_robot_4_wheel` | 4/4 | position | RESET, RETURN_HOME | SCENE_CAMERA, ROBOT_CAMERA | yes |
| `stage_ground_robot` | 2/2 | position | *(none — Stage has no reset)* | none | no |
| `wpilib_ground_robot` | 2/2 | position | RESET, RETURN_HOME | none | no |
| `flight_robot_4_motor` | 0/4 | position, pose, location | TAKEOFF, LAUNCH | SCENE_CAMERA, ROBOT_CAMERA | no |

## Test commands by capability

All commands below assume the app's node name is `app_sim_connector` under
`/nepi/device1` — adjust if yours differs. `setup_action`/`go_action` take
the **index** into that config's own list above (e.g. `ground_robot_2_wheel`:
0 = RESET, 1 = RETURN_HOME).

**Position control** (any config with `has_goto_position`):
```bash
rostopic pub -1 /nepi/device1/app_sim_connector/sim/goto_position nepi_interfaces/GotoPosition \
  "x_meters: 2.0
y_meters: 0.0
z_meters: 0.0
yaw_deg: 0.0"
```

**Attitude/pose control** (`flight_robot_4_motor` only — `has_goto_pose`):
```bash
rostopic pub -1 /nepi/device1/app_sim_connector/sim/goto_pose nepi_interfaces/GotoPose \
  "roll_deg: 0.0
pitch_deg: 0.0
yaw_deg: 45.0"
```

**Global location** (`flight_robot_4_motor` only — `has_goto_location`):
```bash
rostopic pub -1 /nepi/device1/app_sim_connector/sim/goto_location nepi_interfaces/GotoLocation \
  "lat: 47.6541208
long: -122.3186620
altitude_m: 10.0
yaw_deg: -999"
```

**Setup actions** (RESET, RETURN_HOME, TAKEOFF, LAUNCH — whichever the
selected config lists, by index):
```bash
rostopic pub -1 /nepi/device1/app_sim_connector/sim/setup_action std_msgs/Int32 "data: 0"
```

**Stop immediately** (any config with `has_go_stop`):
```bash
rostopic pub -1 /nepi/device1/app_sim_connector/sim/go_stop std_msgs/Empty "{}"
```

**Manual per-motor control** (any config with `motor_count > 0` — index is
0-based, `speed_ratio` is 0.0-1.0 magnitude, no direction bit per the wire
format):
```bash
rostopic pub -1 /nepi/device1/app_sim_connector/sim/set_motor_control nepi_interfaces/MotorControl \
  "motor_ind: 0
speed_ratio: 0.5"
```

**Camera view mode** (configs with `has_camera_view_control`, e.g.
`ground_robot_2_wheel`/`ground_robot_4_wheel`/`flight_robot_4_motor` — value
must be one of that config's `available_camera_view_modes`):
```bash
rostopic pub -1 /nepi/device1/app_sim_connector/sim/set_camera_view_mode std_msgs/String "data: 'SCENE_CAMERA'"
rostopic pub -1 /nepi/device1/app_sim_connector/sim/set_camera_view_mode std_msgs/String "data: 'ROBOT_CAMERA'"
```
Then view the actual image stream: `rostopic hz
/nepi/device1/app_sim_connector/color_2d_image` to confirm frames are
arriving, or open it in the RUI's image viewer / `web_video_server`.

**Environment options** (configs with `has_environment_controls`, e.g. the
reference rover bridge's obstacle course toggle — the live option list is
refreshed from the bridge itself, not fixed in the params yaml; check
current options via the app's own status topic):
```bash
rostopic echo -n1 /nepi/device1/app_sim_connector/sim/status | grep -A5 available_environment_options
rostopic pub -1 /nepi/device1/app_sim_connector/sim/set_environment_option std_msgs/String "data: 'obstacle_course'"
```

**Sensor topics** (also live-refreshed from the connected bridge, not
fixed): the app relays whatever the bridge reports as available; check
`/nepi/device1/app_sim_connector/sim/status`'s `available_sensor_topics`
field, then `sim/set_active_image_topic` (`std_msgs/String`) to select one.

**Home position** (any config with `has_set_home`/`has_go_home`):
```bash
rostopic pub -1 /nepi/device1/app_sim_connector/sim/set_home_current std_msgs/Empty "{}"
rostopic pub -1 /nepi/device1/app_sim_connector/sim/go_home std_msgs/Empty "{}"
```

**Check overall status** (position, connection state, current config, all
capability flags in one message):
```bash
rostopic echo -n1 /nepi/device1/app_sim_connector/sim/status
```

## Bringing your own simulator

The app doesn't care what's on the other end of the bridge as long as it
speaks the same wire protocol: newline-delimited JSON over one persistent
TCP connection, velocity/position commands one direction, telemetry the
other, plus the small number of typed lines (`camera_settings`, `reset`,
`obstacle_course`) documented in `sim_bridge_node.py`'s own module docstring.
`demo_bridge_client.py` (in this app's `scripts/` folder) is a minimal,
dependency-free reference implementation — it generates synthetic motion
rather than driving a real simulator, but it speaks the exact protocol, so
pointing the app at it (`python3 demo_bridge_client.py --profile rover`)
lets you confirm the NEPI side works before writing a single line of your
own simulator integration. Copy its shape and swap in real calls into your
simulator once you're ready.

Add a new `robot_configs` entry in `sim_connector_app_params.yaml`
describing your robot's real capabilities (see the table above for the
existing shapes to copy from) rather than reusing a mismatched one — a
robot with no camera claiming `has_camera_view_control: true` will show a
camera selector that silently does nothing.

## Appendix: legacy hardware/SITL command reference

The commands above all go through `nepi_app_sim_connector`'s own `sim/*`
topics. The ArduPilot path is different: `rbx_ardupilot_node.py` talks
straight to `mavros`, with no bridge/app layer in between. These raw
`rostopic`/`rosservice` commands (originally from a 2023 ArduPilot tutorial)
hit mavros directly — useful for debugging the ArduPilot RBX driver itself,
or for confirming what mavros exposes/accepts before it's wired into NEPI.
They're not part of the sim connector protocol and don't apply to the
Gazebo rover path described above.

Set the mavros namespace once per terminal:
```bash
source /opt/nepi/ros/setup.bash
MAV=/nepi/device1/mavlink_sitl     # confirm real name with: rostopic list | grep mavlink
```

**Discover what's available**
```bash
rostopic list | grep mavlink
rosservice list | grep mavlink
rostopic list | grep setpoint
```

**Confirm mavros is connected**
```bash
rostopic echo $MAV/state          # expect connected: True
```

**Read nav/pose data**
```bash
rostopic echo $MAV/global_position/global      # lat/lon/alt (WGS84)
rostopic echo $MAV/global_position/local       # ENU pose/odom
rostopic echo $MAV/global_position/compass_hdg # heading
# NEPI-side fused solution:
rosservice call /nepi/device1/nav_pose_query "query_time: {secs: 0, nsecs: 0}
transform: false"
```

**Arm / mode / takeoff** (normally done via the RUI; shown here for direct
CLI testing):
```bash
rosservice call $MAV/set_mode "base_mode: 0
custom_mode: 'GUIDED'"
rosservice call $MAV/cmd/arming "value: true"
rosservice call $MAV/cmd/takeoff "{min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0, altitude: 10.0}"
```

**Autonomous setpoint moves**
```bash
rostopic pub $MAV/setpoint_position/local  geometry_msgs/PoseStamped ...
rostopic pub $MAV/setpoint_position/global geographic_msgs/GeoPoseStamped ...
rostopic pub $MAV/setpoint_raw/attitude    mavros_msgs/AttitudeTarget ...
# hit space+Tab after the message type to auto-fill the body
```

**Direct motor drive** — there's no single blessed mavros CLI command for
per-motor drive; confirm what a given ArduPilot build actually accepts:
```bash
rostopic list | grep -i actuator     # e.g. $MAV/actuator_control
```
Candidates: `mavros_msgs/ActuatorControl`, or `MAV_CMD_DO_MOTOR_TEST` via
`$MAV/cmd/command`. Params to tame spin speed while testing (set in Mission
Planner / SITL params): `MOT_PWM_MAX=1500`, `MOT_SPIN_ARM=0.03`,
`MOT_SPIN_MAX=0.5`, `MOT_SPIN_MIN=0.15`.

**Fake GPS — skip for SITL** (SITL supplies its own GPS). Kept only for the
no-GPS hardware path:
```bash
rostopic pub -r 10 $MAV/hil/gps mavros_msgs/HilGPS '{header: auto, fix_type: 3, geo: {latitude: 47.6541, longitude: -122.31894, altitude: 0.005}, eph: 0, epv: 0, vel: 0, vn: 0, ve: 0, vd: 0, cog: 0, satellites_visible: 9}'
```

**NEPI motor command** (once the RBX driver's motor-control hooks are
wired — `getMotorControlRatios`/`setMotorControlRatio` in
`rbx_ardupilot_node.py`):
```bash
rostopic pub -1 /nepi/device1/ardupilot_sitl/rbx/set_motor_control \
  nepi_interfaces/MotorControl "{motor_ind: 2, speed_ratio: 0.4}"
```
