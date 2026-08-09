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
