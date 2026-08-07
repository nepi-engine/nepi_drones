# Timed operational status report for a SimDeviceIF connector node, published at
# STATUS_UPDATE_RATE_HZ (2 Hz, matching device_if_rbx.py). Mirrors
# nepi_interfaces/DeviceRBXStatus.msg's shape (reused fields kept unchanged, so
# error reporting and data recording are inherited rather than new work) plus the
# SimDeviceIF additions at the bottom.

string device_name
string device_path
string device_node_name

string serial_num
string hw_version
string sw_version

string data_source_description
string data_ref_description

string settings_topic
string navpose_topic
string save_data_topic
string[] data_products
string[] data_product_topics
string[] data_product_image_topics

# Human readable description of any current control process, "None" if no
# active process
string process_current

# Human readable description of last control process, "None" if no previous
# process has happened
string process_last

# Ready/Busy -- check ready state, then busy during a command control action,
# then ready again on completion
bool ready

# Battery charge state (0.0-1.0), -999 if not available
float32 battery

# Signed error values for current control process
nepi_interfaces/GotoErrors errors_current

# Signed error values for previous control process
nepi_interfaces/GotoErrors errors_prev

# Success or failure of last command control (met error bounds before cmd_timeout)
bool cmd_success

# Manual and autonomous control ready states
bool manual_control_mode_ready
bool autonomous_control_mode_ready

# Current motor controls for manual motor control mode
nepi_interfaces/MotorControl[] current_motor_control_settings

# Last command string -- can be added to an automation script file
string last_cmd_string

# Last error message
string last_error_message

# Transform from device native ref frame to the nepi_frame
nepi_interfaces/Transform navpose_frame_transform

##############################
# SimDeviceIF additions: connection/telemetry health. A live bridge socket
# proves the far side is reachable but not that telemetry is current -- these
# two fields let a stalled-but-connected bridge (simulator paused, a hung
# callback) be distinguished from a healthy one.

# Is the bridge to the simulator-side script currently connected
bool bridge_connected

# Seconds since the last telemetry line was received over that bridge.
# Grows monotonically while bridge_connected is True but nothing arrives;
# resets to ~0 on every fresh telemetry line. -1 if never connected.
float32 telemetry_age_sec

##############################
# SimDeviceIF additions: live, typed sensor-topic list. Re-derived on every
# publish, never cached past construction (see api/device_if_sim.py).

SensorTopicInfo[] available_sensor_topics

##############################
# SimDeviceIF additions: the two selectors. Same reported-list-plus-active-
# selection shape as available_image_topics / active_image_topic.
#
# Simulator sources currently available on the live ROS graph. Discovery is
# generic: sim-capable devices are matched by device type and by their own
# declared data source description, never by a simulator product name. An empty
# list and an empty selection are both valid and safe.

string[] available_simulators
string[] available_simulator_names
string selected_simulator

# Named robot configs this deployment offers, and which one is selected.
# Selecting a config tells the simulator which kind of robot is wanted, which is
# what determines the capability profile reported by the capabilities query.

string[] available_robot_configs
string selected_robot_config
