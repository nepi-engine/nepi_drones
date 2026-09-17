# Fake GPS App Status

# Per-device target list: every discovered mavros node, and whether HilGPS/
# GPS_INPUT injection is currently enabled/bound for it -- independently
# toggleable per device (requested live 2026-09-16: "it needs to just be
# enabled on the devices it should matter for"; previously a single global
# enabled+selected_mavros_node meant two ArduPilot devices running at once
# (a physical drone + its SITL twin) fought over the same shared toggle).
# All three arrays are parallel to available_mavros_nodes.
string[] available_mavros_nodes
bool[] mavros_node_enabled
bool[] mavros_node_bound

# Simulated GPS position (WGS84)
float64 current_latitude
float64 current_longitude
float64 current_altitude_m

# Configured start/home position (WGS84)
float64 start_latitude
float64 start_longitude
float64 start_altitude_m

# Simulated NED position relative to last reset (meters)
float32 current_point_x
float32 current_point_y
float32 current_point_z

# Simulated orientation (derived from direction of travel)
# current_heading_deg: degrees true north; current_yaw_deg: ENU yaw degrees
float32 current_heading_deg
float32 current_yaw_deg

# Active move state
bool moving

# Fake GPS publish settings
int32 satellites_visible
float32 gps_pub_rate_hz
