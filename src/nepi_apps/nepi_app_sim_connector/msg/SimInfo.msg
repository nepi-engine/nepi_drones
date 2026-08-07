# On-demand identity/info report for a SimDeviceIF connector node. Mirrors
# nepi_interfaces/DeviceRBXInfo.msg's shape (reused fields kept unchanged) plus
# the sim-specific fields at the bottom.

# Connected to the simulator bridge
bool connected

# Device identifiers
string device_name
string serial_num
string hw_version
string sw_version

# In standby or not
bool standby

# Index to enumerated state options
int32 state

# Index to enumerated mode options
int32 mode

# Abs error bounds for command controls
nepi_interfaces/ErrorBounds error_bounds

# Timeout time for goto commands
float32 cmd_timeout

# Home Location Lat, Long, altitude_meters
float32 home_lat
float32 home_long
float32 home_alt

##############################
# SimDeviceIF additions

# Which of the capabilities report's available_image_topics is currently
# selected/streaming -- mirrors DeviceRBXInfo's own image_source field
string active_image_topic

# Current camera view/rig mode, one of the capabilities report's
# available_camera_view_modes
string camera_view_mode
