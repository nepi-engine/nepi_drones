# On-demand identity/info report for a device_if_sim connector node. Mirrors
# nepi_interfaces/DeviceRBXInfo.msg's shape (reused fields kept, per the spec's
# "Reuse as-is" call on the RBX command surface) plus one new field.

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

# Which of the capabilities report's available_image_topics is currently
# selected/streaming -- mirrors DeviceRBXInfo's own image_source field,
# renamed per the spec's device_if_sim field table
string active_image_topic
