# One entry in a SimDeviceIF "available sensor topics" list.
#
# The available-topics list is typed (topic + msg type) rather than images-only,
# so a future sensor modality (lidar, depth, IMU) is just another entry with a
# different msg_type, not a contract change. Cameras are the has_camera == True
# subset of this same list, filtered by msg_type == 'sensor_msgs/Image' -- not a
# separately maintained field.

# The full ROS topic name, e.g. "/nepi/device1/app_sim_connector/scene_camera"
string topic_name

# The ROS message type string as returned by nepi_sdk.find_topics_by_msgs(),
# e.g. "sensor_msgs/Image", "sensor_msgs/LaserScan", "sensor_msgs/Imu"
string msg_type
