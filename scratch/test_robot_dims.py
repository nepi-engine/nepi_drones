#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

rospy.init_node('test_robot_dims_pub', anonymous=True)
pub = rospy.Publisher('/nepi/device1/app_sim_connector/sim/set_robot_dimensions', String, queue_size=1, latch=True)
rospy.sleep(1)

yaml_text = (
    "wheel_radius_m: 0.15\n"
    "wheel_width_m: 0.08\n"
    "track_width_m: 0.35\n"
    "wheelbase_m: 0.32\n"
    "chassis_length_m: 0.55\n"
    "chassis_width_m: 0.32\n"
    "chassis_height_m: 0.22\n"
    "weight_kg: 6\n"
    "camera_horizontal_fov_deg: 80\n"
    "wheel_independence_enabled: 0\n"
)
pub.publish(String(data=yaml_text))
rospy.sleep(1)
print("published set_robot_dimensions")
