#!/usr/bin/env python3
import json
import rospy
from std_msgs.msg import String

rospy.init_node('test_save_dims_pub', anonymous=True)
pub = rospy.Publisher('/nepi/device1/app_sim_connector/sim/save_robot_dimensions_config', String, queue_size=1, latch=True)
rospy.sleep(1)

payload = {
    "name": "test_config_verify",
    "yaml": "wheel_radius_m: 0.15\nweight_kg: 6\n",
}
pub.publish(String(data=json.dumps(payload)))
rospy.sleep(1)
print("published save_robot_dimensions_config")
