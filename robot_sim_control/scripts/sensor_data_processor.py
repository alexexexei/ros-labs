#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

min_range = 0.03
max_range = 3.0

class LidarDataHandler:
    def __init__(self):
        sub_topic_name = '/diffrobot/laser/scan'
        self.lidar_sub = rospy.Subscriber(sub_topic_name, LaserScan, self.lidar_cb)

        pub_topic_name_wrc = '/diffrobot/wheel_right_controller/command'
        pub_topic_name_wlc = '/diffrobot/wheel_left_controller/command'
        self.wheel_right_pub = rospy.Publisher(pub_topic_name_wrc, Float64, queue_size=10)
        self.wheel_left_pub = rospy.Publisher(pub_topic_name_wlc, Float64, queue_size=10)

    def lidar_cb(self, data):
        ranges = np.array(data.ranges)
        if np.any((min_range <= ranges) & (ranges <= max_range)):
            self.publish_wheel_commands(2.5, -2.5)
        else:
            self.publish_wheel_commands(-1.5, -1.5)

    def publish_wheel_commands(self, cv_right, cv_left):
        self.wheel_right_pub.publish(Float64(data=cv_right))
        self.wheel_left_pub.publish(Float64(data=cv_left))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('sensor_data_proc', anonymous=True)
    ldh = LidarDataHandler()
    ldh.run()
