#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

delta_min = 0.1
delta_max = 0.5


class LidarDataHandler:
    def __init__(self):
        sub_topic_name_right = '/diff_drive_robot/laser_1/scan'
        sub_topic_name_left = '/diff_drive_robot/laser_2/scan'

        self.lidar_1_sub = message_filters.Subscriber(sub_topic_name_right, LaserScan)
        self.lidar_2_sub = message_filters.Subscriber(sub_topic_name_left, LaserScan)
        self.ats = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_1_sub, self.lidar_2_sub], 1, 0.1)

        self.ats.registerCallback(self.lidar_cb)
        pub_topic_name = "/cmd_vel"
        self.wheel_pub = rospy.Publisher(pub_topic_name, Twist, queue_size=2)

    def lidar_cb(self, data1, data2):
        ranges1 = data1.ranges
        ranges2 = data2.ranges

        min_1 = min(ranges1)
        min_2 =  min(ranges2)

        if (abs(min_1 - min_2) < 0.1):
            self.publish_wheel_command(0.5, 0, 0, 0, 0, 0.0)
            rospy.loginfo("for")
        elif    min_1 < min_2:
            self.publish_wheel_command(0.25, 0, 0, 0, 0, -1.5)
            rospy.loginfo("left")
        else:
            self.publish_wheel_command(0.25, 0, 0, 0, 0, 1.5) 
            rospy.loginfo("right")   
        
    def publish_wheel_command(self, x, y, z, ax, ay, az):
        twist_msg = Twist()

        twist_msg.linear.x = x
        twist_msg.linear.y = y
        twist_msg.linear.z = z

        twist_msg.angular.x = ax
        twist_msg.angular.y = ay
        twist_msg.angular.z = az

        self.wheel_pub.publish(twist_msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('katafalk_runner', anonymous=True)
    ldh = LidarDataHandler()
    ldh.run()
