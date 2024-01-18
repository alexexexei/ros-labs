#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

class TwoSubOnePub(object):

    def __init__(self):
        self.int_val_topic1 = 0
        self.int_val_topic2 = 0

        rospy.Subscriber("topic1", Int32, self.topic1_callback)
        rospy.Subscriber("topic2", Int32, self.topic2_callback)

        self.result_pub = rospy.Publisher("result_368731", Int32, queue_size=10)

    def topic1_callback(self, data):
        rospy.loginfo("received from topic1: %d", data.data)    
        self.int_val_topic1 = data.data

    def topic2_callback(self, data):
        rospy.loginfo("received from topic2: %d", data.data)
        self.int_val_topic2 = data.data

    def calculate_and_log_result(self):
        result = self.int_val_topic1 + self.int_val_topic2
        rospy.loginfo("result = %d", result)
        return result

    def publish_result(self, result):
        self.result_pub.publish(result)

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            result = self.calculate_and_log_result()
            self.publish_result(result)
            rate.sleep()
        
if __name__ == '__main__':
    rospy.init_node("node3", anonymous=True)
    n3s = TwoSubOnePub()

    n3s.loop()
