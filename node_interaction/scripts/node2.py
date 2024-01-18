#!/usr/bin/env python3

import random
import rospy
from std_msgs.msg import Int32

def talker():
    rospy.init_node("node2", anonymous=True)
    pub = rospy.Publisher("topic2", Int32, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        int_to_send = random.randint(1, 1000)
        rospy.loginfo("published: %d", int_to_send)
        pub.publish(int_to_send)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass  
