#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64




def talker():
    pub = rospy.Publisher('setpoint', Float64, queue_size=1)
    rospy.init_node('setpoint_pub', anonymous=True)
    speed = rospy.get_param("~speed", 1)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # setpoint = speed
        pub.publish(speed * 0.51444)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
