#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

def control_joints():
    rospy.init_node('joint_position_controller')
    pub_joint1 = rospy.Publisher('/joint1_position_controller/command', Float64, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        joint1_position = 0.5  # Desired position in radians
        pub_joint1.publish(joint1_position)
        rate.sleep()

if __name__ == '__main__':
    try:
        control_joints()
    except rospy.ROSInterruptException:
        pass
