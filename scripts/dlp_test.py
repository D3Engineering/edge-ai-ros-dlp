#!/usr/bin/env python3

# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from d3_inventory_demo.robot_state import RobotState


def talker():
    rospy.init_node('dlp_tester', anonymous=True)

    vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    state_pub = rospy.Publisher("/robot_state", String, queue_size=10)

    state = "TEST|DRIVE"
    vel = Twist()

    vel.linear.x = 1.0
    vel.angular.z = 0.0

    rospy.loginfo(state)
    rospy.loginfo(vel)
    state_pub.publish(state)
    vel_pub.publish(vel)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        state_pub.publish(state)
        vel_pub.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


