#!/usr/bin/env python3

########
# DLP Publisher for the purpose of debugging
# Should be run after starting up the DLP listener
# use W-A-S-D-Q-E to send various commands to the dlp_command topic
########
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
        char = input("Waiting for direction (W,A,S,D):").lower()
        if char == "w":
            state = "TEST|DRIVE"
            vel.linear.x = 1.0
            vel.angular.z = 0.0
        elif char == "a":
            state = "TEST|DRIVE"
            vel.linear.x = 0.0
            vel.angular.z = 1.0
        elif char == "s":
            state = "TEST|DRIVE"
            vel.linear.x = -1.0
            vel.angular.z = 0.0
        elif char == "d":
            state = "TEST|DRIVE"
            vel.linear.x = 0.0
            vel.angular.z = -1.0
        elif char == "q":
            state = "TEST|DRIVE"
            vel.linear.x = 0.0
            vel.angular.z = 0.0
        elif char == "e":
            state = "TEST|SCAN"
            vel.linear.x = 0.0
            vel.angular.z = 0.0
        else:
            state = "TEST|TRACK"
            vel.linear.x = 0.0
            vel.angular.z = 0.0

        rospy.loginfo("State: " + state)
        rospy.loginfo("Velocity: Linear:" + str(vel.linear.x) + "  Angular:" + str(vel.angular.z))
        state_pub.publish(state)
        vel_pub.publish(vel)
        #rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


