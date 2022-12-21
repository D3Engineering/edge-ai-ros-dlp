#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

LIN_STOP_VAL = 0.1 # m/s
ANG_STOP_VAL = 0.1 # deg/s


def cmd_vel_callback(data):
    global dlp_pub
    global last_cmd

    linear_vel = data.linear.x
    angular_vel = data.angular.z

    if abs(linear_vel) < LIN_STOP_VAL and abs(angular_vel) < ANG_STOP_VAL:
        cmd = "stop"
    elif angular_vel <= -0.33:
        cmd = "turn_left"
    elif angular_vel >= 0.33:
        cmd = "turn_right"
    else:
        cmd = "go"

    if cmd != last_cmd:
        dlp_pub.publish(cmd)
        last_cmd = cmd


def dlp_interface():
    rospy.init_node('dlp_interface', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)

    global dlp_pub
    dlp_pub = rospy.Publisher('/dlp_test_string', String, queue_size=2)

    global last_cmd
    last_cmd = "stop"
    dlp_pub.publish(last_cmd)

    rospy.spin()


if __name__ == '__main__':
    try:
        dlp_interface()
    except rospy.ROSInterruptException:
        pass
