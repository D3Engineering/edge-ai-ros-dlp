#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from d3_inventory_demo.robot_state import RobotState

LIN_STOP_VAL = 0.025 # m/s
ANG_STOP_VAL = 0.1 # deg/s
ANG_THRESH_VAL = 0.33

dlp_pub = None
robot_state = None
cmd_vel = None
last_cmd = None

def robot_state_callback(data):
    global robot_state
    # Format is <OBJECTIVE_NAME|STATE>, we just want state
    robot_state = data.data.split("|")[1]
    dlp_command()

def cmd_vel_callback(data):
    global cmd_vel
    cmd_vel = data
    dlp_command()

def dlp_command():
    global last_cmd
    state = RobotState[robot_state]

    if state is RobotState.SCAN:
        cmd = "scan"
    elif state is RobotState.DRIVE:
        cmd = vel_to_cmd(cmd_vel)
    else:
        cmd = "logo"

    if cmd != last_cmd:
        dlp_pub.publish(cmd)
        last_cmd = cmd

def vel_to_cmd(velocity):

    if cmd_vel is None:
        linear_vel = 0
        angular_vel = 0
    else:
        linear_vel = velocity.linear.x
        angular_vel = velocity.angular.z

    if abs(linear_vel) < LIN_STOP_VAL and abs(angular_vel) < ANG_STOP_VAL:
        cmd = "stop"
    elif angular_vel <= -ANG_THRESH_VAL:
        cmd = "turn_left"
    elif angular_vel >= ANG_THRESH_VAL:
        cmd = "turn_right"
    elif linear_vel > 0.:
        cmd = "forward"
    elif linear_vel < 0.:
        cmd = "backward"
    else:
        cmd = "go"

    return cmd



def dlp_interface():
    rospy.init_node('dlp_interface', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("/robot_state", String, robot_state_callback)

    global dlp_pub
    dlp_pub = rospy.Publisher('dlp_command', String, queue_size=2)

    global last_cmd
    last_cmd = "logo"
    dlp_pub.publish(last_cmd)

    rospy.spin()


if __name__ == '__main__':
    try:
        dlp_interface()
    except rospy.ROSInterruptException:
        pass
