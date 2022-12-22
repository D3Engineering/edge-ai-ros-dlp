#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from d3_inventory_demo.robot_state import RobotState

LIN_STOP_VAL = 0.025 # m/s
ANG_STOP_VAL = 0.1 # deg/s
ANG_THRESH_VAL = 0.33

vel_data = []
dlp_pub = None
robot_state = None
cmd_vel = None

def robot_state_callback(data):
    global robot_state
    # Format is <OBJECTIVE_NAME|STATE>, we just want state
    robot_state = data.data.split("|")[1]

def cmd_vel_callback(data):
    global cmd_vel
    cmd_vel = data
    vel_data.append(data)
    if len(vel_data) > 5:
        vel_data.pop(0)

def update_dlp_command():
    state = None
    if RobotState.exists(robot_state):
        state = RobotState[robot_state]

    if state is RobotState.SCAN:
        cmd = "scan"
    elif state is RobotState.DRIVE:
        cmd = vel_to_cmd_v2()
    else:
        cmd = "logo"

    return cmd

def vel_to_cmd():
    if cmd_vel is None:
        linear_vel = 0
        angular_vel = 0
    else:
        linear_vel = velocity.linear.x
        angular_vel = velocity.angular.z

    # Try one where if abs ang + abs linear > 0.025 or something
    # then whichever is greater, linear to angular
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


def vel_to_cmd_v2():
    linear_vel = 0
    angular_vel = 0
    for vel in vel_data:
        linear_vel += vel.linear.x
        angular_vel += vel.angular.z

    if len(vel_data) > 0:
        linear_vel / len(vel_data)
        angular_vel / len(vel_data)

    if cmd_vel is not None and cmd_vel.linear.x == 0 and cmd_vel.angular.z == 0:
        linear_vel = 0
        angular_vel = 0

    total_vel = abs(linear_vel) + abs(angular_vel)
    if total_vel > 0.025:
        # Bias towards moving forward if possible
        if (1.25 * abs(linear_vel)) > abs(angular_vel):
            if linear_vel > 0.0:
                cmd = "forward"
            else:
                cmd = "backward"
        else:
            # This is backwards, positive ang vel is ccw rotation lmao
            if angular_vel > 0.0:
                cmd = "turn_left"
            else:
                cmd = "turn_right"
    elif total_vel > 0.0 and total_vel <= 0.025:
        cmd = "go"
    else:
        cmd = "stop"

    return cmd



def dlp_interface():
    global dlp_pub

    dlp_cmd = None
    rospy.init_node('dlp_interface', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("/robot_state", String, robot_state_callback)

    dlp_pub = rospy.Publisher('dlp_command', String, queue_size=2)

    dlp_cmd = "logo"

    rate = rospy.Rate(5) # in hz
    while not rospy.is_shutdown():
        dlp_cmd = update_dlp_command()
        dlp_pub.publish(dlp_cmd)
        rate.sleep()


if __name__ == '__main__':
    try:
        dlp_interface()
    except rospy.ROSInterruptException:
        pass
