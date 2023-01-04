#!/usr/bin/env python3

########
# Publisher for controlling the DLP
# Listens to the robot_state and cmd_vel topics and
# publishes an image name for the DLP to display.
########

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from d3_inventory_demo.robot_state import RobotState

vel_data = []
dlp_pub = None
robot_state = None
cmd_vel = None

# Read and save the last robot state
def robot_state_callback(data):
    global robot_state
    # Format is <OBJECTIVE_NAME|STATE>, we just want state
    robot_state = data.data.split("|")[1]

# Read and save the last 5 velocity commands
def cmd_vel_callback(data):
    global cmd_vel
    cmd_vel = data
    vel_data.append(data)
    if len(vel_data) > 5:
        vel_data.pop(0)

# Returns a dlp command corresponding to the
# current velocity command & robot state.
def update_dlp_command():
    state = None
    if RobotState.exists(robot_state):
        state = RobotState[robot_state]

    if state is RobotState.SCAN:
        cmd = "scan"
    elif state is RobotState.DRIVE:
        cmd = vel_to_cmd()
    else:
        cmd = "logo"

    return cmd

# Converts the current command velocity to
# a corresponding direction arrow.
def vel_to_cmd():
    linear_vel = 0
    angular_vel = 0
    # Average linear & angular velocity of the last 5 commands
    for vel in vel_data:
        linear_vel += vel.linear.x
        angular_vel += vel.angular.z

    if len(vel_data) > 0:
        linear_vel / len(vel_data)
        angular_vel / len(vel_data)

    # If the robot is totally stopped, don't wait for the average
    # to settle at 0 - immediately set it to 0.
    if cmd_vel is not None and cmd_vel.linear.x == 0 and cmd_vel.angular.z == 0:
        linear_vel = 0
        angular_vel = 0

    # This condition breaks up the velocity in the following way:
    # If the total velocity is greater than 0.025 m/s: the robot is moving in a direction
    # If the total velocity is above 0 and below 0.025 m/s, the robot is moving ("go")
    # otherwise the robot is stopped
    total_vel = abs(linear_vel) + abs(angular_vel)
    if total_vel > 0.025:
        # Bias towards moving forward if possible:
        # If the 125% of the forward velocity is greater than the angular velocity
        # we'll consider the robot as "moving forward" (or backward)
        if (1.25 * abs(linear_vel)) > abs(angular_vel):
            if linear_vel > 0.0:
                cmd = "forward"
            else:
                cmd = "backward"
        else:
            if angular_vel > 0.0:
                cmd = "turn_left"
            else:
                cmd = "turn_right"
    elif total_vel > 0.0 and total_vel <= 0.025:
        cmd = "go"
    else:
        cmd = "stop"

    return cmd

# Main method - publishes the dlp command at a rate of 5 hz
def dlp_publisher():
    global dlp_pub

    dlp_cmd = None
    rospy.init_node('dlp_publisher', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("/robot_state", String, robot_state_callback)

    dlp_pub = rospy.Publisher('dlp_command', String, queue_size=2)

    dlp_cmd = "logo"

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        dlp_cmd = update_dlp_command()
        dlp_pub.publish(dlp_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        dlp_publisher()
    except rospy.ROSInterruptException:
        pass
