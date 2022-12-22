#!/usr/bin/env python3

# ROS wrapper around dlp.py

import rospy
from std_msgs.msg import String
from d3_dlp.dlp import DLPDemo

dlp_demo = None
last_dlp_cmd = None

def dlp_callback(data):
    global last_dlp_cmd, dlp_demo

    dlp_cmd = data.data
    if dlp_cmd != last_dlp_cmd:
        dlp_demo.update_dlp(dlp_cmd)
        last_dlp_cmd = dlp_cmd

if __name__ == '__main__':
    dlp_demo = DLPDemo()
    rospy.init_node('dlp_ros', anonymous=True)
    rospy.Subscriber('dlp_command', String, dlp_callback)
    rospy.spin()
