#!/usr/bin/env python3

"""
DLP Listener - sends commands to the DLP
based on dlp_command topic
"""

import rospy
from std_msgs.msg import String
from d3_dlp.dlp import DLPDemo

dlp_demo = None
last_dlp_cmd = None

def dlp_callback(data):
    """
    Update the DLP when a new command is issued
    :param data: incoming DLP command
    :return: None
    """
    global last_dlp_cmd, dlp_demo
    dlp_cmd = data.data
    # Only update the DLP if the command has changed
    # important cause otherwise the image will be stuck at it's first frame
    if dlp_cmd != last_dlp_cmd:
        dlp_demo.update_dlp(dlp_cmd)
        last_dlp_cmd = dlp_cmd

if __name__ == '__main__':
    dlp_demo = DLPDemo()
    rospy.init_node('dlp_ros', anonymous=True)
    rospy.Subscriber('dlp_command', String, dlp_callback)
    rospy.spin()
