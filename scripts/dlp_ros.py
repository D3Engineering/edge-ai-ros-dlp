#!/usr/bin/env python3

# ROS wrapper around dlp.py

import rospy
from std_msgs.msg import String
from d3_dlp.dlp import DLPDemo
        
if __name__ == '__main__':
    dlp_demo = DLPDemo()
    rospy.init_node('dlp_ros', anonymous=True)
    rospy.Subscriber('dlp_command', String, dlp_demo.callback)
    rospy.spin()

