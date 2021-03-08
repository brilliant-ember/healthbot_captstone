#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

bridge = CvBridge()
def callback(data):
    print("in the callback")
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow('Video Feed', frame)
    cv2.waitKey(1)
    rospy.loginfo('Image feed received!')
    
    
def listener():
    print("in the listner")
    rospy.init_node('vid_rec')
    #first parameter is the topic you want to subcribe sensor_msgs/Image from
    rospy.Subscriber('/orb_slam2_mono/debug_image', Image, callback)
    rospy.spin()
    
    
if __name__ == '__main__':
    print("In the python")
    listener()
