#!/usr/bin/env python

# this node subscribes to the video from copelliaSim
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image  # Image is a message type in ROS
from cv_bridge import CvBridge  # to convert between ROS and OpenCV Images
import cv2  # OpenCV library
from matplotlib import pyplot as plt
import numpy as np
import message_filters

right_video, left_video = None, None


def left_callback(data):
  global left_video
   # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   # Used to convert between ROS and OpenCV images
  br = CvBridge()
  rospy.loginfo("receiving left video frame")

  # Convert ROS Image message to OpenCV image
  current_frame = br.imgmsg_to_cv2(data)
  left_video = current_frame
  # Display image
  # cv2.imshow("cameraLeft", current_frame)
  # cv2.waitKey(1)


def show_both_videos():
  print("yay")
  print("aaa", left_video)
  if left_video:
    cv2.imshow("cameraLeft", left_video)
    # cv2.imshow("cameraright", right_video)
    cv2.waitKey(1)




# def callback(data):
#    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#      # Used to convert between ROS and OpenCV images
#   br = CvBridge()
 
#   # Output debugging information to the terminal
#   rospy.loginfo("receiving video frame")
   
#   # Convert ROS Image message to OpenCV image
#   current_frame = br.imgmsg_to_cv2(data)
   
#   # Display image
#   cv2.imshow("camera", current_frame)
   
#   cv2.waitKey(1)
# 
# def combined_callback(right_video, left_video):
#    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#      # Used to convert between ROS and OpenCV images
#   br = CvBridge()
 
#   # Output debugging information to the terminal
#   rospy.loginfo("receiving video frame")
   
#   # Convert ROS Image message to OpenCV image
#   right_frame = br.imgmsg_to_cv2(right_video)
#   left_frame = br.imgmsg_to_cv2(left_video)
   
#   # Display image
#   # cv2.imshow("camera1", right_frame)
#   cv2.imshow("camera2", right_frame)
   
#   cv2.waitKey(1)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('cams_listener', anonymous=False)
    rospy.loginfo("started cams_listner node")
    right_video = message_filters.Subscriber('hbot_camera1', Image)
    left_video = message_filters.Subscriber('hbot_camera2', Image)
    # ts = message_filters.TimeSynchronizer([right_video, left_video], 10)
    # ts.registerCallback(combined_callback)

    # rospy.Subscriber("hbot_camera", Image, callback)
    rospy.Subscriber("hbot_camera2", Image, left_callback)
    show_both_videos()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
