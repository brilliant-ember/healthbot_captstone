#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

import serial
from time import sleep

COM = '/dev/ttyUSB1'
BAUD = 115200
MAXPWM = 1023 - 70
MINPWM = 350
distance_right_wheel, distance_left_wheel, vel_right, vel_left = 0,0,0,0
conv_factor = 0.0001 # convert micrometers to cm
publish_rate = 60
wheel_diameter = 6.5# 6.5 cm
wheel_rad = wheel_diameter/2
distance_btwn_wheels = 17 # 17cm
# assume robot starts at origin and doesnt move initially
x = 0.0
y = 0.0
th = 0.0

def ros_stuff():
    global x,y,th, distance_right_wheel, distance_left_wheel, vel_right, vel_left
    rospy.init_node('odometry_publisher')
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    last_time = rospy.Time.now()
    r = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        if (read_serial_data(ser)):
            current_time = rospy.Time.now()
            w = wheel_rad/ distance_btwn_wheels *(vel_right - vel_left) # angular speed
            v = wheel_rad/2 * (vel_right + vel_left)

            # given hypthenus v and Vtheta w,  i can find Vx and Vy
            vx = v * cos(w)
            vy = v* sin(w)
            
            dt = (current_time - last_time).to_sec()
            delta_x = (vx * cos(w) - vy * sin(w)) * dt
            delta_y = (vx * sin(w) + vy * cos(w)) * dt
            delta_th = w * dt

            x += delta_x
            y += delta_y
            th += delta_th
            
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

            # first, we'll publish the transform over tf
            odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
            # set the velocity
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, w))
            # publish the message
            odom_pub.publish(odom)
            last_time = current_time
            r.sleep()
        else: # no new serial data do nothing
            # print("no serial")
            # sleep(0.001)
            pass

def read_serial_data(ser):
    global distance_right_wheel, distance_left_wheel, vel_right, vel_left, cm
    try:
        if ser.in_waiting > 0:  # if there is new data in the receive buffer, ie only trigger when u get new data
            # print(ser.in_waiting, end=' ')
            val = str(ser.readline().decode().strip('\r\n'))
            val = val.split()
            if (val[0] == "dx_vel" and len(val) == 5):
                # distance in micrometers, velocity in micrometer/milisec, so convert 
                distance_right_wheel = float(val[1]) * conv_factor
                distance_left_wheel = float(val[2]) * conv_factor
                vel_right = float(val[3]) * conv_factor # no this  is conv_factor/milisec
                vel_left = float(val[4]) * conv_factor
                # print(val, end="\n")
                return True
            # print(dx, end="\n")
            
        else:
            # pass
            return False
    except(UnicodeDecodeError):
        pass

# def write_serial_data(ser):
#     # print("writing")
#     c = b"500,600\n"
#     # c = c.encode(encoding = 'utf-8')
#     ser.write(c)
#     ser.flush()
#     print(c)
#     sleep(1000)
#     # sleep(.001) # waiting for incomming data
#     # print("pass, is waiting")

    
with serial.Serial(COM, BAUD, timeout=1) as ser:
    print('Waiting for device')
    sleep(1)
    print("found {}".format(ser.name))
    ser.reset_output_buffer()
    ser.reset_input_buffer()
    ros_stuff()

    
    # while True:
    #     read_serial_data(ser)



    

        
        # write_serial_data(ser)
        
        
        # else:
        #     print("is not wainting")
