#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# constants, units in cm
publish_rate = 60
wheel_diameter = 6.5 #cm
ticks_per_rev = 231
distance_btwn_wheels = 17 #cm

def get_wheel_traveled_disntace(ticks):
    ''' ticks is the number of pulses captured by the hall sensor on the wheel
    look at the read me section odometry for info'''
    traveled_distance = wheel_diameter * pi * ticks_per_rev / ticks
    return traveled_distance

def get_linear_velocity(left_wheel_distance, right_wheel_distance, time):
    ''' v = deltaX/deltaT '''
    return (left_wheel_distance + right_wheel_distance)/time

def get_angular_velocity(left_wheel_distance, right_wheel_distance, time):
    '''ang velocity, counterclock wise direction is positive '''
    return ( right_wheel_distance - left_wheel_distance)/time

rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

# assume robot starts at origin and doesnt move initially
x = 0.0
y = 0.0
th = 0.0

vx = 0
vy = 0
vth = 0

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(publish_rate)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt

    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
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
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
