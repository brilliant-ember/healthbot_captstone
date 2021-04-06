#!/usr/bin/env python

import math
from math import sin, cos, pi
from random import randint
import time

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float32MultiArray

# constants
raw_speed = 5  # this is the linear velocity
wheel_radius = 0.0325  # 3cm but in meters
wheel_cirumference = 2 * 3.14 * wheel_radius
L = 0.07  # distace between wheels 7cm
max_limit = 10  # coppeliasim can go further, so this limit is arbitrary
min_limit = -10
# init variables
integrated_error = 0  # this is the accumulated error ie integral sum error
previous_error = 0
previous_time = time.time()  # used to calculate time delta



def callback(data):
    # gets feedback data from coppeliaSim sensors and publishes drive commands back
    rospy.loginfo("received data")
    # this is angular speed from the simulation feedback sensor
    wR = data.data[0]  # right motor angular speed
    wL = data.data[1]  # left motor angular speed
    # closed loop call goes here
    wd = pi/2
    update_wheel_velocity_open_loop(wd)


def simulation_listener():
    rospy.Subscriber("sim_encoder", Float32MultiArray, callback)
    # rospy.Subscriber("odom", Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()


def publsih_command(vr, vl):
    '''publishes right and left wheel velocities to coppeliaSim'''
    pub = rospy.Publisher("vel_wheels", Float32MultiArray, queue_size=10)
    # if is_open_loop: # if sys is closed loop, we follow the listner rate
    #     rate = rospy.Rate(10) # 10hz 

    my_msg = Float32MultiArray()
    my_msg.data = [vr, vl]
    if not rospy.is_shutdown():
        pub.publish(my_msg)
        rospy.loginfo("published vr {}, vl {}".format(vr,vl))
        # if is_open_loop:
        #     rate.sleep()
    else:
        rospy.logerr("rospy is shutdown, didnt publish")


def clamp_velocity_to_limit(v):
    if v > max_limit:
        return max_limit
    elif v < min_limit:
        return min_limit
    return v


def get_linear_velocity(vr, vl):
    # uses the unicycle kinmatic model to get linear velocity
    return (wheel_radius/2 * (vr + vl))


def get_angular_velocity(vr, vl):
    # uses unicycle kinmatic model
    w = wheel_radius/L * (vr-vl)
    return w


def update_wheel_velocity_open_loop(desired_w):
    ''' from the kinamatic model we have two equations
    that solve for linear velocity and angular velocity
    assuming constant linear velocity raw_speed, we use the the
    desired_w and the linear vel equations with the two unknowns
    vl and vr, and solve for them
    v_lin = radius/2(vr + vl) eq 1
    v_ang = radius/distance_btwn_wheels (vr - vl)
    do the math and you will have two equations for vl and vr
    get the desired_w from the pid, this is like the ref signal'''
    # init wheels speed based on raw_speed plus some real life error.
    # randint gives some reallife error
    left_speed = raw_speed + (5 * (randint(1, 10)/10))
    right_speed = raw_speed + (0.9 * (randint(1, 10)/10))
    vr = (desired_w*L + 2*right_speed)/(2*wheel_radius)
    vl = (2*left_speed - desired_w*L)/(2*wheel_radius)
    rate_limiter = 0.015 # to avoid saturating the vel, reduce it to x%
    vr, vl = vr*rate_limiter, vl*rate_limiter
    print("vl {}, vr {} ".format(vl, vr))
    vr = clamp_velocity_to_limit(vr)
    vl = clamp_velocity_to_limit(vl)
    print("clamped: vl {}, vr {} ".format(vl, vr))
    publsih_command(vr, vl)


if __name__ == '__main__':
    try:
        print('main')
        # publish initial command
        rospy.init_node("diff_drive_pub")
        simulation_listener()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
