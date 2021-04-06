#!/usr/bin/env python

import math
from math import sin, cos, pi
from random import randint
import time

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float32MultiArray, Float64

# constants
raw_speed = 1  # this is the linear velocity
wheel_radius = 0.0325  # 3cm but in meters
wheel_cirumference = 2 * 3.14 * wheel_radius
L = 0.07  # distace between wheels 7cm
max_limit = 10  # coppeliasim can go further, so this limit is arbitrary
min_limit = -10
queue_size = 10
# init variables
integrated_error = 0  # this is the accumulated error ie integral sum error
previous_error = 0
previous_time = time.time()  # used to calculate time delta
left_wheel_velocity = 0.0
right_wheel_velocity = 0.0
left_ns = 'left_wheel'#left wheel namespace
right_ns = 'right_wheel'#right wheel namespace
target_omega = 0
# publishers
## publishers to PID node 
setpoint_pub_right = None
setpoint_pub_left = None
feedback_encoder_pub_right = None
feedback_encoder_pub_left = None
## publishers to coppeliasim
wheel_vel_pub = None

def init_simulation_publishers():
    global setpoint_pub_right, setpoint_pub_left, feedback_encoder_pub_right, feedback_encoder_pub_left, wheel_vel_pub
    #for pid controller
    setpoint_pub_right = rospy.Publisher(right_ns+"/setpoint", Float64, queue_size=queue_size)
    setpoint_pub_left = rospy.Publisher(left_ns+"/setpoint", Float64, queue_size=queue_size)
    feedback_encoder_pub_right = rospy.Publisher(right_ns+"/state", Float64, queue_size=queue_size)
    feedback_encoder_pub_left = rospy.Publisher(left_ns+"/state", Float64, queue_size=queue_size)
    # for coppeliasim
    wheel_vel_pub = rospy.Publisher("vel_wheels", Float32MultiArray, queue_size=queue_size)
def encoder_callback(data):
    # gets feedback data from coppeliaSim sensors and publishes drive commands back
    rospy.loginfo("received data")
    # this is angular speed from the simulation feedback sensor
    wR = data.data[0]  # right motor angular speed
    wL = data.data[1]  # left motor angular speed
    # closed loop call goes here
    wd = pi/2
    # calculate_desired_velocities(wd)
    publish_feedback_pid(wR, wL) # publish the feedback so the pid controller node can compensate
    vr, vl = calculate_desired_velocities(wd)
    publish_pid_input(vr, vl)

def init_simulation_listeners():
    rospy.Subscriber("sim_encoder", Float32MultiArray, encoder_callback)
    # these two below translate control effort to something that coppeliasim understands
    rospy.Subscriber(right_ns+"/control_effort", Float64, sim_right_velocity_callback)
    rospy.Subscriber(left_ns+"/control_effort", Float64, sim_left_velocity_callback)

    # rospy.Subscriber("odom", Odometry, encoder_callback)
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

def sim_right_velocity_callback(v):
    ''' publishes pid commands to the coppeliasim simulation'''
    global right_wheel_velocity, left_wheel_velocity
    right_wheel_velocity = v.data
    publish_simulation_command(right_wheel_velocity, left_wheel_velocity)

def sim_left_velocity_callback(v):
    ''' publishes pid commands to the coppeliasim simulation'''
    global right_wheel_velocity, left_wheel_velocity
    left_wheel_velocity = v.data
    publish_simulation_command(right_wheel_velocity, left_wheel_velocity)
### ----

def publish_pid_input(desired_vr, desired_vl):
    '''publishes the desired input to the control loop'''
    global setpoint_pub_right, setpoint_pub_left, feedback_encoder_pub_right, feedback_encoder_pub_left, wheel_vel_pub
    my_msg = Float64()
    my_msg2 = Float64()
    my_msg.data =desired_vr
    my_msg2.data =desired_vl
    if not rospy.is_shutdown():
        setpoint_pub_right.publish(my_msg)
        setpoint_pub_left.publish(my_msg2)
    else:
        rospy.logerr("rospy is shutdown, didnt publish")

def publish_feedback_pid(vr,vl):
    '''publishes the feedback to the PID controller node'''
    global setpoint_pub_right, setpoint_pub_left, feedback_encoder_pub_right, feedback_encoder_pub_left, wheel_vel_pub
    my_msg = Float64()
    my_msg2 = Float64()
    my_msg.data =vr
    my_msg2.data =vl
    if not rospy.is_shutdown():
        feedback_encoder_pub_right.publish(my_msg)
        feedback_encoder_pub_left.publish(my_msg2)
        # rospy.loginfo("published vr {}, vl {}".format(vr,vl))
    else:
        rospy.logerr("rospy is shutdown, didnt publish")

def publish_simulation_command(vr, vl):
    '''publishes right and left wheel velocities to coppeliaSim'''
    global wheel_vel_pub
    # rate = rospy.Rate(10) # 10hz 
    my_msg = Float32MultiArray()
    my_msg.data = [vr, vl]
    if not rospy.is_shutdown():
        wheel_vel_pub.publish(my_msg)
        rospy.loginfo("published vr {}, vl {}".format(vr,vl))
        # rate.sleep()
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


def calculate_desired_velocities(desired_w):
    ''' from the kinamatic model we have two equations that solve for linear velocity and angular velocity
    assuming constant linear velocity raw_speed, we use the the
    desired_w and the linear vel equations with the two unknowns vl and vr, and solve for them
    v_lin = radius/2(vr + vl) eq 1
    v_ang = radius/distance_btwn_wheels (vr - vl)
    do the math and you will have two equations for vl and vr get the desired_w from the pid, this is like the ref signal'''
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
    # publish_simulation_command(vr, vl)
    return (vr, vl)
    

if __name__ == '__main__':
    try:
        node_name = "diff_drive_head"
        rospy.init_node(node_name)
        rospy.loginfo("Starting " + node_name)

        init_simulation_listeners()
        init_simulation_publishers()
        rate= rospy.Rate(10)
        target_omega = 0
        while not rospy.is_shutdown():
            vr, vl = calculate_desired_velocities(target_omega)
            publish_simulation_command(vr, vl)
            rate.sleep()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
