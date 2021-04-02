"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from random import randint
import math
import time
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
# timestep = int(robot.getBasicTimeStep())
timestep = 64
raw_speed = 4
previous_left_distance = 0
previous_right_distance = 0
wheel_radius = 0.0325
wheel_cirumference = 2 * 3.14 * wheel_radius
L = 0.07 # distace between wheels

 # randint gives some reallife error
left_speed = raw_speed * 0.5 * (randint(1, 10)/10)
right_speed = raw_speed *0.45 * (randint(1, 10)/10)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
left_motor = robot.getDevice("motor_2")
right_motor = robot.getDevice("motor_1")
right_encoder = robot.getDevice("encoder_1")
left_encoder = robot.getDevice("encoder_2")
 # enable motors
left_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setPosition(float('inf'))
right_motor.setVelocity(0.0)
# enable positiions
right_encoder.enable(timestep)
left_encoder.enable(timestep)

left_encoder_val = 0
right_encoder_val = 0


def get_angular_velocity(right_distance, left_distance, dt):
    # init timers to get delta t
    current_time = int(time.time())
    left_dx = left_distance - previous_left_distance
    right_dx = right_distance - previous_right_distance
    
    vr = right_dx/dt
    vl = left_dx/dt
    
    w = wheel_radius/L * (vl-vr)
    return w
    
previous_time = time.time()

# Main loop:
while robot.step(timestep) != -1:
# ---- odometry ------
    left_encoder_val = left_encoder.getValue()
    right_encoder_val = right_encoder.getValue() # 0 to 2pi
    distance_right = right_encoder_val * wheel_radius
    distance_left = left_encoder_val * wheel_radius
    #print("right distance {} left distnace {}".format(distance_right, distance_left))
    # ------------
    current_time = time.time()
    dt = current_time - previous_time
    w = get_angular_velocity(distance_right, distance_left, dt)
    print("w = {}".format(w))
    
    # remember these for next time
    previous_right_distance = distance_right
    previous_left_distance = distance_left
    previous_time  = current_time
    
    right_motor.setVelocity(right_speed)
    left_motor.setVelocity(left_speed)

# Enter here exit cleanup code.
