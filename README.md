# Capstone project healthbot

https://www.youtube.com/playlist?list=PLD1pHgB031ck666_LJpps_zopNTI3XrwL

Deep learning depth point cloud from monocular camera was the best pointcloud produced, but there's no video for it in the playlist above.

## PID control
this package is used http://wiki.ros.org/pid
to start PID control with coppeliasim
1. start roscore
2. open coppeliasim file `rosControllerCoppeliaSimDiffDrivettt.ttt`
3. `roslaunch motor_control pid_coppeliaSim.launch`
4. run this file hbot_ws/src/motor_control/src/pid_simulation.py
5. For manuel tunning use `rosrun rqt_reconfigure rqt_reconfigure`


## Motor Control
I am using a NodeMCU board, but you can use an arduino or any board instead but make sure to use the correct pins and set the PWM limit for your board, also remove the term`ICACHE_RAM_ATTR` from befor the interrupt functions as that may cause errors depending on what board you're using. Also if you're using an Arduino Uno change the MAXPWM to 255 and the MINPWM to 0.
In case you're using a NodeMCU make sure to power _off the motors when you boot or upload code_ to the mcu because having power on certain pins during launch can cause failures and weird behavior for the NodeMCU. Also the pin numbers printed on the board don't match the acutal GPIO numbers, hence different pin numbers in code, use a pin out diagram that has the GPIO pin numbers.

I am only using two hall sensors, one for each motor, so we only need two interupt pins to run the motor encoders for odometry instead of 4 interupt pins
![](screenshots/2021-03-25-19-17-35.png)*motor control connections, without sensors*last_time

### Odometry calculation
[this blog was good ](https://hackernoon.com/feedback-odometry-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-3-e9d8e4df6df1 )and also [the youtbe tutorial by adventures in STEM](https://www.youtube.com/watch?v=oLBYHbLO8W0)


For my motor I have:
- gear box ratio 1:21, so for each wheel rotation (output) I have 21 motor rotations. 21 motor revs = 1 output rev
- 11 pulses (ticks) per motor revlotion
- 21 x 11 = 231 pulses per wheel rev
- wheel diameter = 6.5 cm
- distnace convered per 1 wheel rev = diameter * pi = 20.4 cm 
- 231 pulses = 20.4 cm, so 1 pulse = 0.08cm, so to go 1 meter we have 1250 pulses
- distance between wheels is 17cm
So we get __distnace traveled per wheel = Diameter\*pi\*counted ticks/231__ or _deltaX = d\*pi\*ticks/tickesPerRev_
now we need linear velocity and angular velocity w, go to the blog post I put it has a neat explanation 

## Battery and Power
### current draw
- motor stall at 12v = 1.2 A
- nodeMCU max at 5v = 0.4A
- dragon board at 5v usb max = 0.9A
- total = 2.5A , my lipo is 3.3A so I am good


---


# Legacy information, kept for old project reference
The Ros and gazebo files are in the src dir, the coppeliaSim files in the coppeliaSim dir
## Navigation
Basic object detection is made with ultrasonic
OpenVslam is used for one camera navigation
RTAB-Map is used for stereo camear and lidar http://wiki.ros.org/rtabmap_ros
useful stereo package http://wiki.ros.org/stereo_image_proc/Tutorials/ChoosingGoodStereoParameters
other useful resources
https://automaticaddison.com/working-with-ros-and-opencv-in-ros-noetic/
https://docs.opencv.org/master/dd/d53/tutorial_py_depthmap.html


## To start the ROS project

1. at the root of the workspace do `catkin_make && source devel/setup.sh`
2. Do `roslaunch pkg launchfile` u can choose from the ones below
   
   
launch rviz
`roslaunch healthbot rviz.launch`

launch gazebo using an urdf model
`roslaunch healthbot urdf_world.launch`

launch gazebo with the sdf model
` roslaunch healthbot sdf_world.launch`

Joint_state_publisher allows you to control the joint values, u can check it out in Rviz


## About the model
At the time of making this robot Rviz has problems with Gazebo's files, .sdf, so one has to convert to urdf files first.
The robot model was constructed in the gazebo gui, so the output is an sdf file, we need to convert that to urdf file to use in ros
use the command to convert, this is already done but if you want to make edits dont edit urdf directly unless u know what u r doing
`rosrun pysdf sdf2urdf.py src/healthbot/sdf/healthbot/model.sdf src/healthbot/urdf/healthbot.xacro`

_Note_ You will NOT be able to convert unless the gazebo sdf verison is 1.6 or below so go to the sdf file and edit it to 1.6 in the sdf verison tag

To check if your urdf is correct use this command
`check_urdf healthbot.xacro`

There are many 'partial' models, both in the sdf and the urdf dirs, but the main robot model is the 'healthbot' model, everything else is just kept there for reference

# I switched to orbslam
openvslam got shutdown so I had to switch to orbslam here are the instructions on how to run it below

follow this but u can run the built in r200 mono launch file
 https://medium.com/@mhamdaan/implementing-orb-slam-on-ubuntu-18-04-ros-melodic-606e668deffa
camera calibration 
`rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.26 image:=/usb_cam/image_raw camera:=/usb_cam --no-service-check`
- if u wanna test your own camera make sure to calibrate it and put the camera.yaml file to the parameter server ( u can use the medium tutorial above, it has a section on how to do it)
- run rviz with the config file in the orbslam2 and you will see params that you can edit
- my launch file has a bunch of remaps this is cuz my camera publishes to topic /usb_cam, while the code listens to topic /camera

## how to run
1. open the usb camera using `roslaunch usb_cam my_usb_cam_test.launch`
2. open orbslam2 docker (the ros fork, not the main repo) and launch the r200 mono launch file (or whatever launch file u made) 
	2-1. `docker run -it --name orbslam --net=host bd0dd564b380` or if u already have the stopped container then just do `docker start -i the-container-id`
	2-2.`roslaunch orb_slam2_ros myLaunch.launch`
3. open rviz with the mono config that I made, it is in the config dir
`rviz -d ./rviz_config_mono.rviz`


# Run SLAM with coppelliaSIM simlation instead of real robot
it is the same as how to run steps from above, only that we are doing it with coppeliasim instead of
real camera
1. run the copelliaSim folder that has slam in its name
2. run the orb slam launch file roslaunch src/orb_slam_2_ros/ros/launch/mySimulationLaunch.launch 
3. run rviz just like from above



## Misc
This is not important, but if u have troubles with gazebo make sure `GAZEBO_RESOURCE_PATH` has the path to the sdf file if you will load from sdf.
https://roboticsbackend.com/oop-with-ros-in-python/ -- good example code
