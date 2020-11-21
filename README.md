# Capstone project healthbot

## To start the project

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

## Misc
This is not important, but if u have troubles with gazebo make sure `GAZEBO_RESOURCE_PATH` has the path to the sdf file if you will load from sdf.
