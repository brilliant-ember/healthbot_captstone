# Capstone project healthbot

launch rviz
`roslaunch healthbot rviz.launch`

launch gazebo
`roslaunch healthbot urdf_world.launch`

Joint_state_publisher allows you to control the joint values, u can check it out in Rviz

Make sure `GAZEBO_RESOURCE_PATH` has the path to the sdf file if you will load from sdf.

In case you want to convert an sdf to urdf u can use the following line as an example
`rosrun pysdf sdf2urdf.py src/gazebo_sim/sdf_models/my_robot/model.sdf src/gazebo_sim/urdf/mobile_base.xacro`
