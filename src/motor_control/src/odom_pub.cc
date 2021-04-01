/*
This file interfaces with the arduino serial and reads the Odometry data, issues
motor commands to the arduino motor driver, and publishes relative topics
good blog explaining it https://hackernoon.com/feedback-odometry-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-3-e9d8e4df6df1
http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
*/
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>

int odom_publish_rate = 60.0; // this is like 60Hz or 60 times a sec

int main (int argc, char** argv){
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle node;
    ros::Publisher odom_pub = node.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

   double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;

  ros::Time current_time, previous_time;
  // init time trackers to current time
  current_time = ros::Time::now();
  previous_time = ros::Time::now();

  ros::Rate r(odom_publish_rate); // this is a constructor to create the r var

  while (node.ok()){
      ros::spinOnce();
      current_time = ros::Time::now(); // update current time

    //change this
    double dt = (current_time - previous_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

//since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    previous_time = current_time;
    r.sleep();


  }

}