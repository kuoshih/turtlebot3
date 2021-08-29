/*
 Copyright 2021 NCU MATH.
 Developer: Kuo-Shih Tseng (kuoshih@math.ncu.edu.tw)
 Description: This code activate a node "main." 
 This node subscribes three topics -- imu, odom, and scan. 
 You can access data from three Callback functions.
 $Revision: 1.0 $,  2021.08.29, revise the file from minibot

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
*/

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#define RAD2DEG(x) ((x)*180./M_PI)

void callback1(const ros::TimerEvent&);
void init_marker(void);

visualization_msgs::Marker marker;
uint32_t shape = visualization_msgs::Marker::CYLINDER;
ros::Publisher marker_pub;
int counter=0;

void callback1(const ros::TimerEvent&)
{// update maker location and publish it. 
    float x=1*cos(0.174*counter);
    float y=1*sin(0.174*counter);
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    //ROS_INFO("x=%f,y=%f\n",x,y);
    counter++;

    marker_pub.publish(marker);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
   /* 
    ROS_INFO("V x: [%f], y: [%f], z: [%f]", msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
    ROS_INFO("W x: [%f], y: [%f], z: [%f]", msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
    */
}


void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
/*
    printf("[YDLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    printf("[YDLIDAR INFO]: angle_increment : [%f] degree\n", RAD2DEG(scan->angle_increment));    
    for(int i = 0; i < 360; i++) 
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
	   if(degree > 0&& degree< 10)
        {printf("[YDLIDAR INFO]: angle-distance : [%f, %f, %i]\n", degree, scan->ranges[i], i);}
    }*/
    
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");


  ros::NodeHandle n;

  ros::Subscriber sub2 = n.subscribe("/imu", 1000, imuCallback);
  ros::Subscriber sub3 = n.subscribe("/odom", 1000, odomCallback);
  ros::Subscriber sub4 = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

  // create a timer callback
  ros::Timer timer1 = n.createTimer(ros::Duration(0.1), callback1);
  // create a topic "visualization_marker"
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  init_marker();

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */

  ros::spin();


  return 0;
}

void init_marker(void)
{
    // Initialize maker's setting.
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "target";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    // Tag(ACTION)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    //Tag(POSE)
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    //Tag(LIFETIME)
    marker.lifetime = ros::Duration();

}
