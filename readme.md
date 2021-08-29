
## Abstract
This sample code is for turtlebot3 simulator and turtlebot3.
This code activates a node "main" and the other sensor nodes. 
Main node subscribes three topics -- imu, odom, and scan. 
Main node publish a marker, which moves around the turtlebot3.
You can access data from three Callback functions in src/main.cpp.
You can see it in rviz.

## About us

Developer:   
* Kuo-Shih Tseng   
Contact: kuoshih@math.ncu.edu.tw   
Date: 2021/08/29  
License: Apache 2.0  


## Compile the code
$ cd catkin_ws/src  
$ git clone https://github.com/kuoshih/turtlebot3   
$ cd ..  
$ catkin_make  or catin build

Or Download this this code to pi\catkin_ws\src.   
Unzip hypharos_minibot.zip to replace the original code.
  
$cd catkin_ws  
$catkin_make  or catin build

## Run the code  (for gazebo simulator)
You can install turtlebot_simulator first.  
See the details in https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/  
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch # Terminal 1  

$ roslaunch turtlebot3 project_sample.launch # Terminal 2  

## Run the code  (for turtlebot3 robot)
Run the bringup in turtlebot3.  
$ roscore  # Terminal 1 in your laptop or PC.  

$ roslaunch turtlebot3_bringup turtlebot3_robot.launch# Terminal 1 in turtlebot3.  

$ roslaunch turtlebot3_gazebo turtlebot3_world.launch # Terminal 2 in your laptop or PC.  

The bashrc configuration about two devices' IP can be found in  
https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup  
https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup  
## rviz
Then, you can see the rviz.  
![alt text](https://github.com/kuoshih/turtlebot3/blob/main/document/rviz.png)  
## Edit code  
You can edit src/main.cpp for your project.  

## About Turtlebot3
More information about Turtlebot3 can be found   https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/ .
