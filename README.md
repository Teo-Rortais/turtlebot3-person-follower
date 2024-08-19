# TurtleBot3 : Person Follower

This project aims to use the Turtlebot3, the Jetson Orin Nano and a USB camera to make a person follower robot. To recognize a person I used the [jetson-inference repository](https://github.com/dusty-nv/jetson-inference). And for the robot to communicate with the Jetson I chose to use ROS Humble. 

For this project you will need to have The TurtleBot3 and the Jetson set up. And install ROS Humble on both devices and the [jetson-inference repository](https://github.com/dusty-nv/jetson-inference) on the Jetson. I made a full guide to install everything here : [SETUP](docs/SETUP.md)

Here is what the robot looks like :

| ![[robot_front.jpg]] | ![[robot_side.jpg]] |
| :------------------- | :-----------------: |

The robot is able to recognize a person and determine it's position on an image. From the position the program will send a command to control the mouvements of the robot.

![[person_follower_demo.gif|700]]

The robot will rotate to follow the person, it will also move backward if the person is too close or forward if the person is too far. The speed values can be modify in the code.

# How to use the project

You will need 3 different terminal. 2 of them must be on the TurtleBot and the third one, on the Jetson.

On the TurtleBot enter those two commands in two different terminal :

``` bash
ros2 launch turtlebot3_bringup robot.launch.py
```

``` bash
ros2 run v4l2_camera v4l2_camera_node
```

And on the Jetson : 

``` bash 
ros2 run person_follower person_follower
```

