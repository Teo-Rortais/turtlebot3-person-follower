# TurtleBot3 : Person Follower

This project aims to use the TurtleBot3, the Jetson Orin Nano and a USB camera to make a person follower robot. To recognize a person I used the [jetson-inference repository](https://github.com/dusty-nv/jetson-inference). And for the robot to communicate with the Jetson I chose to use ROS Humble. 

For this project you will need to have The TurtleBot3 and the Jetson set up. And install ROS Humble on both devices and the [jetson-inference repository](https://github.com/dusty-nv/jetson-inference) on the Jetson. I made a full guide to install everything here : [SETUP](docs/SETUP.md)

Here is what the robot looks like:

| <img src="https://github.com/Teo-Rortais/turtlebot3-person-follower/blob/main/docs/images/robot_front.jpg" width="100%"> | <img src="https://github.com/Teo-Rortais/turtlebot3-person-follower/blob/main/docs/images/robot_side.jpg" width="100%"> |
| :----------------------------------------------------------------------------------------------------------------------- | :---------------------------------------------------------------------------------------------------------------------: |

The robot is able to recognize a person and determine its position on an image. From the position the program will send a command to control the movements of the robot.

<img src="https://github.com/Teo-Rortais/turtlebot3-person-follower/blob/main/docs/images/person_follower_demo.gif" width="100%">

The robot will rotate to follow the person, it will also move backwards if the person is too close or forwards if the person is too far. The speed values can be modified in the code.

# How to use the project

You will need 3 different terminals. 2 of them must be on the TurtleBot and the third one, on the Jetson.

On the TurtleBot, enter these two commands in two different terminals:

``` bash
ros2 launch turtlebot3_bringup robot.launch.py
```

``` bash
ros2 run v4l2_camera v4l2_camera_node
```

And on the Jetson, in the project directory: 

``` bash 
source install/setup.bash
ros2 run person_follower person_follower
```


# Useful links 

To build this project, I primarily referred to the following resources and tutorials. Additionally, I frequently consulted various forums to navigate the errors and challenges I encountered.

- Jetson-Inference repository : https://github.com/dusty-nv/jetson-inference
- TurtleBot3 Manual : https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
- ROS 2 Documentation (Humble) : https://docs.ros.org/en/humble/