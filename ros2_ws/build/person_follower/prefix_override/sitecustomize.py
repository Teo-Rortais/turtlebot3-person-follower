import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/orinnano/turtlebot3-person-follower/ros2_ws/install/person_follower'
