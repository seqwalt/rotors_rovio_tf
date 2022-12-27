# This node does the following:
# 1. Publishes /est_odom node first using ground truth, and then using ROVIO once its available
# 2. Provides a transformation from the original rovio odometry to the quadrotor body-frame odometry

# Installation process of this package (see http://wiki.ros.org/catkin/Tutorials/CreatingPackage)
1) >>> cd ~/ROS/rovio_ws
2) >>> catkin_create_pkg rotors_rovio_tf std_msgs rospy roscpp
3) Update CMakeLists.txt file
4) Move est_odom_node.cpp into rotors_rovio_tf/src/ directory
5) >>> rosrun rotors_rovio_tf est_odom_node.cpp

# Note: est_odom_node.py was moved into the rotors_rovio_tf/ directory.
