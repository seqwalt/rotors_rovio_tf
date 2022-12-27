# ROS node has following features:
1. Publishes /est_odom node first using ground truth, and then using ROVIO once its available
2. Provides a transformation from the original rovio odometry to the quadrotor body-frame odometry

# Usage:  
```rosrun rotors_rovio_tf est_odom_node.cpp```

# Dependencies:  
odom_predictor (link coming soon)
