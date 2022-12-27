#!/usr/bin/env python3

# This file does the following:
# 1. Publishes /est_odom node first using ground truth, and then using ROVIO once its available
# 2. Provides a transformation from the original rovio odometry to the quadrotor body-frame odometry

# Notes on profiling this script
# using pprofile >>> sudo apt install python3-pprofile
# run node with  >>> pprofile3 --out est_odom_node.txt est_odom_node.py
# in est_odom_node.txt, Ctrl-f for "est_odom_node.py" to view timing data

import rospy
import time
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from rovio.srv import SrvResetToPose
from geometry_msgs.msg import Quaternion, Point
import tf.transformations as TF

global est_odom_unavail
global gt_sub
global first_rovio_callback
global first_gt_callback

def ground_truth_callback(data):
    global est_odom_unavail
    global first_gt_callback
    global last_gt_msg

    msg = data
    last_gt_msg = msg # will be used to initialize est_odom_callback
    if est_odom_unavail:
        if first_gt_callback:
            first_gt_callback = 0
            rospy.loginfo("Using Ground Truth") # only display once to avoid lag
        pub.publish(msg)

def est_odom_callback(data):
    global est_odom_unavail
    global first_rovio_callback
    global gt_sub

    msg = data
    data_pose = data.pose.pose
    IB_q = [0.0, 0.38268, 0.0, -0.92388] # tf quaternion [x,y,z,w] for rotation from imu to quad body (45 deg about y-axis)

    if first_rovio_callback:
        global last_gt_msg
        last_gt_pose = last_gt_msg.pose.pose

        # ROTATE rovio orientation to align with gt orientation (since rovio orientation aligns with down-tilted imu)
        # ----------------------------------------------------------------
        sWrW_q_data = last_gt_pose.orientation # quaternion for rotation from simulation world (sW) to rovio world (rW) (rovio is initialized when quad is at last_gt_pose)
        sWrW_q = [sWrW_q_data.x, sWrW_q_data.y, sWrW_q_data.z, sWrW_q_data.w]
        rWI_q_data = data_pose.orientation # quaternion for rotation from rovio world (rW) x-axis to imu (I)
        rWI_q = [rWI_q_data.x, rWI_q_data.y, rWI_q_data.z, rWI_q_data.w]
        sWI_q = q_mult(sWrW_q, rWI_q) # quaternion for rotation from simulation world to imu
        msg.pose.pose.orientation = Quaternion(*sWI_q) # convert to correct message format
        # ----------------------------------------------------------------

        # TRANSLATE rovio to start at last groundtruth position
        # ----------------------------------------------------------------
        gt_pos = [last_gt_pose.position.x, last_gt_pose.position.y, last_gt_pose.position.z] # last ground truth position
        msg.pose.pose.position = Point(*gt_pos) # translate to last ground truth position
        # ----------------------------------------------------------------

        # Call ROVIO's reset pose service to apply the correct tranlation/rotation once
        # ---------------------------------
        rospy.wait_for_service('/rovio/reset_to_pose')
        try:
            resetToPose = rospy.ServiceProxy('/rovio/reset_to_pose', SrvResetToPose) # create service call function handle
            resetToPose(msg.pose.pose)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        # ---------------------------------

        print('Initializing Rovio...')
        time.sleep(2)
        rospy.loginfo("Using Estimated Odometry") # only display once to avoid lag
        est_odom_unavail = 0
        first_rovio_callback = 0

        # Shift from imu to body frame for visualization and state estimation
        sWI_q_data = msg.pose.pose.orientation
        sWI_q = [sWI_q_data.x, sWI_q_data.y, sWI_q_data.z, sWI_q_data.w]
        sWB_q = q_mult(IB_q, sWI_q) # rotate sW x-axis by IB_q, then by sWI_q
        msg.pose.pose.orientation = Quaternion(*sWB_q) # message format

        pub.publish(msg)
        gt_sub.unregister() # stop subscribing to ground truth
    else:
        # Shift from imu to body frame for visualization and state estimation
        sWI_q_data = data_pose.orientation
        sWI_q = [sWI_q_data.x, sWI_q_data.y, sWI_q_data.z, sWI_q_data.w]
        sWB_q = q_mult(IB_q, sWI_q) # rotate sW x-axis by IB_q, then by sWI_q
        msg.pose.pose.orientation = Quaternion(*sWB_q) # message format
        pub.publish(msg)

def q_mult(quaternion2, quaternion1):
    # return quaternion multiplication to rotate with q2, then q1
    # (i.e. Hamilton product)
    # Note: input quaternions are lists --> [x, y, z, w]
    result_quaternion = TF.quaternion_multiply(quaternion1, quaternion2)
    return result_quaternion

def q_inv(quaternion):
    # return inverse of quaternion. Applies rotation in opposite direction
    # Note: quaternion is a list --> [x, y, z, w]
    q = quaternion
    q[3] = -q[3] # flip sign of w
    return q

if __name__ == '__main__':
    est_odom_unavail=1
    first_gt_callback=1
    first_rovio_callback=1

    rospy.init_node('est_odom_node')

    pub = rospy.Publisher("/est_odometry", Odometry, queue_size=1)
    gt_sub = rospy.Subscriber("/hummingbird/ground_truth/odometry", Odometry, ground_truth_callback)
    rospy.Subscriber("/rovio/odometry", Odometry, est_odom_callback) # uncomment for rovio

    try:
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
