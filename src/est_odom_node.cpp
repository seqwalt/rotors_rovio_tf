#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <rovio/SrvResetToPose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <thread>

tf2::Quaternion q_mult(tf2::Quaternion q2, tf2::Quaternion q1);
tf2::Quaternion q_inv(tf2::Quaternion q);

ros::Subscriber gt_sub;
ros::Publisher est_odom;
nav_msgs::Odometry last_gt_msg;
nav_msgs::Odometry new_msg;
bool est_odom_unavail = true;
bool first_gt_callback = true;
bool first_rovio_callback = true;

void ground_truth_callback(const nav_msgs::Odometry &msg){
  last_gt_msg = msg; // will be used to initialize est_odom_callback
  if (est_odom_unavail){
    if (first_gt_callback){
      first_gt_callback = false;
      ROS_INFO("Using Ground Truth");
    }
    est_odom.publish(msg);
  }
}

void est_odom_callback(const nav_msgs::Odometry &msg){
  new_msg = msg;
  geometry_msgs::Quaternion msg_q = msg.pose.pose.orientation; // quaternion part of msg
  tf2::Quaternion IB_q = tf2::Quaternion(0.0, 0.38268, 0.0, -0.92388); // tf quaternion [x,y,z,w] for rotation from imu to quad body (45 deg about y-axis)

  if (first_rovio_callback){
    // ROTATE rovio orientation to align with gt orientation (since rovio orientation aligns with down-tilted imu)
    // ----------------------------------------------------------------
    geometry_msgs::Quaternion msg_gt_q = last_gt_msg.pose.pose.orientation; // last ground truth quaternion
    tf2::Quaternion sWrW_q; // quaternion for rotation from simulation world (sW) to rovio world (rW)
    tf2::fromMsg(msg_gt_q, sWrW_q); // load sWrW_q with data

    tf2::Quaternion rWI_q; // quaternion for rotation from rovio world (rW) x-axis to imu (I)
    tf2::fromMsg(msg_q, rWI_q); // load rWI_q with data

    tf2::Quaternion sWI_q = q_mult(sWrW_q, rWI_q); // quaternion for rotation from simulation world to imu
    new_msg.pose.pose.orientation = tf2::toMsg(sWI_q); // convert to message format
    // ----------------------------------------------------------------

    // ROTATE rovio angular and linear velocity
    // ----------------------------------------------------------------
    tf2::Vector3 angVel, linVel;
    tf2::fromMsg(msg.twist.twist.angular, angVel); // load angVel with data
    tf2::fromMsg(msg.twist.twist.linear, linVel); // load linVel with data

    new_msg.twist.twist.angular = tf2::toMsg(quatRotate(IB_q.inverse(), angVel));
    new_msg.twist.twist.linear  = tf2::toMsg(quatRotate(IB_q.inverse(), linVel));
    // ----------------------------------------------------------------

    // TRANSLATE rovio to start at last groundtruth position
    // ----------------------------------------------------------------
    geometry_msgs::Point msg_gt_p = last_gt_msg.pose.pose.position; // last ground truth position
    new_msg.pose.pose.position = msg_gt_p; // translate to last ground truth position
    // ----------------------------------------------------------------

    // Call ROVIO's reset pose service to apply the correct tranlation/rotation once
    // ---------------------------------
    ros::service::waitForService("/rovio/reset_to_pose", -1);
    // reset rovio pose to the adjusted pose
    rovio::SrvResetToPose::Request req;
    rovio::SrvResetToPose::Response resp;
    req.T_WM = new_msg.pose.pose;
    if (!ros::service::call("/rovio/reset_to_pose", req, resp)){
      ROS_ERROR("Service call failed");
    }
    // ---------------------------------

    std::cout << "Initializing Rovio ..." << std::endl;
    for (int i=15; i > 0; i--){
      std::cout << "Rovio starts in " << i << " seconds" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(1)); // wait
    }
    ROS_INFO("Using Estimated Odometry");
    est_odom_unavail = false;
    first_rovio_callback = false;

    // Shift from imu to body frame for visualization and state estimation
    tf2::Quaternion sWB_q = q_mult(IB_q, sWI_q); // rotate sW x-axis by IB_q, then by sWI_q
    new_msg.pose.pose.orientation = tf2::toMsg(sWB_q); // message format

    //est_odom.publish(new_msg);
    gt_sub.shutdown(); // stop subscribing to ground truth
  }
  else{
    // Shift from imu to body frame for visualization and state estimation
    tf2::Quaternion sWI_q;
    tf2::fromMsg(msg_q, sWI_q);
    tf2::Quaternion sWB_q = q_mult(IB_q, sWI_q); // rotate sW x-axis by IB_q, then by sWI_q
    new_msg.pose.pose.orientation = tf2::toMsg(sWB_q); // message format

    // ROTATE rovio angular and linear velocity
    // ----------------------------------------------------------------
    tf2::Vector3 angVel, linVel;
    tf2::fromMsg(msg.twist.twist.angular, angVel); // load angVel with data
    tf2::fromMsg(msg.twist.twist.linear, linVel); // load linVel with data
    new_msg.twist.twist.angular = tf2::toMsg(quatRotate(IB_q.inverse(), angVel));
    new_msg.twist.twist.linear  = tf2::toMsg(quatRotate(IB_q.inverse(), linVel));
    // ----------------------------------------------------------------

    est_odom.publish(new_msg);

  }
}

tf2::Quaternion q_mult(tf2::Quaternion q2, tf2::Quaternion q1){
  // return quaternion multiplication to rotate with q2, then q1
  // (i.e. Hamilton product)
  return q1 * q2;
}

tf2::Quaternion q_inv(tf2::Quaternion q){
  // return inverse of quaternion. Applies rotation in opposite direction
  return q.inverse();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "est_odom_node");
  ros::NodeHandle n;
  est_odom = n.advertise<nav_msgs::Odometry>("/est_odometry", 1);
  gt_sub = n.subscribe("/hummingbird/ground_truth/odometry", 1, ground_truth_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber rovio_imuInt_sub = n.subscribe("/odom_predictor_node/imu_integrated_odometry", 1, est_odom_callback, ros::TransportHints().tcpNoDelay());

  ros::MultiThreadedSpinner spinner(2); // Use a threads for each subscriber
  spinner.spin();

  return 0;
}
