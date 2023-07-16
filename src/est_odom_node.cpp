#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <rovio/SrvResetToPose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <thread>
#include <string>

tf2::Quaternion q_mult(tf2::Quaternion q2, tf2::Quaternion q1);
tf2::Quaternion q_inv(tf2::Quaternion q);

ros::Subscriber gt_sub;
ros::Publisher est_odom;
nav_msgs::Odometry last_gt_msg;
nav_msgs::Odometry new_msg;
bool est_odom_unavail = true;
bool first_gt_callback = true;
std::string frame_gt;
bool first_rovio_callback = true;
float rovio_start_time;
bool do_once = true;
double tot_secs = 60.0; // take one minute to initialize Rovio
double sec_counter = tot_secs;
tf2::Vector3 init_rovio_pos;
tf2::Quaternion init_rWI_q;
tf2::Vector3 init_angVel, init_linVel;

void ground_truth_callback(const nav_msgs::Odometry &msg){
  last_gt_msg = msg; // will be used to initialize est_odom_callback
  if (est_odom_unavail){
    if (first_gt_callback){
      first_gt_callback = false;
      frame_gt = msg.header.frame_id;
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
    rovio_start_time = msg.header.stamp.toSec();
    first_rovio_callback = false;
    ROS_INFO("ROVIO initialization done in:");
  }

  double time_diff = msg.header.stamp.toSec() - rovio_start_time;
  bool initializing_rovio = time_diff < tot_secs; // rovio is initialized for the first tot_secs seconds
  if (initializing_rovio){
    // enter this if-statement multiple times until rovio is sufficiently initialized
    if (tot_secs - time_diff <= sec_counter){
      ROS_INFO("%.1f seconds",tot_secs - time_diff);
      sec_counter -= 1.0;
    }
  } else {
    // ROTATE rovio orientation to align with gt orientation (since rovio orientation aligns with down-tilted imu)
    // ----------------------------------------------------------------
    geometry_msgs::Quaternion msg_gt_q = last_gt_msg.pose.pose.orientation; // last ground truth quaternion
    tf2::Quaternion sWrW_q; // quaternion for rotation from simulation world (sW) to rovio world (rW)
    tf2::fromMsg(msg_gt_q, sWrW_q); // load sWrW_q with data

    tf2::Quaternion rWI_q; // quaternion for rotation from rovio world (rW) x-axis to imu (I)
    tf2::fromMsg(msg_q, rWI_q); // load rWI_q with data

    if (do_once){
      tf2::fromMsg(msg.pose.pose.position, init_rovio_pos);
      init_rWI_q = rWI_q;
      est_odom_unavail = false;
      gt_sub.shutdown(); // stop subscribing to ground truth
    }

    tf2::Quaternion quat_diff = q_mult(sWrW_q, init_rWI_q.inverse());
    new_msg.pose.pose.orientation = tf2::toMsg(q_mult(quat_diff,rWI_q));
    // ----------------------------------------------------------------

    // ROTATE rovio angular and linear velocity
    // ----------------------------------------------------------------
    tf2::Vector3 angVel, linVel, gt_angVel, gt_linVel;
    tf2::fromMsg(msg.twist.twist.angular, angVel); // load angVel with data
    tf2::fromMsg(msg.twist.twist.linear, linVel); // load linVel with data
    tf2::fromMsg(last_gt_msg.twist.twist.angular, gt_angVel);
    tf2::fromMsg(last_gt_msg.twist.twist.linear, gt_linVel);

    if (do_once){
      init_angVel = quatRotate(quat_diff.inverse(), angVel);
      init_linVel = quatRotate(quat_diff.inverse(), linVel);
      do_once = false;
    }

    new_msg.twist.twist.angular = tf2::toMsg(quatRotate(quat_diff.inverse(), angVel) - init_angVel + gt_angVel);
    new_msg.twist.twist.linear  = tf2::toMsg(quatRotate(quat_diff.inverse(), linVel) - init_linVel + gt_linVel);
    // ----------------------------------------------------------------

    // TRANSLATE rovio to start at last groundtruth position
    // ----------------------------------------------------------------
    tf2::Vector3 gt_pos;
    tf2::fromMsg(last_gt_msg.pose.pose.position, gt_pos); // last ground truth position
    tf2::Vector3 curr_rovio_pos;
    tf2::fromMsg(msg.pose.pose.position, curr_rovio_pos);
    tf2::toMsg(curr_rovio_pos - init_rovio_pos + gt_pos, new_msg.pose.pose.position); // translate to last ground truth position
    // ----------------------------------------------------------------

    // publish msg
    new_msg.header.frame_id = frame_gt; // update to frame_id used in ground truth odometry
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
  gt_sub = n.subscribe("/mocap_node/vioquad/odom", 1, ground_truth_callback, ros::TransportHints().tcpNoDelay());
  ros::Subscriber rovio_imuInt_sub = n.subscribe("/odom_predictor_node/imu_integrated_odometry", 1, est_odom_callback, ros::TransportHints().tcpNoDelay());

  ros::MultiThreadedSpinner spinner(2); // Use a threads for each subscriber
  spinner.spin();

  return 0;
}
