#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <thread>

tf2::Quaternion q_mult(const tf2::Quaternion& q2, const tf2::Quaternion& q1);
tf2::Quaternion q_inv(const tf2::Quaternion& q);
void negativeQuat(geometry_msgs::Quaternion& q);

ros::Subscriber gt_sub;
ros::Publisher est_odom;
nav_msgs::Odometry gt_msg_odom, new_msg_odom;
const bool twist_in_body_frame = true;
bool est_odom_unavail = true;
bool first_gt_callback = true;
bool first_rovio_callback = true;
float rovio_start_time;
bool do_once = true;
double tot_secs = 15.0; // seconds to initialize Rovio
double sec_counter = tot_secs;
tf2::Vector3 init_rovio_pos;
tf2::Quaternion init_rWI_q;
tf2::Vector3 init_angVel, init_linVel;

void ground_truth_callback(const nav_msgs::Odometry &msg){
  gt_msg_odom = msg;
  negativeQuat(gt_msg_odom.pose.pose.orientation);    // so PAMPC doesn't yaw quad unnecessarily
  gt_msg_odom.header.frame_id = "odom";     // need to conform to mavros standard, for px4
  gt_msg_odom.child_frame_id = "base_link"; // need to conform to mavros standard, for px4
  gt_msg_odom.pose.pose.position.z -= 0.15; // remove 8 cm to align better with EKF2 estimation

  // Update twist to body frame
  if (twist_in_body_frame){
    tf2::Quaternion quat;
    tf2::fromMsg(msg.pose.pose.orientation, quat); // load quat with data
    tf2::Vector3 angVel, linVel, local_angVel, local_linVel;
    tf2::fromMsg(msg.twist.twist.angular, angVel); // load angVel with data
    tf2::fromMsg(msg.twist.twist.linear, linVel); // load linVel with data
    local_angVel = quatRotate(quat.inverse(), angVel);
    local_linVel = quatRotate(quat.inverse(), linVel);
    gt_msg_odom.twist.twist.angular = tf2::toMsg(local_angVel);
    gt_msg_odom.twist.twist.linear = tf2::toMsg(local_linVel);
  }

  // Publish in VIO is not ready
  if (est_odom_unavail){
    if (first_gt_callback){
      first_gt_callback = false;
      ROS_INFO("Using Ground Truth");
    }
    est_odom.publish(gt_msg_odom);
  }
}

void est_odom_callback(const nav_msgs::Odometry &msg){
  new_msg_odom = msg;
  new_msg_odom.header.frame_id = "odom";     // need to conform to mavros standard, for px4
  new_msg_odom.child_frame_id = "base_link"; // need to conform to mavros standard, for px4
  geometry_msgs::Quaternion msg_q = msg.pose.pose.orientation; // quaternion part of msg
  //tf2::Quaternion IB_q = tf2::Quaternion(0.0, 0.38268, 0.0, -0.92388); // tf quaternion [x,y,z,w] for rotation from imu to quad body (45 deg about y-axis)
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
    geometry_msgs::Quaternion msg_gt_q = gt_msg_odom.pose.pose.orientation; // last ground truth quaternion
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
    tf2::Quaternion quat_world2body = q_mult(quat_diff,rWI_q);
    new_msg_odom.pose.pose.orientation = tf2::toMsg(quat_world2body);
    // ----------------------------------------------------------------

    // ROTATE rovio angular and linear velocity
    // ----------------------------------------------------------------
    tf2::Vector3 angVel, linVel, gt_angVel, gt_linVel;
    tf2::fromMsg(msg.twist.twist.angular, angVel); // load angVel with data
    tf2::fromMsg(msg.twist.twist.linear, linVel);  // load linVel with data
    tf2::fromMsg(gt_msg_odom.twist.twist.angular, gt_angVel);
    tf2::fromMsg(gt_msg_odom.twist.twist.linear, gt_linVel);
    if (do_once){
      init_angVel = quatRotate(quat_diff.inverse(), angVel);
      init_linVel = quatRotate(quat_diff.inverse(), linVel);
      do_once = false;
    }
    if (twist_in_body_frame) {
      tf2::Vector3 local_angVel, local_linVel;
      local_angVel = quatRotate(quat_diff.inverse(), angVel);
      local_linVel = quatRotate(quat_diff.inverse(), linVel);
      new_msg_odom.twist.twist.angular = tf2::toMsg(local_angVel);
      new_msg_odom.twist.twist.linear  = tf2::toMsg(local_linVel);
    } else {
      tf2::Vector3 world_angVel, world_linVel;
      world_angVel = quatRotate(quat_diff.inverse(), angVel) - init_angVel + gt_angVel;
      world_linVel = quatRotate(quat_diff.inverse(), linVel) - init_linVel + gt_linVel;
      new_msg_odom.twist.twist.angular = tf2::toMsg(world_angVel);
      new_msg_odom.twist.twist.linear  = tf2::toMsg(world_linVel);
    }
    // ----------------------------------------------------------------

    // TRANSLATE rovio to start at last groundtruth position
    // ----------------------------------------------------------------
    tf2::Vector3 gt_pos;
    tf2::fromMsg(gt_msg_odom.pose.pose.position, gt_pos); // last ground truth position
    tf2::Vector3 curr_rovio_pos;
    tf2::fromMsg(msg.pose.pose.position, curr_rovio_pos);
    tf2::toMsg(curr_rovio_pos - init_rovio_pos + gt_pos, new_msg_odom.pose.pose.position); // translate to last ground truth position
    // ----------------------------------------------------------------

    // publish msg
    est_odom.publish(new_msg_odom);
  }
}

tf2::Quaternion q_mult(const tf2::Quaternion& q2, const tf2::Quaternion& q1){
  // return quaternion multiplication to rotate with q2, then q1
  // (i.e. Hamilton product)
  return q1 * q2;
}

tf2::Quaternion q_inv(const tf2::Quaternion& q){
  // return inverse of quaternion. Applies rotation in opposite direction
  return q.inverse();
}

void negativeQuat(geometry_msgs::Quaternion& q) {
  // Multiply (-1) for EACH term of quaternion. Note quaternion meaning will remain the same
  // Useful since acado PAMPC considers each quaternion coefficient as a separate variable
  q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z;
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
