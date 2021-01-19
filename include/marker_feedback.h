//
// Created by inderjeet on 1/26/20.
//
// Contributed by indraneel

#ifndef MARKER_FEEDBACK_H_
#define MARKER_FEEDBACK_H_

#include <iostream>
#include <condition_variable>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <thread>
#include <gor_map_server/MapService.h>
#include <nav_2_0_common_msgs/DMData.h>
#include <spdlog/spdlog.h>
#include <queue>
#include <cmath>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

#include "behaviortree_cpp_v3/bt_factory.h"

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)

// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)


/** Compile time config macros */
#define DEBUG_MODE false
#define LOCALISATION_FREQ  150 // In Hz
#define X_CORRECTION  true
#define Y_CORRECTION  true
#define THETA_CORRECTION true
#define LOCALISATION_ENABLED true

class MarkerFeedback {
 private:
  std::condition_variable tfDataCondition;

  std::thread update_ipu_thread_;
  std::thread localisation_thread_;
  std::condition_variable ipu_data_cv_;
  std::mutex ipu_data_mutex_;
  bool ipu_data_available_;
  bool node_alive;

  geometry_msgs::Pose2D ipu_ground_truth;
  geometry_msgs::PoseArray global_poses_viz;

  //tf::TransformBroadcaster map_to_odom_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  nav_2_0_common_msgs::DMData marker_data_;
  std::queue<nav_2_0_common_msgs::DMData> message_queue;
  std::shared_ptr<spdlog::logger> logger_;
  std::string processed_dm_str;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::TransformStamped transform_localisation;

  void localisation_handler();
  float x_error_buffer;
  float y_error_buffer;
  float yaw_error_buffer;
  float localisation_heading;


 public:
  ros::Subscriber ipu_msgs_subs_;
  ros::Publisher global_pose_pub;
  ros::ServiceClient global_map_client_;
  std::shared_ptr<ros::NodeHandle> nodehandle_ptr;
  std::shared_ptr<BT::Blackboard> blackboard;


  MarkerFeedback(std::shared_ptr<ros::NodeHandle> &nh_ptr_,\
                  tf2_ros::StaticTransformBroadcaster sb);
  ~MarkerFeedback();

  void process_ipu_data();
  void wait_for_ipu_data();
  void update_ipu_data();
  void ipu_msg_callback(const nav_2_0_common_msgs::DMData &ipu_msg);

};

#endif //MARKER_FEEDBACK_H_
