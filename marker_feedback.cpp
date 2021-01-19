//
// Created by inderjeet on 1/26/20.
//
// Contributed by indraneel

#include "marker_feedback.h"

MarkerFeedback::MarkerFeedback(std::shared_ptr<ros::NodeHandle> &nh_ptr_,\
              tf2_ros::StaticTransformBroadcaster sb):nodehandle_ptr(nh_ptr_),static_broadcaster(sb),\
              tfListener(tfBuffer,*nh_ptr_) {
  
  logger_ = spdlog::get("move_controller")->clone("marker_feedback");
  global_map_client_ = nodehandle_ptr->serviceClient<gor_map_server::MapService>("gor_map_server");

  logger_->info("Marker feedback object created!");

  ipu_msgs_subs_ = nodehandle_ptr->subscribe("/ipu_data", 1, &MarkerFeedback::ipu_msg_callback, this);
  global_pose_pub = nodehandle_ptr->advertise<geometry_msgs::PoseArray>("/butler_navigation/global_pose_viz", 1);
  node_alive = true;
  ipu_data_available_ = false;
  update_ipu_thread_ = std::thread(&MarkerFeedback::wait_for_ipu_data, this);
  processed_dm_str = "000.000";

  // Initialise the global poses array for rviz visualisation
  global_poses_viz = geometry_msgs::PoseArray();
  global_poses_viz.header.seq = 0; 

#if DEBUG_MODE == true
  // Publish a null odom->base_link transform for testing
  logger_->info("Debug Mode is enabled!!");
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "odom";
  static_transformStamped.child_frame_id = "base_footprint";
  static_transformStamped.transform.translation.x = 0.0f;
  static_transformStamped.transform.translation.y = 0.0f;
  static_transformStamped.transform.translation.z = 0.0f;
  tf2::Quaternion quat;
  quat.setRPY(0.0f, 0.0f, 0.0f);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster.sendTransform(static_transformStamped);

#endif

  // Initialisation
  x_error_buffer = 0.0f;
  y_error_buffer = 0.0f;
  yaw_error_buffer = 0.0f; // In degrees
  localisation_heading = 0.0f; // in degrees
  transform_localisation.transform.translation.x = 0.0f;
  transform_localisation.transform.translation.y = 0.0f;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transform_localisation.transform.rotation.x = q.x();
  transform_localisation.transform.rotation.y = q.y();
  transform_localisation.transform.rotation.z = q.z();
  transform_localisation.transform.rotation.w = q.w();

  // Start localisation
  localisation_thread_ = std::thread(&MarkerFeedback::localisation_handler, this);
}

MarkerFeedback::~MarkerFeedback() {
  node_alive = false;
  update_ipu_thread_.join();
  localisation_thread_.join();
}


void MarkerFeedback::wait_for_ipu_data() {

   logger_->info("Waiting for IPU data!");
   while(node_alive)
   {
      std::unique_lock<std::mutex> ipu_data_lock(ipu_data_mutex_);
      ipu_data_cv_.wait(ipu_data_lock, [this] { return ipu_data_available_;});
      if(!message_queue.empty())
      {
        marker_data_ = message_queue.front();
        message_queue.pop();
        if(marker_data_.camera_name == "bottom")
        {
          // Process data
          process_ipu_data();
        }
        if(message_queue.empty())
          ipu_data_available_ = false;
      }
      else
      {
        logger_->error("Message queue empty!!");
      }
   }
}

void MarkerFeedback::process_ipu_data() {

  // Only process the IPU reading if you have not already processed it before else just print it
  if(marker_data_.dm != processed_dm_str )
  {
    logger_->info("Bottom Camera Data: DM: {} , deltaX = {}, deltaY = {},"
                      " dtheta= {}",
                      marker_data_.dm, marker_data_.deltaX, marker_data_.deltaY,
                      marker_data_.heading);
    // Inform the tree
    blackboard->set<bool>("is_got_bot_DM", true); // NOLINT

    processed_dm_str = marker_data_.dm;
    gor_map_server::MapService map_service;
    map_service.request.datamatrix_string = marker_data_.dm;

    if(global_map_client_.call(map_service)) {

      if(map_service.response.marker_found)
      {
        logger_->debug("2DPose of {} is X: {}, Y: {}",map_service.request.datamatrix_string,
                              map_service.response.x,map_service.response.y);

        // Calculate global pose of butler from the 2D Marker Pose and IPU reading
        // Apply GOR frame to REP-105 frame Transformations
        // CAUTION : These calculation are butler face specific, currently only while moving towards suspension side
        float delta_x_ros_frame = marker_data_.deltaY / 100.0f;
        float delta_y_ros_frame = marker_data_.deltaX / 100.0f;

        // -ve sign is for convert clockwise(+ve) to anticlockwise(-ve ) and subtracted with 90 to align with ROS Frame #
        float ipu_heading_ros_frame = -1*marker_data_.heading + 360.0f + 90.0f;
        //if(ipu_heading_ros_frame < -180.0f)
        //    ipu_heading_ros_frame += 360.0;
        
        //if(ipu_heading_ros_frame > 180.0)
        //    ipu_heading_ros_frame -= 360.0;
        
        float ipu_heading_ros_frame_rad = degreesToRadians(ipu_heading_ros_frame);

        //TODO: Calculate global pose according to orientation
        geometry_msgs::Pose global_pose = geometry_msgs::Pose();
        global_pose.position.x = map_service.response.x + (delta_y_ros_frame * sin(ipu_heading_ros_frame_rad) + (delta_x_ros_frame)*cos(ipu_heading_ros_frame_rad) );
        global_pose.position.y = map_service.response.y + (delta_y_ros_frame * cos(ipu_heading_ros_frame_rad) - (delta_x_ros_frame)*sin(ipu_heading_ros_frame_rad) );
        tf::Quaternion simple_quat = tf::Quaternion();
        simple_quat.setEuler(ipu_heading_ros_frame_rad,0.0f,0.0f); 
        global_pose.orientation.x = simple_quat.getX();
        global_pose.orientation.y = simple_quat.getY();
        global_pose.orientation.z = simple_quat.getZ();
        global_pose.orientation.w = simple_quat.getW();

        logger_->info("Ground Truth Pose of butler in map X: {} Y:{} theta: {} deg", global_pose.position.x,global_pose.position.y,ipu_heading_ros_frame);

        // Publish pose to view in rviz
        global_poses_viz.header.seq++;
        global_poses_viz.header.stamp = marker_data_.header.stamp;
        global_poses_viz.header.frame_id = "map"; 
        global_poses_viz.poses.push_back(global_pose);
        global_pose_pub.publish(global_poses_viz);

        // Find error in localisation from the tf tree and push error to buffer
        try{
          transformStamped = tfBuffer.lookupTransform("odom", "base_footprint",
                                  ros::Time(0));
          double yaw, pitch, roll;
          tf::Quaternion qcurrent(transformStamped.transform.rotation.x,transformStamped.transform.rotation.y,\
                                  transformStamped.transform.rotation.z,transformStamped.transform.rotation.w);
          //qcurrent.normalize();
          tf::Matrix3x3 mat(qcurrent);
          mat.getRPY(roll, pitch, yaw);
          logger_->debug("Current Odometry Pose is X: {} Y:{} T:{} degrees",transformStamped.transform.translation.x,\
                                          transformStamped.transform.translation.y,radiansToDegrees(yaw));

          /** @brief: Localisation
           *  tf_map_to_odom_new * tf_odom_to_base = tf_ground_truth 
           * tf_map_to_odom_new = tf_ground_truth * tf_odom_to_base_inv
          */
          if(LOCALISATION_ENABLED)
          {
            logger_->debug("Residual error buf in X:{} Y:{} T:{}",x_error_buffer,y_error_buffer,yaw_error_buffer);

            tf::Transform tf_odom_to_base = tf::Transform(tf::createQuaternionFromRPY(0, 0, yaw), tf::Vector3(transformStamped.transform.translation.x, transformStamped.transform.translation.y, 0.0));
            tf::Transform tf_ground_truth = tf::Transform(tf::createQuaternionFromRPY(0, 0, ipu_heading_ros_frame_rad), tf::Vector3(global_pose.position.x, global_pose.position.y, 0.0));
            tf::Transform tf_map_to_odom_new = tf_ground_truth * tf_odom_to_base.inverse();

            // Sanity Check calculations
            tf::Transform tf_bot_position = tf_map_to_odom_new*tf_odom_to_base;
            tf::Quaternion bot_rotation = tf_bot_position.getRotation();
            tf::Matrix3x3 mat_rot(bot_rotation);
            mat_rot.getRPY(roll, pitch, yaw);
            logger_->debug("[SanityCheck] New bot position in tf tree is X:{} Y:{} theta:{} ",tf_bot_position.getOrigin().getX(),tf_bot_position.getOrigin().getY(),radiansToDegrees(yaw));

            tf::Quaternion map_to_odom_new_rot = tf_map_to_odom_new.getRotation();
            tf::Matrix3x3 mat_temp(map_to_odom_new_rot);
            mat_temp.getRPY(roll, pitch, yaw);
            
            // Update the error buffers
            if(X_CORRECTION)
              x_error_buffer+= (tf_map_to_odom_new.getOrigin().getX() - transform_localisation.transform.translation.x);
            if(Y_CORRECTION)
              y_error_buffer+= (tf_map_to_odom_new.getOrigin().getY() - transform_localisation.transform.translation.y);
            float a = radiansToDegrees(yaw)-localisation_heading;
            a += (a>180.0f) ? -360.0f : (a<-180.0f) ? 360.0f : 0.0f;
            if(THETA_CORRECTION)
              yaw_error_buffer += a;

            logger_->debug("Updated localisation error buffers in X:{} Y:{} T:{}",x_error_buffer,y_error_buffer,yaw_error_buffer,radiansToDegrees(yaw));
          }
            
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          logger_->warn("map->base_footprint look up failed!");
          ros::Duration(1.0).sleep();
        }


      }
      else
      {
        logger_->error("DM string not in the gor map. DM: {}", map_service.request.datamatrix_string);
      }
    }

    else {
      logger_->error("Gor map server service call failed!");
    }
  }
  else
  {
    logger_->debug("Bottom Camera Data: DM: {} , deltaX = {}, deltaY = {},"
                      " dtheta= {}",
                      marker_data_.dm, marker_data_.deltaX, marker_data_.deltaY,
                      marker_data_.heading);
  }
}

void MarkerFeedback::ipu_msg_callback(const nav_2_0_common_msgs::DMData &ipu_msg) {

  message_queue.push(ipu_msg);
  ipu_data_available_ = true;
  ipu_data_cv_.notify_one();
}

/** Runs at a fixed frequency tries to maintain the localisation error at zero using map->odom transform */
void MarkerFeedback::localisation_handler()
{
  logger_->info("Localisation is game!");
    while(node_alive)
   {
      
      static tf2_ros::TransformBroadcaster br;
      transform_localisation.header.stamp = ros::Time::now();
      transform_localisation.header.frame_id = "map";
      transform_localisation.child_frame_id = "odom";

      // Correct localisation error if any
      transform_localisation.transform.translation.x += (x_error_buffer);
      transform_localisation.transform.translation.y += (y_error_buffer);
      transform_localisation.transform.translation.z = 0.0;
      tf2::Quaternion q;

      localisation_heading += yaw_error_buffer;
      if(localisation_heading < -180.0f)
          localisation_heading += 360.0;
      
      if(localisation_heading > 180.0)
          localisation_heading -= 360.0;
      q.setRPY(0, 0, degreesToRadians(localisation_heading));
      transform_localisation.transform.rotation.x = q.x();
      transform_localisation.transform.rotation.y = q.y();
      transform_localisation.transform.rotation.z = q.z();
      transform_localisation.transform.rotation.w = q.w();

      // Update error buffers
      x_error_buffer=x_error_buffer*(1-1);
      y_error_buffer=y_error_buffer*(1-1);
      yaw_error_buffer=yaw_error_buffer*(1-1);

      br.sendTransform(transform_localisation);

      std::this_thread::sleep_for(std::chrono::milliseconds(1000/LOCALISATION_FREQ));
   }

}

