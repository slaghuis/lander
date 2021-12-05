// Copyright 2021 Xeni Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* *********************************************************************************
    Seek the landing target.
  
    The general strategy here is adjust the altitude and the pose of the drone to 
    a safe flying height, and increase velocity towards the landing target.
    Once we are where the target should be we check to see if the target is found.
    If not, we hang around untill a target is sighted.
    With the target on sight, we transition to the APPROACH state.    
* *********************************************************************************/
#include "lander/state.hpp"
#include "lander/lander_node.h"
#include "lander/utils.hpp"

void SeekingState::execute_mission() 
{
  RCLCPP_INFO(node_->get_logger(), "Seek: Flying to target zone.");
  
  //Read the parameters needed in this object
  node_->get_parameter("target_seek_altitude", target_seek_altitude_);
  node_->get_parameter("max_yaw_speed", max_yaw_speed_);
  node_->get_parameter("max_speed_xy", max_speed_xy_);
  node_->get_parameter("max_speed_z", max_speed_z_);
  node_->get_parameter("waypoint_radius_error", waypoint_radius_error_);
  node_->get_parameter("yaw_threshold", yaw_threshold_);
  node_->get_parameter("altitude_threshold", altitude_threshold_); 
   
  // and the vector based parameters
  rclcpp::Parameter pid_xy_settings_param = node_->get_parameter("pid_xy");
  std::vector<double> pid_xy_settings = pid_xy_settings_param.as_double_array(); 
  pid_x   = std::make_shared<PID>(0.5, max_speed_xy_, -max_speed_xy_, (float)pid_xy_settings[0], (float)pid_xy_settings[1], (float)pid_xy_settings[2]);

  rclcpp::Parameter pid_z_settings_param = node_->get_parameter("pid_z");
  std::vector<double> pid_z_settings = pid_z_settings_param.as_double_array(); 
  pid_z   = std::make_shared<PID>(0.5, max_speed_z_, -max_speed_z_, (float)pid_z_settings[0], (float)pid_z_settings[1], (float)pid_z_settings[2]);

  rclcpp::Parameter pid_yaw_settings_param = node_->get_parameter("pid_yaw");
  std::vector<double> pid_yaw_settings = pid_yaw_settings_param.as_double_array(); 
  pid_yaw   = std::make_shared<PID>(0.5, max_yaw_speed_, -max_yaw_speed_, (float)pid_yaw_settings[0], (float)pid_yaw_settings[1], (float)pid_yaw_settings[2]);
  
  // Fly to where the target should be
  fly_to_waypoint( node_->landing_target_position() );
  
  // Read the transform to see if the target is in view (i.e has been seen in the last second.
  // If not, hang around here.  Maybe the flight controller will pick up a flat battery and land.
  // What if the wind blows?  Maybe I can put x,y,z PID's down to hover right here?
  rclcpp::Rate loop_rate(2);
  while (! node_->target_is_close() ) {
    RCLCPP_ERROR(node_->get_logger(), "Seek: Target is out of sight!");
    loop_rate.sleep();
  }
  node_->TransitionTo(new ApproachingState);  
}

void SeekingState::abort()
{
  // Stabilise flight
  node_->stop_movement();
  
  // Go sit under a tree till you are called 
  node_->TransitionTo(new PendingState);
}


bool SeekingState::is_working() 
{
  return true;  // Only pending state gets to sit under a tree and do nothing.
}
  
bool SeekingState::fly_to_waypoint(std::shared_ptr<geometry_msgs::msg::PoseStamped> wp) {
    rclcpp::Rate loop_rate(2);

    bool waypoint_is_close_, altitude_is_close_, pose_is_close_;
    float target_x_, target_y_, target_z_;

    target_x_ = wp->pose.position.x;
    target_y_ = wp->pose.position.y;
    target_z_ = target_seek_altitude_;                 //wp->pose.position.z  will crash us into the ground!
   
    geometry_msgs::msg::Twist setpoint = geometry_msgs::msg::Twist();
    setpoint.linear.x = 0.0;
    setpoint.linear.y = 0.0;
    setpoint.linear.z = 0.0;
    setpoint.angular.x = 0.0;
    setpoint.angular.y = 0.0;
    setpoint.angular.z = 0.0;

    double x, y, z, w;
    float err_x, err_y, err_z, err_dist;
    double yaw_to_target;
    double yaw_error;

    // First correct the yaw        
    pid_yaw->restart_control();

    do {
      node_->read_position(&x, &y, &z, &w);  // Current position according to tf2

      err_x = target_x_ - x; 
      err_y = target_y_ - y;
      yaw_to_target = atan2(err_y, err_x);
      
      yaw_error = getDiff2Angles(yaw_to_target, w, M_PI);
      pose_is_close_ = (fabs(yaw_error) < yaw_threshold_);
      if (pose_is_close_) {
        break;
      }
      setpoint.angular.z = pid_yaw->calculate(0, -yaw_error);         // correct yaw error down to zero  
      
      node_->set_velocity(setpoint);
      loop_rate.sleep();  // Give the drone time to move
    } while (!pose_is_close_);  

    // Now that we are ponting, keep on adjusting yaw, but include altitude and foreward velocity
    pid_x->restart_control();
    pid_z->restart_control();
    do {
      node_->read_position(&x, &y, &z, &w);  // Current position according to tf2

      err_x = target_x_ - x; 
      err_y = target_y_ - y;
      err_z = target_z_ - z;

      err_dist = sqrt(pow(err_x,2) + pow(err_y,2));        
      waypoint_is_close_ = (err_dist < waypoint_radius_error_);

      altitude_is_close_ = ( abs(err_z) < altitude_threshold_);
    
      yaw_to_target = atan2(err_y, err_x);      
      yaw_error = getDiff2Angles(yaw_to_target, w, M_PI);
      pose_is_close_ = (fabs(yaw_error) < yaw_threshold_);
      
      if (!(waypoint_is_close_ && altitude_is_close_)) {
        setpoint.angular.z = pid_yaw->calculate(0, -yaw_error);         // correct yaw error down to zero  
        setpoint.linear.z = pid_z->calculate(0, -err_z);                // correct altitude
        setpoint.linear.x = pid_x->calculate(0, -err_dist);              // fly
      } else {
        // Stop motion
        setpoint.angular.z = 0;
        setpoint.linear.z = 0;
        setpoint.linear.x = 0;
      }         
      node_->set_velocity(setpoint);
      loop_rate.sleep();  // Give the drone time to move
    }  while (!(waypoint_is_close_ && altitude_is_close_)); 
    
    return true;

  }
