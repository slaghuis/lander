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

/* ****************************************************************************************
 * Descend towards the landing target, maintaining control over lateral movement
 *  This node depends on keeping the landing target in the camera frame.  If the
 *    target is lost, change back to Seeking state
 *  All navigation is done in the base_link frame using a FLU coordinate frame 
 * ****************************************************************************************/
#include "lander/state.hpp"
#include "lander/lander_node.h"
  
void DescendingState::execute_mission() 
{
  RCLCPP_INFO(node_->get_logger(), "Descend: Keeping over the goal, reduce altitude.");
  
  // Read the parameters needed in this object . . 
  node_->get_parameter("max_yaw_speed", max_yaw_speed_);
  node_->get_parameter("max_speed_xy", max_speed_xy_);
  node_->get_parameter("max_speed_z", max_speed_z_);
  node_->get_parameter("land_altitude", land_altitude_);
  node_->get_parameter("waypoint_radius_error", waypoint_radius_error_);
  node_->get_parameter("yaw_threshold", yaw_threshold_);
  node_->get_parameter("altitude_threshold", altitude_threshold_);

  // . . . and the vector based parameters
  rclcpp::Parameter pid_xy_settings_param = node_->get_parameter("pid_xy");
  std::vector<double> pid_xy_settings = pid_xy_settings_param.as_double_array(); 
  pid_x   = std::make_shared<PID>(0.5, max_speed_xy_, -max_speed_xy_, (float)pid_xy_settings[0], (float)pid_xy_settings[1], (float)pid_xy_settings[2]);
  pid_y   = std::make_shared<PID>(0.5, max_speed_xy_, -max_speed_xy_, (float)pid_xy_settings[0], (float)pid_xy_settings[1], (float)pid_xy_settings[2]);

  rclcpp::Parameter pid_z_settings_param = node_->get_parameter("pid_z");
  std::vector<double> pid_z_settings = pid_z_settings_param.as_double_array(); 
  pid_z   = std::make_shared<PID>(0.5, max_speed_z_, -max_speed_z_, (float)pid_z_settings[0], (float)pid_z_settings[1], (float)pid_z_settings[2]);

  rclcpp::Parameter pid_yaw_settings_param = node_->get_parameter("pid_yaw");
  std::vector<double> pid_yaw_settings = pid_yaw_settings_param.as_double_array(); 
  pid_yaw   = std::make_shared<PID>(0.5, max_yaw_speed_, -max_yaw_speed_, (float)pid_yaw_settings[0], (float)pid_yaw_settings[1], (float)pid_yaw_settings[2]);
  
  // Fly as low as the landing target can still be sigthed. If the target is lost, return to seeking state and try again.
  if (correct_altitude() ) {
    node_->TransitionTo(new LandingState);
  } else {
    node_->TransitionTo(new SeekingState);
  }    
  
}

void DescendingState::abort()
{
  // Stabilise flight
  node_->stop_movement();  
  
  // Go sit under a tree till you are called 
  node_->TransitionTo(new PendingState);
}

bool DescendingState::is_working() 
{
  return true;  // Only pending state gets to sit under a tree and do nothing.
}
  
bool DescendingState::correct_altitude() {  
  rclcpp::Rate loop_rate(2);
  
  bool  pose_is_close, position_is_close, altitude_is_close;
    
  geometry_msgs::msg::Twist setpoint = geometry_msgs::msg::Twist();
  setpoint.linear.x = 0.0;
  setpoint.linear.y = 0.0;
  setpoint.linear.z = 0.0;
  setpoint.angular.x = 0.0;
  setpoint.angular.y = 0.0;
  setpoint.angular.z = 0.0;

  double yaw_error, x_error, y_error, z_error;
      
  do {
    if (! node_->target_is_close() ) {
      // Have lost sight of the target
      return false;
    }
 
    node_->read_target_position("base_link", &x_error, &y_error, &z_error, &yaw_error);  
  
    pose_is_close = (fabs(yaw_error) < yaw_threshold_);

    position_is_close = ( (fabs(x_error) < waypoint_radius_error_) && (fabs(x_error) < waypoint_radius_error_) );
    
    z_error = (z_error - land_altitude_);   // We are only descending to the land_altitude parameter
    altitude_is_close = fabs(z_error) < altitude_threshold_;
    
    if ( pose_is_close && position_is_close && altitude_is_close ) {
      return true;
    }
    // correct position error down to zero  
    setpoint.linear.x = pid_x->calculate(0, -x_error);             
    setpoint.linear.y = pid_y->calculate(0, -y_error);               
    setpoint.linear.z = pid_z->calculate(0, -z_error);        
    setpoint.angular.z = pid_yaw->calculate(0, -yaw_error);          
      
    node_->set_velocity(setpoint);
    loop_rate.sleep();  // Give the drone time to move
  } while (true);  //(!pose_is_close_); 
    
  return true;
}