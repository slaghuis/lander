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

#include "target_lander/descending_state.h"
#include "target_lander/target_lander.h"

#include <cmath> // sqrt, pow
#include <complex> // complex numbers for coorinate rotation

/*
 *   Descend toward the landing target.
 *
 *   The general strategy here is to control vehicle velocity using a  PID
 *   controller, using target position error as input, and constraining vertical
 *   velocity to within sane limits.
 *
 */

namespace TargetLander {

  DescendingState::DescendingState(rclcpp::Node::SharedPtr node) 
  : TargetLanderState(std::string("Descending")), node_(node) {
  
    // Read the parameters   
    node_->get_parameter("max_speed_xy", max_speed_xy_);
    node_->get_parameter<float>("max_accel_xy", max_accel_xy_);
    node_->get_parameter("max_speed_z", max_speed_z_);
    node_->get_parameter<float>("max_accel_z", max_accel_z_);
    node_->get_parameter<float>("descend_radius", descend_radius_);
    node_->get_parameter<float>("land_max_speed_xy", land_max_speed_xy_);
    node_->get_parameter<float>("land_altitude", land_altitude_);
    node_->get_parameter<float>("max_yaw_speed", max_yaw_speed_);
    node_->get_parameter<float>("camera_yaw_correction", camera_yaw_correction_);
    node_->get_parameter<float>("camera_vertical_fov", camera_vertical_fov_);

    pid_x = std::make_shared<PID>(0.1, max_speed_xy_, -max_speed_xy_, 0.1, 0.00, 0.0);
    pid_y = std::make_shared<PID>(0.1, max_speed_xy_, -max_speed_xy_, 0.1, 0.00, 0.0);  // Was 0.3, but did not work great
    pid_z = std::make_shared<PID>(0.1, max_speed_z_, -max_speed_z_, 0.25, 0.00, 0.0);
    
    //pid_yaw = std::make_shared<PID>(0.5, max_yaw_speed_, -max_yaw_speed_, 0.7, 0.00, 0); 
     
    // Set up publisher for twist information (Velocities to make the drone move)
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel", 10); 
  
    // Set a callback timer
    descend_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&DescendingState::descend_timer_callback, this));

  }
  
  DescendingState::~DescendingState() {
  }
  
  void DescendingState::pend( TargetLanderNode * lander)   {
    lander->set_state(TargetLanderNode::ST_PEND);
  }
  
  void DescendingState::seek( TargetLanderNode * lander)   {
    lander->set_state(TargetLanderNode::ST_SEEK);
  }
  
  void DescendingState::land( TargetLanderNode * lander)   {
    lander->set_state(TargetLanderNode::ST_LAND);
  }
  
  
  void DescendingState::handle_track_message(TargetLanderNode * lander, const lander_interfaces::msg::Track::SharedPtr msg, const geometry_msgs::msg::Pose::SharedPtr pose, const geometry_msgs::msg::Twist::SharedPtr twist) {
  
    // Implement control logic to correct for velocity and position errors.
    
    // Abort if we are no longer tracking the target
    // RISK: One could fall into a death spiral of Seek->Approach->Descend->Seek
    if (!msg->tracking.data) {
      lander->seek();
      return;  // Explicit return to prevent fall through to rest of code. 
    }
    
    double err_x, err_y, err_z;   

    // Scale the numbers according on the altitude from the percentaes passed from the tracking node.
    double scaling = pose->position.z * 2 * tan(camera_vertical_fov_/2);
    double target_x = msg->position.x * scaling;
    double target_y = msg->position.y * scaling;
                
    // Rotate the tracking messages around the origin to compensate for the mount angle of the camera.
    // No need to rotate the velocity, as we only use it to check for stability, hence direction is irrelevant.
    complex<double> p(target_x, target_y);
    complex<double> p_rotated = p * polar(1.0, (double)camera_yaw_correction_); //camera_yaw_correction_);      
    err_x = (double)p_rotated.real();  
    err_y = (double)p_rotated.imag();    
    
    err_z = land_altitude_ - pose->position.z;     

    double distance = abs(p_rotated);  //sqrt(pow(err_x,2) + pow(err_y,2));
    double velocity = sqrt(pow(twist->linear.x * scaling,2) + pow(twist->linear.y*scaling,2));
    
    bool vehicle_is_right_above_target = (pose->position.z < land_altitude_);
    bool vehicle_is_stable = (velocity < land_max_speed_xy_);
    // Transition to LAND state if:
    //   - the vehicle speed is within the landing speed threshold
    //   - the vehicle is low enough to switch to flight controller landing
    if( vehicle_is_right_above_target && vehicle_is_stable ) {
      lander->land();
            
      return;  // Explicit return to prevent fall through to rest of code.      
    }        
        
    setpoint.angular.x = 0.0;
    setpoint.angular.y = 0.0;
    setpoint.angular.z = 0.0;
    //setpoint.angular.z = pid_yaw->calculate( yaw_to_target, yaw);         // correct yaw

    setpoint.linear.x = pid_x->calculate(0.0, err_x);
    setpoint.linear.y = pid_y->calculate(0.0, err_y);
    setpoint.linear.z = pid_z->calculate(land_altitude_, pose->position.z);

    RCLCPP_INFO(node_->get_logger(), "[DESCEND] error: (%6.2f, %6.2f, %6.2f) setpoint: (%6.2f, %6.2f, %6.2f) speed: %6.2f  distance: %6.2f",
                        err_x, err_y, err_z, setpoint.linear.x, setpoint.linear.y, setpoint.linear.z, velocity, distance);

  }
  
  void DescendingState::descend_timer_callback() {
    
    // Publish location setpoint once per control loop iteration.
      
    twist_pub_->publish(setpoint);    
    
  }
}  // namespace
