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

#include "target_lander/approaching_state.h"
#include "target_lander/target_lander.h"

#include <cmath> // sqrt, pow
#include <complex> // complex numbers for coorinate rotation

/*
 *  Apploach the landing target while maintaining altitude
 *
 *  The general strategy here is to first minimize the velocity error,
 *  and then minimize position error.
 */
 
namespace TargetLander {

  ApproachingState::ApproachingState(rclcpp::Node::SharedPtr node) 
  : TargetLanderState(std::string("Approaching")), node_(node) {
  
    // Read the parameters
    node_->get_parameter("max_speed_xy", max_speed_xy_);
    node_->get_parameter<float>("max_accel_xy", max_accel_xy_);
    node_->get_parameter<float>("descend_radius", descend_radius_);
    node_->get_parameter<float>("descend_max_speed_xy", descend_max_speed_xy_);
    node_->get_parameter<int>("descend_holddown", descend_holddown_);    
    node_->get_parameter<float>("camera_yaw_correction", camera_yaw_correction_);
    node_->get_parameter<float>("camera_vertical_fov", camera_vertical_fov_);
      
    // Is this too agressive
    pid_x = std::make_shared<PID>(0.5, descend_max_speed_xy_, -descend_max_speed_xy_, 0.4, 0.00, 0.0);
    pid_y = std::make_shared<PID>(0.5, descend_max_speed_xy_, -descend_max_speed_xy_, 0.4, 0.00, 0.0);
    
    // Set up the hoddown timer
    descend_holddown_timer = std::make_shared<HolddownTimer>(descend_holddown_);

    // Set up publisher for twist information (Velocities to make the drone move)
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel", 10); 
  
    // Set a callback timer
    approach_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&ApproachingState::approach_timer_callback, this));      
  }
  
  ApproachingState::~ApproachingState() {
  }
  
  void ApproachingState::pend( TargetLanderNode * lander)  {
    lander->set_state(TargetLanderNode::ST_PEND);
  }
  
  void ApproachingState::seek( TargetLanderNode * lander)  {
    lander->set_state(TargetLanderNode::ST_SEEK);
  }
  
  void ApproachingState::descend( TargetLanderNode * lander)  {
    lander->set_state(TargetLanderNode::ST_DESCEND);
  }
  
  void ApproachingState::handle_track_message(TargetLanderNode * lander, const lander_interfaces::msg::Track::SharedPtr msg, const geometry_msgs::msg::Pose::SharedPtr pose, const geometry_msgs::msg::Twist::SharedPtr twist) {
  
    // Implement control logic to correct for velocity and position errors.
    
    // Abort if we are no longer tracking the target
    // RISK: One could fall into a death spiral of Seek->Approach->Seek->Approach
    if (!msg->tracking.data) {
      lander->seek();
      return;  // Explicit return to prevent fall through to rest of code. 
    }
    
    // Guidance provided by sight of the landing traget only.        
    double err_x, err_y;
    double velocity;

    // Scale the numbers according on the altitude from the percentaes passed from the tracking node.
    double scaling = pose->position.z * 2 * tan(camera_vertical_fov_/2);
    double target_x = msg->position.x * scaling;
    double target_y = msg->position.y * scaling;
    
    velocity = sqrt(pow(twist->linear.x * scaling,2) + pow(twist->linear.y*scaling,2));
            
    // Rotate the tracking messages around the origin to compensate for the mount angle of the camera.
    // No need to rotate the velocity, as we only use it to check for stability, hence direction is irrelevant.
    complex<double> p(target_x, target_y);
    complex<double> p_rotated = p * polar(1.0, (double)camera_yaw_correction_); //camera_yaw_correction_);      
    err_x = (double)p_rotated.real();  
    err_y = (double)p_rotated.imag();    
    
    double distance = abs(p_rotated);  //sqrt(pow(err_x,2) + pow(err_y,2));
    
    bool target_is_close = (distance < descend_radius_);
    bool vehicle_is_stable = (velocity < descend_max_speed_xy_);
    // Transition to DESCEND state if:
    //   - we're within the approach radius of the target
    //   - the vehicle speed is within the approach speed threshold
    //   - those conditions have been true for the holddown period
    if( descend_holddown_timer->test(target_is_close && vehicle_is_stable)) {
      lander->descend();
      return;  // Explicit return to prevent fall through to rest of code.      
    }        

    setpoint.linear.x = pid_x->calculate(0.0, err_x);
    setpoint.linear.y = pid_y->calculate(0.0, err_y);
    setpoint.linear.z = 0.0;  //pid_z->calculate(target_seek_altitude_, msg->position.z);  // correct altitude
            
    setpoint.angular.x = 0.0;
    setpoint.angular.y = 0.0;
    setpoint.angular.z = 0.0;
    //setpoint.angular.z = pid_yaw->calculate( yaw_to_target, yaw);         // correct yaw
    
    RCLCPP_INFO(node_->get_logger(), "[APPRAOCH] error: (%6.2f, %6.2f, 000.00) setpoint: (%6.2f, %6.2f, %6.2f) speed: %6.2f  distance: %6.2f",
                        err_x, err_y, setpoint.linear.x, setpoint.linear.y, setpoint.linear.z, velocity, distance);

  }
  
  void ApproachingState::approach_timer_callback() {
    
    // Publish location setpoint once per control loop iteration.      
    twist_pub_->publish(setpoint);    
    
  }
  
  
} // Namespace
