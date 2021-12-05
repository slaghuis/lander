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

/* *********************************************************************************************
 *  Land on the landing target.
 *
 *  Landing is tricky. Ground effect causes random perturbations in
 *  vehicle velocity, and we can't correct for that very well because
 *  we're so close to the ground we don't have much room to maneuver
 *  or margin for error.
 *
 *  So we just ask the FCU via the drone node to land and then disarm the motors.
 * *********************************************************************************************/
#include "lander/state.hpp"
#include "lander/lander_node.h"

void LandingState::execute_mission() 
{
  RCLCPP_INFO(node_->get_logger(), "Landing: Handing control over th the flight controller.");
    
  client_ptr_ = rclcpp_action::create_client<DroneLand>( node_, "/drone/land");
  
  
  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting.  Going up where it is safe.");
    node_->TransitionTo(new SeekingState);
  }

  auto goal_msg = DroneLand::Goal();
  goal_msg.gear_down = true;

  RCLCPP_INFO(node_->get_logger(), "Sending goal");
  using namespace std::placeholders;
  auto send_goal_options = rclcpp_action::Client<DroneLand>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&LandingState::goal_response_callback, this, _1);  
  send_goal_options.feedback_callback =
    std::bind(&LandingState::feedback_callback, this, _1, _2);
    
  send_goal_options.result_callback =
    std::bind(&LandingState::result_callback, this, _1);
      
  client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void LandingState::abort()
{
  // Stabilise flight
  // No idea what to do if I cancel  
  // Go sit under a tree till you are called 
  node_->TransitionTo(new PendingState);
}

bool LandingState::is_working() 
{
  return true;  // Only pending state gets to sit under a tree and do nothing.
}
  
// ACTION CLIENT //////////////////////////////////////////////////////////////////////////////////

void LandingState::goal_response_callback(std::shared_future<GoalHandleDroneLand::SharedPtr> future)  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

void LandingState::feedback_callback(
  GoalHandleDroneLand::SharedPtr,
  const std::shared_ptr<const DroneLand::Feedback> feedback)
{
  RCLCPP_INFO(node_->get_logger(), "Landing altitude: %f", feedback->current_altitude );
}

void LandingState::result_callback(const GoalHandleDroneLand::WrappedResult & result)
{
  this->goal_done_ = true;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return;
  }
  
  node_->TransitionTo(new PendingState);

  RCLCPP_INFO(node_->get_logger(), "Result received");  
}