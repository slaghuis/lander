#include "target_lander/landing_state.h"
#include "target_lander/target_lander.h"

/*
 *  Land on the landing target.
 *
 *  Landing is tricky. Ground effect causes random perturbations in
 *  vehicle velocity, and we can't correct for that very well because
 *  we're so close to the ground we don't have much room to maneuver
 *  or margin for error.
 *
 *  So we just ask the FCU to land and then disarm the motors.
 */
 
namespace TargetLander {

  LandingState::LandingState(rclcpp::Node::SharedPtr node) 
  : TargetLanderState(std::string("Landing")), node_(node) {

    client_ptr_ = rclcpp_action::create_client<Land>( node_, "/drone/land");
    
    // Set a callback timer
    land_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&LandingState::send_goal, this));

  }
  
  LandingState::~LandingState() {  
  }
  
  void LandingState::pend( TargetLanderNode * lander)   {
    lander->set_state(TargetLanderNode::ST_PEND);
  }
  
  void LandingState::send_goal() {
  
    using namespace std::placeholders;
    
    land_timer_->cancel();
    
    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Land::Goal();
    goal_msg.descend_speed = 0;  // This is ignored

    RCLCPP_INFO(node_->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Land>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&LandingState::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&LandingState::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&LandingState::result_callback, this, _1);
      
      
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
    
  }
  
  void LandingState::goal_response_callback(std::shared_future<GoalHandleLand::SharedPtr> future)  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void LandingState::feedback_callback(
    GoalHandleLand::SharedPtr,
    const std::shared_ptr<const Land::Feedback> feedback)
  {
    RCLCPP_INFO(node_->get_logger(), "[LANDING] altitude: %f", feedback->current_altitude );
  }

  void LandingState::result_callback(const GoalHandleLand::WrappedResult & result)
  {
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
    RCLCPP_INFO(node_->get_logger(), "The eagle has landed!");
    rclcpp::shutdown();
  }
  
}  //namespace  
