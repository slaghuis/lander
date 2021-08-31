#pragma once

#include <functional>
#include <future>
#include <memory>
  
#include "target_lander/target_lander_state.h"

#include "drone_interfaces/action/land.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


namespace TargetLander {

  class TargetLanderNode;

  class LandingState : public TargetLanderState {
  public:
  
    using Land = drone_interfaces::action::Land;
    using GoalHandleLand = rclcpp_action::ClientGoalHandle<Land>;
        
    LandingState(rclcpp::Node::SharedPtr node);
    virtual ~LandingState();
  
    virtual void pend( TargetLanderNode *)  ;
  private:
    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
    
    rclcpp_action::Client<Land>::SharedPtr client_ptr_;

     // ROS2 Wall Timers
    rclcpp::TimerBase::SharedPtr land_timer_;

    void send_goal();
    void goal_response_callback(std::shared_future<GoalHandleLand::SharedPtr> future);
    void feedback_callback(GoalHandleLand::SharedPtr, const std::shared_ptr<const Land::Feedback> feedback);
    void result_callback(const GoalHandleLand::WrappedResult & result);

  };

}  //namespace
