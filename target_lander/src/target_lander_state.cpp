#include "target_lander/target_lander_state.h"

#include <iostream>

namespace TargetLander {

  TargetLanderState::TargetLanderState(std::string name)
  : _name(name) {
  }

  TargetLanderState::~TargetLanderState() {
  }

  void TargetLanderState::pend( TargetLanderNode * )  {
    //RCLCPP_ERROR(node_->get_logger(), "Illegal state transition from %s to Pend", get_name());
  }

  void TargetLanderState::seek( TargetLanderNode * )   {
    //RCLCPP_ERROR(node_->get_logger(), "Illegal state transition from %s to Seek", get_name());
  }

  void TargetLanderState::approach( TargetLanderNode * )  {
    //RCLCPP_ERROR(node_->get_logger(), "Illegal state transition from %s to Approach", get_name());
  }

  void TargetLanderState::descend( TargetLanderNode * )  {
    //RCLCPP_ERROR(node_->get_logger(), "Illegal state transition from %s to Descend", get_name());
  }

  void TargetLanderState::land( TargetLanderNode * )   {
    //RCLCPP_ERROR(lander_->get_logger(), "Illegal state transition from %s to Land", get_name());
  }
  
//  void TargetLanderState::handle_track_message(TargetLanderNode * , const lander_interfaces::msg::Track::SharedPtr ) {
  void TargetLanderState::handle_track_message(TargetLanderNode * , const lander_interfaces::msg::Track::SharedPtr, const geometry_msgs::msg::Pose::SharedPtr, const geometry_msgs::msg::Twist::SharedPtr) {
    //RCLCPP_ERROR(lander_->get_logger(), "Illegal track handler call");
  }

}  // Namespace
