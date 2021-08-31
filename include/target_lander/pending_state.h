#pragma once

#include "target_lander/target_lander_state.h"

namespace TargetLander {

  class TargetLanderNode;

  class PendingState : public TargetLanderState {
  public:
    PendingState(rclcpp::Node::SharedPtr node);
    virtual ~PendingState();
  
    virtual void seek( TargetLanderNode * ) ; 
  private:
    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
  };
  
} // namespace

