#include "target_lander/pending_state.h"
#include "target_lander/target_lander.h"

namespace TargetLander {

  PendingState::PendingState(rclcpp::Node::SharedPtr node) 
  : TargetLanderState(std::string("Pending")), node_(node) {
  }
  
  PendingState::~PendingState() {
  }
  
  void PendingState::seek( TargetLanderNode * lander)   {  
    lander->set_state(TargetLanderNode::ST_SEEK);
  }
} // Namespace
