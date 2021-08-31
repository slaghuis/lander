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
