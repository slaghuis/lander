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

#include "lander/state.hpp"

void PendingState::execute_mission() 
{
  //  Do nothing.  Light one up
}


void PendingState::abort()
{
  // Do nothing
}

bool PendingState::is_working() 
{
  return false;  // In pending state, sitting under a tree having a joint.
}
  