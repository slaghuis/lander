#pragma once
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

#include <memory>  // shared_ptr

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "lander/pid.hpp"
#include "lander/utils.hpp"
#include "lander/holddown_timer.hpp"

#include "drone_interfaces/action/land.hpp"

class LanderServer;

class State {
  protected:
    LanderServer * node_;
  
  public:  
    virtual ~State()
    {};
  
    void set_context(LanderServer * node) {
      this->node_ = node;
    }  
    virtual void execute_mission() = 0;
    virtual void abort() = 0;
    virtual bool is_working() { return false; };
};

// PENDING STATE /////////////////////////////////////////////////////////////////////////////////
class PendingState : public State {
 public:
  void execute_mission() override;
  void abort() override;
  bool is_working() override;
};

// SEEKING STATE /////////////////////////////////////////////////////////////////////////////////
class SeekingState : public State {
  public:
    void execute_mission() override;
    void abort() override;
    bool is_working() override;
  
  protected:
      // Node parameters
    float target_seek_altitude_;
    float max_yaw_speed_;
    float max_speed_xy_;
    float max_speed_z_;
    float waypoint_radius_error_;
    float yaw_threshold_;
    float altitude_threshold_;
  
    // PID Controllers
    std::shared_ptr<PID> pid_x;
    std::shared_ptr<PID> pid_z;
    std::shared_ptr<PID> pid_yaw;
  
  private:
    bool fly_to_waypoint(std::shared_ptr<geometry_msgs::msg::PoseStamped> wp);
};

// APPROACHING STATE /////////////////////////////////////////////////////////////////////////////////
class ApproachingState : public State {
  public:
    void execute_mission() override;
    void abort() override;
    bool is_working() override;
  
  protected:
      // Node parameters
    float target_seek_altitude_;
    float max_yaw_speed_;
    float max_speed_xy_;
    float max_speed_z_;
    float waypoint_radius_error_;
    float yaw_threshold_;
    float altitude_threshold_;
    int holddown_;
  
    // PID Controllers
    std::shared_ptr<PID> pid_x;
    std::shared_ptr<PID> pid_y;
    std::shared_ptr<PID> pid_z;
    std::shared_ptr<PID> pid_yaw;
  
    // Hoddown Timer
    std::shared_ptr<HolddownTimer> descend_holddown_timer;
  
  private:
    bool correct_yaw();

};

class DescendingState : public State {
  public:
    void execute_mission() override;
    void abort() override;
    bool is_working() override;
  
  protected:
    float max_yaw_speed_;
    float max_speed_xy_;
    float max_speed_z_;
    float land_altitude_;
    float waypoint_radius_error_;
    float yaw_threshold_;
    float altitude_threshold_;
  
    // PID Controllers
    std::shared_ptr<PID> pid_x;
    std::shared_ptr<PID> pid_y;
    std::shared_ptr<PID> pid_z;
    std::shared_ptr<PID> pid_yaw;
  
  private:
    bool correct_altitude();
};

class LandingState : public State {
  public:
  
    using DroneLand = drone_interfaces::action::Land;
    using GoalHandleDroneLand = rclcpp_action::ClientGoalHandle<DroneLand>;
  
    void execute_mission() override;
    void abort() override;
    bool is_working() override;
  private:
    rclcpp_action::Client<DroneLand>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_done_;

    void goal_response_callback(std::shared_future<GoalHandleDroneLand::SharedPtr> future);
  //void goal_response_callback(GoalHandleDroneLand::SharedPtr goal_handle);
    void feedback_callback(
        GoalHandleDroneLand::SharedPtr,
        const std::shared_ptr<const DroneLand::Feedback> feedback);
    void result_callback(const GoalHandleDroneLand::WrappedResult & result);
 
};


