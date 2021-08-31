#pragma once

#include "target_lander/target_lander_state.h"
#include "target_lander/pid.h"
#include "target_lander/holddown_timer.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nav_msgs/msg/odometry.hpp>

namespace TargetLander {

  class TargetLanderNode;

  class SeekingState : public TargetLanderState {
  public:
    SeekingState(rclcpp::Node::SharedPtr node);
    virtual ~SeekingState();
  
    virtual void pend( TargetLanderNode * ) ;
    virtual void approach( TargetLanderNode * ) ;
    
    virtual void handle_track_message(TargetLanderNode * , const lander_interfaces::msg::Track::SharedPtr msg, const geometry_msgs::msg::Pose::SharedPtr pose, const geometry_msgs::msg::Twist::SharedPtr twist);
    
    void seek_timer_callback();
  private:
    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
      
    // ROS2 Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    
    // ROS2 Wall Timers
    rclcpp::TimerBase::SharedPtr seek_timer_;
    
    // ROS2 Parameters
    std::vector<double> target_local_position_ = {0.0, 0.0, 0.0};
    float target_seek_altitude_;
    int approach_holddown_;
    float approach_speed_;
    float approach_radius_;
    float max_yaw_speed_;     
    float camera_yaw_correction_;
    float camera_vertical_fov_;
    
    // PID Controllers  
    std::shared_ptr<PID> pid_x;
    std::shared_ptr<PID> pid_z;
    std::shared_ptr<PID> pid_yaw;

    // Hoddown Timer
    std::shared_ptr<HolddownTimer> approach_holddown_timer;
    
    // Global variables
    geometry_msgs::msg::Twist setpoint = geometry_msgs::msg::Twist();
    

  };

}
