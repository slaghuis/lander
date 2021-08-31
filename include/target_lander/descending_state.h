#pragma once
#include "target_lander/target_lander_state.h"
#include "target_lander/pid.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nav_msgs/msg/odometry.hpp>

namespace TargetLander {


  class TargetLanderNode;

  class DescendingState : public TargetLanderState {
  public:
    DescendingState(rclcpp::Node::SharedPtr node);
    virtual ~DescendingState();
  
    virtual void pend( TargetLanderNode * )  ;
    virtual void land( TargetLanderNode * )  ;
    virtual void seek( TargetLanderNode * )  ;
    
    virtual void handle_track_message(TargetLanderNode * , const lander_interfaces::msg::Track::SharedPtr msg, const geometry_msgs::msg::Pose::SharedPtr pose, const geometry_msgs::msg::Twist::SharedPtr twist);
    
    void descend_timer_callback();

  private:
    // Pointer to the ROS node
    rclcpp::Node::SharedPtr node_;
    
        // ROS2 Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    
    // ROS2 Wall Timers
    rclcpp::TimerBase::SharedPtr descend_timer_;
    
    // ROS2 Parameters
    float max_speed_xy_;
    float max_accel_xy_;
    float max_speed_z_;
    float max_accel_z_;
    float descend_radius_;
    float land_max_speed_xy_;
    float land_altitude_;
    float max_yaw_speed_;
    float camera_yaw_correction_;
    float camera_vertical_fov_;
    
    // PID Controllers
    std::shared_ptr<PID> pid_x;
    std::shared_ptr<PID> pid_y;
    std::shared_ptr<PID> pid_z;
    
    // Global Variables
    geometry_msgs::msg::Twist setpoint = geometry_msgs::msg::Twist();  


  };

} // namespace
