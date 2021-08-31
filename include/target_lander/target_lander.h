#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_interface.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "lander_interfaces/msg/track_stamped.hpp"
#include "lander_interfaces/action/target_land.hpp"

#include "target_lander/echo_listener.h"

namespace TargetLander {

  class TargetLanderState;

  class TargetLanderNode : public rclcpp::Node 
  {
  public:
  
    using TargetLand = lander_interfaces::action::TargetLand;
    using GoalHandleTargetland = rclcpp_action::ServerGoalHandle<TargetLand>;

    enum State { ST_PEND, ST_SEEK, ST_APPROACH, ST_DESCEND, ST_LAND };
    
    explicit TargetLanderNode();  //const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
    
    void pend();
    void seek();
    void approach();
    void descend();
    void land();
    
    void set_state(State state); 
    
    std::shared_ptr<geometry_msgs::msg::Pose> last_pose = std::make_shared<geometry_msgs::msg::Pose>();
    std::shared_ptr<geometry_msgs::msg::Twist> last_twist = std::make_shared<geometry_msgs::msg::Twist>();
    
    std::shared_ptr<EchoListener> echo_listener_; 
  private:    
    rclcpp::TimerBase::SharedPtr one_off_timer_;
    std::string source_frameid = "odom";
    std::string target_frameid = "base_link";
    
    void init();
    
    std::shared_ptr<TargetLanderState> t_l_state;
    
    rclcpp_action::Server<TargetLand>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const TargetLand::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleTargetland> goal_handle);
    void execute(const std::shared_ptr<GoalHandleTargetland> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleTargetland> goal_handle);
    
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const; 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    
    void tracking_callback(const lander_interfaces::msg::TrackStamped::SharedPtr msg) ; 
    rclcpp::Subscription<lander_interfaces::msg::TrackStamped>::SharedPtr tracking_;
    
  };  // class TargetLander

} // namespace

