#pragma once

#include <chrono>     // seconds, milliseconds
#include <functional>
#include <memory>     // std::shared_ptr
#include <thread>     // std::thread
#include <typeinfo>
#include <cmath>      // sqrt. pow

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "lander_interfaces/action/land.hpp"
#include "lander_interfaces/srv/set_target.hpp"

#include "lander/state.hpp"
static const float DEFAULT_TARGET_SEEK_ALTITUDE = 5.0;  // The altitude at which to approach the target (in meters)
static const float DEFAULT_MAX_SPEED_XY = 2.0;          // Maximum horizontal speed, in m/s
static const float DEFAULT_MAX_SPEED_Z = 0.33;          // Maximum vertical speed, in m/s
static const float DEFAULT_MAX_YAW_SPEED = 0.5;         // Maximum yaw speed in radians/s 
static const float DEFAULT_WAYPOINT_RADIUS_ERROR = 0.3; // Acceptable XY distance to waypoint deemed as close enough
static const float DEFAULT_YAW_THRESHOLD = 0.025;       // Acceptible YAW to start foreward acceleration
static const float DEFAULT_ALTITUDE_THRESHOLD = 0.3;    // Acceptible Z distance to altitude deemed as close enough 
static const int DEFAULT_HOLDDOWN = 2;                  // Time to ensure stability in flight is attained

using SetTarget = lander_interfaces::srv::SetTarget;

class LanderServer : public rclcpp::Node
{
public:
  using Land = lander_interfaces::action::Land;
  using GoalHandleLand = rclcpp_action::ServerGoalHandle<Land>;
  
  explicit LanderServer(const rclcpp::NodeOptions &); 

  void TransitionTo(State *state);
  void execute_mission();
  void abort();
  bool is_working();
  
  bool read_position(double *x, double *y, double *z);
  bool read_position(double *x, double *y, double *z, double *w);
  bool read_target_position(double *x, double *y, double *z, double *w);  
  bool target_is_close();
  void set_velocity(geometry_msgs::msg::Twist setpoint);
  void stop_movement();
  std::shared_ptr<geometry_msgs::msg::PoseStamped> landing_target_position();
  double read_distance_to_target();
  
private:
   
  // Global Variables
  State *state_;
  std::shared_ptr<geometry_msgs::msg::PoseStamped> target_position_;
  
  // Node Parameters
  std::string map_frame_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr one_off_timer_;
  void init();

  // Publishers  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  
  // Simple clients
  rclcpp::Client<SetTarget>::SharedPtr set_landing_target_client;
  bool set_landing_target(int target_number);
  
  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
  // Action Server
  rclcpp_action::Server<Land>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Land::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleLand> goal_handle);

  void execute(const std::shared_ptr<GoalHandleLand> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleLand> goal_handle);
    
};  // class LanderServer

