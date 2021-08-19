#pragma once

#include <string>

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <string>
#include <functional>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "lander_interfaces/msg/track.hpp"
#include "lander_interfaces/msg/track_stamped.hpp"

namespace TargetLander {

  // The landing target is located at the origin, i.e., where the vehicle's
  // inertial navigation system was initialized (in meters from home)
  static const float DEFAULT_TARGET_LOCAL_POSITION [] = {0.0, 0.0, 0.0};  

  // The altitude at which to approach the target (in meters)
  static const float DEFAULT_TARGET_SEEK_ALTITUDE = 15.0;

  // The period of time we need to have continuously seen the target
  // before transitioning to APPROACH state (in seconds)
  static const int DEFAULT_APPROACH_HOLDDOWN = 5;

  // The speed at which we'll transition to APPROACH state
  // (assuming we can see the landing target) (in m/s)
  static const float DEFAULT_APPROACH_SPEED = 1.0;

  // The radius within which we'll transition to APPROACH state
  // (assuming we can see the landing target) (in meters)
  static const float DEFAULT_APPROACH_RADIUS = 1.5;

  // Maximum horizontal speed, in m/s
  static const float DEFAULT_MAX_SPEED_XY = 0.25;
  
  // Maximum yaw speed in radians/s
  static const float DEFAULT_MAX_YAW_SPEED = 0.25;

  // Maximum horizontal acceleration, in m/s/s
  static const float DEFAULT_MAX_ACCEL_XY = 0.1;

  static const float DEFAULT_DESCEND_RADIUS = 0.33;
  static const float DEFAULT_DESCEND_MAX_SPEED_XY = 0.10;
  static const int DEFAULT_DESCEND_HOLDDOWN = 3;
  

  // Maximum vertical speed, in m/s
  static const float DEFAULT_MAX_SPEED_Z = 0.33;

  // Maximum vertical acceleration, in m/s/s
  static const float DEFAULT_MAX_ACCEL_Z = 0.05;

  static const float DEFAULT_LAND_MAX_SPEED_XY = 0.05;
  static const float DEFAULT_LAND_ALTITUDE = 0.5;

class TargetLanderNode;

class TargetLanderState {
public:
  TargetLanderState(std::string name);
  virtual ~TargetLanderState();
  
  virtual void pend( TargetLanderNode * node) ;
  virtual void seek( TargetLanderNode * node) ;
  virtual void approach( TargetLanderNode * node) ;
  virtual void descend( TargetLanderNode * node) ;
  virtual void land( TargetLanderNode * node) ;
  
//  virtual void handle_track_message(TargetLanderNode * node, const lander_interfaces::msg::TrackStamped::SharedPtr msg);
  virtual void handle_track_message(TargetLanderNode * , const lander_interfaces::msg::Track::SharedPtr msg, const geometry_msgs::msg::Pose::SharedPtr pose, const geometry_msgs::msg::Twist::SharedPtr twist);
  
  std::string get_name() { return _name; }
private:
  std::string _name;
  
};  

} // Namespace
