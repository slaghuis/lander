#include "target_lander/seeking_state.h"
#include "target_lander/target_lander.h"

#include <cmath> // sqrt, pow

/* 
 * Seek the landing target.

    The general strategy here is adjust the altitude and the pose of the drone to 
    a safe flying height, and increase velocity towards the landing target.
    The tracker node, which is constantly surveiling the ground for the
    landing target, should start to provide target tracking estimates once we're
    in proximity to the target. We'll use that as an indication that it's time
    to transition to the APPROACH state of the flight program.
*/

using std::placeholders::_1;

namespace TargetLander {

  SeekingState::SeekingState(rclcpp::Node::SharedPtr node) 
  : TargetLanderState(std::string("Seeking")), node_(node) {
    
    // Read the parameters  
    rclcpp::Parameter target_local_position_param("target_local_position", std::vector<double>({}));

    node_->get_parameter("target_local_position", target_local_position_param);    
    node_->get_parameter("target_seek_altitude", target_seek_altitude_);
    node_->get_parameter("approach_holddown", approach_holddown_);
    node_->get_parameter("approach_speed", approach_speed_);
    node_->get_parameter("approach_radius", approach_radius_);
    node_->get_parameter("max_yaw_speed", max_yaw_speed_);

    RCLCPP_ERROR(node_->get_logger(), "Ascending to %f meters", target_seek_altitude_);

    target_local_position_ = target_local_position_param.as_double_array();
    
//    pid_x = std::make_shared<PID>(1.0, approach_speed_, -approach_speed_, 0.7, 0.00, 0.4);
//    pid_z = std::make_shared<PID>(1.0, approach_speed_, -approach_speed_, 0.9, 0.00, 0.5);
//    pid_yaw = std::make_shared<PID>(1.0, max_yaw_speed_, -max_yaw_speed_, 0.1, 0.01, 0.5);
    
    pid_x = std::make_shared<PID>(0.5, approach_speed_, -approach_speed_, 0.7, 0.00, 0.0);
    pid_z = std::make_shared<PID>(0.5, 1.0, -1.0, 0.7, 0.00, 0.0);
    pid_yaw = std::make_shared<PID>(0.5, max_yaw_speed_, -max_yaw_speed_, 0.7, 0.00, 0);  
    
    // Set up publisher for twist information (Velocities to make the drone move
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel", 10); 
  
    // Set a callback timer
    seek_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&SeekingState::seek_timer_callback, this));
      
  }
  
  SeekingState::~SeekingState() {
  }
  
  void SeekingState::pend( TargetLanderNode * lander)  {
    lander->set_state(TargetLanderNode::ST_PEND);
  }
  
  void SeekingState::approach( TargetLanderNode * lander)  {
    lander->set_state(TargetLanderNode::ST_APPROACH);
  }
    
  // was const TargetLanderNode & lander  
  void SeekingState::handle_track_message(TargetLanderNode * lander, const lander_interfaces::msg::Track::SharedPtr msg, const geometry_msgs::msg::Pose::SharedPtr pose, const geometry_msgs::msg::Twist::SharedPtr twist) {
        
    double err_x, err_y;
    if (msg->tracking.data) {  // Can see the target, use the camera observation to guide  
      err_x = msg->position.x; // - pose->position.x;
      err_y = msg->position.y; // - pose->position.y;
    } else {                   // Can't see the target, use the parameters as guidance 
      err_x = pose->position.x - target_local_position_[0];
      err_y = pose->position.y - target_local_position_[1];
    }
    // This is a DEBUG Override
    err_x = pose->position.x - target_local_position_[0];
    err_y = pose->position.y - target_local_position_[1];
    
    double distance = sqrt(pow(err_x,2) + pow(err_y,2));
    double velocity = sqrt(pow(twist->linear.x,2) + pow(twist->linear.y,2));
    
    bool target_is_stable = msg->tracking.data;
    bool target_is_close = (distance < approach_radius_);
    bool vehicle_is_stable = (velocity < approach_speed_);
    
    if( target_is_stable && target_is_close && vehicle_is_stable ) {
      lander->approach();
      
      return;  // Explicit return to prevent fall through to rest of code.      
    }        
    
    // RCLCPP_INFO(node_->get_logger(), "Calculating a setpoint");
    
    // Implement control logic to correct for velocity and position errors.    
    // Landing target is in a world coordinate system, reltive to North and  the landing target
    // Drone flies in a body coordinate system.
    // Strategy: adjust altitude to fly home altitude and adjust yaw to look at the target.
    // If yaw is close enough, start increasing foreward velocity (x) whilst monitoring the yaw
    // and altitude.
   
    // Orientation quaternion
    tf2::Quaternion q(
        pose->orientation.x,
        pose->orientation.y,
        pose->orientation.z,
        pose->orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
                    
    // Calculate direction (desired yaw angle in radians) to the landing target
    // NOTE:  -M_PI <= yaw <= M_PI
     
     double yaw_to_target = (err_x == 0.0) ? 0.0 : atan(err_y / err_x);
     if (err_x > 0.0) {
       if(err_y > 0.0) {
         yaw_to_target -= M_PI;
//         RCLCPP_WARN(node_->get_logger(), "[SEEKING] X>0 Y>0 MINUS PI: Yaw %f", yaw_to_target);
       } else {
         yaw_to_target += M_PI;
//         RCLCPP_WARN(node_->get_logger(), "[SEEKING] X<0 Y>0 PLUS PI : Yaw %f", yaw_to_target);
       }
     }

     setpoint.angular.x = 0.0;
     setpoint.angular.y = 0.0;
     setpoint.angular.z = pid_yaw->calculate( yaw_to_target, yaw);         // correct yaw

     setpoint.linear.x = 0.0;
     setpoint.linear.y = 0.0;  
     setpoint.linear.z = pid_z->calculate(target_seek_altitude_, pose->position.z);  // correct altitude
    
     bool vehicle_is_high_enough = abs(pose->position.z - target_seek_altitude_) < 0.5;
     bool vehicle_pose_is_good = abs(setpoint.angular.z) < 0.02; 
     if(vehicle_is_high_enough && vehicle_pose_is_good) {      
        // The PID will return a negative, as we are trying to close the distance down to 0.  (Unless we have overshot the target)
        // for that reason we send a negative distance
        setpoint.linear.x = pid_x->calculate(0.0, -distance);                  // fly closer to the target
     }      
              
     RCLCPP_INFO(node_->get_logger(), "[SEEKING] setpoint: (%6.2f, %6.2f, %6.2f) speed: %6.2f  distance: %6.2f",
                        setpoint.linear.x, setpoint.linear.y, setpoint.linear.z, velocity, distance);                

  }
  
    
  void SeekingState::seek_timer_callback() {
    
    /*   Publish location setpoint once per control loop iteration.
    *
    *    For now, we have a static landing target position, so repeatedly setting
    *    the same location setpoint is like beating a dead horse like beating a
    *    dead horse like beating a dead horse like -- you get the idea.
    *
    *    But eventually we'll receive dynamic updates of the target position, and
    *    then this will be more useful. Besides, PX4 requires a constant stream of
    *    heartbeat messages, so this is not for naught.        
    */
      
    twist_pub_->publish(setpoint);    
    
  }
  
}  // Namespace
