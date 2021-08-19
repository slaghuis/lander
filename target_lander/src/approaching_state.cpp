#include "target_lander/approaching_state.h"
#include "target_lander/target_lander.h"

#include <cmath> // sqrt, pow

/*
 *  Apploach the landing target while maintaining altitude
 *
 *  The general strategy here is to first minimize the velocity error,
 *  and then minimize position error.
 */
 
namespace TargetLander {

  ApproachingState::ApproachingState(rclcpp::Node::SharedPtr node) 
  : TargetLanderState(std::string("Approaching")), node_(node) {
  
    // Read the parameters
    node_->get_parameter("max_speed_xy", max_speed_xy_);
    node_->get_parameter<float>("max_accel_xy", max_accel_xy_);
    node_->get_parameter<float>("descend_radius", descend_radius_);
    node_->get_parameter<float>("descend_max_speed_xy", descend_max_speed_xy_);
    node_->get_parameter<int>("descend_holddown", descend_holddown_);    
      
    // Is this too agressive
    pid_x = std::make_shared<PID>(0.5, descend_max_speed_xy_, -descend_max_speed_xy_, 0.4, 0.00, 0.0);
    pid_y = std::make_shared<PID>(0.5, descend_max_speed_xy_, -descend_max_speed_xy_, 0.4, 0.00, 0.0);

    // Set up publisher for twist information (Velocities to make the drone move)
    twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel", 10); 
  
    // Set a callback timer
    approach_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&ApproachingState::approach_timer_callback, this));      
  }
  
  ApproachingState::~ApproachingState() {
  }
  
  void ApproachingState::pend( TargetLanderNode * lander)  {
    lander->set_state(TargetLanderNode::ST_PEND);
  }
  
  void ApproachingState::seek( TargetLanderNode * lander)  {
    lander->set_state(TargetLanderNode::ST_SEEK);
  }
  
  void ApproachingState::descend( TargetLanderNode * lander)  {
    lander->set_state(TargetLanderNode::ST_DESCEND);
  }
  
  void ApproachingState::handle_track_message(TargetLanderNode * lander, const lander_interfaces::msg::Track::SharedPtr msg, const geometry_msgs::msg::Pose::SharedPtr pose, const geometry_msgs::msg::Twist::SharedPtr twist) {
  
    // Implement control logic to correct for velocity and position errors.
    
    if ((pose->position.x > 5.0) || (pose->position.x > 5.0)) {
      RCLCPP_WARN(node_->get_logger(), "[APPRAOCH] Drone has blown away");
    }
    
    // Abort if we are no longer tracking the target
    // RISK: One could fall into a death spiral of Seek->Approach->Seek->Approach
    if (!msg->tracking.data) {
      lander->seek();
      return;  // Explicit return to prevent fall through to rest of code. 
    }
    
    // Guidance provided by sight only.    
    double err_x = msg->position.x; // - pose->position.x;
    double err_y = msg->position.y; // - pose->position.y;
    
    double distance = sqrt(pow(err_x,2) + pow(err_y,2));
    double velocity = sqrt(pow(twist->linear.x,2) + pow(twist->linear.y,2));
    
    bool target_is_close = (distance < descend_radius_);
    bool vehicle_is_stable = (velocity < descend_max_speed_xy_);
    // Transition to DESCEND state if:
    //   - we're within the approach radius of the target
    //   - the vehicle speed is within the approach speed threshold
    //   - those conditions have been true for the holddown period  NOT IMPLIEMENTED YET!!

    if( target_is_close && vehicle_is_stable ) {
      lander->descend();
      
      return;  // Explicit return to prevent fall through to rest of code.      
    }        

    setpoint.linear.x = pid_x->calculate(0.0, err_x);
    setpoint.linear.y = pid_y->calculate(0.0, err_y);
    setpoint.linear.z = 0.0;  //pid_z->calculate(target_seek_altitude_, msg->position.z);  // correct altitude
            
    setpoint.angular.x = 0.0;
    setpoint.angular.y = 0.0;
    setpoint.angular.z = 0.0;
    //setpoint.angular.z = pid_yaw->calculate( yaw_to_target, yaw);         // correct yaw
    
    RCLCPP_INFO(node_->get_logger(), "[APPRAOCH] error: (%6.2f, %6.2f, 000.00) setpoint: (%6.2f, %6.2f, %6.2f) speed: %6.2f  distance: %6.2f",
                        err_x, err_y, setpoint.linear.x, setpoint.linear.y, setpoint.linear.z, velocity, distance);

  }
  
  void ApproachingState::approach_timer_callback() {
    
    // Publish location setpoint once per control loop iteration.      
    twist_pub_->publish(setpoint);    
    
  }
  
  
} // Namespace
