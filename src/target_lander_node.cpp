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

/* ************************************************************************
   * State machine coding pattern to change node behaviour based on the 
   * appraoch to the landing target.
   *
   * Action server to initiate the landing process and provide feedback
   * Subscribes to Tracker messages published by a landing target tracker node
   * Subscribes to Odometry information from the drone to determine posiion
   * Publishes velocity commands to te drone to move it
   *
   ************************************************************************ */
    
#include "target_lander/target_lander.h"

#include "target_lander/pending_state.h"
#include "target_lander/approaching_state.h"
#include "target_lander/descending_state.h"
#include "target_lander/landing_state.h"
#include "target_lander/seeking_state.h"

#include <cmath>  // sqrt. pow

namespace TargetLander {

using namespace std::chrono_literals;

//  TargetLanderNode::TargetLanderNode(const rclcpp::NodeOptions & options)
//  : Node("lander", options)
  TargetLanderNode::TargetLanderNode() : Node("lander") { 
       
    one_off_timer_ = this->create_wall_timer(
      1000ms, std::bind(&TargetLanderNode::init, this));
  
  }

  void TargetLanderNode::init() {
    using namespace std::placeholders;
    
    // Only run this once.  Stop the timer that triggered this.
    this->one_off_timer_->cancel();
    
    // Declare all the parameters
    this->declare_parameter("target_local_position");
    this->declare_parameter<float>("target_seek_altitude", DEFAULT_TARGET_SEEK_ALTITUDE);
    this->declare_parameter<int>("approach_holddown", DEFAULT_APPROACH_HOLDDOWN);
    this->declare_parameter<float>("approach_speed", DEFAULT_APPROACH_SPEED);
    this->declare_parameter<float>("approach_radius", DEFAULT_APPROACH_RADIUS);
    this->declare_parameter<float>("max_yaw_speed", DEFAULT_MAX_YAW_SPEED);
    this->declare_parameter<float>("camera_yaw_correction", DEFAULT_CAMERA_YAW_CORRECTION);
    this->declare_parameter<float>("camera_vertical_fov", DEFAULT_CAMERA_VERTICAL_FOV);

    this->declare_parameter<float>("max_speed_xy", DEFAULT_MAX_SPEED_XY);
    this->declare_parameter<float>("max_accel_xy", DEFAULT_MAX_ACCEL_XY);
    this->declare_parameter<float>("descend_radius", DEFAULT_DESCEND_RADIUS);
    this->declare_parameter<float>("descend_max_speed_xy", DEFAULT_DESCEND_MAX_SPEED_XY);
    this->declare_parameter<int>("descend_holddown", DEFAULT_DESCEND_HOLDDOWN);

    this->declare_parameter<float>("max_speed_z", DEFAULT_MAX_SPEED_Z);
    this->declare_parameter<float>("max_accel_z", DEFAULT_MAX_ACCEL_Z);
    this->declare_parameter<float>("land_max_speed_xy", DEFAULT_LAND_MAX_SPEED_XY);
    this->declare_parameter<float>("land_altitude", DEFAULT_LAND_ALTITUDE);
    
    // Initialise the state machine in "PENDING" mode.
    auto node_ptr = shared_from_this();        
    t_l_state = std::make_shared<PendingState>(node_ptr);

    // Set a listener to read the drone position  
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
       "drone/odometry", 10, std::bind(&TargetLanderNode::odometry_callback, this, _1));
              
    // Set a listener to read target tracking         
    tracking_ = this->create_subscription<lander_interfaces::msg::TrackStamped>(
        "tracker/track", 10, std::bind(&TargetLanderNode::tracking_callback, this, _1));

    // Declare the action server
    this->action_server_ = rclcpp_action::create_server<TargetLand>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "lander/land",
      std::bind(&TargetLanderNode::handle_goal, this, _1, _2),
      std::bind(&TargetLanderNode::handle_cancel, this, _1),
      std::bind(&TargetLanderNode::handle_accepted, this, _1));
      
           
    // Set a transform Listener
    rclcpp::Clock::SharedPtr clock = this->get_clock();
    echo_listener_ = std::make_shared<EchoListener>(clock);
     
    // Wait for the first transforms to become avaiable.      
    // read rate parameter
    // ros::NodeHandle p_nh("~");
    // p_nh.param("rate", rate_hz, 1.0);
    rclcpp::Rate rate(1.0);   // Set a random rate at 1 Hz
      
    std::string warning_msg;
    while (rclcpp::ok() && !echo_listener_->buffer_.canTransform(
        source_frameid, target_frameid, tf2::TimePoint(), &warning_msg))
      {
        RCLCPP_INFO_THROTTLE( this->get_logger(), *clock, 1000, "Waiting for transform %s ->  %s: %s",
          source_frameid.c_str(), target_frameid.c_str(), warning_msg.c_str());
        rate.sleep();
      }
  
  }
        
  void TargetLanderNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
     // The odom messages will come in "odom" frame.  Use tf to turn into "base_link" frame.    

     // Lookup the transform
     try {
       geometry_msgs::msg::PoseStamped odom_pose, base_link_pose;

       odom_pose.header.frame_id = msg->header.frame_id;
       odom_pose.pose.position.x = msg->pose.pose.position.x;
       odom_pose.pose.position.y = msg->pose.pose.position.y;
       odom_pose.pose.position.z = msg->pose.pose.position.z;

       odom_pose.pose.orientation.x = msg->pose.pose.orientation.x;
       odom_pose.pose.orientation.y = msg->pose.pose.orientation.y;
       odom_pose.pose.orientation.z = msg->pose.pose.orientation.z;
       odom_pose.pose.orientation.w = msg->pose.pose.orientation.w;
       
       echo_listener_->buffer_.transform(odom_pose, base_link_pose, "base_link", tf2::durationFromSec(0.1));
       
       last_pose->position.x = base_link_pose.pose.position.x;
       last_pose->position.y = base_link_pose.pose.position.y;
       last_pose->position.z = base_link_pose.pose.position.z;
             
       last_pose->orientation.x = base_link_pose.pose.orientation.x;
       last_pose->orientation.y = base_link_pose.pose.orientation.y;
       last_pose->orientation.z = base_link_pose.pose.orientation.z;
       last_pose->orientation.w = base_link_pose.pose.orientation.w;

       
     } catch (const tf2::TransformException & ex) {
       RCLCPP_ERROR(this->get_logger(), "Exception thrown: %s", ex.what() ); 
       RCLCPP_ERROR(this->get_logger(), "The current list of frames is: %s", echo_listener_->buffer_.allFramesAsString().c_str() ); 
      //std::cout << "Failure at " << clock->now().seconds() << std::endl;
      //std::cout << "Exception thrown:" << ex.what() << std::endl;
      //std::cout << "The current list of frames is:" << std::endl;
      //std::cout << echoListener.buffer_.allFramesAsString() << std::endl;
     }
      
      // Orientation quaternion
      tf2::Quaternion q(
        last_pose->orientation.x,
        last_pose->orientation.y,
        last_pose->orientation.z,
        last_pose->orientation.w);

      // 3x3 Rotation matrix from quaternion
      tf2::Matrix3x3 m(q);

      // Roll Pitch and Yaw from rotation matrix
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
    
     last_twist->linear.x = msg->twist.twist.linear.x;
     last_twist->linear.y = msg->twist.twist.linear.y;
     last_twist->linear.z = msg->twist.twist.linear.z;
     
     last_twist->angular.x = msg->twist.twist.angular.x;
     last_twist->angular.y = msg->twist.twist.angular.y;
     last_twist->angular.z = msg->twist.twist.angular.z;
                      
  }
    
  void TargetLanderNode::tracking_callback(const lander_interfaces::msg::TrackStamped::SharedPtr msg) 
  {     
     //RCLCPP_INFO(this->get_logger(), "[%s] x:%f y:%f", t_l_state->get_name().c_str(), msg->track.position.x, msg->track.position.y);
     
     // TODO Use tf2 to transform this message to base_node frame.  It most liekely comes off tracker (camera) frame
        
     auto base_node_msg = std::make_shared<lander_interfaces::msg::Track>();
                                           
     base_node_msg->tracking.data = msg->track.tracking.data;
     base_node_msg->position.x = msg->track.position.x;
     base_node_msg->position.y = msg->track.position.y;
     base_node_msg->position.z = msg->track.position.z;
     
     t_l_state->handle_track_message(this, base_node_msg, last_pose, last_twist);
  }

  void TargetLanderNode::pend() {
    t_l_state->pend(this);
  }
  
  void TargetLanderNode::seek() {
    t_l_state->seek(this);
  }

  void TargetLanderNode::approach() {
    t_l_state->approach(this);
  }

  void TargetLanderNode::descend() {
    t_l_state->descend(this);
  }

  void TargetLanderNode::land()  {
    t_l_state->land(this); 
  }

  void TargetLanderNode::set_state(State state) {
    RCLCPP_INFO(this->get_logger(), "Leaving state '%s'", t_l_state->get_name().c_str());
    
    auto node_ptr = shared_from_this();        
     
    if(state == ST_PEND) {
      t_l_state = std::make_shared<PendingState>(node_ptr);
    } else if(state == ST_SEEK) {
      t_l_state = std::make_shared<SeekingState>(node_ptr);
    } else if(state == ST_APPROACH) {
      t_l_state = std::make_shared<ApproachingState>(node_ptr);
    } else if(state == ST_DESCEND) {
      t_l_state = std::make_shared<DescendingState>(node_ptr);
    } else if(state == ST_LAND) {
      t_l_state = std::make_shared<LandingState>(node_ptr);
    }

    RCLCPP_INFO(this->get_logger(), "Entering state '%s'", t_l_state->get_name().c_str());
  }

  rclcpp_action::GoalResponse TargetLanderNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TargetLand::Goal> )
  {
    RCLCPP_DEBUG(this->get_logger(), "Received goal request to land.");
    (void)uuid;
    
    // Set the state machine to seeking state.
    seek();  // set_state(ST_SEEK);
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse TargetLanderNode::handle_cancel(
    const std::shared_ptr<GoalHandleTargetland> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    
    pend();
    
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void TargetLanderNode::execute(const std::shared_ptr<GoalHandleTargetland> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1.0);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TargetLand::Feedback>();
    auto & distance_to_target = feedback->distance_to_target;
    auto result = std::make_shared<TargetLand::Result>();
    
    distance_to_target = 100.0;  // Initialise to save the loop.

    while (rclcpp::ok() && (distance_to_target > 0.3 )) {

      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->result = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }

      // Publish feedback
      float xy_error = sqrt(pow(last_pose->position.x,2) + pow(last_pose->position.y,2));
      distance_to_target = sqrt(pow(xy_error,2) + pow(last_pose->position.z,2));
      goal_handle->publish_feedback(feedback);

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->result = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }

  void TargetLanderNode::handle_accepted(const std::shared_ptr<GoalHandleTargetland> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TargetLanderNode::execute, this, _1), goal_handle}.detach();
  }
    
} // namespace TargetLander

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<TargetLander::TargetLanderNode>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}

