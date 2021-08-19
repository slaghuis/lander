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

// Assume center of image is 0.0.  
// Report back FLU coordinates indicate the error of the center of the landing target

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "lander_interfaces/msg/track.hpp"
#include "lander_interfaces/msg/track_stamped.hpp"

#include <lander/pid.h>

using namespace std::chrono_literals;

class TrackerNode : public rclcpp::Node
{
  public:
    TrackerNode()
    : Node("tracker"), count_(0)
    {
      x_val=95.0;
      y_val=85.0;
      
      publisher_ = this->create_publisher<lander_interfaces::msg::TrackStamped>("tracker/track", 10);
      timer_ = this->create_wall_timer(
        500ms, std::bind(&TrackerNode::timer_callback, this));
    }

  private:
    void timer_callback()
    {    
      auto message = lander_interfaces::msg::TrackStamped();
    //      message.header.stamp=this->get_clock().now().to_msg();
      message.header.frame_id="camera";
      
      float x_inc = x_pid.calculate(0, x_val);
      float y_inc = y_pid.calculate(0, y_val);
      x_val += x_inc;
      y_val += y_inc;

      message.track.tracking.data = (x_val < 5.0) && (y_val < 5.0);
            
      message.track.position.x = x_val;
      message.track.position.y = y_val;
      message.track.position.z = 0.0;
      message.track.velocity.x = 0.0;
      message.track.velocity.y = 0.0;
      message.track.velocity.z = 0.0;
      
      RCLCPP_INFO(this->get_logger(), "Distance x:'%f' y:'%f;", message.track.position.x, message.track.position.y);
      publisher_->publish(message);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<lander_interfaces::msg::TrackStamped>::SharedPtr publisher_;
    size_t count_;
    
    PID x_pid = PID(0.5, 5, -5, 0.7, 0.01, 0.5);
    PID y_pid = PID(0.5, 5, -5, 0.7, 0.01, 0.5);
    float x_val, y_val;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackerNode>());
  rclcpp::shutdown();
  return 0;
}
