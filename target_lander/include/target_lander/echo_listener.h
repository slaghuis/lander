#pragma once

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>

namespace TargetLander {

  class EchoListener
  {
  public:
    tf2_ros::Buffer buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;

    explicit EchoListener(rclcpp::Clock::SharedPtr clock)
    : buffer_(clock)
    {
      tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
    }

    ~echoListener()
    {
    }
  };  
}
