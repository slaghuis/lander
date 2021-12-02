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

/*
SetTarget.srv

int64 target_num
---
bool result
*/

#include <inttypes.h>
#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <vector>
#include <sstream>

#include "cv_bridge/cv_bridge.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>

// #include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <message_filters/subscriber.h>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#ifdef TF2_CPP_HEADERS
  #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
  #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#include "lander_interfaces/srv/set_target.hpp"

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using SetTarget = lander_interfaces::srv::SetTarget;

class TrackerNode : public rclcpp::Node
{
public:
  TrackerNode()
  : Node("tracker_node"), target_number_(0)
  {
    // Instantiate the transform landscape
    tf_buffer_ = 
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);   
    
    // Declare parameters
    this->declare_parameter<std::string>("camera_parameters_file", "/home/eric/ros_ws/src/lander/camera_info/calibration_params.yml");
    this->declare_parameter<float>("marker_length", 0.00245);  // the length of the markers' side. The returning translation vectors will be in the same unit. Normally, unit is meters.
    
    // Give the node a second to start up before initiating
    one_off_timer_ = this->create_wall_timer(1000ms, std::bind(&TrackerNode::init, this));

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("lander/target_pose", 10);      
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("tracker/image", 10);  
      
      
    service_ = create_service<SetTarget>("lander/set_target", std::bind(&TrackerNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));
   
  }

private:
  void init()
  {
    // Only run this once.  Stop the timer that triggered this.
    this->one_off_timer_->cancel();
    
    std::string filename;
    this->get_parameter("camera_parameters_file", filename);
    this->get_parameter("marker_length", marker_length_);
    
    RCLCPP_INFO(this->get_logger(), "Reading camera calibration parameters from: %s", filename.c_str());
    bool readOk = readCameraParameters(filename, cameraMatrix, distCoeffs);
    
    if(!readOk) {
      RCLCPP_ERROR(this->get_logger(), "Invalid camera parameters file: %s", filename.c_str());
    } else {
    
      dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
      params = cv::aruco::DetectorParameters::create();
      
      image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&TrackerNode::image_callback, this, std::placeholders::_1));
    }

  }
  
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(),
            "Image received\t Timestamp: %u.%u sec ",msg->header.stamp.sec,msg->header.stamp.nanosec);
            
    // Frame acquisition
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s",e.what());
        return;
    }

    cv::Mat image, imageCopy;
    cv_ptr->image.copyTo(image);   // BGR image coming from Raspberry Pi Camera via ROSinputVideo.retrieve(image);
    image.copyTo(imageCopy);
    
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f> > corners;
    
    cv::aruco::detectMarkers(image, dictionary, corners, ids);
    RCLCPP_DEBUG(this->get_logger(), "%i Markers found.", ids.size());
    
    // if at least one marker detected
    if (ids.size() > 0) {
      RCLCPP_DEBUG(this->get_logger(), "Draw Markers");
      cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
      
      std::vector<cv::Vec3d> rvecs, tvecs;
      cv::aruco::estimatePoseSingleMarkers(corners, marker_length_, cameraMatrix, distCoeffs, rvecs, tvecs);

      // draw axis for each marker
      int selected_index = 0;
      bool found = false;
      for (size_t i = 0; i<(size_t)ids.size(); i++) {
        RCLCPP_DEBUG(this->get_logger(), "Seen ARUCO marker number %i", ids[i]); 
        if(ids[i] == target_number_) {
          selected_index = (int)i;
          found = true;
        }  
        cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
      }    

      //compile a message, and send it
      auto message = geometry_msgs::msg::PoseStamped();
      rclcpp::Time now = this->get_clock()->now();
      message.header.stamp = now;
      message.header.frame_id = "base_camera";

      // Build identity rotation matrix as a cv::Mat
      cv::Mat rot(3, 3, CV_64FC1);
      cv::Rodrigues(rvecs[selected_index], rot);
      
      // Convert to a tf2::Matrix3x3
      tf2::Matrix3x3 tf2_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
                             rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
                             rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));

      // Create a transform and convert to a Pose
      tf2::Transform tf2_transform(tf2_rot, tf2::Vector3());
      tf2::toMsg(tf2_transform, message.pose);
      
      message.pose.position.x = tvecs[selected_index](0);
      message.pose.position.y = tvecs[selected_index](1);
      message.pose.position.z = tvecs[selected_index](2);

      float z;
      if ( read_height(&z) ) { // Lookup a transform from base_radar
        // message.pose.position.z = z;  
        RCLCPP_INFO(this->get_logger(), "Camera height = %.3f Sensor height = %.3f",tvecs[selected_index](2),z);
      } 
      
      if (found || (target_number_ == 0)) {
        pose_publisher_->publish(message); 
      }  
    } else {
      //cv::Point textPosition(imageWidth / 4, imageHeight / 4);
      cv::Point textPosition(100, 100);
      int fontSize = 1;
      cv::Scalar fontColor(0,0,255);
      int fontWeight = 1;
      std::ostringstream s;
      s << "Target ";
      
      if (target_number_ == 0) {
        s << "not found.";
      } else {
        s << target_number_  << " not found.";
      }

      cv::putText(imageCopy, s.str(), textPosition, cv::FONT_HERSHEY_SIMPLEX, fontSize, fontColor, fontWeight);
         
    }
      
    // Publish to ROS      
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg; // >> message to be sent

    std_msgs::msg::Header header; // empty header
    header.stamp = this->get_clock()->now();
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, imageCopy);
    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::msg::Image
    image_publisher_->publish(img_msg);      
  }
  
  void handle_service(
  const std::shared_ptr<SetTarget::Request> request,
  const std::shared_ptr<SetTarget::Response> response)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "set landing target to: %i", request->target);
    target_number_ = request->target;                         // Aruco number on the target   
    response->result = true;
    
  }

  bool read_height(float *z)
  {  
    std::string from_frame = "base_radar"; 
    std::string to_frame = "base_camera";   //map_frame_.c_str();
      
    geometry_msgs::msg::TransformStamped transformStamped;
    
    // Look up for the transformation between map and base_link_ned frames
    // and returns the last position in the 'map' frame
    try {
      transformStamped = tf_buffer_->lookupTransform(
        to_frame, from_frame,
        tf2::TimePointZero);
        *z = transformStamped.transform.translation.z;
    } catch (tf2::TransformException & ex) {
      RCLCPP_DEBUG(
        this->get_logger(), "Could not transform %s to %s: %s",
        to_frame.c_str(), from_frame.c_str(), ex.what());
      return false;  
    }
    return true;
  }
  
  // OpenCV Procssing //////////////////////////////////////////////////////////////////////////////
  
  static bool readCameraParameters(std::string filename, cv::Mat& camMatrix, cv::Mat& distCoeffs)
  {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return (camMatrix.size() == cv::Size(3,3)) ;
  }
    
  // Private Variables ///////////////////////////////////////////////////////////////////////////
  
  rclcpp::TimerBase::SharedPtr one_off_timer_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  
  rclcpp::Service<SetTarget>::SharedPtr service_;
  
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  
  int target_number_;
  
  cv::Mat cameraMatrix, distCoeffs;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  cv::Ptr<cv::aruco::DetectorParameters> params;
  float marker_length_;
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackerNode>());
  rclcpp::shutdown();
  return 0;
}
