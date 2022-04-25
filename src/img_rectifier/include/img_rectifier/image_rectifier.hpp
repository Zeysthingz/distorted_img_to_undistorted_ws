#ifndef DISTORTED_IMG_TO_UNDISTORTED_WS_IMAGE_RECTIFIER_NODE_HPP
#define DISTORTED_IMG_TO_UNDISTORTED_WS_IMAGE_RECTIFIER_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include "std_msgs/msg/header.hpp"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include "opencv2/highgui/highgui.hpp"


using namespace std::chrono_literals;

namespace ImageCorrector {

class ImageRectifierNode : public rclcpp::Node {
 public:
  ImageRectifierNode(const rclcpp::NodeOptions &node_options);
  double counter_ = 0;

// private:
  // Publishers
  cv::Mat camera_matrix_;
  cv::Mat distCoeffs;
  cv::Mat undistorted_image;
//
//  These headers will allow us to load an image using OpenCV and convert it to the ROS message format.

//   std::shared_ptr<image_transport::ImageTransport> deneme_;
//   image_transport::Publisher undistorted_img_publisher_;



  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr undistorted_img_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr distorted_img_subscription_;

  void CameraRectifierCallback(const sensor_msgs::msg::Image::SharedPtr &msg_image);
};
} // namespace ground_sep

#endif //  DISTORTED_IMG_TO_UNDISTORTED_WS_IMAGE_RECTIFIER_NODE_HPP

