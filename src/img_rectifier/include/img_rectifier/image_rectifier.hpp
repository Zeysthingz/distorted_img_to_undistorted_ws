#ifndef DISTORTED_IMG_TO_UNDISTORTED_WS_IMAGE_RECTIFIER_NODE_HPP
#define DISTORTED_IMG_TO_UNDISTORTED_WS_IMAGE_RECTIFIER_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include "std_msgs/msg/header.hpp"
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>
#include <iostream>

namespace ImageCorrector {

class ImageRectifierNode : public rclcpp::Node {
 public:
  ImageRectifierNode(const rclcpp::NodeOptions &node_options);

// private:
  // Publishers
  cv::Mat m_camera_matrix;
  cv::Mat m_distCoeffs;
  cv::Mat m_undistorted_image;

//vektorun içinde double türünde veriler var
  std::vector<double> m_camera_matrix_param;
  std::vector<double> m_distortion_coefs_param;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr undistorted_img_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr distorted_img_subscription_;

  void CameraRectifierCallback(const sensor_msgs::msg::Image::SharedPtr &msg_image);
};
} // namespace ground_sep

#endif //  DISTORTED_IMG_TO_UNDISTORTED_WS_IMAGE_RECTIFIER_NODE_HPP

