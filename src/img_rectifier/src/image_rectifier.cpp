#include "img_rectifier/image_rectifier.hpp"
#include "opencv2/opencv.hpp"

namespace ImageCorrector {
ImageRectifierNode::ImageRectifierNode(const rclcpp::NodeOptions &node_options)
    : Node("image_corrector", node_options) {
//     counter_= 0;
  undistorted_img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("undistorted_img", 10);

  std::cout << "OpenCV version : " << CV_VERSION << std::endl;
  auto callback = [this](sensor_msgs::msg::Image::SharedPtr msg_image) {
    this->CameraRectifierCallback(msg_image);
  };
  distorted_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/right/velodyne_points", 10, callback);

}

void ImageRectifierNode::CameraRectifierCallback(const sensor_msgs::msg::Image::SharedPtr &msg_image) {
//  auto message = std_msgs::msg::String();
////  std::allocator<double> counter_;
////  counter_ = 0;
//  message.data = "Hello, world! " + std::to_string(counter_);
//  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//  publisher_->publish(message);

}
} // for namespace

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ImageCorrector::ImageRectifierNode)