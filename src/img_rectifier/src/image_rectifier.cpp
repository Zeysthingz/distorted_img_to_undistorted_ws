#include "img_rectifier/image_rectifier.hpp"
#include "opencv2/opencv.hpp"

namespace ImageCorrector {
ImageRectifierNode::ImageRectifierNode(const rclcpp::NodeOptions &node_options)
    : Node("image_corrector", node_options) {

//  distCoeffs = (cv::Mat1d(1, 5)
//      << -0.013081, 0.070711, 0.002003, -0.017107, 0.000000);
//  camera_matrix_ = (cv::Mat1d(3, 3)
//      << 544.222867, 0.000000, 298.213388, 0.000000, 544.289845,
//      230.837500, 0.000000, 0.000000, 1.000000
//  );
  std::cout << "denemeeee " << std::endl;

  camera_matrix = this->declare_parameter<std::vector<double>>("camera_matrix.data");
  distortion_coefs = this->declare_parameter<std::vector<double>>("distortion_coefficients.data");

  distCoeffs = (cv::Mat1d(distortion_coefs));
//  std::cout << distCoeffs << std::endl;


//  std::cout<<camera_matrix.data()<<std::endl;
  camera_matrix_ = cv::Mat1d(3, 3, camera_matrix.data());
  std::cout << camera_matrix_ << std::endl;


  cv::Size s = camera_matrix_.size();
  std::cout<<s<<std::endl;


//vektor elemanı bastırmak ıcın
//  std::cout << camera_matrix.at(0) << std::endl;
//  std::cout << distortion_coefs.at(0) << std::endl;
//  std::cout << distCoeffs.size << std::endl;
//  std::cout << distCoeffs << std::endl;
//  std::cout << camera_matrix_ << std::endl;

  undistorted_img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("undistorted_img", 10);

  std::cout << "OpenCV version : " << CV_VERSION << std::endl;
  auto callback = [this](sensor_msgs::msg::Image::SharedPtr msg_image) {
    this->CameraRectifierCallback(msg_image);
  };
  distorted_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, callback);

}

void ImageRectifierNode::CameraRectifierCallback(const sensor_msgs::msg::Image::SharedPtr &msg_image) {

//  std::cout << "Type of msg_image : " << typeid(msg_image).name() << std::endl;
//
  std_msgs::msg::Header msg_header = msg_image->header;
  std::string frame_id = msg_header.frame_id.c_str();
  std::cout << "New Image from " << frame_id << std::endl;



// When converting a ROS sensor_msgs/Image message into a CvImage
  cv::Mat cv_image = cv_bridge::toCvCopy(msg_image, "bgr8")->image;

  cv::undistort(cv_image, undistorted_image, camera_matrix_, distCoeffs);
  std::cout << undistorted_image.rows << " " << undistorted_image.cols << " " << undistorted_image.channels()
            << std::endl;

  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp = this->get_clock()->now();
  sensor_msgs::msg::Image::SharedPtr ros_image_ptr(new sensor_msgs::msg::Image());

  ros_image_ptr = cv_bridge::CvImage(header,
                                     sensor_msgs::image_encodings::BGR8, // MONO8 or BGR8
                                     undistorted_image).toImageMsg();
  undistorted_img_publisher_->publish(*ros_image_ptr);

}
} // for namespace

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ImageCorrector::ImageRectifierNode)