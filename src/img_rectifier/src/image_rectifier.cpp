#include "img_rectifier/image_rectifier.hpp"

namespace ImageCorrector {
ImageRectifierNode::ImageRectifierNode(const rclcpp::NodeOptions &node_options)
    : Node("image_corrector", node_options) {

  m_camera_matrix_param = this->declare_parameter<std::vector<double>>("camera_matrix.data");
  m_distortion_coefs_param = this->declare_parameter<std::vector<double>>("distortion_coefficients.data");

  m_distCoeffs = (cv::Mat1d(m_distortion_coefs_param));
  std::cout << m_distCoeffs << std::endl;

//  std::cout<<camera_matrix.data()<<std::endl;
  m_camera_matrix = cv::Mat1d(3, 3, m_camera_matrix_param.data());
  std::cout << m_camera_matrix << std::endl;

  cv::Size s = m_camera_matrix.size();
  std::cout << s << std::endl;

  undistorted_img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("undistorted_img", 10);

  std::cout << "OpenCV version : " << CV_VERSION << std::endl;

//  auto callback = [this](sensor_msgs::msg::Image::SharedPtr msg_image) {
//    this->CameraRectifierCallback(msg_image);
//  };
//  distorted_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
//      "/image_raw", 10, callback);
  functionSub = std::bind(&ImageRectifierNode::CameraRectifierCallback, this, std::placeholders::_1);
  distorted_img_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", 10, functionSub);
}

void ImageRectifierNode::CameraRectifierCallback(const sensor_msgs::msg::Image::SharedPtr &msg_image) {

//  std::cout << "Type of msg_image : " << typeid(msg_image).name() << std::endl;

//takes publishers header data
  std_msgs::msg::Header msg_header = msg_image->header;
  std::string frame_id = msg_header.frame_id.c_str();
  std::cout << "New Image from " << frame_id << std::endl;

// When converting a ROS sensor_msgs/Image message into a CvImage

  cv_bridge::CvImagePtr cv_ptr;

//pointer tanımladım bu sayede boyutu buyuk data tutabılırım.
  try {
    cv_ptr = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::RGB8);

  }
  catch (cv_bridge::Exception &e) {
    std::cerr << "cv_bridge exception: " << e.what() << std::endl;
  }

  cv::Mat image = cv_ptr->image;
  std::cout << image.rows << std::endl;
  std::cout << image.cols << std::endl;
  cv::Mat m_undistorted_image;

//undistortion function
  cv::undistort(image, m_undistorted_image, m_camera_matrix, m_distCoeffs);
  std::cout << m_undistorted_image.rows << " " << m_undistorted_image.cols << " " << m_undistorted_image.channels()
            << std::endl;
  cv::Mat img = m_undistorted_image;
  std_msgs::msg::Header header;
  header.frame_id = frame_id;

//takes publishers tımestamp
  header.stamp = msg_header.stamp;

//  creates new pointer data includes ımages
  sensor_msgs::msg::Image::SharedPtr ros_image_ptr(new sensor_msgs::msg::Image());

//converts opencv ımages to ros messages by toImageMsg()
  ros_image_ptr = cv_bridge::CvImage(header,
                                     sensor_msgs::image_encodings::RGB8, // MONO8 or BGR8
                                     img).toImageMsg();

//  publishing
  undistorted_img_publisher_->publish(*ros_image_ptr);

}
} // for namespace

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ImageCorrector::ImageRectifierNode)