#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

int main(int argc, char **argv) {
  // Initialize ROS2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cv_image_publisher");

  // Load the PNG images using OpenCV
  cv::Mat rgb_image = cv::imread("rgb.png", cv::IMREAD_UNCHANGED);
  cv::Mat depth_image = cv::imread("depth.png", cv::IMREAD_UNCHANGED);

  // Convert the cv images to ROS2 messages
  sensor_msgs::msg::Image::Ptr rgb_image_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_image)
          .toImageMsg();
  sensor_msgs::msg::Image::Ptr depth_image_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", depth_image)
          .toImageMsg();

  // Initialize ROS2 image publishers
  image_transport::ImageTransport image_transport(node);
  auto rgb_publisher = image_transport.advertise("rgb_topic", 1);
  auto depth_publisher = image_transport.advertise("depth_topic", 1);

  // Publish the ROS2 messages on the RGB and depth topics
  rclcpp::Rate loop_rate(30);
  while (rclcpp::ok()) {
    rgb_publisher.publish(rgb_image_msg);
    depth_publisher.publish(depth_image_msg);
    loop_rate.sleep();
  }

  return 0;
}
