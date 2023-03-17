// #include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>

// class MyNode : public rclcpp::Node {
// public:
//   MyNode() : Node("my_node") {
//     subscription_ = create_subscription<sensor_msgs::msg::Image>(
//         "/image", 10,
//         std::bind(&MyNode::callback, this, std::placeholders::_1));
//     cv_bridge_ = std::make_shared<cv_bridge::CvImage>();
//   }

//   void callback(const sensor_msgs::msg::Image::SharedPtr msg) {
//     latest_image_ = msg;
//   }

//   cv::Mat get_latest_cv_image() {
//     if (latest_image_ == nullptr) {
//       return cv::Mat();
//     }
//     cv_bridge_ = cv_bridge::toCvCopy(latest_image_, "bgr8");
//     return cv_bridge_->image;
//   }

// private:
//   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
//   sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
//   std::shared_ptr<cv_bridge::CvImage> cv_bridge_;
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<MyNode>();
//   while (rclcpp::ok()) {
//     cv::Mat cv_image = node->get_latest_cv_image();
//     if (!cv_image.empty()) {
//       // Process the latest image using OpenCV
//       cv::imwrite("o.png", cv_image);
//       cv::imshow("Image", cv_image);
//       cv::waitKey(1);
//     }
//     rclcpp::spin_some(node);
//   }
//   rclcpp::shutdown();
//   return 0;
// }

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node("my_node") {
    subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "/depth", 10,
        std::bind(&MyNode::callback, this, std::placeholders::_1));
    cv_bridge_ = std::make_shared<cv_bridge::CvImage>();
  }

  void callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    latest_image_ = msg;
  }

  cv::Mat get_latest_cv_image() {
    if (latest_image_ == nullptr) {
      return cv::Mat();
    }
    cv_bridge_ = cv_bridge::toCvCopy(latest_image_,
                                     sensor_msgs::image_encodings::TYPE_32FC1);
    cv::Mat depth_image = cv_bridge_->image;
    // Scale the depth values to the range [0, 255] and convert to 8-bit format
    cv::Mat depth_image_8bit;
    depth_image.convertTo(depth_image_8bit, CV_8UC1,
                          255.0 / 10.0); // assuming max depth is 10 meters
    return depth_image_8bit;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  std::shared_ptr<cv_bridge::CvImage> cv_bridge_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  while (rclcpp::ok()) {
    cv::Mat cv_image = node->get_latest_cv_image();
    if (!cv_image.empty()) {
      // Process the latest image using OpenCV
      cv::imwrite("depth.png", cv_image);
      cv::imshow("Image", cv_image);
      cv::waitKey(1);
    }
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}
