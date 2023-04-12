// #ifdef DEPTH_IMAGE_H
// #define DEPTH_IMAGE_H

#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
class DepthImage : public rclcpp::Node {

public:
  DepthImage(rs2::pipeline &pipe) : Node("Depth_image_publisher") {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("depth", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [&]() {
      // rs2::frameset frames = pipe.wait_for_frames();

      // // Publish the depth image
      // rs2::depth_frame depth_frame = frames.get_depth_frame();
      // sensor_msgs::msg::Image depth_image;
      // depth_image.header.frame_id = "realsense_camera";
      // depth_image.header.stamp = this->now();
      // depth_image.width = depth_frame.get_width();
      // depth_image.height = depth_frame.get_height();
      // depth_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      // depth_image.step = depth_image.width * sizeof(uint16_t);
      // depth_image.data.resize(depth_image.height * depth_image.step);
      // memcpy(depth_image.data.data(), depth_frame.get_data(),
      //        depth_image.data.size());
      // publisher_->publish(depth_image);

      rs2::frameset frames = pipe.wait_for_frames();

      // Publish the depth image
      rs2::depth_frame depth_frame = frames.get_depth_frame();
      sensor_msgs::msg::Image depth_image;
      depth_image.header.frame_id = "realsense_camera";
      depth_image.header.stamp = this->now();
      depth_image.width = depth_frame.get_width();
      depth_image.height = depth_frame.get_height();
      depth_image.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      depth_image.step = depth_image.width * sizeof(uint16_t);

      cv::Mat depth_(cv::Size(640, 480), CV_16UC1,
                     (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
      depth_.convertTo(depth_, CV_8UC1,
                       255.0 / 1000.0); // Convert to 8-bit depth image
      cv::applyColorMap(depth_, depth_,
                        cv::COLORMAP_JET); // Apply colormap
      sensor_msgs::msg::Image::SharedPtr msg =
          cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", depth_)
              .toImageMsg();
      depth_image.data.resize(depth_image.height * depth_image.step);
      memcpy(depth_image.data.data(), depth_frame.get_data(),
             depth_image.data.size());
      publisher_->publish(*msg);
    });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
// #endif // DEPTH_IMAGE_H
