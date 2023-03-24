#include "webrtc_ros/ros_pcl_capturer.h"
#include "webrtc/rtc_base/bind.h"
#include <boost/enable_shared_from_this.hpp>
#include <cv_bridge/cv_bridge.h>
#include <nlohmann/json.hpp>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace webrtc_ros {

RosPCLCapturer::RosPCLCapturer(rclcpp::Node::SharedPtr &nh,
                               const std::string &topic)
    : impl_(new RosPCLCapturerImpl(nh, topic)) {}

RosPCLCapturer::~RosPCLCapturer() {
  Stop(); // Make sure were stopped so callbacks stop
}

void RosPCLCapturer::Start(
    rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) {
  // std::cout << data_channel->state() << std::endl;
  data_channel_ = data_channel;

  // webrtc::DataBuffer buf("sssssssssssss");
  // data_channel->Send(buf);

  std::async(std::launch::async, [&]() { impl_->Start(this); });
}

void RosPCLCapturer::Stop() { impl_->Stop(); }

void RosPCLCapturer::pclCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromPCLPointCloud2(pcl_pc2, cloud);
  // Save the PCL point cloud to a JSON string
  // Convert the PCL point cloud to a JSON string
  nlohmann::json json_obj;
  for (const auto &point : cloud.points) {
    nlohmann::json point_obj;
    point_obj["x"] = point.x;
    point_obj["y"] = point.y;
    point_obj["z"] = point.z;
    // point_obj["r"] = point.r;
    // point_obj["g"] = point.g;
    // point_obj["b"] = point.b;
    json_obj.push_back(point_obj);
  }
  // Create a JSON object to hold the message data
  // nlohmann::json json_obj;

  // // Copy the point cloud fields to the JSON object
  // json_obj["height"] = msg->height;
  // json_obj["width"] = msg->width;
  // json_obj["is_bigendian"] = msg->is_bigendian;
  // json_obj["point_step"] = msg->point_step;
  // json_obj["row_step"] = msg->row_step;

  // // Create a JSON array to hold the point cloud data
  // auto point_array = nlohmann::json::array();

  // // Iterate over the point cloud data and add it to the array
  // for (sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x"),
  //      iter_y(*msg, "y"), iter_z(*msg, "z");
  //      iter_x != iter_x.end() && iter_y != iter_y.end() &&
  //      iter_z != iter_z.end();
  //      iter_x + 2, ++iter_y + 2, ++iter_z + 2) {
  //   // Add the x, y, and z values to the array as a JSON object
  //   point_array.push_back({{"x", *iter_x}, {"y", *iter_y}, {"z", *iter_z}});
  // }
  // std::vector<uint8_t> data_vec(msg->data.begin(), msg->data.end());

  // // Create a JSON object that represents the message data
  // json_obj = {{"data", data_vec}};

  // // Convert the JSON object to a string
  std::string json_str = json_obj.dump();

  // Add the point cloud array to the JSON object
  // json_obj["points"] = point_array;

  // std::string json_str = json_obj.dump();
  // Set the maximum chunk size (in bytes)
  const int kMaxChunkSize = 16384; // 16 KB

  // Set the maximum buffer size (in bytes)
  const int kMaxBufferSize = 67108864; // 64 MB

  // Calculate the threshold for the data channel buffer
  const int kBufferThreshold = kMaxBufferSize - kMaxChunkSize;

  // Send the JSON data in chunks
  // for (size_t offset = 0; offset < json_str.size(); offset += kMaxChunkSize)
  // {
  //   size_t remaining_bytes = json_str.size() - offset;
  //   size_t chunk_size =
  //       std::min(remaining_bytes, static_cast<size_t>(kMaxChunkSize));
  //   std::string chunk = json_str.substr(offset, chunk_size);
  //   webrtc::DataBuffer buffer(chunk);
  //   while (data_channel_->buffered_amount() + chunk_size >= kBufferThreshold)
  //   {
  //     std::cout << "Buffered amount: " << data_channel_->buffered_amount()
  //               << std::endl;
  //     std::this_thread::sleep_for(
  //         std::chrono::milliseconds(100)); // wait for 100ms
  //   }
  //   if (!data_channel_->Send(buffer)) {
  //     std::cout << "Failed to send data" << std::endl;
  //     break;
  //   }
  // }
  webrtc::DataBuffer buf(json_str);
  data_channel_->Send(buf);
};

RosPCLCapturerImpl::RosPCLCapturerImpl(rclcpp::Node::SharedPtr &nh,
                                       const std::string &topic)
    : nh_(nh), topic_(topic), capturer_(nullptr) {}

void RosPCLCapturerImpl::Start(RosPCLCapturer *capturer) {
  std::unique_lock<std::mutex> lock(state_mutex_);

  sub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_, 10,
      std::bind(&RosPCLCapturerImpl::pclCallback, shared_from_this(),
                std::placeholders::_1));
  capturer_ = capturer;
}

void RosPCLCapturerImpl::Stop() {
  // Make sure to do this before aquiring lock so we don't deadlock with
  // callback This needs to aquire a lock that is heald which callbacks are
  // dispatched
  sub_ = nullptr;

  std::unique_lock<std::mutex> lock(state_mutex_);
  if (capturer_ == nullptr)
    return;

  capturer_ = nullptr;
}

void RosPCLCapturerImpl::pclCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::unique_lock<std::mutex> lock(state_mutex_);
  if (capturer_ == nullptr)
    return;
  capturer_->pclCallback(msg);
}

} // namespace webrtc_ros
