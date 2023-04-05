#include "webrtc_ros/ros_pcl_capturer.h"
#include "lz4.h"
#include "webrtc/rtc_base/bind.h"
#include <boost/enable_shared_from_this.hpp>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <nlohmann/json.hpp>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sys/wait.h>
#include <vector>
#include <zlib.h>
namespace webrtc_ros {

RosPCLCapturer::RosPCLCapturer(rclcpp::Node::SharedPtr &nh,
                               const std::string &topic)
    : impl_(new RosPCLCapturerImpl(nh, topic)) {}

RosPCLCapturer::~RosPCLCapturer() {
  Stop(); // Make sure were stopped so callbacks stop
}

void RosPCLCapturer::Start(
    rtc::scoped_refptr<webrtc::DataChannelInterface> data_channel) {
  trigger_ = true;
  data_channel_ = data_channel;
  std::async(std::launch::async, [&]() { impl_->Start(this); });
}

bool RosPCLCapturer::GetStatus() { return trigger_; }

void RosPCLCapturer::Stop() {
  trigger_ = false;
  impl_->Stop();
}

std::string RosPCLCapturer::splitPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr &msg) {
  // Split the point cloud into several smaller point clouds
  // ...
  // std::vector<pcl::PointCloud<pcl::PointXYZRGB>> pcl_vec;
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  pcl::fromPCLPointCloud2(pcl_pc2, cloud);

  // Here are the points for each vector
  // int threshHold = 200;

  // std::vector<nlohmann::json> json_objs;
  // std::vector<std::string> json_strs;
  nlohmann::json json_obj;

  int count = 0;
  for (const auto &point : cloud.points) {
    if (point.x == 0.0 && point.y == 0.0 && point.z == 0.0)
      continue;
    nlohmann::json point_obj;
    // point_obj["x"] = int(point.x * 100);
    // point_obj["y"] = int(point.y * 100);
    // point_obj["z"] = int(point.z * 100);
    point_obj["x"] = ((int(point.x * 100) + 100) << 16) |
                     ((int(point.y * 100) + 100) << 8) |
                     (int(point.z * 100) + 100);
    point_obj["r"] = (point.r << 16) | (point.g << 8) | point.b;
    json_obj.push_back(point_obj);
    count++;
  }

  std::string json_str = json_obj.dump();

  // 压缩字符串
  // int max_compressed_size = LZ4_compressBound(json_str.size());
  // std::string compressed_data(max_compressed_size, '\0');
  // int compressed_size =
  //     LZ4_compress_default(json_str.data(), compressed_data.data(),
  //                          json_str.size(), max_compressed_size);

  // // 输出压缩后的数据
  // std::cout << "Original size: " << json_str.size() << std::endl;
  // std::cout << "Compressed size: " << compressed_size << std::endl;
  // std::cout << "Compressed data: " << compressed_data.substr(0,
  // compressed_size)
  //           << ros_PCL_->Start(data_channel_);std::endl;
  // 发送二进制数据
  // webrtc::DataBuffer buffer(
  //     rtc::CopyOnWriteBuffer(compressed_data.data(), compressed_size), true);
  // webrtc::DataBuffer buffer(
  //     rtc::CopyOnWriteBuffer(compressed_data.data(), compressed_size), true);
  // data_channel_->Send(buffer);

  webrtc::DataBuffer buffer(json_str);
  data_channel_->Send(buffer);

  return json_str;
}

void RosPCLCapturer::processPointCloud(
    pcl::PointCloud<pcl::PointXYZRGB> &cloud) {
  // Process the sub-cloud
  // ...
}

void RosPCLCapturer::pclCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  std::future<std::string> result = std::async(
      std::launch::async, &RosPCLCapturer::splitPointCloud, this, msg);
  // Wait for the result from splitPointCloud and send it over the data channel
  // std::string a = result.get();
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
