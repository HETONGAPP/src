// #ifdef PCI_IMAGE_H
// #define PCI_IMAGE_H

#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <opencv2/highgui.hpp>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using PointT = pcl::PointXYZRGB;
using PointCloudT = pcl::PointCloud<PointT>;
using cloud_pointer = PointCloudT::Ptr;
using prevCloud = PointCloudT::Ptr;

std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture,
                                      rs2::texture_coordinate Texture_XY) {
  // Get Width and Height coordinates of texture
  int width = texture.get_width();   // Frame width in pixels
  int height = texture.get_height(); // Frame height in pixels

  // Normals to Texture Coordinates conversion
  int x_value =
      std::min(std::max(int(Texture_XY.u * width + .5f), 0), width - 1);
  int y_value =
      std::min(std::max(int(Texture_XY.v * height + .5f), 0), height - 1);

  int bytes =
      x_value * texture.get_bytes_per_pixel(); // Get # of bytes per pixel
  int strides =
      y_value * texture.get_stride_in_bytes(); // Get line width in bytes
  int Text_Index = (bytes + strides);

  const auto New_Texture =
      reinterpret_cast<const uint8_t *>(texture.get_data());

  // RGB components to save in tuple
  int NT1 = New_Texture[Text_Index];
  int NT2 = New_Texture[Text_Index + 1];
  int NT3 = New_Texture[Text_Index + 2];

  return std::tuple<int, int, int>(NT1, NT2, NT3);
}

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud object with depth and RGB data
// from a single frame captured using the Realsense.
//===================================================
cloud_pointer PCL_Conversion(const rs2::points &points,
                             const rs2::video_frame &color) {

  // Object Declaration (Point Cloud)
  cloud_pointer cloud(new PointCloudT);

  // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
  std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

  //================================
  // PCL Cloud Object Configuration
  //================================
  // Convert data captured from Realsense camera to Point Cloud
  auto sp = points.get_profile().as<rs2::video_stream_profile>();

  cloud->width = static_cast<uint32_t>(sp.width());
  cloud->height = static_cast<uint32_t>(sp.height());
  cloud->is_dense = false;
  cloud->points.resize(points.size());

  auto Texture_Coord = points.get_texture_coordinates();
  auto Vertex = points.get_vertices();

  // Iterating through all points and setting XYZ coordinates
  // and RGB values
  for (int i = 0; i < points.size(); i++) {
    //===================================
    // Mapping Depth Coordinates
    // - Depth data stored as XYZ values
    //===================================
    cloud->points[i].x = Vertex[i].x;
    cloud->points[i].y = Vertex[i].y;
    cloud->points[i].z = Vertex[i].z;

    // Obtain color texture for specific point
    RGB_Color = RGB_Texture(color, Texture_Coord[i]);

    // Mapping Color (BGR due to Camera Model)
    cloud->points[i].r = std::get<2>(RGB_Color); // Reference tuple<2>
    cloud->points[i].g = std::get<1>(RGB_Color); // Reference tuple<1>
    cloud->points[i].b = std::get<0>(RGB_Color); // Reference tuple<0>
  }

  return cloud; // PCL RGB Point Cloud generated
}
class PCImage : public rclcpp::Node {

public:
  PCImage(rs2::pipeline &pipe) : Node("PC_image_publisher") {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [&]() {
      rs2::frameset frames = pipe.wait_for_frames();
      rs2::depth_frame depth_frame = frames.get_depth_frame();
      rs2::video_frame color_frame = frames.get_color_frame();
      // Convert the depth and color images to a point cloud
      rs2::pointcloud pc;
      pc.map_to(color_frame);
      rs2::points points = pc.calculate(depth_frame);

      PointCloudT::Ptr cloud(new PointCloudT);

      cloud = PCL_Conversion(points, color_frame);

      pcl::PassThrough<PointT> pass;
      pass.setInputCloud(cloud);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.0, 1.0);

      PointCloudT::Ptr cloud_filtered(new PointCloudT);
      pass.filter(*cloud_filtered);
      cloud->swap(*cloud_filtered);

      // Create a new point cloud to store the downsampled data
      PointCloudT::Ptr cloud_downsampled(new PointCloudT);

      // Create a voxel grid filter object
      // pcl::VoxelGrid<PointT> sor;
      pcl::RandomSample<PointT> rs;

      std::cout << "PCL original points size: " << cloud->size() << std::endl;
      if (cloud->size() >= 2500) {
        // Set the input cloud for the filter
        // sor.setInputCloud(cloud);

        // // Set the voxel grid size
        // sor.setLeafSize(0.03f, 0.03f, 0.03f);

        // sor.filter(*cloud_downsampled);

        // Set the input cloud for the filter
        rs.setInputCloud(cloud);

        // Set the ratio of points to be randomly selected
        rs.setSample(2500);

        // Apply the filter to obtain the downsampled point cloud
        rs.filter(*cloud_downsampled);

        cloud->swap(*cloud_downsampled);
      }

      // Remove NaN values from the point cloud
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

      sensor_msgs::msg::PointCloud2 point_cloud;
      pcl::toROSMsg(*cloud, point_cloud);

      point_cloud.header.frame_id = "realsense_camera";
      point_cloud.header.stamp = this->now();
      publisher_->publish(point_cloud);
      // std::cout << "PCL points size: " << cloud->size() << std::endl;
    });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
// #endif // PCI_IMAGE_H
