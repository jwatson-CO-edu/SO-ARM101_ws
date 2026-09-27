#include "lerobot_perception/color_point_cloud_service_node.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include <omp.h>

#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>

namespace lerobot_perception
{

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

ColorPointCloudServiceNode::ColorPointCloudServiceNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("color_point_cloud_service", options)
{
  // Topic names are parameters, not hardcoded: the exact topics depthai_ros_driver publishes
  // have changed across releases. Verify these with `ros2 topic list` against your installed
  // version and override via this node's params file (see config/) if they differ.
  const auto rgb_topic = declare_parameter<std::string>("rgb_topic", "/oak/rgb/image_raw");
  const auto depth_topic = declare_parameter<std::string>(
    "depth_topic", "/oak/stereo/image_raw");
  const auto camera_info_topic = declare_parameter<std::string>(
    "camera_info_topic", "/oak/rgb/camera_info");
  const auto sync_queue_size = declare_parameter<int>("sync_queue_size", 10);

  rgb_sub_.subscribe(this, rgb_topic, rmw_qos_profile_sensor_data);
  depth_sub_.subscribe(this, depth_topic, rmw_qos_profile_sensor_data);
  info_sub_.subscribe(this, camera_info_topic, rmw_qos_profile_sensor_data);

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(sync_queue_size), rgb_sub_, depth_sub_, info_sub_);
  sync_->registerCallback(std::bind(&ColorPointCloudServiceNode::frameCallback, this, _1, _2, _3));

  // Reentrant: lets the executor (must be a MultiThreadedExecutor - see main.cpp) run more
  // than one handleRequest() call at a time when multiple clients request a cloud close
  // together. This is safe because handleRequest() only ever touches its own local copies
  // and stack-allocated PCL objects - no state shared across concurrent calls - beyond the
  // one lock-guarded read of the latest cached frame, which was already thread-safe.
  // frameCallback() is left on the node's default (mutually exclusive) callback group: it
  // doesn't need to overlap with itself, and different groups already run concurrently
  // under a MultiThreadedExecutor regardless of each other's type.
  service_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  service_ = create_service<GetColorPointCloud>(
    "get_color_point_cloud",
    std::bind(&ColorPointCloudServiceNode::handleRequest, this, _1, _2),
    rmw_qos_profile_services_default,
    service_callback_group_);

  RCLCPP_INFO(
    get_logger(), "color_point_cloud_service ready (rgb='%s', depth='%s', camera_info='%s')",
    rgb_topic.c_str(), depth_topic.c_str(), camera_info_topic.c_str());
}

void ColorPointCloudServiceNode::frameCallback(
  const Image::ConstSharedPtr & rgb_msg,
  const Image::ConstSharedPtr & depth_msg,
  const CameraInfo::ConstSharedPtr & info_msg)
{
  std::lock_guard<std::mutex> lock(frame_mutex_);
  latest_rgb_ = rgb_msg;
  latest_depth_ = depth_msg;
  latest_info_ = info_msg;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorPointCloudServiceNode::buildCloud(
  const Image::ConstSharedPtr & rgb_msg,
  const Image::ConstSharedPtr & depth_msg,
  const CameraInfo::ConstSharedPtr & info_msg,
  std::string & error) const
{
  if (rgb_msg->width != depth_msg->width || rgb_msg->height != depth_msg->height) {
    error =
      "rgb (" + std::to_string(rgb_msg->width) + "x" + std::to_string(rgb_msg->height) +
      ") and depth (" + std::to_string(depth_msg->width) + "x" +
      std::to_string(depth_msg->height) +
      ") image sizes differ - enable RGB/depth alignment in the camera driver so every "
      "depth pixel corresponds to the same pixel in the color image";
    return nullptr;
  }

  cv_bridge::CvImageConstPtr rgb_cv;
  try {
    rgb_cv = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
  } catch (const cv_bridge::Exception & e) {
    error = std::string("cv_bridge conversion of rgb image failed: ") + e.what();
    return nullptr;
  }

  const bool depth_is_uint16 = depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1;
  const bool depth_is_float = depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1;
  if (!depth_is_uint16 && !depth_is_float) {
    error = "unsupported depth encoding '" + depth_msg->encoding +
      "' (expected 16UC1 millimeters or 32FC1 meters)";
    return nullptr;
  }
  cv_bridge::CvImageConstPtr depth_cv = cv_bridge::toCvShare(depth_msg);

  // Pinhole camera model: X = (u - cx) * Z / fx, Y = (v - cy) * Z / fy. K is row-major
  // [fx 0 cx; 0 fy cy; 0 0 1] per sensor_msgs/CameraInfo.
  const double fx = info_msg->k[0];
  const double fy = info_msg->k[4];
  const double cx = info_msg->k[2];
  const double cy = info_msg->k[5];
  if (fx == 0.0 || fy == 0.0) {
    error = "camera_info has zero focal length - is the camera publishing valid intrinsics?";
    return nullptr;
  }

  const int num_rows = rgb_cv->image.rows;
  const int num_cols = rgb_cv->image.cols;
  const std::size_t total_pixels = static_cast<std::size_t>(num_rows) * num_cols;

  // Parallelized per-row: every row is independent (each pixel only reads its own rgb/
  // depth value), but every thread appends to its own std::vector rather than one shared
  // cloud, so there's no synchronization inside the parallel region and no fixed-size/
  // organized cloud with NaN placeholders - just a plain unordered vector of valid points
  // per thread, concatenated into the final cloud once all threads finish. Point order in
  // the output is therefore grouped by thread/row-range rather than strict row-major, which
  // doesn't matter: the cloud is unorganized (height 1) and nothing downstream depends on
  // point order.
  const int num_threads = std::max(1, omp_get_max_threads());
  std::vector<std::vector<pcl::PointXYZRGB>> per_thread_points(num_threads);
  for (auto & buffer : per_thread_points) {
    buffer.reserve(total_pixels / static_cast<std::size_t>(num_threads));
  }

  #pragma omp parallel for schedule(static)
  for (int v = 0; v < num_rows; ++v) {
    std::vector<pcl::PointXYZRGB> & local_points = per_thread_points[omp_get_thread_num()];
    for (int u = 0; u < num_cols; ++u) {
      double depth_m = 0.0;
      if (depth_is_uint16) {
        const std::uint16_t raw = depth_cv->image.at<std::uint16_t>(v, u);
        if (raw == 0) {continue;}  // 0 is the standard "no measurement" sentinel
        depth_m = static_cast<double>(raw) * 0.001;
      } else {
        const float raw = depth_cv->image.at<float>(v, u);
        if (!std::isfinite(raw) || raw <= 0.0f) {continue;}
        depth_m = static_cast<double>(raw);
      }

      pcl::PointXYZRGB point;
      point.x = static_cast<float>((u - cx) * depth_m / fx);
      point.y = static_cast<float>((v - cy) * depth_m / fy);
      point.z = static_cast<float>(depth_m);
      const cv::Vec3b & bgr = rgb_cv->image.at<cv::Vec3b>(v, u);
      point.b = bgr[0];
      point.g = bgr[1];
      point.r = bgr[2];
      local_points.push_back(point);
    }
  }

  auto cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud->reserve(total_pixels);
  for (const auto & buffer : per_thread_points) {
    cloud->insert(cloud->end(), buffer.begin(), buffer.end());
  }

  cloud->is_dense = true;
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->header.frame_id = rgb_msg->header.frame_id;
  // pcl::PCLHeader stores stamp as microseconds since epoch; rclcpp::Time::nanoseconds()
  // gives nanoseconds since epoch, so divide down rather than depending on a
  // pcl_conversions overload that may not exist in every ROS 2 distro's port.
  cloud->header.stamp = static_cast<std::uint64_t>(
    rclcpp::Time(rgb_msg->header.stamp).nanoseconds() / 1000);

  return cloud;
}

void ColorPointCloudServiceNode::handleRequest(
  const std::shared_ptr<GetColorPointCloud::Request> request,
  std::shared_ptr<GetColorPointCloud::Response> response)
{
  Image::ConstSharedPtr rgb_msg;
  Image::ConstSharedPtr depth_msg;
  CameraInfo::ConstSharedPtr info_msg;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    rgb_msg = latest_rgb_;
    depth_msg = latest_depth_;
    info_msg = latest_info_;
  }

  if (!rgb_msg || !depth_msg || !info_msg) {
    response->success = false;
    response->message = "no synchronized camera frame received yet";
    return;
  }

  std::string error;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = buildCloud(rgb_msg, depth_msg, info_msg, error);
  if (!cloud) {
    response->success = false;
    response->message = error;
    return;
  }

  if (request->downsample) {
    if (request->voxel_leaf_size_m <= 0.0) {
      response->success = false;
      response->message = "voxel_leaf_size_m must be > 0 when downsample is requested";
      return;
    }
    auto downsampled = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    const float leaf = static_cast<float>(request->voxel_leaf_size_m);
    voxel_grid.setLeafSize(leaf, leaf, leaf);
    voxel_grid.filter(*downsampled);
    downsampled->header = cloud->header;
    cloud = downsampled;
  }

  std::size_t num_clusters = 0;
  if (request->segment) {
    if (cloud->empty()) {
      response->success = false;
      response->message = "cloud is empty after downsampling, nothing to segment";
      return;
    }

    // Color-Based Region Growing Segmentation (pcl::RegionGrowingRGB): unlike the plain
    // curvature-based pcl::RegionGrowing, this variant does not need precomputed surface
    // normals - it grows regions from spatial proximity (distance_threshold) and per-channel
    // color similarity (point_color_threshold within a region, region_color_threshold when
    // merging two regions) alone.
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> region_growing;
    region_growing.setInputCloud(cloud);
    region_growing.setSearchMethod(tree);
    region_growing.setDistanceThreshold(static_cast<float>(request->distance_threshold));
    region_growing.setPointColorThreshold(static_cast<float>(request->point_color_threshold));
    region_growing.setRegionColorThreshold(static_cast<float>(request->region_color_threshold));
    region_growing.setMinClusterSize(request->min_cluster_size);

    std::vector<pcl::PointIndices> clusters;
    region_growing.extract(clusters);
    num_clusters = clusters.size();

    // getColoredCloud() re-colors every point by its cluster (one solid color per cluster;
    // points too small/isolated to form a min_cluster_size-sized cluster are left black) -
    // this is PCL's own standard output for this algorithm, returned here in place of the
    // true camera colors, per the request's documented behavior.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored = region_growing.getColoredCloud();
    if (!colored) {
      response->success = false;
      response->message = "region growing RGB segmentation produced no output";
      return;
    }
    colored->header = cloud->header;
    cloud = colored;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud, cloud_msg);
  response->cloud = std::move(cloud_msg);
  response->success = true;
  response->message = request->segment ?
    ("returned " + std::to_string(cloud->size()) + " points in " +
    std::to_string(num_clusters) + " region-growing-RGB cluster(s)") :
    ("returned " + std::to_string(cloud->size()) + " points");
}

}  // namespace lerobot_perception
