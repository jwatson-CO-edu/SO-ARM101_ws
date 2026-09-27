#ifndef LEROBOT_PERCEPTION__COLOR_POINT_CLOUD_SERVICE_NODE_HPP_
#define LEROBOT_PERCEPTION__COLOR_POINT_CLOUD_SERVICE_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "lerobot_perception_interfaces/srv/get_color_point_cloud.hpp"

namespace lerobot_perception
{

/// Service node: on request, builds an XYZRGB point cloud from the OAK-D Lite's latest
/// synchronized (color image, depth image, camera_info) frame, in the camera's own optical
/// frame, optionally voxel-downsampled and/or PCL Region-Growing-RGB segmented.
///
/// Design note: frameCallback() only caches the latest synchronized message triplet (an O(1)
/// pointer copy under a short-held lock) - it does no image or point-cloud work. All of the
/// actual per-pixel and PCL processing happens inside handleRequest(), and only then, so
/// there is no continuous background CPU cost between requests.
///
/// Parallelized:
///  - buildCloud()'s depth->cloud pixel loop, across rows, via OpenMP (see the .cpp file).
///  - Concurrent handleRequest() calls, via a Reentrant callback group (see the .cpp file's
///    constructor) - requires main() to spin with a MultiThreadedExecutor, not
///    rclcpp::spin().
/// Not parallelized (identified, not implemented): the independent rgb/depth cv_bridge
/// conversions in buildCloud(); pcl::VoxelGrid's point-to-voxel-index assignment;
/// pcl::RegionGrowingRGB's region-growing walk (also the least parallelizable of the four,
/// since each seed's growth order affects the result).
class ColorPointCloudServiceNode : public rclcpp::Node
{
public:
  explicit ColorPointCloudServiceNode(const rclcpp::NodeOptions & options);

private:
  using Image = sensor_msgs::msg::Image;
  using CameraInfo = sensor_msgs::msg::CameraInfo;
  using GetColorPointCloud = lerobot_perception_interfaces::srv::GetColorPointCloud;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<Image, Image, CameraInfo>;

  /// message_filters callback: cache the latest synchronized frame, nothing else.
  void frameCallback(
    const Image::ConstSharedPtr & rgb_msg,
    const Image::ConstSharedPtr & depth_msg,
    const CameraInfo::ConstSharedPtr & info_msg);

  /// Service callback: convert the cached frame to a cloud, then optionally downsample and
  /// segment it per the request, and fill in the response.
  void handleRequest(
    const std::shared_ptr<GetColorPointCloud::Request> request,
    std::shared_ptr<GetColorPointCloud::Response> response);

  /// Builds an unorganized, dense XYZRGB cloud in rgb_msg's optical frame from one
  /// synchronized frame. Pixels with no valid depth are dropped rather than kept as NaN.
  /// Returns nullptr and fills `error` on any failure (mismatched image sizes, unsupported
  /// depth encoding, degenerate camera intrinsics, ...).
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr buildCloud(
    const Image::ConstSharedPtr & rgb_msg,
    const Image::ConstSharedPtr & depth_msg,
    const CameraInfo::ConstSharedPtr & info_msg,
    std::string & error) const;

  message_filters::Subscriber<Image> rgb_sub_;
  message_filters::Subscriber<Image> depth_sub_;
  message_filters::Subscriber<CameraInfo> info_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Reentrant so a MultiThreadedExecutor (see main.cpp) can run multiple handleRequest()
  // calls concurrently instead of queuing them one at a time.
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Service<GetColorPointCloud>::SharedPtr service_;

  // Guards the three latest_* pointers below: frameCallback() (subscriber thread) writes
  // them, handleRequest() (service thread) reads them. Everything else each does with its
  // own copy is lock-free.
  mutable std::mutex frame_mutex_;
  Image::ConstSharedPtr latest_rgb_;
  Image::ConstSharedPtr latest_depth_;
  CameraInfo::ConstSharedPtr latest_info_;
};

}  // namespace lerobot_perception

#endif  // LEROBOT_PERCEPTION__COLOR_POINT_CLOUD_SERVICE_NODE_HPP_
