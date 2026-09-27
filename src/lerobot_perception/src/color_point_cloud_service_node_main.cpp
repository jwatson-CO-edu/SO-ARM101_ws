#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lerobot_perception/color_point_cloud_service_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<lerobot_perception::ColorPointCloudServiceNode>(options);

  // MultiThreadedExecutor, not rclcpp::spin(): the service's Reentrant callback group (see
  // the node's constructor) only lets concurrent get_color_point_cloud requests actually run
  // in parallel if something is spinning more than one thread to run them on.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
