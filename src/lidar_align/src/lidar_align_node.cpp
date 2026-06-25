#include <rclcpp/rclcpp.hpp>

#include <algorithm>

#include "lidar_align/aligner.h"
#include "lidar_align/loader.h"
#include "lidar_align/sensors.h"

using namespace lidar_align;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("lidar_align");

  Loader loader(Loader::getConfig(node));

  Lidar lidar;
  Odom odom;

  std::string input_bag_path;
  RCLCPP_INFO(node->get_logger(), "Loading Pointcloud Data...");
  node->declare_parameter<std::string>("input_bag_path", "");
  node->get_parameter("input_bag_path", input_bag_path);
  if (input_bag_path.empty()) {
    RCLCPP_FATAL(node->get_logger(),
                 "Could not find input_bag_path parameter, exiting");
    exit(EXIT_FAILURE);
  } else if (!loader.loadPointcloudFromROSBag(
                 input_bag_path, Scan::getConfig(node), &lidar)) {
    RCLCPP_FATAL(node->get_logger(), "Error loading pointclouds from ROS bag.");
    exit(0);
  }

  bool transforms_from_csv;
  node->declare_parameter<bool>("transforms_from_csv", false);
  node->get_parameter("transforms_from_csv", transforms_from_csv);
  std::string input_csv_path;
  RCLCPP_INFO(node->get_logger(), "Loading Transformation Data...");
  if (transforms_from_csv) {
    node->declare_parameter<std::string>("input_csv_path", "");
    node->get_parameter("input_csv_path", input_csv_path);
    if (input_csv_path.empty()) {
      RCLCPP_FATAL(node->get_logger(),
                   "Could not find input_csv_path parameter, exiting");
      exit(EXIT_FAILURE);
    } else if (!loader.loadTformFromMaplabCSV(input_csv_path, &odom)) {
      RCLCPP_FATAL(node->get_logger(), "Error loading transforms from CSV.");
      exit(0);
    }
  } else if (!loader.loadTformFromROSBag(input_bag_path, &odom)) {
    RCLCPP_FATAL(node->get_logger(), "Error loading transforms from ROS bag.");
    exit(0);
  }

  if (lidar.getNumberOfScans() == 0) {
    RCLCPP_FATAL(node->get_logger(), "No data loaded, exiting");
    exit(0);
  }

  RCLCPP_INFO(node->get_logger(), "Interpolating Transformation Data...");
  lidar.setOdomOdomTransforms(odom);

  Aligner aligner(Aligner::getConfig(node));

  aligner.lidarOdomTransform(&lidar, &odom);

  rclcpp::shutdown();
  return 0;
}
