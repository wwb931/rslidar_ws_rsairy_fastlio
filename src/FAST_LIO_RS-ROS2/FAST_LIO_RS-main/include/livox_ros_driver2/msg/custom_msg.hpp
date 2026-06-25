#pragma once

#include "livox_ros_driver2/msg/custom_point.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include <std_msgs/msg/header.hpp>

namespace livox_ros_driver2
{
namespace msg
{

struct CustomMsg
{
  using SharedPtr = std::shared_ptr<CustomMsg>;
  using ConstSharedPtr = std::shared_ptr<const CustomMsg>;
  using UniquePtr = std::unique_ptr<CustomMsg>;

  using _header_type = std_msgs::msg::Header;
  using _timebase_type = std::uint64_t;
  using _point_num_type = std::uint32_t;
  using _lidar_id_type = std::uint8_t;
  using _rsvd_type = std::array<std::uint8_t, 3>;
  using _points_type = std::vector<CustomPoint>;

  _header_type header;
  _timebase_type timebase = 0;
  _point_num_type point_num = 0;
  _lidar_id_type lidar_id = 0;
  _rsvd_type rsvd{};
  _points_type points;
};

}  // namespace msg
}  // namespace livox_ros_driver2

