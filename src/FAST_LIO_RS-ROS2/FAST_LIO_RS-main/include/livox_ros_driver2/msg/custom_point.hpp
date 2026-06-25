#pragma once

#include <cstdint>
#include <memory>

namespace livox_ros_driver2
{
namespace msg
{

struct CustomPoint
{
  using SharedPtr = std::shared_ptr<CustomPoint>;
  using ConstSharedPtr = std::shared_ptr<const CustomPoint>;
  using UniquePtr = std::unique_ptr<CustomPoint>;

  using _offset_time_type = std::uint32_t;
  using _x_type = float;
  using _y_type = float;
  using _z_type = float;
  using _reflectivity_type = std::uint8_t;
  using _tag_type = std::uint8_t;
  using _line_type = std::uint8_t;

  _offset_time_type offset_time = 0;
  _x_type x = 0.0f;
  _y_type y = 0.0f;
  _z_type z = 0.0f;
  _reflectivity_type reflectivity = 0;
  _tag_type tag = 0;
  _line_type line = 0;
};

}  // namespace msg
}  // namespace livox_ros_driver2
