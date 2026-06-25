#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cstring>
#include <limits>
#include <unordered_map>

#include "lidar_align/loader.h"
#include "lidar_align/transform.h"

namespace lidar_align {

namespace {

template <typename T>
bool deserializeBagMessage(
    const std::shared_ptr<rosbag2_storage::SerializedBagMessage>& bag_message,
    T* msg) {
  rclcpp::Serialization<T> serializer;
  rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
  serializer.deserialize_message(&serialized_msg, msg);
  return true;
}

int64_t toMicroseconds(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<int64_t>(stamp.sec) * 1000000ll +
         static_cast<int64_t>(stamp.nanosec) / 1000ll;
}

template <typename T>
void declareAndGet(const rclcpp::Node::SharedPtr& node, const std::string& name,
                   T* value) {
  node->declare_parameter<T>(name, *value);
  node->get_parameter(name, *value);
}

std::unordered_map<std::string, std::string> topicTypeMap(
    const std::vector<rosbag2_storage::TopicMetadata>& topics) {
  std::unordered_map<std::string, std::string> topic_types;
  for (const auto& topic : topics) {
    topic_types[topic.name] = topic.type;
  }
  return topic_types;
}

}  // namespace

Loader::Loader(const Config& config) : config_(config) {}

Loader::Config Loader::getConfig(const rclcpp::Node::SharedPtr& node) {
  Loader::Config config;
  declareAndGet(node, "use_n_scans", &config.use_n_scans);
  return config;
}

void Loader::parsePointcloudMsg(const sensor_msgs::msg::PointCloud2& msg,
                                LoaderPointcloud* pointcloud) {
  bool has_timing = false;
  bool has_intensity = false;
  bool has_timestamp = false;
  for (const sensor_msgs::msg::PointField& field : msg.fields) {
    if (field.name == "time_offset_us") {
      has_timing = true;
    } else if (field.name == "timestamp" || field.name == "time" ||
               field.name == "t") {
      has_timestamp = true;
    } else if (field.name == "intensity") {
      has_intensity = true;
    }
  }

  if (has_timing) {
    pcl::fromROSMsg(msg, *pointcloud);
    return;
  } else if (has_intensity) {
    Pointcloud raw_pointcloud;
    pcl::fromROSMsg(msg, raw_pointcloud);

    int timestamp_offset = -1;
    uint8_t timestamp_type = 0;
    double frame_start = std::numeric_limits<double>::infinity();
    if (has_timestamp) {
      for (const auto& field : msg.fields) {
        if (field.name == "timestamp" || field.name == "time" ||
            field.name == "t") {
          timestamp_offset = field.offset;
          timestamp_type = field.datatype;
          break;
        }
      }
      if (timestamp_offset >= 0) {
        for (size_t i = 0; i < msg.width * msg.height; ++i) {
          const uint8_t* ptr = msg.data.data() + i * msg.point_step;
          double t = 0.0;
          if (timestamp_type == sensor_msgs::msg::PointField::FLOAT64) {
            std::memcpy(&t, ptr + timestamp_offset, sizeof(double));
          } else if (timestamp_type == sensor_msgs::msg::PointField::FLOAT32) {
            float tf = 0.0f;
            std::memcpy(&tf, ptr + timestamp_offset, sizeof(float));
            t = tf;
          }
          if (t > 0.0) {
            frame_start = std::min(frame_start, t);
          }
        }
      }
    }

    for (size_t i = 0; i < raw_pointcloud.size(); ++i) {
      const Point& raw_point = raw_pointcloud[i];
      PointAllFields point;
      point.x = raw_point.x;
      point.y = raw_point.y;
      point.z = raw_point.z;
      point.intensity = raw_point.intensity;
      point.time_offset_us = 0;

      if (timestamp_offset >= 0 && std::isfinite(frame_start)) {
        const uint8_t* ptr = msg.data.data() + i * msg.point_step;
        double t = frame_start;
        if (timestamp_type == sensor_msgs::msg::PointField::FLOAT64) {
          std::memcpy(&t, ptr + timestamp_offset, sizeof(double));
        } else if (timestamp_type == sensor_msgs::msg::PointField::FLOAT32) {
          float tf = 0.0f;
          std::memcpy(&tf, ptr + timestamp_offset, sizeof(float));
          t = tf;
        }
        point.time_offset_us = static_cast<int32_t>((t - frame_start) * 1e6);
      }

      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
        continue;
      }

      pointcloud->push_back(point);
    }
    pointcloud->header = raw_pointcloud.header;
  } else {
    pcl::PointCloud<pcl::PointXYZ> raw_pointcloud;
    pcl::fromROSMsg(msg, raw_pointcloud);

    for (const pcl::PointXYZ& raw_point : raw_pointcloud) {
      PointAllFields point;
      point.x = raw_point.x;
      point.y = raw_point.y;
      point.z = raw_point.z;

      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }

      pointcloud->push_back(point);
    }
    pointcloud->header = raw_pointcloud.header;
  }
}

bool Loader::loadPointcloudFromROSBag(const std::string& bag_path,
                                      const Scan::Config& scan_config,
                                      Lidar* lidar) {
  rosbag2_cpp::Reader reader;
  try {
    reader.open(bag_path);
  } catch (const std::exception& e) {
    std::cerr << "LOADING BAG FAILED: " << e.what() << std::endl;
    return false;
  }

  size_t scan_num = 0;
  const auto topic_types = topicTypeMap(reader.get_all_topics_and_types());
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    const auto type_iter = topic_types.find(bag_message->topic_name);
    if (type_iter == topic_types.end() ||
        type_iter->second != "sensor_msgs/msg/PointCloud2") {
      continue;
    }

    sensor_msgs::msg::PointCloud2 msg;
    try {
      deserializeBagMessage(bag_message, &msg);
    } catch (...) {
      continue;
    }

    std::cout << " Loading scan: \e[1m" << scan_num++ << "\e[0m from ros bag"
              << '\r' << std::flush;

    LoaderPointcloud pointcloud;
    parsePointcloudMsg(msg, &pointcloud);

    lidar->addPointcloud(pointcloud, scan_config);

    if (lidar->getNumberOfScans() >= config_.use_n_scans) {
      break;
    }
  }
  if (lidar->getTotalPoints() == 0) {
    std::cerr << "No points were loaded, verify that the bag contains "
                 "populated sensor_msgs/msg/PointCloud2 messages."
              << std::endl;
    return false;
  }

  return true;
}

bool Loader::loadTformFromROSBag(const std::string& bag_path, Odom* odom) {
  rosbag2_cpp::Reader reader;
  try {
    reader.open(bag_path);
  } catch (const std::exception& e) {
    std::cerr << "LOADING BAG FAILED: " << e.what() << std::endl;
    return false;
  }

  size_t imu_num = 0;
  size_t transform_num = 0;
  double shiftX = 0, shiftY = 0, shiftZ = 0, velX = 0, velY = 0, velZ = 0;
  rclcpp::Time last_time(0, 0, RCL_ROS_TIME);
  double timeDiff, lastShiftX, lastShiftY, lastShiftZ;

  const auto topic_types = topicTypeMap(reader.get_all_topics_and_types());
  while (reader.has_next()) {
    auto bag_message = reader.read_next();
    const auto type_iter = topic_types.find(bag_message->topic_name);
    if (type_iter == topic_types.end()) {
      continue;
    }

    if (type_iter->second == "geometry_msgs/msg/TransformStamped") {
      geometry_msgs::msg::TransformStamped transform_msg;
      deserializeBagMessage(bag_message, &transform_msg);
      Timestamp stamp = toMicroseconds(transform_msg.header.stamp);
      Transform T(Transform::Translation(transform_msg.transform.translation.x,
                                         transform_msg.transform.translation.y,
                                         transform_msg.transform.translation.z),
                  Transform::Rotation(transform_msg.transform.rotation.w,
                                      transform_msg.transform.rotation.x,
                                      transform_msg.transform.rotation.y,
                                      transform_msg.transform.rotation.z));
      odom->addTransformData(stamp, T);
      ++transform_num;
      continue;
    }

    if (type_iter->second != "sensor_msgs/msg/Imu") {
      continue;
    }

    sensor_msgs::msg::Imu imu;
    deserializeBagMessage(bag_message, &imu);

    std::cout << "Loading imu: \e[1m" << imu_num++ << "\e[0m from ros bag"
              << '\r' << std::flush;

    Timestamp stamp = toMicroseconds(imu.header.stamp);
    rclcpp::Time current_time(imu.header.stamp);
    if (imu_num == 1) {
      last_time = current_time;
      Transform T(Transform::Translation(0, 0, 0),
                  Transform::Rotation(1, 0, 0, 0));
      odom->addTransformData(stamp, T);
    } else {
      timeDiff = (current_time - last_time).seconds();
      last_time = current_time;
      velX = velX + imu.linear_acceleration.x * timeDiff;
      velY = velY + imu.linear_acceleration.y * timeDiff;
      velZ = velZ + (imu.linear_acceleration.z - 9.801) * timeDiff;

      lastShiftX = shiftX;
      lastShiftY = shiftY;
      lastShiftZ = shiftZ;
      shiftX =
          lastShiftX + velX * timeDiff +
          imu.linear_acceleration.x * timeDiff * timeDiff / 2;
      shiftY =
          lastShiftY + velY * timeDiff +
          imu.linear_acceleration.y * timeDiff * timeDiff / 2;
      shiftZ = lastShiftZ + velZ * timeDiff +
               (imu.linear_acceleration.z - 9.801) * timeDiff * timeDiff / 2;

      Transform T(Transform::Translation(shiftX, shiftY, shiftZ),
                  Transform::Rotation(imu.orientation.w, imu.orientation.x,
                                      imu.orientation.y, imu.orientation.z));
      odom->addTransformData(stamp, T);
    }
  }

  if (odom->empty()) {
    std::cerr << "No imu or transform messages found!" << std::endl;
    return false;
  }

  return true;
}

bool Loader::loadTformFromMaplabCSV(const std::string& csv_path, Odom* odom) {
  std::ifstream file(csv_path, std::ifstream::in);

  size_t tform_num = 0;
  while (file.peek() != EOF) {
    std::cout << " Loading transform: \e[1m" << tform_num++
              << "\e[0m from csv file" << '\r' << std::flush;

    Timestamp stamp;
    Transform T;

    if (getNextCSVTransform(file, &stamp, &T)) {
      odom->addTransformData(stamp, T);
    }
  }

  return true;
}

// lots of potential failure cases not checked
bool Loader::getNextCSVTransform(std::istream& str, Timestamp* stamp,
                                 Transform* T) {
  std::string line;
  std::getline(str, line);

  // ignore comment lines
  if (line[0] == '#') {
    return false;
  }

  std::stringstream line_stream(line);
  std::string cell;

  std::vector<std::string> data;
  while (std::getline(line_stream, cell, ',')) {
    data.push_back(cell);
  }

  if (data.size() < 9) {
    return false;
  }

  constexpr size_t TIME = 0;
  constexpr size_t X = 2;
  constexpr size_t Y = 3;
  constexpr size_t Z = 4;
  constexpr size_t RW = 5;
  constexpr size_t RX = 6;
  constexpr size_t RY = 7;
  constexpr size_t RZ = 8;

  *stamp = std::stoll(data[TIME]) / 1000ll;
  *T = Transform(Transform::Translation(std::stod(data[X]), std::stod(data[Y]),
                                        std::stod(data[Z])),
                 Transform::Rotation(std::stod(data[RW]), std::stod(data[RX]),
                                     std::stod(data[RY]), std::stod(data[RZ])));

  return true;
}

}  // namespace lidar_align
