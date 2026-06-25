#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

class CameraInitVisualTfNode : public rclcpp::Node
{
public:
  CameraInitVisualTfNode()
  : Node("camera_init_visual_tf")
  {
    parent_frame_ = this->declare_parameter<std::string>("parent_frame", "camera_init_lidar");
    child_frame_ = this->declare_parameter<std::string>("child_frame", "camera_init");

    const double roll = this->declare_parameter<double>("roll", 3.141592653589793);
    const double pitch = this->declare_parameter<double>("pitch", 0.0);
    const double yaw = this->declare_parameter<double>("yaw", -1.570796326794897);

    const double x = this->declare_parameter<double>("x", 0.0);
    const double y = this->declare_parameter<double>("y", 0.0);
    const double z = this->declare_parameter<double>("z", 0.0);

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    transform_.header.frame_id = parent_frame_;
    transform_.child_frame_id = child_frame_;
    transform_.transform.translation.x = x;
    transform_.transform.translation.y = y;
    transform_.transform.translation.z = z;
    transform_.transform.rotation.x = q.x();
    transform_.transform.rotation.y = q.y();
    transform_.transform.rotation.z = q.z();
    transform_.transform.rotation.w = q.w();

    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publishTransform();

    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&CameraInitVisualTfNode::publishTransform, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing visual TF %s -> %s, rpy=[%.6f, %.6f, %.6f]",
      parent_frame_.c_str(), child_frame_.c_str(), roll, pitch, yaw);
  }

private:
  void publishTransform()
  {
    transform_.header.stamp = this->now();
    broadcaster_->sendTransform(transform_);
  }

  std::string parent_frame_;
  std::string child_frame_;
  geometry_msgs::msg::TransformStamped transform_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInitVisualTfNode>());
  rclcpp::shutdown();
  return 0;
}
