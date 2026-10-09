// TF 转发：源变换就绪前发布初始位姿，之后持续发布最近一次源变换。
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

class TfRelayNode : public rclcpp::Node
{
public:
  TfRelayNode() : Node("tf_relay_node")
  {
    source_parent_ = declare_parameter<std::string>("source_parent", "map1");
    source_child_ = declare_parameter<std::string>("source_child", "odom1");
    target_parent_ = declare_parameter<std::string>("target_parent", "map");
    target_child_ = declare_parameter<std::string>("target_child", "odom");
    const double rate = declare_parameter<double>("publish_rate_hz", 20.0);
    time_offset_ = declare_parameter<double>("time_offset_sec", 0.1);
    auto pose =
      declare_parameter<std::vector<double>>("init_pose", {-5.08, -1.5, 0.0, 0.0, 0.0, -1.5708});

    if (!std::isfinite(rate) || rate <= 0.0 || rate > 1000.0) {
      throw std::invalid_argument("publish_rate_hz must be in (0, 1000]");
    }
    if (!std::isfinite(time_offset_)) {
      throw std::invalid_argument("time_offset_sec must be finite");
    }
    if (
      source_parent_.empty() || source_child_.empty() || target_parent_.empty() ||
      target_child_.empty() || source_parent_ == source_child_ || target_parent_ == target_child_) {
      throw std::invalid_argument(
        "TF frames must be nonempty and each parent must differ from child");
    }
    if (pose.size() != 6) {
      RCLCPP_WARN(get_logger(), "init_pose requires [x, y, z, roll, pitch, yaw]; using identity");
      pose.assign(6, 0.0);
    }
    if (!std::all_of(pose.begin(), pose.end(), [](double value) { return std::isfinite(value); })) {
      throw std::invalid_argument("init_pose values must be finite");
    }

    latest_.translation.x = pose[0];
    latest_.translation.y = pose[1];
    latest_.translation.z = pose[2];
    tf2::Quaternion rotation;
    rotation.setRPY(pose[3], pose[4], pose[5]);
    latest_.rotation.x = rotation.x();
    latest_.rotation.y = rotation.y();
    latest_.rotation.z = rotation.z();
    latest_.rotation.w = rotation.w();

    buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, this, false);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    // 两个回调由单线程执行器依次运行；查询不阻塞，缓存无需额外互斥锁。
    lookup_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(0.02), [this]() { lookupSource(); });
    publish_timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate),
      [this]() { publishTransform(); });
    RCLCPP_INFO(
      get_logger(),
      "TF relay: %s -> %s ==> %s -> %s @ %.1f Hz; using init_pose until source is ready",
      source_parent_.c_str(), source_child_.c_str(), target_parent_.c_str(), target_child_.c_str(),
      rate);
  }

private:
  void lookupSource()
  {
    try {
      latest_ =
        buffer_->lookupTransform(source_parent_, source_child_, tf2::TimePointZero).transform;
      if (!received_source_) {
        received_source_ = true;
        RCLCPP_INFO(get_logger(), "Source TF received, switching to live transform");
      }
    } catch (const tf2::TransformException &) {
      // 首次收到源 TF 之前保留初始变换，之后保留最近一次有效源变换。
    }
  }

  void publishTransform()
  {
    geometry_msgs::msg::TransformStamped output;
    output.header.stamp = now() + rclcpp::Duration::from_seconds(time_offset_);
    output.header.frame_id = target_parent_;
    output.child_frame_id = target_child_;
    output.transform = latest_;
    broadcaster_->sendTransform(output);
  }

  std::string source_parent_, source_child_, target_parent_, target_child_;
  double time_offset_;
  bool received_source_{false};
  geometry_msgs::msg::Transform latest_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr lookup_timer_, publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = EXIT_SUCCESS;
  try {
    rclcpp::spin(std::make_shared<TfRelayNode>());
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(rclcpp::get_logger("tf_relay_node"), "%s", exception.what());
    result = EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return result;
}
