#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2/time.h"
class Pc2Transformer : public rclcpp::Node
{
public:
  Pc2Transformer()
  : Node("pc2_transformer_node")
  {
    this->declare_parameter<std::string>("target_frame", "odom");
    this->declare_parameter<std::string>("input_topic", "/cloud_registered_body");
    this->declare_parameter<std::string>("output_topic", "/livox/lidar/pointcloud_odom");

    std::string target_frame = this->get_parameter("target_frame").as_string();
    std::string input_topic = this->get_parameter("input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 5);
    subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, 10,
      std::bind(&Pc2Transformer::cloud_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "点云坐标系转换节点已启动!");
    RCLCPP_INFO(this->get_logger(), "监听话题: %s", input_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "目标坐标系: %s", target_frame.c_str());
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::string target_frame = this->get_parameter("target_frame").as_string();
    sensor_msgs::msg::PointCloud2 transformed_cloud;

    try {
      // 使用最新可用 TF 转换整帧点云；此处不执行逐点运动畸变补偿。
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
     target_frame,
      msg->header.frame_id,
      tf2::TimePointZero,
      tf2::durationFromSec(0.5));
      tf2::doTransform(*msg, transformed_cloud, transform);
      
      transformed_cloud.header.frame_id = target_frame;
      
      publisher_->publish(transformed_cloud);

    } catch (const tf2::TransformException & ex) {
      // TF 不可用时丢弃本帧，告警间隔为 2 秒。
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, 
        "坐标系转换失败 (这在刚启动时很正常): %s", ex.what());
      return;
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pc2Transformer>());
  rclcpp::shutdown();
  return 0;
}