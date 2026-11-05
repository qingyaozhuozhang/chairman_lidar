#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "custom_msg/msg/pose_euler.hpp" 
#include <cmath> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class OdometryTransformMath : public rclcpp::Node
{
public:
    OdometryTransformMath() : Node("odometry_transform_math")
    {
        
        
        this->declare_parameter("map_origin_x", -5.543);
        this->declare_parameter("map_origin_y", -5.998);
        this->declare_parameter("map_origin_z", 0.000); 
        this->declare_parameter("map_origin_qz", 0.000);
        this->declare_parameter("map_origin_qw", 1.000);

        this->declare_parameter("lidar_init_x", -4.397);
        this->declare_parameter("lidar_init_y", -3.917);
        this->declare_parameter("lidar_init_z", 0.000); 
        this->declare_parameter("lidar_init_qz", 0.000);
        this->declare_parameter("lidar_init_qw", 1.000);


        float mx = static_cast<float>(this->get_parameter("map_origin_x").as_double());
        float my = static_cast<float>(this->get_parameter("map_origin_y").as_double());
        float mz = static_cast<float>(this->get_parameter("map_origin_z").as_double());
        float mqz = static_cast<float>(this->get_parameter("map_origin_qz").as_double());
        float mqw = static_cast<float>(this->get_parameter("map_origin_qw").as_double());

        float lx = static_cast<float>(this->get_parameter("lidar_init_x").as_double());
        float ly = static_cast<float>(this->get_parameter("lidar_init_y").as_double());
        float lz = static_cast<float>(this->get_parameter("lidar_init_z").as_double());
        float lqz = static_cast<float>(this->get_parameter("lidar_init_qz").as_double());
        float lqw = static_cast<float>(this->get_parameter("lidar_init_qw").as_double());

        // 偏移量计算
        float map_yaw = 2.0f * std::atan2(mqz, mqw);
        float lidar_yaw = 2.0f * std::atan2(lqz, lqw);

        float dx = lx - mx;
        float dy = ly - my;
        
        offset_x_ = dx * std::cos(map_yaw) - dy * std::sin(map_yaw);
        offset_y_ = dx * std::sin(map_yaw) + dy * std::cos(map_yaw);
        offset_z_ = lz - mz; 
        offset_yaw_ = lidar_yaw - map_yaw;

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom_raw", 10, std::bind(&OdometryTransformMath::odom_callback, this, std::placeholders::_1));
        
        pose_pub_ = this->create_publisher<custom_msg::msg::PoseEuler>("odom_map", 10);
        init_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    }

private:
    void publish_init_pose(float x,float y,float z,float w){
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = 0.0;
        tf2::Quaternion q(0,0,z,w);
        msg.pose.pose.orientation = tf2::toMsg(q);
        init_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "发布初始位姿");
    };

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 1. 获取 odom_raw 原始局部坐标
        float rx = static_cast<float>(msg->pose.pose.position.x);
        float ry = static_cast<float>(msg->pose.pose.position.y);
        float rz = static_cast<float>(msg->pose.pose.position.z);

        float raw_qz = static_cast<float>(msg->pose.pose.orientation.z);
        float raw_qw = static_cast<float>(msg->pose.pose.orientation.w);
        float raw_yaw = 2.0f * std::atan2(raw_qz, raw_qw);

        // 2. 通过 TF 实时获取小车中心
        float map_x = 0.0f;
        float map_y = 0.0f;
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "map", "base_footprint", tf2::TimePointZero);
            map_x = static_cast<float>(transform.transform.translation.x);
            map_y = static_cast<float>(transform.transform.translation.y);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                "等待 TF (map -> base_footprint)... 原因: %s", ex.what());
            map_x = rx;
            map_y = ry;
        }

        // 4. 坐标系转换：计算【自定义坐标系】下的位置
        float final_x = offset_x_ + (rx * std::cos(offset_yaw_) - ry * std::sin(offset_yaw_));
        float final_y = offset_y_ + (rx * std::sin(offset_yaw_) + ry * std::cos(offset_yaw_));
        float final_yaw = offset_yaw_ + raw_yaw;
        float final_z = offset_z_ + rz; 

        custom_msg::msg::PoseEuler pure_pose;
        pure_pose.x = final_x;            
        pure_pose.y = final_y;            
        pure_pose.z = final_z;
        pure_pose.yaw = final_yaw;

        pose_pub_->publish(pure_pose);

        RCLCPP_INFO(this->get_logger(), 
            "X: %.3f m, Y: %.3f m, Z: %.3f m, Yaw: %.3f rad | (rx,ry): (%.3f, %.3f)",
            pure_pose.x, pure_pose.y, pure_pose.z, pure_pose.yaw,
            map_x, map_y);

        if (sign==0){
            ix = static_cast<float>(this->get_parameter("lidar_init_x").as_double());
            iy = static_cast<float>(this->get_parameter("lidar_init_y").as_double());
            iz = static_cast<float>(this->get_parameter("lidar_init_qz").as_double());
            iw = static_cast<float>(this->get_parameter("lidar_init_qw").as_double());
            if(std::abs(map_x-ix)>0.01 || std::abs(map_y-iy)>0.01){
                publish_init_pose(ix,iy,iz,iw);
            }else{
                sign =1;
            }
        }
    }

    float offset_x_, offset_y_, offset_z_, offset_yaw_,ix,iy,iz,iw;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<custom_msg::msg::PoseEuler>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pub_;
    
    // TF 监听相关
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    int sign = 0;

    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryTransformMath>());
    rclcpp::shutdown();
    return 0;
}