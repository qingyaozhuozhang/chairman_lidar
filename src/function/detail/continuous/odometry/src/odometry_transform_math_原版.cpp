#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "custom_msg/msg/pose_euler.hpp" 
#include <cmath> 
#include <vector>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

struct Region {
    int id;
    std::string name;
    float x_min, x_max, y_min, y_max;
};

class OdometryTransformMath : public rclcpp::Node
{
public:
    OdometryTransformMath() : Node("odometry_transform_math")
    {
        this->declare_parameter("selected_pose", 1);
        
        this->declare_parameter("default_inflation_radius", 0.5); 
        
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

        int selected_pose = static_cast<int>(this->get_parameter("selected_pose").as_int());
        default_inflation_radius_ = this->get_parameter("default_inflation_radius").as_double();

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

        // 加载 YAML 区域配置
        try {
            std::string pkg_share = ament_index_cpp::get_package_share_directory("odometry");
            std::string yaml_path = pkg_share + "/config/regions.yaml";
            
            YAML::Node config = YAML::LoadFile(yaml_path);
            std::string region_key = (selected_pose == 1 || selected_pose == 2) ? "regions_red" : "regions_blue";
            
            if (config[region_key]) {
                for (const auto& node : config[region_key]) {
                    Region r;
                    r.id = node["id"].as<int>();
                    r.name = node["name"].as<std::string>();
                    r.x_min = node["x_min"].as<float>();
                    r.x_max = node["x_max"].as<float>();
                    r.y_min = node["y_min"].as<float>();
                    r.y_max = node["y_max"].as<float>();
                    regions_.push_back(r);
                }
                RCLCPP_INFO(this->get_logger(), "成功加载了 %zu 个检测区域 (当前使用方案: %s)", regions_.size(), region_key.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "在 regions.yaml 中找不到 '%s' 键！请检查文件格式。", region_key.c_str());
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "无法加载 regions.yaml! 报错: %s", e.what());
        }

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        local_costmap_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/local_costmap/local_costmap");
        global_costmap_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/global_costmap/global_costmap");

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

    // 封装动态修改膨胀半径的方法
    void set_inflation_radius(double radius) {
        RCLCPP_WARN(this->get_logger(), "🚧 动态调整代价地图膨胀半径为: %.2f m", radius);

        // 修改局部代价地图
        if (local_costmap_param_client_->service_is_ready()) {
            local_costmap_param_client_->set_parameters({
                rclcpp::Parameter("inflation_layer.inflation_radius", radius)
            });
        } else {
            RCLCPP_WARN(this->get_logger(), "local_costmap 参数服务未就绪，动态调整失效！");
        }

        // 修改全局代价地图
        if (global_costmap_param_client_->service_is_ready()) {
            global_costmap_param_client_->set_parameters({
                rclcpp::Parameter("inflation_layer.inflation_radius", radius)
            });
        } else {
            RCLCPP_WARN(this->get_logger(), "global_costmap 参数服务未就绪，动态调整失效！");
        }
    }

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

        // 3. 使用栅格地图坐标判断区域
        int current_region_id = -1; // 默认 -1 表示不在任何区域
        std::string current_region_name = "未知区域";

        for (const auto& r : regions_) {
            if (map_x >= r.x_min && map_x <= r.x_max && 
                map_y >= r.y_min && map_y <= r.y_max) {
                current_region_id = r.id;
                current_region_name = r.name;
                break; 
            }
        }

        // 基于 ID 的跨区域状态检测与参数推送
        // 判断当前区域是否属于“零膨胀区域”（ID 从 1 到 13）
        bool current_in_merlin = false;
        if (current_region_id >= 1 && current_region_id <= 13) {
            current_in_merlin = true;
        }

        // 仅在零膨胀区与普通区域之间切换时更新代价地图参数。
        if (current_in_merlin != last_in_merlin_) {
            if (current_in_merlin) { 
                RCLCPP_INFO(this->get_logger(), "🌟 小车进入梅林区域 [ID: %d, %s]，设置膨胀半径为 0.0", current_region_id, current_region_name.c_str());
                set_inflation_radius(0.0); 
            } 
            else { 
                RCLCPP_INFO(this->get_logger(), "🌟 小车离开梅林区域 (当前进入 [ID: %d, %s])，恢复膨胀半径为 %.2f...", current_region_id, current_region_name.c_str(), default_inflation_radius_);
                set_inflation_radius(default_inflation_radius_); 
            }
            last_in_merlin_ = current_in_merlin; 
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
            "X: %.3f m, Y: %.3f m, Z: %.3f m, Yaw: %.3f rad | ID: %d (%s) | (rx,ry): (%.3f, %.3f)", 
            pure_pose.x, pure_pose.y, pure_pose.z, pure_pose.yaw,
            current_region_id, current_region_name.c_str(),
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
    std::vector<Region> regions_; 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<custom_msg::msg::PoseEuler>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pub_;
    
    // TF 监听相关
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    int sign = 0;

    // 动态参数相关
    rclcpp::AsyncParametersClient::SharedPtr local_costmap_param_client_;
    rclcpp::AsyncParametersClient::SharedPtr global_costmap_param_client_;
    
    bool last_in_merlin_ = false; 
    double default_inflation_radius_ = 0.5;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryTransformMath>());
    rclcpp::shutdown();
    return 0;
}