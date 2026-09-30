#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "custom_msg/msg/pose_euler.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cmath>
#include <vector>
#include <string>
#include <functional>


struct Region {
    int id;
    std::string name;
    float x_min;
    float x_max;
    float y_min;
    float y_max;
};


class OdometryTransformMath : public rclcpp::Node
{
public:
    OdometryTransformMath() : Node("odometry_transform_math")
    {
        // ================= 参数声明 =================
        this->declare_parameter("selected_pose", 1);

        this->declare_parameter("default_inflation_radius", 0.5);

        // 初始位姿参数（用于 /initialpose 发布）
        this->declare_parameter("lidar_init_x", -5.106);
        this->declare_parameter("lidar_init_y", -1.4115);
        this->declare_parameter("lidar_init_z", 0.000);
        this->declare_parameter("lidar_init_qz", -0.707);
        this->declare_parameter("lidar_init_qw", 0.707);

        // ================= 启动阶段 TF 稳定检测参数 =================
        this->declare_parameter("required_tf_stable_count", 20);
        this->declare_parameter("tf_jump_threshold", 0.01);

        // TF 第一次稳定后，强制发布几次 /initialpose
        this->declare_parameter("initialpose_publish_after_stable_count", 1);

        // ================= 读取参数 =================
        selected_pose_ = static_cast<int>(this->get_parameter("selected_pose").as_int());

        default_inflation_radius_ =
            this->get_parameter("default_inflation_radius").as_double();

        lidar_init_x_ =
            static_cast<float>(this->get_parameter("lidar_init_x").as_double());
        lidar_init_y_ =
            static_cast<float>(this->get_parameter("lidar_init_y").as_double());
        lidar_init_z_ =
            static_cast<float>(this->get_parameter("lidar_init_z").as_double());
        lidar_init_qz_ =
            static_cast<float>(this->get_parameter("lidar_init_qz").as_double());
        lidar_init_qw_ =
            static_cast<float>(this->get_parameter("lidar_init_qw").as_double());

        required_tf_stable_count_ =
            static_cast<int>(this->get_parameter("required_tf_stable_count").as_int());

        tf_jump_threshold_ =
            this->get_parameter("tf_jump_threshold").as_double();

        initialpose_publish_after_stable_count_ =
            static_cast<int>(
                this->get_parameter("initialpose_publish_after_stable_count").as_int()
            );

        if (required_tf_stable_count_ < 1) {
            required_tf_stable_count_ = 1;
        }

        if (tf_jump_threshold_ <= 0.0) {
            tf_jump_threshold_ = 0.01;
        }

        if (initialpose_publish_after_stable_count_ < 0) {
            initialpose_publish_after_stable_count_ = 0;
        }

        // ================= 加载 YAML 区域配置 =================
        try {
            std::string pkg_share =
                ament_index_cpp::get_package_share_directory("odometry");

            std::string yaml_path = pkg_share + "/config/regions.yaml";

            YAML::Node config = YAML::LoadFile(yaml_path);

            std::string region_key =
                (selected_pose_ == 1 || selected_pose_ == 2) ? "regions_red" : "regions_blue";

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

                RCLCPP_INFO(
                    this->get_logger(),
                    "成功加载了 %zu 个检测区域 (当前使用方案: %s)",
                    regions_.size(),
                    region_key.c_str()
                );
            } else {
                RCLCPP_WARN(
                    this->get_logger(),
                    "在 regions.yaml 中找不到 '%s' 键！请检查文件格式。",
                    region_key.c_str()
                );
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(
                this->get_logger(),
                "无法加载 regions.yaml! 报错: %s",
                e.what()
            );
        }

        // ================= 初始化 TF =================
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ================= 参数客户端 =================
        local_costmap_param_client_ =
            std::make_shared<rclcpp::AsyncParametersClient>(
                this,
                "/local_costmap/local_costmap"
            );

        global_costmap_param_client_ =
            std::make_shared<rclcpp::AsyncParametersClient>(
                this,
                "/global_costmap/global_costmap"
            );

        // ================= 订阅发布 =================
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&OdometryTransformMath::odom_callback, this, std::placeholders::_1)
        );

        pose_pub_ =
            this->create_publisher<custom_msg::msg::PoseEuler>("odom_map", 10);

        init_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/initialpose",
                rclcpp::QoS(10).reliable().transient_local()
            );

        RCLCPP_INFO(
            this->get_logger(),
            "odometry_transform_math 启动完成：目标初始 map=(%.4f, %.4f), qz=%.4f, qw=%.4f",
            lidar_init_x_,
            lidar_init_y_,
            lidar_init_qz_,
            lidar_init_qw_
        );

        RCLCPP_INFO(
            this->get_logger(),
            "启动保护参数：required_tf_stable_count=%d, tf_jump_threshold=%.4f m, initialpose_publish_after_stable_count=%d",
            required_tf_stable_count_,
            tf_jump_threshold_,
            initialpose_publish_after_stable_count_
        );
    }

private:
    void reset_startup_tf_stability()
    {
        has_last_startup_tf_ = false;
        startup_tf_stable_count_ = 0;
        last_startup_tf_x_ = 0.0f;
        last_startup_tf_y_ = 0.0f;
    }

    bool lookup_map_base_tf(
        float& map_x,
        float& map_y,
        float& map_z,
        float& map_yaw)
    {
        try {
            geometry_msgs::msg::TransformStamped transform =
                tf_buffer_->lookupTransform(
                    "map",
                    "base_footprint",
                    tf2::TimePointZero
                );

            map_x = static_cast<float>(transform.transform.translation.x);
            map_y = static_cast<float>(transform.transform.translation.y);
            map_z = static_cast<float>(transform.transform.translation.z);
            map_yaw = static_cast<float>(tf2::getYaw(transform.transform.rotation));

            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "读取 TF (map -> base_footprint) 失败，本帧丢弃。原因: %s",
                ex.what()
            );

            return false;
        }
    }

    bool wait_startup_tf_stable_and_get(
        float& map_x,
        float& map_y,
        float& map_z,
        float& map_yaw)
    {
        bool ok = lookup_map_base_tf(map_x, map_y, map_z, map_yaw);

        if (!ok) {
            reset_startup_tf_stability();
            return false;
        }

        if (!has_last_startup_tf_) {
            last_startup_tf_x_ = map_x;
            last_startup_tf_y_ = map_y;
            has_last_startup_tf_ = true;
            startup_tf_stable_count_ = 0;

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "启动初始化：读取到第一帧 TF，map=(%.4f, %.4f)，开始等待连续稳定...",
                map_x,
                map_y
            );

            return false;
        }

        double jump = std::hypot(
            static_cast<double>(map_x - last_startup_tf_x_),
            static_cast<double>(map_y - last_startup_tf_y_)
        );

        last_startup_tf_x_ = map_x;
        last_startup_tf_y_ = map_y;

        if (jump <= tf_jump_threshold_) {
            startup_tf_stable_count_++;
        } else {
            startup_tf_stable_count_ = 0;

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "启动初始化：TF 仍在跳变，暂不发布 /initialpose。jump=%.4f m, 当前 map=(%.4f, %.4f)",
                jump,
                map_x,
                map_y
            );

            return false;
        }

        if (startup_tf_stable_count_ < required_tf_stable_count_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,
                "启动初始化：等待 TF 连续稳定：%d/%d，jump=%.4f m，map=(%.4f, %.4f)",
                startup_tf_stable_count_,
                required_tf_stable_count_,
                jump,
                map_x,
                map_y
            );

            return false;
        }

        if (!startup_tf_stable_done_) {
            startup_tf_stable_done_ = true;

            RCLCPP_INFO(
                this->get_logger(),
                "✅ 启动初始化：TF 已连续稳定，准备发布 /initialpose。当前 map=(%.4f, %.4f), yaw=%.4f",
                map_x,
                map_y,
                map_yaw
            );
        }

        return true;
    }

    void publish_init_pose()
    {
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();

        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";

        msg.pose.pose.position.x = lidar_init_x_;
        msg.pose.pose.position.y = lidar_init_y_;
        msg.pose.pose.position.z = 0.0;

        tf2::Quaternion q(0.0, 0.0, lidar_init_qz_, lidar_init_qw_);
        q.normalize();

        msg.pose.pose.orientation = tf2::toMsg(q);

        msg.pose.covariance[0] = 0.05 * 0.05;
        msg.pose.covariance[7] = 0.05 * 0.05;
        msg.pose.covariance[35] = 0.05 * 0.05;

        init_pub_->publish(msg);

        initialpose_publish_count_++;

        RCLCPP_WARN(
            this->get_logger(),
            "发布 /initialpose #%d/%d：map=(%.4f, %.4f), qz=%.4f, qw=%.4f",
            initialpose_publish_count_,
            initialpose_publish_after_stable_count_,
            lidar_init_x_,
            lidar_init_y_,
            lidar_init_qz_,
            lidar_init_qw_
        );
    }

    bool handle_startup_initialpose_stage(
        float map_x,
        float map_y)
    {
        if (initialpose_stage_done_) {
            return true;
        }

        double current_error = std::hypot(
            static_cast<double>(map_x - lidar_init_x_),
            static_cast<double>(map_y - lidar_init_y_)
        );

        if (initialpose_publish_count_ < initialpose_publish_after_stable_count_) {
            RCLCPP_WARN(
                this->get_logger(),
                "启动初始化：TF 已稳定，强制发布 /initialpose。当前 map=(%.4f, %.4f)，目标初始 map=(%.4f, %.4f)，当前误差=%.4f m",
                map_x,
                map_y,
                lidar_init_x_,
                lidar_init_y_,
                current_error
            );

            publish_init_pose();

            if (initialpose_publish_count_ >= initialpose_publish_after_stable_count_) {
                initialpose_stage_done_ = true;

                RCLCPP_WARN(
                    this->get_logger(),
                    "✅ 启动初始化完成：已发布 /initialpose，后续进入正常里程计输出阶段。"
                );
            }

            return false;
        }

        initialpose_stage_done_ = true;

        RCLCPP_WARN(
            this->get_logger(),
            "启动初始化完成：initialpose_publish_after_stable_count=0，没有发布 /initialpose，直接进入正常输出。当前误差=%.4f m",
            current_error
        );

        return true;
    }

    void set_inflation_radius(double radius)
    {
        RCLCPP_WARN(
            this->get_logger(),
            "🚧 调整代价地图膨胀半径为: %.2f m",
            radius
        );

        if (local_costmap_param_client_->service_is_ready()) {
            local_costmap_param_client_->set_parameters({
                rclcpp::Parameter("inflation_layer.inflation_radius", radius)
            });
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "local_costmap 参数服务未就绪，动态调整失效！"
            );
        }

        if (global_costmap_param_client_->service_is_ready()) {
            global_costmap_param_client_->set_parameters({
                rclcpp::Parameter("inflation_layer.inflation_radius", radius)
            });
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "global_costmap 参数服务未就绪，动态调整失效！"
            );
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 读取 TF 获得 map 坐标系下的位姿
        float map_x = 0.0f;
        float map_y = 0.0f;
        float map_z = 0.0f;
        float map_yaw = 0.0f;

        // 启动阶段：只在这里等待 TF 连续稳定
        if (!startup_tf_stable_done_) {
            bool startup_tf_ready =
                wait_startup_tf_stable_and_get(map_x, map_y, map_z, map_yaw);

            if (!startup_tf_ready) {
                return;
            }
        } else {
            // 正常运行阶段：只读取 TF，不再判断 jump
            bool tf_ok = lookup_map_base_tf(map_x, map_y, map_z, map_yaw);

            if (!tf_ok) {
                return;
            }
        }

        // 启动阶段：TF 第一次稳定后，发布 /initialpose
        if (!initialpose_stage_done_) {
            bool can_continue =
                handle_startup_initialpose_stage(map_x, map_y);

            if (!can_continue) {
                return;
            }
        }

        // 使用真实 map 坐标判断区域
        int current_region_id = -1;
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
        bool current_in_merlin = false;

        if (current_region_id >= 1 && current_region_id <= 13) {
            current_in_merlin = true;
        }

        if (current_in_merlin != last_in_merlin_) {
            if (current_in_merlin) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "🌟 小车进入梅林区域 [ID: %d, %s]，设置膨胀半径为 0.0",
                    current_region_id,
                    current_region_name.c_str()
                );

                set_inflation_radius(0.0);
            } else {
                RCLCPP_INFO(
                    this->get_logger(),
                    "🌟 小车离开梅林区域，当前进入 [ID: %d, %s]，恢复膨胀半径为 %.2f",
                    current_region_id,
                    current_region_name.c_str(),
                    default_inflation_radius_
                );

                set_inflation_radius(default_inflation_radius_);
            }

            last_in_merlin_ = current_in_merlin;
        }

        // 发布 /odom_map（使用真实 map 坐标）
        custom_msg::msg::PoseEuler pure_pose;
        pure_pose.x = map_x;
        pure_pose.y = map_y;
        pure_pose.z = map_z;
        pure_pose.yaw = map_yaw;

        pose_pub_->publish(pure_pose);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            300,
            "X: %.3f m, Y: %.3f m, Z: %.3f m, Yaw: %.3f rad | ID: %d (%s)",
            pure_pose.x,
            pure_pose.y,
            pure_pose.z,
            pure_pose.yaw,
            current_region_id,
            current_region_name.c_str()
        );
    }

private:
    // ================= 基础参数 =================
    int selected_pose_ = 1;

    float lidar_init_x_ = 0.0f;
    float lidar_init_y_ = 0.0f;
    float lidar_init_z_ = 0.0f;
    float lidar_init_qz_ = 0.0f;
    float lidar_init_qw_ = 1.0f;

    std::vector<Region> regions_;

    // ================= ROS 通信 =================
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<custom_msg::msg::PoseEuler>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pub_;

    // ================= TF =================
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ================= 启动阶段 TF 稳定检测 =================
    bool startup_tf_stable_done_ = false;
    bool has_last_startup_tf_ = false;

    float last_startup_tf_x_ = 0.0f;
    float last_startup_tf_y_ = 0.0f;

    int startup_tf_stable_count_ = 0;
    int required_tf_stable_count_ = 20;

    double tf_jump_threshold_ = 0.01;

    // ================= initialpose 阶段 =================
    bool initialpose_stage_done_ = false;

    int initialpose_publish_count_ = 0;
    int initialpose_publish_after_stable_count_ = 1;

    // ================= 动态参数客户端 =================
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