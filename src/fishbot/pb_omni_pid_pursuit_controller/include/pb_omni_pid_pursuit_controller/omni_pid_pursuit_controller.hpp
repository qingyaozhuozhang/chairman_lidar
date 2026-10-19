// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_
#define PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pb_omni_pid_pursuit_controller/pid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace pb_omni_pid_pursuit_controller
{

/**
 * @class pb_omni_pid_pursuit_controller::OmniPidPursuitController
 * @brief Regulated pure pursuit controller plugin
 */
class OmniPidPursuitController : public nav2_core::Controller
{
public:
  OmniPidPursuitController() = default;
  ~OmniPidPursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);

  bool transformPose(
    const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose) const;

  double getCostmapMaxExtent() const;

  std::unique_ptr<geometry_msgs::msg::PointStamped> createCarrotMsg(
    const geometry_msgs::msg::PoseStamped & carrot_pose);

  geometry_msgs::msg::PoseStamped getLookAheadPoint(
    const double & lookahead_dist, const nav_msgs::msg::Path & transformed_plan);

  geometry_msgs::msg::Point circleSegmentIntersection(
    const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2, double r);

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  double getLookAheadDistance(const geometry_msgs::msg::Twist & speed);

  // 保留声明：函数体内仍存在，但 computeVelocityCommands 已注释掉调用
  double approachVelocityScalingFactor(const nav_msgs::msg::Path & path) const;

  // 保留声明：函数体内仍存在，但 computeVelocityCommands 已注释掉调用
  void applyApproachVelocityScaling(
    const nav_msgs::msg::Path & path, double & linear_vel_x, double & linear_vel_y) const;

  bool isCollisionDetected(const nav_msgs::msg::Path & path);

private:
  void applyCurvatureLimitation(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double & linear_vel_x, double & linear_vel_y);

  double calculateCurvature(
    const nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & lookahead_pose,
    double forward_dist, double backward_dist) const;

  double calculateCurvatureRadius(
    const geometry_msgs::msg::Point & near_point, const geometry_msgs::msg::Point & current_point,
    const geometry_msgs::msg::Point & far_point) const;

  void visualizeCurvaturePoints(
    const geometry_msgs::msg::PoseStamped & backward_pose,
    const geometry_msgs::msg::PoseStamped & forward_pose) const;

  std::vector<double> calculateCumulativeDistances(const nav_msgs::msg::Path & path) const;

  geometry_msgs::msg::PoseStamped findPoseAtDistance(
    const nav_msgs::msg::Path & path, const std::vector<double> & cumulative_distances,
    double target_distance) const;
  void applyLowSpeedDecelLimit(
    const geometry_msgs::msg::PoseStamped & carrot_pose, double & lin_vel_x, double & lin_vel_y);

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_{rclcpp::get_logger("OmniPidPursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  std::mutex mutex_;

  tf2::Duration transform_tolerance_;
  double control_duration_;

  // x/y 分离的平动 PID（误差为 carrot 在 base 下的坐标，天然带符号）
  std::shared_ptr<PID> move_pid_x;
  std::shared_ptr<PID> move_pid_y;
  std::shared_ptr<PID> heading_pid_;
  // 末端速度
  double mid_distance_;
  double last_distance_;
  double mid_speed_;
  double last_speed_;
  double low_speed_threshold_;
  double max_decel_step_;

  // 平动 PID 参数（x/y 独立）
  double translation_kp_x, translation_ki_x, translation_kd_x;
  double translation_kp_y, translation_ki_y, translation_kd_y;
  double v_linear_min_x, v_linear_max_x;
  double v_linear_min_y, v_linear_max_y;

  // 角向 PID 参数
  double rotation_kp_, rotation_ki_, rotation_kd_;
  double v_angular_min_, v_angular_max_;
  bool enable_rotation_;

  double min_max_sum_error_;

  // lookahead
  double lookahead_dist_;
  bool use_velocity_scaled_lookahead_dist_;
  double min_lookahead_dist_;
  double max_lookahead_dist_;
  double lookahead_time_;
  bool use_interpolation_;
  double last_pub_vel_x_{0.0};
  double last_pub_vel_y_{0.0};
  // 朝向控制
  bool use_rotate_to_heading_;
  bool rotate_to_goal_heading_only_;
  double yaw_hold_tolerance_;
  double use_rotate_to_heading_treshold_;

  // 最小可驱动速度
  double min_approach_linear_velocity_;
  double min_approach_angular_velocity_;
  double min_approach_angular_velocity_tolerance_;

  // 保留声明以兼容旧 YAML；本版控制器内部不使用
  bool enable_final_pose_zero_stop_;
  double final_xy_goal_tolerance_;
  double final_yaw_goal_tolerance_;
  double approach_velocity_scaling_dist_;

  double max_robot_pose_search_dist_;

  // 曲率限速
  double curvature_min_;
  double curvature_max_;
  double reduction_ratio_at_high_curvature_;
  double curvature_forward_dist_;
  double curvature_backward_dist_;
  double max_velocity_scaling_factor_rate_;
  // 上一周期限完的速度（m/s），分别用于 x/y 斜坡基准
  double last_velocity_scaling_factor_x;
  double last_velocity_scaling_factor_y;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr carrot_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    curvature_points_pub_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  nav_msgs::msg::Path global_plan_;
};

}  // namespace pb_omni_pid_pursuit_controller

#endif  // PB_OMNI_PID_PURSUIT_CONTROLLER__OMNI_PID_PURSUIT_CONTROLLER_HPP_