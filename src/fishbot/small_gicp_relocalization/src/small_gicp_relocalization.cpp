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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <chrono>
#include <fstream>

#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/create_timer.hpp"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp_relocalization/map_preprocessing.hpp"
#include "small_gicp_relocalization/pose_transform.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options), result_t_(Eigen::Isometry3d::Identity())
{
  num_threads_ = declare_parameter("num_threads", 16);
  num_neighbors_ = declare_parameter("num_neighbors", 10);
  max_dist_sq_ = declare_parameter("max_dist_sq", 4.0);
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
  robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "base_footprint");
  prior_pcd_file_ = declare_parameter<std::string>("prior_pcd_file", "");
  input_cloud_topic_ = declare_parameter<std::string>("input_cloud_topic", "/cloud_registered");
  initial_map_pose_ = poseFromXyzRpy(
    declare_parameter<std::vector<double>>("initial_map_pose", std::vector<double>{}));
  map_to_pcd_ =
    poseFromXyzRpy(declare_parameter<std::vector<double>>("map_to_pcd", std::vector<double>{}));
  const double publish_rate = declare_parameter("publish_rate_hz", 20.0);
  const double registration_rate = declare_parameter("registration_rate_hz", 2.0);
  time_offset_ = declare_parameter("time_offset_sec", 0.1);
  if (
    map_frame_.empty() || odom_frame_.empty() || robot_base_frame_.empty() ||
    map_frame_ == odom_frame_ || map_frame_ == robot_base_frame_ ||
    odom_frame_ == robot_base_frame_) {
    throw std::invalid_argument("map, odom and robot_base_frame must be distinct nonempty frames");
  }
  if (
    num_threads_ < 1 || num_neighbors_ < 3 || !std::isfinite(max_dist_sq_) || max_dist_sq_ <= 0.0 ||
    !std::isfinite(publish_rate) || publish_rate <= 0.0 || publish_rate > 1000.0 ||
    !std::isfinite(registration_rate) || registration_rate <= 0.0 || registration_rate > 100.0 ||
    !std::isfinite(time_offset_) || time_offset_ < 0.0) {
    throw std::invalid_argument("invalid GICP sampling, matching or timer parameters");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = 10;

  using SteadyClock = std::chrono::steady_clock;
  const auto map_begin = SteadyClock::now();
  RCLCPP_INFO(get_logger(), "[地图准备] 读取 PCD：%s", prior_pcd_file_.c_str());
  loadGlobalMap(prior_pcd_file_);
  const auto map_loaded = SteadyClock::now();
  RCLCPP_INFO(
    get_logger(),
    "[配准参数] 地图不再降采样；实时点云使用 PCD 记录的体素边长=%.3fm，"
    "num_neighbors=%d，最大对应距离=%.3fm，迭代上限=%d",
    map_leaf_size_, num_neighbors_, std::sqrt(max_dist_sq_), register_->optimizer.max_iterations);
  target_ = toCovariancePoints(*global_map_);
  const auto map_prepared = SteadyClock::now();
  RCLCPP_INFO(
    get_logger(), "[地图准备] 使用离线点云，地图点数 %zu -> %zu；开始建立搜索树和计算协方差",
    global_map_->size(), target_->size());
  global_map_.reset();
  if (target_->size() < static_cast<size_t>(num_neighbors_)) {
    throw std::runtime_error("PCD map has too few finite points for covariance estimation");
  }
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));
  const auto map_indexed = SteadyClock::now();
  // 协方差估计与后续配准复用同一棵地图搜索树。
  small_gicp::estimate_covariances_omp(*target_, *target_tree_, num_neighbors_, num_threads_);
  const auto map_ready = SteadyClock::now();
  const auto seconds = [](auto start, auto end) {
    return std::chrono::duration<double>(end - start).count();
  };
  RCLCPP_INFO(
    get_logger(), "[地图就绪] 读取 %.3fs，点类型转换 %.3fs，搜索树 %.3fs，协方差 %.3fs，总计 %.3fs",
    seconds(map_begin, map_loaded), seconds(map_loaded, map_prepared),
    seconds(map_prepared, map_indexed), seconds(map_indexed, map_ready),
    seconds(map_begin, map_ready));

  pcd_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));
  // 只接收当前发布的人工初始位姿，避免重启时重放历史定位指令。
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::QoS(10).reliable().durability_volatile(),
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));
  register_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / registration_rate),
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));
  // 配准可能耗时；独立回调组配合多线程执行器保持 TF 定时发布。
  transform_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  transform_timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(1.0 / publish_rate),
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this), transform_group_);
  RCLCPP_INFO(
    get_logger(), "等待本次 %s -> %s TF，以所选地图位姿初始化 GICP", odom_frame_.c_str(),
    robot_base_frame_.c_str());
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  std::ifstream header(file_name, std::ios::binary);
  if (!header) {
    throw std::runtime_error("Unable to read PCD map: " + file_name);
  }
  map_leaf_size_ = readMapVoxelSize(header);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    throw std::runtime_error("Unable to load PCD map: " + file_name);
  }
  auto & points = global_map_->points;
  points.erase(
    std::remove_if(
      points.begin(), points.end(),
      [](const pcl::PointXYZ & p) {
        return !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z);
      }),
    points.end());
  global_map_->width = points.size();
  global_map_->height = 1;
  global_map_->is_dense = true;
  // 保留 PCD 自身坐标，map_to_pcd 只用于初值换算和最终 TF 输出。
  RCLCPP_INFO(get_logger(), "Loaded PCD map with %zu finite points", points.size());
}

bool SmallGicpRelocalizationNode::initializeFromPose(
  const Eigen::Isometry3d & map_to_robot, const rclcpp::Time & stamp, bool force)
{
  try {
    const auto tf = tf_buffer_->lookupTransform(
      odom_frame_, robot_base_frame_, stamp, rclcpp::Duration::from_seconds(force ? 0.2 : 0.0));
    const Eigen::Isometry3d odom_to_robot(tf2::transformToEigen(tf.transform).matrix());
    const auto seed = initialAlignment(map_to_pcd_, map_to_robot, odom_to_robot);
    if (!seed.matrix().allFinite()) {
      throw std::runtime_error("initial alignment contains nonfinite values");
    }
    std::lock_guard<std::mutex> guard(transform_mutex_);
    if (initialized_ && !force) {
      return true;
    }
    result_t_ = seed;
    map_to_start_ = map_to_robot;
    pcd_to_start_ = seed * odom_to_robot;
    initialized_ = true;
    matched_ = false;
    consecutive_registration_failures_ = 0;
    const auto & pcd_to_robot = pcd_to_start_;
    const auto yaw = [](const Eigen::Isometry3d & pose) {
      return std::atan2(pose(1, 0), pose(0, 0));
    };
    RCLCPP_INFO(
      get_logger(),
      "[定位初始化] %s：map=(%.3f, %.3f, %.3f)，yaw=%.4f rad；"
      "PCD=(%.3f, %.3f, %.3f)，yaw=%.4f rad",
      robot_base_frame_.c_str(), map_to_robot.translation().x(), map_to_robot.translation().y(),
      map_to_robot.translation().z(), yaw(map_to_robot), pcd_to_robot.translation().x(),
      pcd_to_robot.translation().y(), pcd_to_robot.translation().z(), yaw(pcd_to_robot));
    RCLCPP_INFO(
      get_logger(),
      "[匹配初值] PCD <- %s=(%.3f, %.3f, %.3f)，yaw=%.4f rad；"
      "已设置配准初值与初始 map -> odom，等待点云配准",
      odom_frame_.c_str(), seed.translation().x(), seed.translation().y(), seed.translation().z(),
      yaw(seed));
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000, "等待局部里程计 TF，尚未初始化定位: %s", ex.what());
    return false;
  }
}

void SmallGicpRelocalizationNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
  if (
    msg->header.frame_id != map_frame_ || !std::isfinite(p.x) || !std::isfinite(p.y) ||
    !std::isfinite(p.z) || !rotation.coeffs().allFinite() || rotation.norm() < 1e-9) {
    RCLCPP_WARN(get_logger(), "拒绝 /initialpose：需使用 map_frame，坐标及非零四元数必须有限");
    return;
  }
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() << p.x, p.y, p.z;
  pose.linear() = rotation.normalized().toRotationMatrix();
  if (initializeFromPose(
        pose, rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type()), true)) {
    accumulated_cloud_->clear();
  }
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> guard(transform_mutex_);
    if (!initialized_) {
      return;
    }
  }
  if (msg->header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "点云缺少 frame_id，跳过该帧");
    return;
  }
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  if (msg->header.frame_id != odom_frame_) {
    try {
      const auto tf = tf_buffer_->lookupTransform(
        odom_frame_, msg->header.frame_id, rclcpp::Time(msg->header.stamp),
        rclcpp::Duration::from_seconds(0.0));
      transform = tf2::transformToEigen(tf.transform).matrix().cast<float>();
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "点云转换到 odom 失败，跳过该帧: %s", ex.what());
      return;
    }
  }
  pcl::PointCloud<pcl::PointXYZ> scan;
  pcl::fromROSMsg(*msg, scan);
  pcl::transformPointCloud(scan, scan, transform);
  for (const auto & point : scan) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      accumulated_cloud_->push_back(point);
    }
  }
}

void SmallGicpRelocalizationNode::performRegistration()
{
  Eigen::Isometry3d seed;
  {
    std::lock_guard<std::mutex> guard(transform_mutex_);
    if (!initialized_ || accumulated_cloud_->empty()) {
      return;
    }
    seed = result_t_;
  }
  const auto registration_begin = std::chrono::steady_clock::now();
  source_ = downsampleLivePoints(*accumulated_cloud_, map_leaf_size_);
  accumulated_cloud_->clear();
  if (source_->size() < static_cast<size_t>(num_neighbors_)) {
    return;
  }
  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);
  const auto result = register_->align(*target_, *source_, *target_tree_, seed);
  const double elapsed_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - registration_begin)
      .count();
  // small_gicp 的 iterations 记录从 0 开始的外层迭代下标。
  const size_t iterations = result.iterations + 1;
  const int iteration_limit = register_->optimizer.max_iterations;
  const bool finite_result =
    result.T_target_source.matrix().allFinite() && std::isfinite(result.error);
  std::lock_guard<std::mutex> guard(transform_mutex_);
  if (
    result.converged && finite_result &&
    result.num_inliers >= static_cast<size_t>(num_neighbors_)) {
    result_t_ = result.T_target_source;
    if (!matched_) {
      matched_ = true;
      RCLCPP_INFO(
        get_logger(),
        "[定位就绪] GICP 首次配准收敛，iterations=%zu/%d，source=%zu，target=%zu，"
        "inliers=%zu，error=%.4f，"
        "耗时=%.1fms；此前连续失败=%zu 轮",
        iterations, iteration_limit, source_->size(), target_->size(), result.num_inliers,
        result.error, elapsed_ms, consecutive_registration_failures_);
    } else if (consecutive_registration_failures_ > 0) {
      RCLCPP_INFO(
        get_logger(),
        "[定位恢复] GICP 配准收敛，iterations=%zu/%d，source=%zu，target=%zu，"
        "inliers=%zu，error=%.4f，"
        "耗时=%.1fms；此前连续失败=%zu 轮",
        iterations, iteration_limit, source_->size(), target_->size(), result.num_inliers,
        result.error, elapsed_ms, consecutive_registration_failures_);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "[定位更新] GICP 配准收敛，iterations=%zu/%d，source=%zu，target=%zu，"
        "inliers=%zu，error=%.4f，耗时=%.1fms",
        iterations, iteration_limit, source_->size(), target_->size(), result.num_inliers,
        result.error, elapsed_ms);
    }
    consecutive_registration_failures_ = 0;
  } else {
    ++consecutive_registration_failures_;
    const char * reason;
    if (!finite_result) {
      reason = "变换或误差含非有限值";
    } else if (result.num_inliers < static_cast<size_t>(num_neighbors_)) {
      reason = "有效对应点不足";
    } else if (iterations >= static_cast<size_t>(iteration_limit)) {
      reason = "未收敛，本轮已运行到迭代上限";
    } else {
      reason = "未收敛，优化器提前退出";
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "GICP 本轮匹配无效，%s；原因=%s；converged=%d，iterations=%zu/%d，"
      "inliers=%zu/%zu (%.1f%%)，target=%zu，error=%.4f，耗时=%.1fms，连续失败=%zu 轮",
      matched_ ? "保持上次成功变换" : "保持启动初值", reason, result.converged, iterations,
      iteration_limit, result.num_inliers, source_->size(),
      100.0 * result.num_inliers / source_->size(), target_->size(), result.error, elapsed_ms,
      consecutive_registration_failures_);
  }
}

void SmallGicpRelocalizationNode::publishTransform()
{
  bool ready;
  {
    std::lock_guard<std::mutex> guard(transform_mutex_);
    ready = initialized_;
  }
  if (
    !ready &&
    !initializeFromPose(initial_map_pose_, rclcpp::Time(0, 0, get_clock()->get_clock_type()))) {
    return;
  }
  Eigen::Isometry3d map_to_odom;
  {
    std::lock_guard<std::mutex> guard(transform_mutex_);
    map_to_odom = mapOdometryFromStart(map_to_start_, pcd_to_start_, result_t_);
  }
  geometry_msgs::msg::TransformStamped output;
  output.header.stamp = now() + rclcpp::Duration::from_seconds(time_offset_);
  output.header.frame_id = map_frame_;
  output.child_frame_id = odom_frame_;
  output.transform = tf2::eigenToTransform(map_to_odom).transform;
  tf_broadcaster_->sendTransform(output);
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
