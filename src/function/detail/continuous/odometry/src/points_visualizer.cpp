#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

// 点位仅作为 map 坐标系中的显示标记，不参与导航控制或代价地图计算。
class PointsVisualizer : public rclcpp::Node
{
public:
  PointsVisualizer() : Node("points_visualizer")
  {
    points_file_ = declare_parameter<std::string>("points_file", "");
    if (points_file_.empty()) {
      throw std::invalid_argument("请通过 points_file 指定 points.yaml，或运行 odometry.launch.py");
    }
    publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/navigation_points", rclcpp::QoS(1).reliable().transient_local());
    reload();
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {reload();});
  }

private:
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  MarkerArray make_markers(const std::string & text)
  {
    const auto config = YAML::Load(text);
    if (!config.IsMap()) {
      throw std::runtime_error("points.yaml 顶层必须是字典");
    }
    MarkerArray result;
    Marker clear;
    clear.header.frame_id = "map";
    clear.action = Marker::DELETEALL;
    result.markers.push_back(clear);
    const auto stamp = now();
    for (const std::string team : {"red", "blue"}) {
      const auto points = config[team];
      if (!points.IsMap()) {
        throw std::runtime_error("缺少点位字典: " + team);
      }
      std::set<int> ids;
      for (const auto & entry : points) {
        const int id = entry.first.as<int>();
        if (id < 0 || id > 127 || !ids.insert(id).second) {
          throw std::runtime_error(team + " 点位编号重复或超出 0～127");
        }
        const auto pose = entry.second["pose"];
        if (!pose.IsSequence() || pose.size() != 4) {
          throw std::runtime_error(team + "/" + std::to_string(id) + ": pose 必须含 x/y/qz/qw");
        }
        bool dynamic = true;
        for (const auto & value : pose) {
          dynamic = dynamic && value.IsNull();
        }
        if (dynamic) {
          continue;  // 动态目标没有固定地图坐标。
        }
        const double x = pose[0].as<double>();
        const double y = pose[1].as<double>();
        if (!std::isfinite(x) || !std::isfinite(y)) {
          throw std::runtime_error(team + "/" + std::to_string(id) + ": 坐标必须为有限数值");
        }
        Marker dot;
        dot.header.frame_id = "map";
        dot.header.stamp = stamp;
        dot.frame_locked = true;
        dot.ns = team + "_points";
        dot.id = id;
        dot.type = Marker::SPHERE;
        dot.action = Marker::ADD;
        dot.pose.position.x = x;
        dot.pose.position.y = y;
        dot.pose.position.z = 0.06;
        dot.pose.orientation.w = 1.0;
        dot.scale.x = dot.scale.y = dot.scale.z = 0.12;
        dot.color.r = team == "red" ? 1.0f : 0.0f;
        dot.color.b = team == "blue" ? 1.0f : 0.0f;
        dot.color.a = 1.0f;
        result.markers.push_back(dot);

        Marker label = dot;
        label.ns = team + "_labels";
        label.type = Marker::TEXT_VIEW_FACING;
        label.pose.position.z = 0.25;
        label.scale.z = 0.16;
        label.text = (team == "red" ? "R" : "B") + std::to_string(id);
        result.markers.push_back(label);
      }
    }
    return result;
  }

  void reload()
  {
    try {
      std::ifstream input(points_file_);
      if (!input) {
        throw std::runtime_error("无法读取 " + points_file_);
      }
      std::ostringstream stream;
      stream << input.rdbuf();
      if (input.bad()) {
        throw std::runtime_error("读取点位文件失败");
      }
      const auto text = stream.str();
      if (loaded_ && text == last_text_) {
        return;
      }
      // 完整解析后再发布，编辑中的无效 YAML 不清空上次有效显示。
      auto markers = make_markers(text);
      publisher_->publish(markers);
      last_text_ = text;
      loaded_ = true;
      RCLCPP_INFO(get_logger(), "已显示 %zu 个红蓝定点：%s",
        (markers.markers.size() - 1) / 2, points_file_.c_str());
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "点位显示暂未更新，保留上次有效标记：%s", error.what());
    }
  }

  std::string points_file_;
  std::string last_text_;
  bool loaded_ = false;
  rclcpp::Publisher<MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PointsVisualizer>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("points_visualizer"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
