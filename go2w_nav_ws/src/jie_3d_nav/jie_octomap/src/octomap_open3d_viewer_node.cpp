#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <open3d/Open3D.h>

#include "octomap/AbstractOcTree.h"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class OctomapOpen3DViewerNode : public rclcpp::Node
{
public:
  OctomapOpen3DViewerNode()
  : Node("octomap_open3d_viewer")
  {
    declare_parameter<std::string>("octomap_topic", "/octomap");
    declare_parameter<std::string>("marker_topic", "/selection_markers");
    declare_parameter<std::string>("path_topic", "/planned_path");
    declare_parameter<std::string>("preblocked_marker_topic", "/preblocked_cells_markers");
    declare_parameter<bool>("freeze_cloud_after_first", true);
    const auto octomap_topic = get_parameter("octomap_topic").as_string();
    const auto marker_topic = get_parameter("marker_topic").as_string();
    const auto path_topic = get_parameter("path_topic").as_string();
    const auto preblocked_marker_topic = get_parameter("preblocked_marker_topic").as_string();

    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&OctomapOpen3DViewerNode::onOctomap, this, std::placeholders::_1));
    marker_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      marker_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&OctomapOpen3DViewerNode::onMarkers, this, std::placeholders::_1));
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&OctomapOpen3DViewerNode::onPath, this, std::placeholders::_1));
    preblocked_sub_ = create_subscription<visualization_msgs::msg::Marker>(
      preblocked_marker_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&OctomapOpen3DViewerNode::onPreblocked, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "octomap_open3d_viewer started. octomap_topic=%s marker_topic=%s path_topic=%s preblocked=%s",
      octomap_topic.c_str(), marker_topic.c_str(), path_topic.c_str(), preblocked_marker_topic.c_str());
  }

  bool consumeLatest(std::vector<Eigen::Vector3d> & out_points)
  {
    std::lock_guard<std::mutex> lock(points_mutex_);
    if (!dirty_) {
      return false;
    }
    out_points = latest_points_;
    dirty_ = false;
    return true;
  }

  struct ArrowState
  {
    bool has_start{false};
    bool has_goal{false};
    bool has_start_cube{false};
    bool has_goal_cube{false};
    Eigen::Vector3d start_base{0.0, 0.0, 0.0};
    Eigen::Vector3d start_tip{0.0, 0.0, 0.0};
    Eigen::Vector3d goal_base{0.0, 0.0, 0.0};
    Eigen::Vector3d goal_tip{0.0, 0.0, 0.0};
    Eigen::Vector3d start_cube_center{0.0, 0.0, 0.0};
    Eigen::Vector3d goal_cube_center{0.0, 0.0, 0.0};
    double start_cube_size{0.30};
    double goal_cube_size{0.30};
  };

  bool consumeArrows(ArrowState & out_arrows)
  {
    std::lock_guard<std::mutex> lock(arrow_mutex_);
    if (!arrow_dirty_) {
      return false;
    }
    out_arrows = arrows_;
    arrow_dirty_ = false;
    return true;
  }

  bool consumePath(std::vector<Eigen::Vector3d> & out_path_points)
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    if (!path_dirty_) {
      return false;
    }
    out_path_points = latest_path_points_;
    path_dirty_ = false;
    return true;
  }

  bool consumePreblocked(std::vector<Eigen::Vector3d> & out_preblocked_points)
  {
    std::lock_guard<std::mutex> lock(preblocked_mutex_);
    if (!preblocked_dirty_) {
      return false;
    }
    out_preblocked_points = latest_preblocked_points_;
    preblocked_dirty_ = false;
    return true;
  }

private:
  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::unique_ptr<octomap::AbstractOcTree> tree_ptr(octomap_msgs::msgToMap(*msg));
    if (!tree_ptr) {
      RCLCPP_ERROR(get_logger(), "Failed to decode octomap message.");
      return;
    }

    auto * oc_tree = dynamic_cast<octomap::OcTree *>(tree_ptr.get());
    if (!oc_tree) {
      RCLCPP_ERROR(get_logger(), "Decoded map is not octomap::OcTree.");
      return;
    }

    std::vector<Eigen::Vector3d> points;
    points.reserve(oc_tree->size());
    for (auto it = oc_tree->begin_leafs(); it != oc_tree->end_leafs(); ++it) {
      if (!oc_tree->isNodeOccupied(*it)) {
        continue;
      }
      points.emplace_back(it.getX(), it.getY(), it.getZ());
    }

    {
      std::lock_guard<std::mutex> lock(points_mutex_);
      latest_points_.swap(points);
      dirty_ = true;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000, "Received OctoMap and converted to %zu points.",
      latest_points_.size());
  }

  void onMarkers(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    ArrowState next;
    for (const auto & m : msg->markers) {
      if (m.type != visualization_msgs::msg::Marker::ARROW || m.points.size() < 2) {
        continue;
      }
      if (m.id == 0) {
        next.has_start = true;
        next.start_base = Eigen::Vector3d(m.points[0].x, m.points[0].y, m.points[0].z);
        next.start_tip = Eigen::Vector3d(m.points[1].x, m.points[1].y, m.points[1].z);
      } else if (m.id == 1) {
        next.has_goal = true;
        next.goal_base = Eigen::Vector3d(m.points[0].x, m.points[0].y, m.points[0].z);
        next.goal_tip = Eigen::Vector3d(m.points[1].x, m.points[1].y, m.points[1].z);
      } else if (m.id == 2 && m.type == visualization_msgs::msg::Marker::CUBE) {
        next.has_start_cube = true;
        next.start_cube_center = Eigen::Vector3d(m.pose.position.x, m.pose.position.y, m.pose.position.z);
        next.start_cube_size = std::max(0.05, static_cast<double>(m.scale.x));
      } else if (m.id == 3 && m.type == visualization_msgs::msg::Marker::CUBE) {
        next.has_goal_cube = true;
        next.goal_cube_center = Eigen::Vector3d(m.pose.position.x, m.pose.position.y, m.pose.position.z);
        next.goal_cube_size = std::max(0.05, static_cast<double>(m.scale.x));
      }
    }
    {
      std::lock_guard<std::mutex> lock(arrow_mutex_);
      arrows_ = next;
      arrow_dirty_ = true;
    }
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(msg->poses.size());
    for (const auto & p : msg->poses) {
      pts.emplace_back(p.pose.position.x, p.pose.position.y, p.pose.position.z);
    }
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      latest_path_points_ = std::move(pts);
      path_dirty_ = true;
    }
  }

  void onPreblocked(const visualization_msgs::msg::Marker::SharedPtr msg)
  {
    if (msg->type != visualization_msgs::msg::Marker::CUBE_LIST) {
      return;
    }
    std::vector<Eigen::Vector3d> pts;
    pts.reserve(msg->points.size());
    for (const auto & p : msg->points) {
      pts.emplace_back(p.x, p.y, p.z);
    }
    {
      std::lock_guard<std::mutex> lock(preblocked_mutex_);
      latest_preblocked_points_ = std::move(pts);
      preblocked_dirty_ = true;
    }
  }

public:
  static std::shared_ptr<open3d::geometry::TriangleMesh> makeArrowMesh(
    const Eigen::Vector3d & base, const Eigen::Vector3d & tip, const Eigen::Vector3d & color)
  {
    const Eigen::Vector3d dir = tip - base;
    const double len = dir.norm();
    const Eigen::Vector3d u = len > 1e-6 ? dir / len : Eigen::Vector3d(0.0, 0.0, 1.0);

    const double cyl_h = std::max(0.1, len * 0.65);
    const double cone_h = std::max(0.05, len * 0.35);
    const double cyl_r = std::max(0.08, len * 0.20);
    const double cone_r = std::max(0.16, len * 0.36);

    auto mesh = open3d::geometry::TriangleMesh::CreateArrow(
      cyl_r, cone_r, cyl_h, cone_h, 24, 4, 1);
    mesh->ComputeVertexNormals();
    mesh->PaintUniformColor(color);

    const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), u);
    mesh->Rotate(q.toRotationMatrix(), Eigen::Vector3d(0.0, 0.0, 0.0));
    mesh->Translate(base);
    return mesh;
  }

  static std::shared_ptr<open3d::geometry::TriangleMesh> makeCubeMesh(
    const Eigen::Vector3d & center, double size, const Eigen::Vector3d & color)
  {
    auto cube = open3d::geometry::TriangleMesh::CreateBox(size, size, size);
    cube->ComputeVertexNormals();
    cube->PaintUniformColor(color);
    cube->Translate(center - Eigen::Vector3d(size * 0.5, size * 0.5, size * 0.5));
    return cube;
  }

private:
  std::mutex points_mutex_;
  std::mutex arrow_mutex_;
  std::mutex path_mutex_;
  std::mutex preblocked_mutex_;
  std::vector<Eigen::Vector3d> latest_points_;
  std::vector<Eigen::Vector3d> latest_path_points_;
  std::vector<Eigen::Vector3d> latest_preblocked_points_;
  ArrowState arrows_;
  bool dirty_{false};
  bool arrow_dirty_{false};
  bool path_dirty_{false};
  bool preblocked_dirty_{false};
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr preblocked_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::InitOptions init_options;
  rclcpp::init(argc, argv, init_options);
  rclcpp::uninstall_signal_handlers();
  auto node = std::make_shared<OctomapOpen3DViewerNode>();

  open3d::visualization::Visualizer vis;
  const bool ok = vis.CreateVisualizerWindow("Open3D OctoMap Viewer (Read-Only)", 1280, 800);
  if (!ok) {
    RCLCPP_ERROR(node->get_logger(), "Failed to create Open3D window.");
    rclcpp::shutdown();
    return 1;
  }
  auto & render_option = vis.GetRenderOption();
  render_option.background_color_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  render_option.point_size_ = 4.0;
  render_option.line_width_ = 4.0;

  auto cloud = std::make_shared<open3d::geometry::PointCloud>();
  auto preblocked_cloud = std::make_shared<open3d::geometry::PointCloud>();
  std::shared_ptr<open3d::geometry::TriangleMesh> start_arrow;
  std::shared_ptr<open3d::geometry::TriangleMesh> goal_arrow;
  std::shared_ptr<open3d::geometry::TriangleMesh> start_cube;
  std::shared_ptr<open3d::geometry::TriangleMesh> goal_cube;
  std::shared_ptr<open3d::geometry::LineSet> path_lines;
  bool geometry_added = false;
  bool preblocked_added = false;
  bool start_added = false;
  bool goal_added = false;
  bool start_cube_added = false;
  bool goal_cube_added = false;
  bool path_added = false;

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    std::vector<Eigen::Vector3d> points;
    if (node->consumeLatest(points)) {
      const bool freeze_after_first = node->get_parameter("freeze_cloud_after_first").as_bool();
      if (!geometry_added || !freeze_after_first) {
        cloud->points_ = points;
        cloud->colors_.assign(points.size(), Eigen::Vector3d(1.0, 1.0, 1.0));
        if (!geometry_added) {
          vis.AddGeometry(cloud);
          vis.GetViewControl().SetZoom(0.35);
          geometry_added = true;
        } else {
          vis.UpdateGeometry(cloud);
        }
      }
    }

    std::vector<Eigen::Vector3d> preblocked_points;
    if (node->consumePreblocked(preblocked_points)) {
      preblocked_cloud->points_ = preblocked_points;
      preblocked_cloud->colors_.assign(preblocked_points.size(), Eigen::Vector3d(0.15, 0.35, 1.0));
      if (!preblocked_added) {
        vis.AddGeometry(preblocked_cloud);
        preblocked_added = true;
      } else {
        vis.UpdateGeometry(preblocked_cloud);
      }
    }

    OctomapOpen3DViewerNode::ArrowState arrows;
    if (node->consumeArrows(arrows)) {
      if (arrows.has_start) {
        auto new_start_arrow = OctomapOpen3DViewerNode::makeArrowMesh(
          arrows.start_base, arrows.start_tip, Eigen::Vector3d(0.1, 0.95, 0.1));
        if (start_added && start_arrow) {
          vis.RemoveGeometry(start_arrow);
        }
        start_arrow = new_start_arrow;
        vis.AddGeometry(start_arrow);
        start_added = true;
      } else if (start_added && start_arrow) {
        vis.RemoveGeometry(start_arrow);
        start_arrow.reset();
        start_added = false;
      }
      if (arrows.has_start_cube) {
        auto new_start_cube = OctomapOpen3DViewerNode::makeCubeMesh(
          arrows.start_cube_center, arrows.start_cube_size, Eigen::Vector3d(0.1, 0.95, 0.1));
        if (start_cube_added && start_cube) {
          vis.RemoveGeometry(start_cube);
        }
        start_cube = new_start_cube;
        vis.AddGeometry(start_cube);
        start_cube_added = true;
      } else if (start_cube_added && start_cube) {
        vis.RemoveGeometry(start_cube);
        start_cube.reset();
        start_cube_added = false;
      }
      if (arrows.has_goal) {
        auto new_goal_arrow = OctomapOpen3DViewerNode::makeArrowMesh(
          arrows.goal_base, arrows.goal_tip, Eigen::Vector3d(0.95, 0.1, 0.1));
        if (goal_added && goal_arrow) {
          vis.RemoveGeometry(goal_arrow);
        }
        goal_arrow = new_goal_arrow;
        vis.AddGeometry(goal_arrow);
        goal_added = true;
      } else if (goal_added && goal_arrow) {
        vis.RemoveGeometry(goal_arrow);
        goal_arrow.reset();
        goal_added = false;
      }
      if (arrows.has_goal_cube) {
        auto new_goal_cube = OctomapOpen3DViewerNode::makeCubeMesh(
          arrows.goal_cube_center, arrows.goal_cube_size, Eigen::Vector3d(0.95, 0.1, 0.1));
        if (goal_cube_added && goal_cube) {
          vis.RemoveGeometry(goal_cube);
        }
        goal_cube = new_goal_cube;
        vis.AddGeometry(goal_cube);
        goal_cube_added = true;
      } else if (goal_cube_added && goal_cube) {
        vis.RemoveGeometry(goal_cube);
        goal_cube.reset();
        goal_cube_added = false;
      }
    }

    std::vector<Eigen::Vector3d> path_points;
    if (node->consumePath(path_points)) {
      if (path_points.size() >= 2) {
        auto new_path = std::make_shared<open3d::geometry::LineSet>();
        new_path->points_ = path_points;
        new_path->lines_.reserve(path_points.size() - 1);
        for (std::size_t i = 0; i + 1 < path_points.size(); ++i) {
          new_path->lines_.push_back(Eigen::Vector2i(static_cast<int>(i), static_cast<int>(i + 1)));
        }
        new_path->colors_.assign(new_path->lines_.size(), Eigen::Vector3d(0.1, 0.95, 0.95));

        if (path_added && path_lines) {
          vis.RemoveGeometry(path_lines);
        }
        path_lines = new_path;
        vis.AddGeometry(path_lines);
        path_added = true;
      } else if (path_added && path_lines) {
        vis.RemoveGeometry(path_lines);
        path_lines.reset();
        path_added = false;
      }
    }

    if (!vis.PollEvents()) {
      break;
    }
    vis.UpdateRender();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  vis.DestroyVisualizerWindow();
  rclcpp::shutdown();
  return 0;
}
