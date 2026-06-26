#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "jie_map_msgs/srv/export_navigation_snapshot.hpp"
#include "jie_map_msgs/srv/get_navigation_map_meta.hpp"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace
{
struct GridIndex
{
  int x;
  int y;
  int z;

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & k) const
  {
    const std::size_t h1 = std::hash<int>{}(k.x);
    const std::size_t h2 = std::hash<int>{}(k.y);
    const std::size_t h3 = std::hash<int>{}(k.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

struct QueueNode
{
  GridIndex idx;
  double f;
  double g;
};

struct QueueNodeCompare
{
  bool operator()(const QueueNode & a, const QueueNode & b) const
  {
    return a.f > b.f;
  }
};

double euclidean(const GridIndex & a, const GridIndex & b)
{
  const double dx = static_cast<double>(a.x - b.x);
  const double dy = static_cast<double>(a.y - b.y);
  const double dz = static_cast<double>(a.z - b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::uint64_t hashOctomapData(const std::vector<int8_t> & data)
{
  // FNV-1a 64-bit
  std::uint64_t h = 1469598103934665603ULL;
  for (const auto v : data) {
    h ^= static_cast<std::uint8_t>(v);
    h *= 1099511628211ULL;
  }
  return h;
}
}  // namespace

class JiePathNode : public rclcpp::Node
{
public:
  JiePathNode()
  : Node("jie_path_node"),
    map_ready_(false),
    has_start_(false),
    has_goal_(false),
    planning_in_progress_(false),
    plan_seq_(0),
    last_success_seq_(0)
  {
    declare_parameter<std::string>("octomap_topic", "/octomap");
    declare_parameter<std::string>("start_topic", "/start_point");
    declare_parameter<std::string>("goal_topic", "/goal_point");
    declare_parameter<std::string>("goal_pose_topic", "/goal_pose");
    declare_parameter<std::string>("path_topic", "/planned_path");
    declare_parameter<std::string>("path_marker_topic", "/planned_path_marker");
    declare_parameter<std::string>("preblocked_marker_topic", "/preblocked_cells_markers");
    declare_parameter<std::string>("external_preblocked_marker_topic", "/edited_preblocked_cells_markers");
    declare_parameter<std::string>("edited_occupied_marker_topic", "/edited_occupied_markers");
    declare_parameter<std::string>("traversable_marker_topic", "/traversable_cells_markers");
    declare_parameter<std::string>("risk_cost_topic", "/risk_cost_cells");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<double>("robot_radius", 0.20);
    declare_parameter<int>("max_iterations", 250000);
    declare_parameter<int>("snap_search_radius_cells", 8);
    declare_parameter<bool>("require_ground_support", true);
    declare_parameter<bool>("strict_direct_ground_support", true);
    declare_parameter<int>("ground_support_xy_radius_cells", 1);
    declare_parameter<int>("ground_support_depth_cells", 2);
    declare_parameter<bool>("enable_preblocked_costmap", true);
    declare_parameter<int>("preblocked_costmap_radius_cells", 3);
    declare_parameter<double>("preblocked_costmap_weight", 1.5);
    declare_parameter<bool>("lowest_traversable_only", false);
    declare_parameter<std::string>("map_id", "navigation_map");
    declare_parameter<std::string>("source_world_file", "");

    const auto octomap_topic = get_parameter("octomap_topic").as_string();
    const auto start_topic = get_parameter("start_topic").as_string();
    const auto goal_topic = get_parameter("goal_topic").as_string();
    const auto goal_pose_topic = get_parameter("goal_pose_topic").as_string();
    const auto path_topic = get_parameter("path_topic").as_string();
    const auto path_marker_topic = get_parameter("path_marker_topic").as_string();
    const auto preblocked_marker_topic = get_parameter("preblocked_marker_topic").as_string();
    const auto external_preblocked_marker_topic =
      get_parameter("external_preblocked_marker_topic").as_string();
    const auto edited_occupied_marker_topic =
      get_parameter("edited_occupied_marker_topic").as_string();
    const auto traversable_marker_topic = get_parameter("traversable_marker_topic").as_string();
    const auto risk_cost_topic = get_parameter("risk_cost_topic").as_string();

    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&JiePathNode::onOctomap, this, std::placeholders::_1));
    start_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      start_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&JiePathNode::onStart, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      goal_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&JiePathNode::onGoal, this, std::placeholders::_1));
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_pose_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&JiePathNode::onGoalPose, this, std::placeholders::_1));
    external_preblocked_sub_ = create_subscription<visualization_msgs::msg::Marker>(
      external_preblocked_marker_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&JiePathNode::onExternalPreblockedMarker, this, std::placeholders::_1));
    edited_occupied_sub_ = create_subscription<visualization_msgs::msg::Marker>(
      edited_occupied_marker_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&JiePathNode::onEditedOccupiedMarker, this, std::placeholders::_1));

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      path_topic, rclcpp::QoS(1).transient_local().reliable());
    path_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      path_marker_topic, rclcpp::QoS(1).transient_local().reliable());
    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      octomap_topic, rclcpp::QoS(1).transient_local().reliable());
    preblocked_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      preblocked_marker_topic, rclcpp::QoS(1).transient_local().reliable());
    traversable_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      traversable_marker_topic, rclcpp::QoS(1).transient_local().reliable());
    risk_cost_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      risk_cost_topic, rclcpp::QoS(1).transient_local().reliable());
    get_meta_srv_ = create_service<jie_map_msgs::srv::GetNavigationMapMeta>(
      "~/get_meta",
      std::bind(
        &JiePathNode::handleGetMapMeta, this, std::placeholders::_1,
        std::placeholders::_2));
    export_snapshot_srv_ = create_service<jie_map_msgs::srv::ExportNavigationSnapshot>(
      "~/export_snapshot",
      std::bind(
        &JiePathNode::handleExportSnapshot, this, std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "jie_path_node started. octomap=%s start=%s goal=%s path=%s preblocked_marker=%s "
      "edited_occupied=%s meta_service=%s export_service=%s",
      octomap_topic.c_str(), start_topic.c_str(), goal_topic.c_str(), path_topic.c_str(),
      preblocked_marker_topic.c_str(), edited_occupied_marker_topic.c_str(), "~/get_meta",
      "~/export_snapshot");
  }

private:
  void fillBounds(
    geometry_msgs::msg::Point & min_bound,
    geometry_msgs::msg::Point & max_bound) const
  {
    if (!octree_) {
      return;
    }
    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);
    min_bound.x = min_x;
    min_bound.y = min_y;
    min_bound.z = min_z;
    max_bound.x = max_x;
    max_bound.y = max_y;
    max_bound.z = max_z;
  }

  void handleGetMapMeta(
    const std::shared_ptr<jie_map_msgs::srv::GetNavigationMapMeta::Request> /*request*/,
    std::shared_ptr<jie_map_msgs::srv::GetNavigationMapMeta::Response> response)
  {
    response->success = map_ready_ && static_cast<bool>(octree_);
    response->message = response->success ? "ok" : "octomap not ready";
    response->map_id = get_parameter("map_id").as_string();
    response->frame_id = get_parameter("frame_id").as_string();
    response->resolution = octree_ ? octree_->getResolution() : 0.0;
    fillBounds(response->min_bound, response->max_bound);
    response->robot_radius = get_parameter("robot_radius").as_double();
    response->snap_search_radius_cells = get_parameter("snap_search_radius_cells").as_int();
    response->require_ground_support = get_parameter("require_ground_support").as_bool();
    response->strict_direct_ground_support =
      get_parameter("strict_direct_ground_support").as_bool();
    response->ground_support_xy_radius_cells =
      get_parameter("ground_support_xy_radius_cells").as_int();
    response->ground_support_depth_cells = get_parameter("ground_support_depth_cells").as_int();
    response->enable_preblocked_costmap = get_parameter("enable_preblocked_costmap").as_bool();
    response->preblocked_costmap_radius_cells =
      get_parameter("preblocked_costmap_radius_cells").as_int();
    response->preblocked_costmap_weight = get_parameter("preblocked_costmap_weight").as_double();
    response->source_world_file = get_parameter("source_world_file").as_string();
  }

  void handleExportSnapshot(
    const std::shared_ptr<jie_map_msgs::srv::ExportNavigationSnapshot::Request> request,
    std::shared_ptr<jie_map_msgs::srv::ExportNavigationSnapshot::Response> response)
  {
    if (!map_ready_ || !octree_) {
      response->success = false;
      response->message = "octomap not ready";
      response->snapshot_stamp = now();
      return;
    }

    if (request->recompute_layers) {
      rebuildPreblockedCells();
      rebuildDerivedLayers();
      rebuildPreblockedCostmap();
    } else {
      publishPreblockedCellsMarker();
      publishCellSetMarker(
        traversable_cells_, traversable_marker_pub_, "traversable_cells", 0.20F, 0.95F, 0.55F,
        0.55F);
      publishRiskCostCloud();
    }

    response->success = true;
    response->message = "snapshot ready";
    response->snapshot_stamp = now();
  }

  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    const std::uint64_t map_hash = hashOctomapData(msg->data);
    if (map_ready_ && map_hash == last_octomap_hash_) {
      return;
    }

    octree_.reset(dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg)));
    if (!octree_) {
      RCLCPP_ERROR(get_logger(), "Failed to convert OctoMap message to OcTree.");
      return;
    }
    map_ready_ = true;
    last_octomap_hash_ = map_hash;
    clearNavigationStateAndPublish(msg->header.frame_id);
    rebuildPreblockedCells();
    rebuildDerivedLayers();
    rebuildPreblockedCostmap();
  }

  void onEditedOccupiedMarker(const visualization_msgs::msg::Marker::SharedPtr msg)
  {
    if (msg->type != visualization_msgs::msg::Marker::CUBE_LIST) {
      RCLCPP_WARN(get_logger(), "Ignored edited occupied marker because it is not CUBE_LIST.");
      return;
    }

    const double resolution = markerResolution(*msg);
    if (resolution <= 0.0) {
      RCLCPP_WARN(get_logger(), "Ignored edited occupied marker because scale is invalid.");
      return;
    }

    auto edited_tree = std::make_shared<octomap::OcTree>(resolution);
    for (const auto & point : msg->points) {
      edited_tree->updateNode(
        octomap::point3d(
          static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)),
        true);
    }
    edited_tree->updateInnerOccupancy();

    octree_ = edited_tree;
    map_ready_ = true;
    last_octomap_hash_ = 0;
    if (!msg->header.frame_id.empty()) {
      set_parameter(rclcpp::Parameter("frame_id", msg->header.frame_id));
    }

    publishCurrentOctomap();
    rebuildPreblockedCells();
    rebuildDerivedLayers();
    rebuildPreblockedCostmap();

    RCLCPP_INFO(
      get_logger(),
      "Edited occupied marker applied. occupied_cells=%zu resolution=%.3f",
      msg->points.size(), resolution);

    if (has_start_ && has_goal_) {
      const bool ok = planAndPublish();
      if (!ok) {
        RCLCPP_WARN(get_logger(), "No path found after edited occupied map refresh.");
      }
    }
  }

  static double markerResolution(const visualization_msgs::msg::Marker & msg)
  {
    const double sx = msg.scale.x > 0.0 ? msg.scale.x : 0.0;
    const double sy = msg.scale.y > 0.0 ? msg.scale.y : 0.0;
    const double sz = msg.scale.z > 0.0 ? msg.scale.z : 0.0;
    if (sx <= 0.0 && sy <= 0.0 && sz <= 0.0) {
      return 0.0;
    }
    if (sx > 0.0) {
      return sx;
    }
    if (sy > 0.0) {
      return sy;
    }
    return sz;
  }

  void publishCurrentOctomap()
  {
    if (!octree_ || !octomap_pub_) {
      return;
    }

    octomap_msgs::msg::Octomap msg;
    msg.header.stamp = now();
    msg.header.frame_id = get_parameter("frame_id").as_string();
    if (!octomap_msgs::fullMapToMsg(*octree_, msg)) {
      RCLCPP_WARN(get_logger(), "Failed to publish edited OctoMap message.");
      return;
    }
    octomap_pub_->publish(msg);
  }

  void onStart(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    start_point_ = *msg;
    has_start_ = true;
    RCLCPP_INFO(
      get_logger(), "Start set to [%.3f, %.3f, %.3f]",
      msg->point.x, msg->point.y, msg->point.z);
  }

  void onGoal(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    goal_point_ = *msg;
    has_goal_ = true;
    RCLCPP_INFO(
      get_logger(), "Goal set to [%.3f, %.3f, %.3f]",
      msg->point.x, msg->point.y, msg->point.z);
    tryPlan();
  }

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_pose_ = *msg;
    has_goal_pose_ = true;
    const double yaw = std::atan2(
      2.0 * (msg->pose.orientation.w * msg->pose.orientation.z +
      msg->pose.orientation.x * msg->pose.orientation.y),
      1.0 - 2.0 * (
        msg->pose.orientation.y * msg->pose.orientation.y +
        msg->pose.orientation.z * msg->pose.orientation.z));
    RCLCPP_INFO(
      get_logger(), "Goal pose yaw set to %.1f deg in frame %s",
      yaw * 180.0 / M_PI,
      msg->header.frame_id.empty() ? get_parameter("frame_id").as_string().c_str() :
      msg->header.frame_id.c_str());

    // GUI publishes goal_point and goal_pose back-to-back. Replan here so the
    // newest terminal orientation is reflected in /planned_path.
    tryPlan();
  }

  void tryPlan()
  {
    if (!map_ready_ || !has_start_ || !has_goal_ || planning_in_progress_) {
      return;
    }
    planning_in_progress_ = true;
    ++plan_seq_;
    const bool ok = planAndPublish();
    planning_in_progress_ = false;
    if (!ok) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "A* planning failed.");
    } else {
      last_success_seq_ = plan_seq_;
    }
  }

  void clearNavigationStateAndPublish(const std::string & frame_id)
  {
    has_start_ = false;
    has_goal_ = false;
    has_goal_pose_ = false;
    planning_in_progress_ = false;
    ++plan_seq_;

    const auto stamp = now();
    const std::string resolved_frame =
      frame_id.empty() ? get_parameter("frame_id").as_string() : frame_id;

    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = stamp;
    empty_path.header.frame_id = resolved_frame;
    path_pub_->publish(empty_path);

    visualization_msgs::msg::Marker delete_path;
    delete_path.header = empty_path.header;
    delete_path.ns = "jie_path";
    delete_path.id = 0;
    delete_path.action = visualization_msgs::msg::Marker::DELETE;
    path_marker_pub_->publish(delete_path);

    RCLCPP_INFO(get_logger(), "Cleared start, goal, and previous path after OctoMap update.");
  }

  GridIndex worldToGrid(double x, double y, double z) const
  {
    const double r = octree_->getResolution();
    return GridIndex{
      static_cast<int>(std::floor(x / r)),
      static_cast<int>(std::floor(y / r)),
      static_cast<int>(std::floor(z / r))};
  }

  octomap::point3d gridToWorld(const GridIndex & idx) const
  {
    const double r = octree_->getResolution();
    return octomap::point3d(
      static_cast<float>((static_cast<double>(idx.x) + 0.5) * r),
      static_cast<float>((static_cast<double>(idx.y) + 0.5) * r),
      static_cast<float>((static_cast<double>(idx.z) + 0.5) * r));
  }

  bool isInsideMetricBounds(const GridIndex & idx) const
  {
    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);
    const auto p = gridToWorld(idx);
    return p.x() >= static_cast<float>(min_x) && p.x() <= static_cast<float>(max_x) &&
           p.y() >= static_cast<float>(min_y) && p.y() <= static_cast<float>(max_y) &&
           p.z() >= static_cast<float>(min_z) && p.z() <= static_cast<float>(max_z);
  }

  bool hasGroundSupport(
    const GridIndex & idx, bool strict_direct_ground_support, int support_xy_radius_cells,
    int support_depth_cells) const
  {
    if (strict_direct_ground_support) {
      GridIndex below{idx.x, idx.y, idx.z - 1};
      if (!isInsideMetricBounds(below)) {
        return false;
      }
      const auto p = gridToWorld(below);
      const octomap::OcTreeNode * node = octree_->search(p);
      return node && octree_->isNodeOccupied(node);
    }

    for (int dz = 1; dz <= std::max(1, support_depth_cells); ++dz) {
      for (int dx = -support_xy_radius_cells; dx <= support_xy_radius_cells; ++dx) {
        for (int dy = -support_xy_radius_cells; dy <= support_xy_radius_cells; ++dy) {
          GridIndex below{idx.x + dx, idx.y + dy, idx.z - dz};
          if (!isInsideMetricBounds(below)) {
            continue;
          }
          const auto p = gridToWorld(below);
          const octomap::OcTreeNode * node = octree_->search(p);
          if (node && octree_->isNodeOccupied(node)) {
            return true;
          }
        }
      }
    }
    return false;
  }

  bool isOccupiedCell(const GridIndex & idx) const
  {
    if (!isInsideMetricBounds(idx)) {
      return false;
    }
    const auto p = gridToWorld(idx);
    const octomap::OcTreeNode * node = octree_->search(p);
    return node && octree_->isNodeOccupied(node);
  }

  bool hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const
  {
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
        if (!isInsideMetricBounds(n)) {
          continue;
        }
        if (!isOccupiedCell(n)) {
          return true;
        }
      }
    }
    return false;
  }

  bool hasSameLevelNeighborWithOccupiedBelow(const GridIndex & idx) const
  {
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
        if (!isInsideMetricBounds(n)) {
          continue;
        }
        const GridIndex n_below{n.x, n.y, n.z - 1};
        if (!isInsideMetricBounds(n_below)) {
          continue;
        }
        if (isOccupiedCell(n_below)) {
          return true;
        }
      }
    }
    return false;
  }

  bool hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const
  {
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const GridIndex n{idx.x + dx, idx.y + dy, idx.z};
        if (!isInsideMetricBounds(n)) {
          continue;
        }
        const GridIndex n_above1{n.x, n.y, n.z + 1};
        if (!isInsideMetricBounds(n_above1)) {
          continue;
        }
        if (isOccupiedCell(n_above1)) {
          return true;
        }
      }
    }
    return false;
  }

  void rebuildPreblockedCells()
  {
    preblocked_cells_.clear();
    if (!octree_) {
      return;
    }

    std::unordered_set<GridIndex, GridIndexHash> candidates;
    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (!octree_->isNodeOccupied(*it)) {
        continue;
      }
      const GridIndex occ = worldToGrid(it.getX(), it.getY(), it.getZ());
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          candidates.insert(GridIndex{occ.x + dx, occ.y + dy, occ.z});
        }
      }
    }

    for (const auto & c : candidates) {
      if (!isInsideMetricBounds(c)) {
        continue;
      }
      if (isOccupiedCell(c)) {
        continue;
      }
      const GridIndex below0{c.x, c.y, c.z - 1};
      const bool below0_occ = isInsideMetricBounds(below0) && isOccupiedCell(below0);
      if (below0_occ && hasSameLevelNeighborWithOccupiedAbove(c)) {
        preblocked_cells_.insert(c);
        continue;
      }
      const GridIndex above1{c.x, c.y, c.z + 1};
      const bool above1_occ = isInsideMetricBounds(above1) && isOccupiedCell(above1);
      if (!hasNonOccupiedNeighborSameLevel(c)) {
        continue;
      }
      if (above1_occ) {
        continue;
      }
      const GridIndex below1{c.x, c.y, c.z - 1};
      if (!isInsideMetricBounds(below1)) {
        continue;
      }
      const bool below1_non_occupied = !isOccupiedCell(below1);
      if (below1_non_occupied) {
        preblocked_cells_.insert(c);
      }
    }

    for (const auto & c : external_preblocked_cells_) {
      if (isInsideMetricBounds(c) && !isOccupiedCell(c)) {
        preblocked_cells_.insert(c);
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Preprocess mask rebuilt. preblocked_cells=%zu external=%zu",
      preblocked_cells_.size(), external_preblocked_cells_.size());
    publishPreblockedCellsMarker();
  }

  void onExternalPreblockedMarker(const visualization_msgs::msg::Marker::SharedPtr msg)
  {
    external_preblocked_cells_.clear();
    if (!octree_) {
      return;
    }

    for (const auto & point : msg->points) {
      const GridIndex idx = worldToGrid(point.x, point.y, point.z);
      if (isInsideMetricBounds(idx) && !isOccupiedCell(idx)) {
        external_preblocked_cells_.insert(idx);
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Received external preblocked marker. cells=%zu",
      external_preblocked_cells_.size());

    rebuildPreblockedCells();
    rebuildDerivedLayers();
    rebuildPreblockedCostmap();

    if (map_ready_ && has_start_ && has_goal_) {
      const bool ok = planAndPublish();
      if (!ok) {
        RCLCPP_WARN(get_logger(), "No path found after external preblocked update.");
      }
    }
  }

  void rebuildPreblockedCostmap()
  {
    preblocked_costmap_.clear();
    if (!octree_) {
      return;
    }
    const bool enable = get_parameter("enable_preblocked_costmap").as_bool();
    if (!enable) {
      return;
    }

    const int radius_cells = std::max(
      1, static_cast<int>(get_parameter("preblocked_costmap_radius_cells").as_int()));
    const double denom = static_cast<double>(radius_cells) + 1.0;

    for (const auto & c : preblocked_cells_) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
          for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }
            const GridIndex n{c.x + dx, c.y + dy, c.z + dz};
            if (!isInsideMetricBounds(n)) {
              continue;
            }
            if (traversable_cells_.find(n) == traversable_cells_.end()) {
              continue;
            }
            if (preblocked_cells_.find(n) != preblocked_cells_.end()) {
              continue;
            }
            const double d = std::sqrt(
              static_cast<double>(dx * dx + dy * dy + dz * dz));
            if (d > static_cast<double>(radius_cells)) {
              continue;
            }
            const double cst = std::max(0.0, (denom - d) / denom);
            auto it = preblocked_costmap_.find(n);
            if (it == preblocked_costmap_.end() || cst > it->second) {
              preblocked_costmap_[n] = cst;
            }
          }
        }
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Preblocked costmap rebuilt. cells=%zu radius=%d",
      preblocked_costmap_.size(), radius_cells);
    publishRiskCostCloud();
  }

  double getPreblockedCost(const GridIndex & idx) const
  {
    const auto it = preblocked_costmap_.find(idx);
    if (it == preblocked_costmap_.end()) {
      return 0.0;
    }
    return it->second;
  }

  void publishCellSetMarker(
    const std::unordered_set<GridIndex, GridIndexHash> & cells,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & publisher,
    const std::string & ns, float r_color, float g_color, float b_color, float a_color) const
  {
    if (!octree_ || !publisher) {
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = get_parameter("frame_id").as_string();
    marker.ns = ns;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    const double resolution = octree_->getResolution();
    marker.scale.x = resolution;
    marker.scale.y = resolution;
    marker.scale.z = resolution;
    marker.color.r = r_color;
    marker.color.g = g_color;
    marker.color.b = b_color;
    marker.color.a = a_color;
    marker.points.reserve(cells.size());
    for (const auto & c : cells) {
      const auto p = gridToWorld(c);
      geometry_msgs::msg::Point q;
      q.x = p.x();
      q.y = p.y();
      q.z = p.z();
      marker.points.push_back(q);
    }
    publisher->publish(marker);
  }

  void publishPreblockedCellsMarker()
  {
    publishCellSetMarker(
      preblocked_cells_, preblocked_marker_pub_, "preblocked_cells", 0.15F, 0.35F, 1.0F, 0.95F);
  }

  void publishRiskCostCloud() const
  {
    if (!risk_cost_pub_) {
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = now();
    cloud_msg.header.frame_id = get_parameter("frame_id").as_string();

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(preblocked_costmap_.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(cloud_msg, "intensity");

    for (const auto & entry : preblocked_costmap_) {
      const auto p = gridToWorld(entry.first);
      *iter_x = p.x();
      *iter_y = p.y();
      *iter_z = p.z();
      *iter_i = static_cast<float>(entry.second);
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_i;
    }

    risk_cost_pub_->publish(cloud_msg);
  }

  void rebuildDerivedLayers()
  {
    traversable_cells_.clear();
    if (!octree_) {
      return;
    }

    const bool require_ground_support = get_parameter("require_ground_support").as_bool();
    const bool strict_direct_ground_support =
      get_parameter("strict_direct_ground_support").as_bool();
    const int support_xy_radius_cells = get_parameter("ground_support_xy_radius_cells").as_int();
    const int support_depth_cells = get_parameter("ground_support_depth_cells").as_int();
    const double robot_radius = get_parameter("robot_radius").as_double();
    const bool lowest_traversable_only = get_parameter("lowest_traversable_only").as_bool();

    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);
    const GridIndex min_idx = worldToGrid(min_x, min_y, min_z);
    const GridIndex max_idx = worldToGrid(max_x, max_y, max_z);

    for (int x = min_idx.x; x <= max_idx.x; ++x) {
      for (int y = min_idx.y; y <= max_idx.y; ++y) {
        for (int z = min_idx.z; z <= max_idx.z; ++z) {
          const GridIndex idx{x, y, z};
          if (!isInsideMetricBounds(idx) || isOccupiedCell(idx)) {
            continue;
          }
          if (isCellTraversable(
              idx, robot_radius, require_ground_support, strict_direct_ground_support,
              support_xy_radius_cells, support_depth_cells))
          {
            traversable_cells_.insert(idx);
            if (lowest_traversable_only) {
              break;
            }
          }
        }
      }
    }

    publishCellSetMarker(
      traversable_cells_, traversable_marker_pub_, "traversable_cells", 0.20F, 0.95F, 0.55F,
      0.55F);
  }

  bool isCellTraversable(
    const GridIndex & idx, double robot_radius, bool require_ground_support,
    bool strict_direct_ground_support,
    int support_xy_radius_cells, int support_depth_cells) const
  {
    if (!isInsideMetricBounds(idx)) {
      return false;
    }

    if (require_ground_support &&
      !hasGroundSupport(
        idx, strict_direct_ground_support, support_xy_radius_cells, support_depth_cells))
    {
      return false;
    }

    for (int z = idx.z - 1; z >= 0; --z) {
      const GridIndex below_idx{idx.x, idx.y, z};
      if (isOccupiedCell(below_idx)) {
        break;
      }
      if (preblocked_cells_.find(below_idx) != preblocked_cells_.end()) {
        return false;
      }
    }

    const octomap::point3d center = gridToWorld(idx);
    const double r = octree_->getResolution();
    const int n = std::max(1, static_cast<int>(std::ceil(robot_radius / r)));
    const double radius_sq = robot_radius * robot_radius;

    // Collision check for vehicle body volume (same height and above),
    // while allowing occupied support cells below. Apply the same footprint
    // rule to preblocked cells so a cell is rejected if the vehicle radius
    // overlaps any preblocked voxel.
    for (int dx = -n; dx <= n; ++dx) {
      for (int dy = -n; dy <= n; ++dy) {
        for (int dz = 0; dz <= n; ++dz) {
          const double dist_x = static_cast<double>(dx) * r;
          const double dist_y = static_cast<double>(dy) * r;
          const double dist_z = static_cast<double>(dz) * r;
          const double dist_sq = dist_x * dist_x + dist_y * dist_y + dist_z * dist_z;
          if (dist_sq > radius_sq) {
            continue;
          }
          const octomap::point3d p(
            center.x() + static_cast<float>(dx * r),
            center.y() + static_cast<float>(dy * r),
            center.z() + static_cast<float>(dz * r));
          const GridIndex nearby_idx = worldToGrid(p.x(), p.y(), p.z());
          if (preblocked_cells_.find(nearby_idx) != preblocked_cells_.end()) {
            return false;
          }
          const octomap::OcTreeNode * node = octree_->search(p);
          if (node && octree_->isNodeOccupied(node)) {
            return false;
          }
        }
      }
    }
    return true;
  }

  bool findNearestFreeCell(
    const GridIndex & seed, double robot_radius, int radius_cells, bool require_ground_support,
    bool strict_direct_ground_support, int support_xy_radius_cells, int support_depth_cells,
    GridIndex & out) const
  {
    if (isCellTraversable(
        seed, robot_radius, require_ground_support, strict_direct_ground_support,
        support_xy_radius_cells, support_depth_cells))
    {
      out = seed;
      return true;
    }

    for (int r = 1; r <= radius_cells; ++r) {
      for (int dz = 0; dz <= r; ++dz) {
        for (int dx = -r; dx <= r; ++dx) {
          for (int dy = -r; dy <= r; ++dy) {
            if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) {
              continue;
            }

            GridIndex c1{seed.x + dx, seed.y + dy, seed.z + dz};
            if (isCellTraversable(
                c1, robot_radius, require_ground_support, strict_direct_ground_support,
                support_xy_radius_cells, support_depth_cells))
            {
              out = c1;
              return true;
            }

            if (dz > 0) {
              GridIndex c2{seed.x + dx, seed.y + dy, seed.z - dz};
              if (isCellTraversable(
                  c2, robot_radius, require_ground_support, strict_direct_ground_support,
                  support_xy_radius_cells, support_depth_cells))
              {
                out = c2;
                return true;
              }
            }
          }
        }
      }
    }
    return false;
  }

  std::vector<GridIndex> make26Directions() const
  {
    std::vector<GridIndex> dirs;
    dirs.reserve(26);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          dirs.push_back(GridIndex{dx, dy, dz});
        }
      }
    }
    return dirs;
  }

  std::vector<GridIndex> reconstructPath(
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
    GridIndex current) const
  {
    std::vector<GridIndex> path;
    path.push_back(current);
    while (came_from.find(current) != came_from.end()) {
      current = came_from.at(current);
      path.push_back(current);
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  bool planAndPublish()
  {
    const double robot_radius = get_parameter("robot_radius").as_double();
    const int max_iterations = get_parameter("max_iterations").as_int();
    const int snap_radius = get_parameter("snap_search_radius_cells").as_int();
    const bool require_ground_support = get_parameter("require_ground_support").as_bool();
    const bool strict_direct_ground_support =
      get_parameter("strict_direct_ground_support").as_bool();
    const int support_xy_radius_cells = get_parameter("ground_support_xy_radius_cells").as_int();
    const int support_depth_cells = get_parameter("ground_support_depth_cells").as_int();
    const bool enable_preblocked_costmap = get_parameter("enable_preblocked_costmap").as_bool();
    const double preblocked_costmap_weight = get_parameter("preblocked_costmap_weight").as_double();
    const std::string frame_id = get_parameter("frame_id").as_string();

    const GridIndex start_raw = worldToGrid(
      start_point_.point.x, start_point_.point.y, start_point_.point.z);
    const GridIndex goal_raw = worldToGrid(
      goal_point_.point.x, goal_point_.point.y, goal_point_.point.z);

    GridIndex start = start_raw;
    GridIndex goal = goal_raw;
    const bool start_ok = findNearestFreeCell(
      start_raw, robot_radius, snap_radius, require_ground_support, strict_direct_ground_support,
      support_xy_radius_cells, support_depth_cells, start);
    const bool goal_ok = findNearestFreeCell(
      goal_raw, robot_radius, snap_radius, require_ground_support, strict_direct_ground_support,
      support_xy_radius_cells, support_depth_cells, goal);

    if (!start_ok) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Start is occupied/out of map and no nearby free cell.");
      return false;
    }
    if (!goal_ok) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Goal is occupied/out of map and no nearby free cell.");
      return false;
    }

    if (!(start == start_raw)) {
      const auto p = gridToWorld(start);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "Start snapped to free cell: [%.2f, %.2f, %.2f]",
        p.x(), p.y(), p.z());
    }
    if (!(goal == goal_raw)) {
      const auto p = gridToWorld(goal);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "Goal snapped to free cell: [%.2f, %.2f, %.2f]",
        p.x(), p.y(), p.z());
    }

    std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
    std::unordered_map<GridIndex, double, GridIndexHash> g_score;
    std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
    std::unordered_set<GridIndex, GridIndexHash> closed_set;

    g_score[start] = 0.0;
    open_set.push(QueueNode{start, euclidean(start, goal), 0.0});

    const std::vector<GridIndex> directions = make26Directions();
    int iters = 0;

    while (!open_set.empty() && iters < max_iterations) {
      const QueueNode current = open_set.top();
      open_set.pop();
      ++iters;

      if (closed_set.find(current.idx) != closed_set.end()) {
        continue;
      }
      closed_set.insert(current.idx);

      if (current.idx == goal) {
        const auto cells = reconstructPath(came_from, current.idx);
        publishPath(cells, frame_id);
        RCLCPP_INFO(get_logger(), "A* path found in %d iterations. waypoints=%zu", iters, cells.size());
        return true;
      }

      for (const auto & d : directions) {
        GridIndex nbr{current.idx.x + d.x, current.idx.y + d.y, current.idx.z + d.z};
        if (closed_set.find(nbr) != closed_set.end()) {
          continue;
        }
        if (!isCellTraversable(
            nbr, robot_radius, require_ground_support, strict_direct_ground_support,
            support_xy_radius_cells, support_depth_cells))
        {
          continue;
        }
        const double step_cost = euclidean(current.idx, nbr);
        double tentative_g = current.g + step_cost;
        if (enable_preblocked_costmap) {
          tentative_g += preblocked_costmap_weight * getPreblockedCost(nbr);
        }

        auto g_it = g_score.find(nbr);
        if (g_it == g_score.end() || tentative_g < g_it->second) {
          came_from[nbr] = current.idx;
          g_score[nbr] = tentative_g;
          const double f = tentative_g + euclidean(nbr, goal);
          open_set.push(QueueNode{nbr, f, tentative_g});
        }
      }
    }

    return false;
  }

  void publishPath(const std::vector<GridIndex> & cells, const std::string & frame_id)
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = frame_id;
    path_msg.poses.reserve(cells.size());

    visualization_msgs::msg::Marker m;
    m.header = path_msg.header;
    m.ns = "jie_path";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.32;
    m.color.r = 0.1F;
    m.color.g = 0.95F;
    m.color.b = 0.95F;
    m.color.a = 1.0F;
    m.pose.orientation.w = 1.0;

    for (std::size_t i = 0; i < cells.size(); ++i) {
      const auto & c = cells[i];
      const auto p = gridToWorld(c);

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.position.z = p.z();
      pose.pose.orientation.w = 1.0;
      if (has_goal_pose_ && i + 1 == cells.size()) {
        pose.pose.orientation = goal_pose_.pose.orientation;
      }
      path_msg.poses.push_back(pose);

      geometry_msgs::msg::Point q;
      q.x = p.x();
      q.y = p.y();
      q.z = p.z();
      m.points.push_back(q);
    }

    path_pub_->publish(path_msg);
    path_marker_pub_->publish(m);
  }

  bool map_ready_;
  bool has_start_;
  bool has_goal_;
  bool has_goal_pose_{false};
  bool planning_in_progress_;
  std::uint64_t plan_seq_;
  std::uint64_t last_success_seq_;
  geometry_msgs::msg::PointStamped start_point_;
  geometry_msgs::msg::PointStamped goal_point_;
  geometry_msgs::msg::PoseStamped goal_pose_;
  std::shared_ptr<octomap::OcTree> octree_;
  std::unordered_set<GridIndex, GridIndexHash> traversable_cells_;
  std::unordered_set<GridIndex, GridIndexHash> preblocked_cells_;
  std::unordered_set<GridIndex, GridIndexHash> external_preblocked_cells_;
  std::unordered_map<GridIndex, double, GridIndexHash> preblocked_costmap_;
  std::uint64_t last_octomap_hash_{0};

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr external_preblocked_sub_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr edited_occupied_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr preblocked_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traversable_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr risk_cost_pub_;
  rclcpp::Service<jie_map_msgs::srv::GetNavigationMapMeta>::SharedPtr get_meta_srv_;
  rclcpp::Service<jie_map_msgs::srv::ExportNavigationSnapshot>::SharedPtr
    export_snapshot_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JiePathNode>());
  rclcpp::shutdown();
  return 0;
}
