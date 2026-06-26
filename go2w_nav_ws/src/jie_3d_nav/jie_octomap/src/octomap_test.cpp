#include <cmath>
#include <memory>
#include <random>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class OctomapTestNode : public rclcpp::Node
{
public:
  OctomapTestNode()
  : Node("octomap_test")
  {
    declare_parameter<double>("resolution", 0.2);
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("octomap_topic", "/octomap");
    declare_parameter<std::string>("marker_topic", "/octomap_occupied_markers");

    const auto octomap_topic = get_parameter("octomap_topic").as_string();
    const auto marker_topic = get_parameter("marker_topic").as_string();

    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      octomap_topic, rclcpp::QoS(1).transient_local().reliable());
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      marker_topic, rclcpp::QoS(1).transient_local().reliable());

    generateMap();
    publishAll();

    // Re-publish periodically so late subscribers can always receive latest data.
    timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&OctomapTestNode::publishAll, this));
  }

private:
  void generateMap()
  {
    const double resolution = get_parameter("resolution").as_double();
    tree_ = std::make_shared<octomap::OcTree>(resolution);
    tree_->setProbHit(0.7);
    tree_->setProbMiss(0.4);
    tree_->setClampingThresMin(0.12);
    tree_->setClampingThresMax(0.97);

    const double x_min = -8.0;
    const double x_max = 8.0;
    const double y_min = -6.0;
    const double y_max = 6.0;

    const int ix_min = static_cast<int>(std::floor(x_min / resolution));
    const int ix_max = static_cast<int>(std::ceil(x_max / resolution));
    const int iy_min = static_cast<int>(std::floor(y_min / resolution));
    const int iy_max = static_cast<int>(std::ceil(y_max / resolution));

    // Continuous sloped ground, filled by voxel indices to avoid row gaps.
    for (int ix = ix_min; ix <= ix_max; ++ix) {
      const double x = (static_cast<double>(ix) + 0.5) * resolution;
      for (int iy = iy_min; iy <= iy_max; ++iy) {
        const double y = (static_cast<double>(iy) + 0.5) * resolution;
        const double gz = groundZ(x, y);
        const int iz_center = static_cast<int>(std::floor(gz / resolution));
        for (int iz = iz_center - 2; iz <= iz_center; ++iz) {
          const double z = (static_cast<double>(iz) + 0.5) * resolution;
          tree_->updateNode(octomap::point3d(x, y, z), true);
        }
      }
    }

    addVerticalWallX(1.5, -4.0, -0.8, 2.8, resolution);
    addVerticalWallX(4.0, 0.5, 4.5, 2.3, resolution);
    addVerticalWallY(-2.0, -6.0, -2.5, 2.6, resolution);
    addVerticalWallY(3.0, -1.0, 5.5, 2.0, resolution);

    // Random short pillars to enrich the scene.
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist_x(-5.5, 5.5);
    std::uniform_real_distribution<double> dist_y(-4.5, 4.5);
    for (int i = 0; i < 14; ++i) {
      const double px = dist_x(gen);
      const double py = dist_y(gen);
      const double base = groundZ(px, py);
      const double height = 0.8 + 0.15 * static_cast<double>(i % 5);
      for (double z = base; z <= base + height; z += resolution) {
        tree_->updateNode(octomap::point3d(px, py, z), true);
      }
    }

    tree_->updateInnerOccupancy();
    RCLCPP_INFO(get_logger(), "Generated random OctoMap for testing.");
  }

  double groundZ(double x, double y) const
  {
    return 0.10 * x + 0.03 * y;
  }

  void addVerticalWallX(
    double x_fixed, double y_start, double y_end, double wall_height, double resolution)
  {
    for (double y = y_start; y <= y_end; y += resolution) {
      const double base = groundZ(x_fixed, y);
      for (double z = base; z <= base + wall_height; z += resolution) {
        tree_->updateNode(octomap::point3d(x_fixed, y, z), true);
      }
    }
  }

  void addVerticalWallY(
    double y_fixed, double x_start, double x_end, double wall_height, double resolution)
  {
    for (double x = x_start; x <= x_end; x += resolution) {
      const double base = groundZ(x, y_fixed);
      for (double z = base; z <= base + wall_height; z += resolution) {
        tree_->updateNode(octomap::point3d(x, y_fixed, z), true);
      }
    }
  }

  void publishAll()
  {
    if (!tree_) {
      return;
    }

    const auto stamp = now();
    const std::string frame_id = get_parameter("frame_id").as_string();

    octomap_msgs::msg::Octomap octomap_msg;
    if (!octomap_msgs::binaryMapToMsg(*tree_, octomap_msg)) {
      RCLCPP_ERROR(get_logger(), "Failed to convert OcTree to octomap message.");
      return;
    }
    octomap_msg.header.stamp = stamp;
    octomap_msg.header.frame_id = frame_id;
    octomap_pub_->publish(octomap_msg);

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame_id;
    marker.ns = "occupied_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = tree_->getResolution();
    marker.scale.y = tree_->getResolution();
    marker.scale.z = tree_->getResolution();
    marker.color.r = 0.95F;
    marker.color.g = 0.45F;
    marker.color.b = 0.15F;
    marker.color.a = 0.95F;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);

    std::size_t occupied_count = 0;
    marker.points.reserve(tree_->size());
    for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
      if (!tree_->isNodeOccupied(*it)) {
        continue;
      }
      geometry_msgs::msg::Point p;
      p.x = it.getX();
      p.y = it.getY();
      p.z = it.getZ();
      marker.points.push_back(p);
      ++occupied_count;
    }
    marker_pub_->publish(marker);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000, "Published OctoMap (%zu occupied leaf voxels).",
      occupied_count);
  }

  std::shared_ptr<octomap::OcTree> tree_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapTestNode>());
  rclcpp::shutdown();
  return 0;
}
