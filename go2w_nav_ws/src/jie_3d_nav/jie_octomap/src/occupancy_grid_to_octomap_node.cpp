#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_set>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"

class OccupancyGridToOctomapNode : public rclcpp::Node
{
public:
  OccupancyGridToOctomapNode()
  : Node("occupancy_grid_to_octomap")
  {
    declare_parameter<std::string>("grid_topic", "/import_occupancy_grid");
    declare_parameter<std::string>("octomap_topic", "/octomap");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<double>("octomap_resolution", 0.2);
    declare_parameter<double>("wall_height_m", 1.0);
    declare_parameter<double>("floor_z_m", 0.0);
    declare_parameter<int>("occupied_threshold", 50);

    const auto grid_topic = get_parameter("grid_topic").as_string();
    const auto octomap_topic = get_parameter("octomap_topic").as_string();

    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      octomap_topic, rclcpp::QoS(1).transient_local().reliable());
    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      grid_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&OccupancyGridToOctomapNode::onGrid, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "occupancy_grid_to_octomap started. grid_topic=%s octomap_topic=%s",
      grid_topic.c_str(), octomap_topic.c_str());
  }

private:
  struct XYKey
  {
    int x;
    int y;

    bool operator==(const XYKey & other) const
    {
      return x == other.x && y == other.y;
    }
  };

  struct XYKeyHash
  {
    std::size_t operator()(const XYKey & key) const
    {
      const std::size_t h1 = std::hash<int>{}(key.x);
      const std::size_t h2 = std::hash<int>{}(key.y);
      return h1 ^ (h2 << 1);
    }
  };

  void onGrid(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const double grid_resolution = msg->info.resolution;
    const double octomap_resolution = get_parameter("octomap_resolution").as_double();
    if (grid_resolution <= 0.0 || octomap_resolution <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Grid and OctoMap resolutions must be positive.");
      return;
    }

    octomap::OcTree tree(octomap_resolution);
    const double wall_height = std::max(
      octomap_resolution, get_parameter("wall_height_m").as_double());
    const double floor_z = get_parameter("floor_z_m").as_double();
    const int occupied_threshold = get_parameter("occupied_threshold").as_int();
    const int height_cells = std::max(
      1, static_cast<int>(std::ceil(wall_height / octomap_resolution)));

    const auto & origin = msg->info.origin.position;
    const std::size_t width = msg->info.width;
    const std::size_t height = msg->info.height;

    std::unordered_set<XYKey, XYKeyHash> known_cells;
    std::unordered_set<XYKey, XYKeyHash> occupied_cells;

    for (std::size_t y = 0; y < height; ++y) {
      for (std::size_t x = 0; x < width; ++x) {
        const std::size_t index = y * width + x;
        if (index >= msg->data.size()) {
          continue;
        }
        const int8_t value = msg->data[index];
        if (value < 0) {
          continue;
        }

        const double world_x = origin.x + (static_cast<double>(x) + 0.5) * grid_resolution;
        const double world_y = origin.y + (static_cast<double>(y) + 0.5) * grid_resolution;
        const int grid_x = static_cast<int>(std::floor(world_x / octomap_resolution));
        const int grid_y = static_cast<int>(std::floor(world_y / octomap_resolution));
        const XYKey key{grid_x, grid_y};
        known_cells.insert(key);

        if (value >= occupied_threshold) {
          occupied_cells.insert(key);
        }
      }
    }

    for (const auto & key : known_cells) {
      const double world_x = (static_cast<double>(key.x) + 0.5) * octomap_resolution;
      const double world_y = (static_cast<double>(key.y) + 0.5) * octomap_resolution;
      tree.updateNode(world_x, world_y, floor_z + 0.5 * octomap_resolution, true);
    }

    for (const auto & key : occupied_cells) {
      const double world_x = (static_cast<double>(key.x) + 0.5) * octomap_resolution;
      const double world_y = (static_cast<double>(key.y) + 0.5) * octomap_resolution;
      for (int z = 1; z <= height_cells; ++z) {
        const double world_z = floor_z + (static_cast<double>(z) + 0.5) * octomap_resolution;
        tree.updateNode(world_x, world_y, world_z, true);
      }
    }

    tree.updateInnerOccupancy();

    octomap_msgs::msg::Octomap octomap_msg;
    octomap_msg.header.stamp = now();
    octomap_msg.header.frame_id = msg->header.frame_id.empty() ?
      get_parameter("frame_id").as_string() : msg->header.frame_id;

    if (!octomap_msgs::binaryMapToMsg(tree, octomap_msg)) {
      RCLCPP_ERROR(get_logger(), "Failed to convert OcTree to Octomap message.");
      return;
    }

    octomap_pub_->publish(octomap_msg);
    RCLCPP_INFO(
      get_logger(),
      "Published OctoMap from OccupancyGrid. width=%zu height=%zu occupied_cells=%zu "
      "grid_resolution=%.3f octomap_resolution=%.3f",
      width, height, occupied_cells.size(), grid_resolution, octomap_resolution);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridToOctomapNode>());
  rclcpp::shutdown();
  return 0;
}
