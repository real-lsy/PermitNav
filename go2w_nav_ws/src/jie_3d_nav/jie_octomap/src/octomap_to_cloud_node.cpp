#include <memory>
#include <sstream>
#include <string>

#include "octomap/AbstractOcTree.h"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class OctomapToCloudNode : public rclcpp::Node
{
public:
  OctomapToCloudNode()
  : Node("octomap_to_cloud")
  {
    declare_parameter<std::string>("octomap_topic", "/octomap");
    declare_parameter<std::string>("cloud_topic", "/octomap_points");
    declare_parameter<std::string>("frame_id", "map");

    const auto octomap_topic = get_parameter("octomap_topic").as_string();
    const auto cloud_topic = get_parameter("cloud_topic").as_string();

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::QoS(1).transient_local().reliable());

    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&OctomapToCloudNode::onOctomap, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "octomap_to_cloud started. octomap_topic=%s, cloud_topic=%s",
      octomap_topic.c_str(), cloud_topic.c_str());
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

    std::size_t occupied_count = 0;
    for (auto it = oc_tree->begin_leafs(); it != oc_tree->end_leafs(); ++it) {
      if (oc_tree->isNodeOccupied(*it)) {
        ++occupied_count;
      }
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = msg->header.stamp;
    cloud_msg.header.frame_id = msg->header.frame_id.empty() ?
      get_parameter("frame_id").as_string() : msg->header.frame_id;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(occupied_count);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    for (auto it = oc_tree->begin_leafs(); it != oc_tree->end_leafs(); ++it) {
      if (!oc_tree->isNodeOccupied(*it)) {
        continue;
      }
      *iter_x = static_cast<float>(it.getX());
      *iter_y = static_cast<float>(it.getY());
      *iter_z = static_cast<float>(it.getZ());
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    cloud_pub_->publish(cloud_msg);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000, "Published PointCloud2 from OctoMap: %zu points",
      occupied_count);
  }

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapToCloudNode>());
  rclcpp::shutdown();
  return 0;
}
