#include <memory>
#include <sstream>
#include <string>

#include "octomap/AbstractOcTree.h"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class OctomapToOccupiedMarkersNode : public rclcpp::Node
{
public:
  OctomapToOccupiedMarkersNode()
  : Node("octomap_to_occupied_markers")
  {
    declare_parameter<std::string>("octomap_topic", "/octomap");
    declare_parameter<std::string>("marker_topic", "/octomap_occupied_markers");
    declare_parameter<std::string>("frame_id", "map");

    const auto octomap_topic = get_parameter("octomap_topic").as_string();
    const auto marker_topic = get_parameter("marker_topic").as_string();

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      marker_topic, rclcpp::QoS(1).transient_local().reliable());

    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&OctomapToOccupiedMarkersNode::onOctomap, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "octomap_to_occupied_markers started. octomap_topic=%s marker_topic=%s",
      octomap_topic.c_str(), marker_topic.c_str());
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

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = msg->header.stamp;
    marker.header.frame_id = msg->header.frame_id.empty() ?
      get_parameter("frame_id").as_string() : msg->header.frame_id;
    marker.ns = "occupied_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = oc_tree->getResolution();
    marker.scale.y = oc_tree->getResolution();
    marker.scale.z = oc_tree->getResolution();
    marker.color.r = 0.95F;
    marker.color.g = 0.45F;
    marker.color.b = 0.15F;
    marker.color.a = 0.95F;

    for (auto it = oc_tree->begin_leafs(); it != oc_tree->end_leafs(); ++it) {
      if (!oc_tree->isNodeOccupied(*it)) {
        continue;
      }
      geometry_msgs::msg::Point point;
      point.x = it.getX();
      point.y = it.getY();
      point.z = it.getZ();
      marker.points.push_back(point);
    }

    marker_pub_->publish(marker);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Published occupied marker from OctoMap: %zu voxels", marker.points.size());
  }

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapToOccupiedMarkersNode>());
  rclcpp::shutdown();
  return 0;
}
