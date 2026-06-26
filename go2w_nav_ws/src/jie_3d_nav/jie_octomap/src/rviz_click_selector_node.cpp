#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class RvizClickSelectorNode : public rclcpp::Node
{
public:
  RvizClickSelectorNode()
  : Node("rviz_click_selector"), expect_start_(true)
  {
    declare_parameter<std::string>("clicked_topic", "/clicked_point");
    declare_parameter<std::string>("marker_topic", "/selection_markers");
    declare_parameter<std::string>("start_topic", "/start_point");
    declare_parameter<std::string>("goal_topic", "/goal_point");
    declare_parameter<double>("arrow_height", 0.6);
    declare_parameter<double>("arrow_length", 0.7);
    declare_parameter<double>("shaft_diameter", 0.16);
    declare_parameter<double>("head_diameter", 0.32);
    declare_parameter<double>("head_length", 0.44);
    declare_parameter<double>("cube_size", 0.20);

    const auto clicked_topic = get_parameter("clicked_topic").as_string();
    const auto marker_topic = get_parameter("marker_topic").as_string();
    const auto start_topic = get_parameter("start_topic").as_string();
    const auto goal_topic = get_parameter("goal_topic").as_string();

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic, rclcpp::QoS(1).transient_local().reliable());
    start_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      start_topic, rclcpp::QoS(1).transient_local().reliable());
    goal_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      goal_topic, rclcpp::QoS(1).transient_local().reliable());
    clicked_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      clicked_topic, 10, std::bind(&RvizClickSelectorNode::onClickedPoint, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "rviz_click_selector started. clicked_topic=%s marker_topic=%s",
      clicked_topic.c_str(), marker_topic.c_str());
  }

private:
  void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (expect_start_) {
      start_point_ = *msg;
      has_start_ = true;
      RCLCPP_INFO(
        get_logger(), "Set START point: [%.3f, %.3f, %.3f]",
        msg->point.x, msg->point.y, msg->point.z);
      start_pub_->publish(*msg);
    } else {
      goal_point_ = *msg;
      has_goal_ = true;
      RCLCPP_INFO(
        get_logger(), "Set GOAL point: [%.3f, %.3f, %.3f]",
        msg->point.x, msg->point.y, msg->point.z);
      goal_pub_->publish(*msg);
    }
    expect_start_ = !expect_start_;
    publishMarkers();
  }

  visualization_msgs::msg::Marker makeArrow(
    int id, const geometry_msgs::msg::PointStamped & p, float r, float g, float b) const
  {
    const double arrow_height = get_parameter("arrow_height").as_double();
    const double arrow_length = get_parameter("arrow_length").as_double();

    visualization_msgs::msg::Marker m;
    m.header = p.header;
    m.ns = "rviz_selector";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = get_parameter("shaft_diameter").as_double();
    m.scale.y = get_parameter("head_diameter").as_double();
    m.scale.z = get_parameter("head_length").as_double();
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0F;
    m.pose.orientation.w = 1.0;

    geometry_msgs::msg::Point base = p.point;
    base.z += arrow_height;
    geometry_msgs::msg::Point tip = base;
    tip.z -= arrow_length;
    m.points.push_back(base);
    m.points.push_back(tip);
    return m;
  }

  visualization_msgs::msg::Marker makeCube(
    int id, const geometry_msgs::msg::PointStamped & p, float r, float g, float b) const
  {
    visualization_msgs::msg::Marker m;
    m.header = p.header;
    m.ns = "rviz_selector";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position = p.point;
    m.pose.orientation.w = 1.0;
    const double cube_size = get_parameter("cube_size").as_double();
    m.scale.x = cube_size;
    m.scale.y = cube_size;
    m.scale.z = cube_size;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 0.95F;
    return m;
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    if (has_start_) {
      arr.markers.push_back(makeArrow(0, start_point_, 0.1F, 0.95F, 0.1F));
      arr.markers.push_back(makeCube(2, start_point_, 0.1F, 0.95F, 0.1F));
    }
    if (has_goal_) {
      arr.markers.push_back(makeArrow(1, goal_point_, 0.95F, 0.1F, 0.1F));
      arr.markers.push_back(makeCube(3, goal_point_, 0.95F, 0.1F, 0.1F));
    }
    marker_pub_->publish(arr);
  }

  bool expect_start_;
  bool has_start_{false};
  bool has_goal_{false};
  geometry_msgs::msg::PointStamped start_point_;
  geometry_msgs::msg::PointStamped goal_point_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr start_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RvizClickSelectorNode>());
  rclcpp::shutdown();
  return 0;
}
