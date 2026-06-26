#include <cmath>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif
#include "tf2_ros/transform_broadcaster.h"

class TestMapToOdomTfNode : public rclcpp::Node
{
public:
  TestMapToOdomTfNode()
  : Node("test_map_to_odom_tf_node"), start_time_(now())
  {
    declare_parameter<double>("radius", 2.0);
    declare_parameter<double>("orbit_period", 20.0);
    declare_parameter<double>("spin_rate", 0.8);
    declare_parameter<std::string>("parent_frame", "map");
    declare_parameter<std::string>("child_frame", "odom");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&TestMapToOdomTfNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "test_map_to_odom_tf_node started. parent=%s child=%s radius=%.2f orbit_period=%.2f spin_rate=%.2f",
      get_parameter("parent_frame").as_string().c_str(),
      get_parameter("child_frame").as_string().c_str(),
      get_parameter("radius").as_double(),
      get_parameter("orbit_period").as_double(),
      get_parameter("spin_rate").as_double());
  }

private:
  void onTimer()
  {
    const double radius = get_parameter("radius").as_double();
    const double orbit_period = std::max(0.1, get_parameter("orbit_period").as_double());
    const double spin_rate = get_parameter("spin_rate").as_double();
    const std::string parent_frame = get_parameter("parent_frame").as_string();
    const std::string child_frame = get_parameter("child_frame").as_string();

    const double t = (now() - start_time_).seconds();
    const double orbit_angle = 2.0 * M_PI * t / orbit_period;
    const double yaw = orbit_angle + spin_rate * t;

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = child_frame;
    tf_msg.transform.translation.x = radius * std::cos(orbit_angle);
    tf_msg.transform.translation.y = radius * std::sin(orbit_angle);
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    tf_msg.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(tf_msg);
  }

  rclcpp::Time start_time_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestMapToOdomTfNode>());
  rclcpp::shutdown();
  return 0;
}
