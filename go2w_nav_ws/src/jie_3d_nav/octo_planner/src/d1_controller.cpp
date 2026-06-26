#include <algorithm>
#include <cmath>
#include <cctype>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/utils.h"
#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

class D1ControllerNode : public rclcpp::Node
{
public:
  D1ControllerNode()
  : Node("d1_controller"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    target_index_(0),
    pose_adjusting_(false),
    goal_reached_(false)
  {
    declare_parameter<std::string>("path_topic", "/planned_path");
    declare_parameter<std::string>("start_navigation_topic", "/start_navigation");
    declare_parameter<std::string>("stop_navigation_topic", "/stop_navigation");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("manual_cmd_vel_topic", "/web_cmd_vel");
    declare_parameter<std::string>("tracking_point_marker_topic", "/tracking_point_marker");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_footprint");
    declare_parameter<std::string>(
      "base_frame_candidates", "odin1_base_link,base_link,base_footprint");
    declare_parameter<std::string>("robot_center_offset_frame", "odin1_base_link");
    declare_parameter<double>("robot_center_offset_x", -0.18);
    declare_parameter<double>("robot_center_offset_y", 0.0);
    declare_parameter<double>("robot_center_offset_z", 0.0);
    declare_parameter<bool>("require_start_command", true);
    declare_parameter<double>("control_frequency", 20.0);
    declare_parameter<double>("lookahead_distance", 0.20);
    declare_parameter<double>("tracking_point_reached_xy_tolerance", 0.20);
    declare_parameter<double>("tracking_point_marker_scale", 0.28);
    declare_parameter<bool>("enable_tracking_debug_view", true);
    declare_parameter<int>("tracking_debug_view_size_px", 640);
    declare_parameter<double>("tracking_debug_view_pixels_per_meter", 80.0);
    declare_parameter<double>("tracking_debug_view_frequency", 10.0);
    declare_parameter<double>("goal_position_tolerance", 0.05);
    declare_parameter<double>("goal_yaw_tolerance", 0.10);
    declare_parameter<double>("linear_gain", 1.5);
    declare_parameter<double>("lateral_gain", 1.5);
    declare_parameter<double>("heading_gain", 2.5);
    declare_parameter<double>("cross_track_angular_gain", 1.0);
    declare_parameter<double>("final_yaw_gain", 0.5);
    declare_parameter<bool>("enable_lateral_motion", true);
    declare_parameter<double>("max_linear_speed", 0.60);
    declare_parameter<double>("max_lateral_speed", 0.60);
    declare_parameter<double>("max_angular_speed", 1.50);
    declare_parameter<bool>("align_final_yaw", true);
    declare_parameter<double>("linear_deadband", 0.05);
    declare_parameter<double>("lateral_deadband", 0.05);
    declare_parameter<double>("angular_deadband", 0.05);

    tf_buffer_.setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));

    const auto path_topic = get_parameter("path_topic").as_string();
    const auto start_navigation_topic = get_parameter("start_navigation_topic").as_string();
    const auto stop_navigation_topic = get_parameter("stop_navigation_topic").as_string();
    const auto cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
    const auto manual_cmd_vel_topic = get_parameter("manual_cmd_vel_topic").as_string();
    const auto tracking_point_marker_topic =
      get_parameter("tracking_point_marker_topic").as_string();
    const double control_frequency = get_parameter("control_frequency").as_double();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&D1ControllerNode::onPath, this, std::placeholders::_1));
    start_navigation_sub_ = create_subscription<std_msgs::msg::Bool>(
      start_navigation_topic, rclcpp::QoS(10).reliable(),
      std::bind(&D1ControllerNode::onStartNavigation, this, std::placeholders::_1));
    stop_navigation_sub_ = create_subscription<std_msgs::msg::Bool>(
      stop_navigation_topic, rclcpp::QoS(10).reliable(),
      std::bind(&D1ControllerNode::onStopNavigation, this, std::placeholders::_1));
    manual_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      manual_cmd_vel_topic, rclcpp::QoS(10).reliable(),
      std::bind(&D1ControllerNode::onManualCmdVel, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    tracking_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      tracking_point_marker_topic, rclcpp::QoS(1).transient_local().reliable());

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, control_frequency));
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&D1ControllerNode::onControlTimer, this));
    const double debug_view_frequency = get_parameter("tracking_debug_view_frequency").as_double();
    const auto debug_period = std::chrono::duration<double>(1.0 / std::max(1.0, debug_view_frequency));
    debug_view_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(debug_period),
      std::bind(&D1ControllerNode::renderTrackingDebugView, this));

    RCLCPP_INFO(
      get_logger(),
      "d1_controller started. path=%s start_navigation=%s stop_navigation=%s cmd_vel=%s "
      "manual_cmd_vel=%s tracking_marker=%s map_frame=%s base_frame=%s require_start_command=%s",
      path_topic.c_str(), start_navigation_topic.c_str(), stop_navigation_topic.c_str(),
      cmd_vel_topic.c_str(), manual_cmd_vel_topic.c_str(), tracking_point_marker_topic.c_str(),
      get_parameter("map_frame").as_string().c_str(),
      get_parameter("base_frame").as_string().c_str(),
      get_parameter("require_start_command").as_bool() ? "true" : "false");
  }

private:
  void onPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) {
      pending_plan_.clear();
      clearActivePlan();
      RCLCPP_WARN(get_logger(), "Received empty planned_path.");
      return;
    }

    if (get_parameter("require_start_command").as_bool()) {
      pending_plan_ = msg->poses;
      clearActivePlan();
      publishCmd(geometry_msgs::msg::Twist());
      RCLCPP_INFO(
        get_logger(),
        "Received planned_path with %zu poses. Waiting for /start_navigation confirmation.",
        pending_plan_.size());
      return;
    }

    activatePlan(msg->poses);
  }

  void onStartNavigation(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      stopNavigation("Navigation start denied/cancelled. Holding position.");
      return;
    }

    if (pending_plan_.empty()) {
      RCLCPP_WARN(get_logger(), "Start navigation requested, but no pending planned_path is available.");
      return;
    }

    try {
      activatePlan(pending_plan_);
      pending_plan_.clear();
    } catch (const std::exception & ex) {
      stopNavigation("Start navigation failed. Holding position.");
      RCLCPP_ERROR(get_logger(), "Start navigation exception: %s", ex.what());
    }
  }

  void onStopNavigation(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }
    stopNavigation("Stop navigation requested. Path tracking aborted and zero velocity sent.");
  }

  void onManualCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (isNavigationActive() && isNonZeroTwist(*msg)) {
      pending_plan_.clear();
      clearActivePlan();
      RCLCPP_INFO(
        get_logger(),
        "Manual web velocity received while navigating. Path tracking aborted without zero burst.");
    } else if (isNavigationActive()) {
      return;
    }

    publishCmd(*msg);
  }

  void activatePlan(const std::vector<geometry_msgs::msg::PoseStamped> & plan)
  {
    global_plan_ = plan;
    target_index_ = findInitialTargetIndex3D();
    pose_adjusting_ = false;
    goal_reached_ = global_plan_.empty();
    publishTrackingPointMarker();
    RCLCPP_INFO(
      get_logger(), "Navigation execution started with %zu poses. initial_target_index=%d",
      global_plan_.size(), target_index_);
  }

  void clearActivePlan()
  {
    global_plan_.clear();
    target_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = true;
    clearTrackingPointMarker();
  }

  void stopNavigation(const char * log_message)
  {
    pending_plan_.clear();
    clearActivePlan();
    publishZeroBurst();
    RCLCPP_INFO(get_logger(), "%s", log_message);
  }

  bool isNavigationActive() const
  {
    return !global_plan_.empty() && !goal_reached_;
  }

  static bool isNonZeroTwist(const geometry_msgs::msg::Twist & twist)
  {
    constexpr double epsilon = 1.0e-6;
    return std::abs(twist.linear.x) > epsilon ||
           std::abs(twist.linear.y) > epsilon ||
           std::abs(twist.linear.z) > epsilon ||
           std::abs(twist.angular.x) > epsilon ||
           std::abs(twist.angular.y) > epsilon ||
           std::abs(twist.angular.z) > epsilon;
  }

  void publishZeroBurst()
  {
    const auto zero = geometry_msgs::msg::Twist();
    for (int i = 0; i < 5; ++i) {
      publishCmd(zero);
    }
  }

  void onControlTimer()
  {
    try {
      onControlTimerImpl();
    } catch (const std::exception & ex) {
      stopNavigation("Control loop exception. Path tracking aborted.");
      RCLCPP_ERROR(get_logger(), "Control loop exception: %s", ex.what());
    }
  }

  void onControlTimerImpl()
  {
    if (global_plan_.empty()) {
      return;
    }

    if (pose_adjusting_) {
      geometry_msgs::msg::PoseStamped final_pose_base;
      if (!transformToBase(global_plan_.back(), final_pose_base)) {
        return;
      }
      trackFinalPose(final_pose_base, global_plan_.back());
      return;
    }

    TrackingTarget target;
    if (!selectTrackingTarget(target)) {
      return;
    }

    if (isFinalTrackingPointReached(target)) {
      pose_adjusting_ = true;
      RCLCPP_INFO(get_logger(), "Final tracking point reached. Switching to final yaw adjustment.");
      geometry_msgs::msg::PoseStamped final_pose_base;
      if (!transformToBase(global_plan_.back(), final_pose_base)) {
        return;
      }
      trackFinalPose(final_pose_base, global_plan_.back());
      return;
    }

    geometry_msgs::msg::Twist cmd_vel;
    const double linear_gain = get_parameter("linear_gain").as_double();
    const double lateral_gain = get_parameter("lateral_gain").as_double();
    const double heading_gain = get_parameter("heading_gain").as_double();
    const double cross_track_angular_gain =
      get_parameter("cross_track_angular_gain").as_double();
    const bool enable_lateral_motion = get_parameter("enable_lateral_motion").as_bool();
    const double heading_error = std::atan2(target.base_y, std::max(1.0e-6, target.base_x));
    cmd_vel.linear.x = clamp(
      target.base_x * linear_gain,
      -get_parameter("max_linear_speed").as_double(),
      get_parameter("max_linear_speed").as_double());
    if (enable_lateral_motion) {
      cmd_vel.linear.y = clamp(
        target.base_y * lateral_gain,
        -get_parameter("max_lateral_speed").as_double(),
        get_parameter("max_lateral_speed").as_double());
    } else {
      cmd_vel.linear.y = 0.0;
    }
    cmd_vel.angular.z = clamp(
      heading_error * heading_gain +
      target.base_y * cross_track_angular_gain,
      -get_parameter("max_angular_speed").as_double(),
      get_parameter("max_angular_speed").as_double());
    cmd_vel.linear.x = applyDeadband(cmd_vel.linear.x, get_parameter("linear_deadband").as_double());
    cmd_vel.linear.y = applyDeadband(cmd_vel.linear.y, get_parameter("lateral_deadband").as_double());
    cmd_vel.angular.z = applyDeadband(cmd_vel.angular.z, get_parameter("angular_deadband").as_double());
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Track target in %s: x=%.3f y=%.3f heading_error=%.3f cmd=(%.3f, %.3f, %.3f)",
      get_parameter("base_frame").as_string().c_str(),
      target.base_x,
      target.base_y,
      heading_error,
      cmd_vel.linear.x,
      cmd_vel.linear.y,
      cmd_vel.angular.z);
    publishCmd(cmd_vel);
  }

  struct RobotPose2D
  {
    double x;
    double y;
    double z;
    double yaw;
  };

  struct TrackingTarget
  {
    double base_x;
    double base_y;
  };

  bool isFinalTrackingPointReached(const TrackingTarget & target) const
  {
    if (global_plan_.empty() || target_index_ != static_cast<int>(global_plan_.size()) - 1) {
      return false;
    }

    const double target_xy_dist = std::hypot(target.base_x, target.base_y);
    return target_xy_dist < get_parameter("tracking_point_reached_xy_tolerance").as_double();
  }

  int findInitialTargetIndex3D()
  {
    if (global_plan_.empty()) {
      return 0;
    }

    RobotPose2D robot_pose;
    if (!lookupRobotPose2D(robot_pose)) {
      RCLCPP_WARN(get_logger(), "Failed to get robot pose. Start tracking from path index 0.");
      return 0;
    }

    int nearest_index = 0;
    double nearest_dist_sq = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < global_plan_.size(); ++i) {
      const auto & point = global_plan_[i].pose.position;
      const double dx = point.x - robot_pose.x;
      const double dy = point.y - robot_pose.y;
      const double dz = point.z - robot_pose.z;
      const double dist_sq = dx * dx + dy * dy + dz * dz;
      if (dist_sq < nearest_dist_sq) {
        nearest_dist_sq = dist_sq;
        nearest_index = static_cast<int>(i);
      }
    }
    return nearest_index;
  }

  bool selectTrackingTarget(TrackingTarget & target)
  {
    if (global_plan_.empty()) {
      return false;
    }

    RobotPose2D robot_pose;
    if (!lookupRobotPose2D(robot_pose)) {
      return false;
    }

    const double reached_tolerance =
      get_parameter("tracking_point_reached_xy_tolerance").as_double();
    const double target_dist = xyDistanceToPlanPoint(robot_pose, target_index_);
    if (target_dist < reached_tolerance && target_index_ < static_cast<int>(global_plan_.size()) - 1) {
      int next_index = target_index_;
      for (int i = target_index_ + 1; i < static_cast<int>(global_plan_.size()); ++i) {
        if (xyDistanceToPlanPoint(robot_pose, i) > reached_tolerance) {
          next_index = i;
          break;
        }
        if (i == static_cast<int>(global_plan_.size()) - 1) {
          next_index = i;
        }
      }
      if (next_index != target_index_) {
        target_index_ = next_index;
        publishTrackingPointMarker();
      }
    }

    const auto & target_point = global_plan_[static_cast<std::size_t>(target_index_)].pose.position;
    const double dx_map = target_point.x - robot_pose.x;
    const double dy_map = target_point.y - robot_pose.y;
    const double cos_yaw = std::cos(robot_pose.yaw);
    const double sin_yaw = std::sin(robot_pose.yaw);
    target.base_x = cos_yaw * dx_map + sin_yaw * dy_map;
    target.base_y = -sin_yaw * dx_map + cos_yaw * dy_map;
    return true;
  }

  bool lookupRobotPose2D(RobotPose2D & robot_pose)
  {
    const std::string map_frame = get_parameter("map_frame").as_string();
    std::string last_error;
    for (const auto & base_frame : getBaseFrameCandidates()) {
      try {
        const auto tf = tf_buffer_.lookupTransform(
          map_frame, base_frame, tf2::TimePointZero, tf2::durationFromSec(0.05));
        if (active_base_frame_ != base_frame) {
          active_base_frame_ = base_frame;
          RCLCPP_INFO(get_logger(), "Using robot base frame for tracking: %s", base_frame.c_str());
        }
        robot_pose.x = tf.transform.translation.x;
        robot_pose.y = tf.transform.translation.y;
        robot_pose.z = tf.transform.translation.z;
        robot_pose.yaw = tf2::getYaw(tf.transform.rotation);
        applyRobotCenterOffset(base_frame, robot_pose);
        return true;
      } catch (const tf2::TransformException & ex) {
        last_error = ex.what();
      }
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Lookup robot pose from %s failed for all base_frame candidates. Last error: %s",
      map_frame.c_str(), last_error.c_str());
    return false;
  }

  double xyDistanceToPlanPoint(const RobotPose2D & robot_pose, int index) const
  {
    const auto & point = global_plan_[static_cast<std::size_t>(index)].pose.position;
    const double dx = point.x - robot_pose.x;
    const double dy = point.y - robot_pose.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  void publishTrackingPointMarker()
  {
    if (global_plan_.empty()) {
      clearTrackingPointMarker();
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = get_parameter("map_frame").as_string();
    marker.header.stamp = now();
    marker.ns = "d1_tracking_point";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = global_plan_[static_cast<std::size_t>(target_index_)].pose;
    const double scale = get_parameter("tracking_point_marker_scale").as_double();
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = 0.1f;
    marker.color.g = 0.65f;
    marker.color.b = 1.0f;
    marker.color.a = 0.95f;
    tracking_marker_pub_->publish(marker);
  }

  void clearTrackingPointMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = get_parameter("map_frame").as_string();
    marker.header.stamp = now();
    marker.ns = "d1_tracking_point";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    tracking_marker_pub_->publish(marker);
  }

  void renderTrackingDebugView()
  {
    try {
      renderTrackingDebugViewImpl();
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "OpenCV tracking debug view exception: %s", ex.what());
    }
  }

  void renderTrackingDebugViewImpl()
  {
    if (!get_parameter("enable_tracking_debug_view").as_bool() || debug_view_disabled_) {
      return;
    }
    if (global_plan_.empty()) {
      return;
    }

    RobotPose2D robot_pose;
    if (!lookupRobotPose2D(robot_pose)) {
      return;
    }

    const int image_size =
      std::max(240, static_cast<int>(get_parameter("tracking_debug_view_size_px").as_int()));
    const double pixels_per_meter =
      std::max(10.0, get_parameter("tracking_debug_view_pixels_per_meter").as_double());
    const cv::Point center(image_size / 2, image_size / 2);
    cv::Mat image(image_size, image_size, CV_8UC3, cv::Scalar(18, 24, 28));

    cv::line(image, cv::Point(center.x, 0), cv::Point(center.x, image_size), cv::Scalar(48, 64, 70), 1);
    cv::line(image, cv::Point(0, center.y), cv::Point(image_size, center.y), cv::Scalar(48, 64, 70), 1);
    cv::arrowedLine(
      image, center, cv::Point(center.x, center.y - 58), cv::Scalar(230, 230, 230), 2,
      cv::LINE_AA, 0, 0.25);
    cv::circle(image, center, 8, cv::Scalar(230, 230, 230), -1, cv::LINE_AA);
    cv::putText(
      image, "robot +X", cv::Point(center.x + 10, center.y - 62),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(230, 230, 230), 1, cv::LINE_AA);

    std::vector<cv::Point> projected_points;
    projected_points.reserve(global_plan_.size());
    for (const auto & pose : global_plan_) {
      projected_points.push_back(projectPlanPoint(robot_pose, pose, center, pixels_per_meter));
    }

    for (std::size_t i = 1; i < projected_points.size(); ++i) {
      cv::line(image, projected_points[i - 1], projected_points[i], cv::Scalar(120, 120, 120), 1, cv::LINE_AA);
    }
    for (const auto & point : projected_points) {
      cv::circle(image, point, 3, cv::Scalar(90, 210, 90), -1, cv::LINE_AA);
    }

    if (target_index_ >= 0 && target_index_ < static_cast<int>(projected_points.size())) {
      const auto target_point = projected_points[static_cast<std::size_t>(target_index_)];
      cv::circle(image, target_point, 12, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
      cv::circle(image, target_point, 4, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
    }

    double final_yaw_error = 0.0;
    if (drawFinalGoalYaw(image, robot_pose, center, pixels_per_meter, final_yaw_error)) {
      char goal_yaw_text[160];
      std::snprintf(
        goal_yaw_text, sizeof(goal_yaw_text), "goal yaw err: %.1f deg",
        final_yaw_error * 180.0 / M_PI);
      cv::putText(
        image, goal_yaw_text,
        cv::Point(16, 108), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 230, 255), 1,
        cv::LINE_AA);
    }

    cv::putText(
      image, "tracking index: " + std::to_string(target_index_),
      cv::Point(16, 28), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(80, 190, 255), 2,
      cv::LINE_AA);
    char pose_text[160];
    std::snprintf(
      pose_text, sizeof(pose_text), "robot map: x=%.2f y=%.2f yaw=%.1f deg",
      robot_pose.x, robot_pose.y, robot_pose.yaw * 180.0 / M_PI);
    cv::putText(
      image, pose_text,
      cv::Point(16, 56), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(220, 220, 220), 1,
      cv::LINE_AA);
    char cmd_text[160];
    std::snprintf(
      cmd_text, sizeof(cmd_text), "cmd vel: x=%.3f y=%.3f wz=%.3f",
      last_cmd_vel_.linear.x, last_cmd_vel_.linear.y, last_cmd_vel_.angular.z);
    cv::putText(
      image, cmd_text,
      cv::Point(16, 82), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(120, 230, 255), 1,
      cv::LINE_AA);
    cv::putText(
      image, "top = robot forward, red = current target",
      cv::Point(16, image_size - 18), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(180, 200, 210), 1,
      cv::LINE_AA);

    try {
      cv::imshow("d1_controller_xy_tracking", image);
      cv::waitKey(1);
    } catch (const cv::Exception & ex) {
      debug_view_disabled_ = true;
      RCLCPP_WARN(get_logger(), "Disable OpenCV tracking debug view: %s", ex.what());
    }
  }

  cv::Point projectPlanPoint(
    const RobotPose2D & robot_pose,
    const geometry_msgs::msg::PoseStamped & pose,
    const cv::Point & center,
    double pixels_per_meter) const
  {
    const double dx_map = pose.pose.position.x - robot_pose.x;
    const double dy_map = pose.pose.position.y - robot_pose.y;
    const double cos_yaw = std::cos(robot_pose.yaw);
    const double sin_yaw = std::sin(robot_pose.yaw);
    const double base_x = cos_yaw * dx_map + sin_yaw * dy_map;
    const double base_y = -sin_yaw * dx_map + cos_yaw * dy_map;

    return cv::Point(
      static_cast<int>(std::round(center.x - base_y * pixels_per_meter)),
      static_cast<int>(std::round(center.y - base_x * pixels_per_meter)));
  }

  bool drawFinalGoalYaw(
    cv::Mat & image,
    const RobotPose2D & robot_pose,
    const cv::Point & center,
    double pixels_per_meter,
    double & yaw_error) const
  {
    if (global_plan_.empty()) {
      return false;
    }

    const auto & final_pose = global_plan_.back();
    const cv::Point final_point =
      projectPlanPoint(robot_pose, final_pose, center, pixels_per_meter);
    const double goal_yaw = tf2::getYaw(final_pose.pose.orientation);
    yaw_error = normalizeAngle(goal_yaw - robot_pose.yaw);
    const double goal_yaw_in_robot_frame = yaw_error;
    const double arrow_length_px = std::max(26.0, pixels_per_meter * 0.35);
    const cv::Point arrow_end(
      static_cast<int>(std::round(final_point.x - std::sin(goal_yaw_in_robot_frame) * arrow_length_px)),
      static_cast<int>(std::round(final_point.y - std::cos(goal_yaw_in_robot_frame) * arrow_length_px)));

    cv::circle(image, final_point, 10, cv::Scalar(0, 230, 255), 2, cv::LINE_AA);
    cv::arrowedLine(
      image, final_point, arrow_end, cv::Scalar(0, 230, 255), 2,
      cv::LINE_AA, 0, 0.30);
    return true;
  }

  void trackFinalPose(
    const geometry_msgs::msg::PoseStamped & final_pose_base,
    const geometry_msgs::msg::PoseStamped & final_pose_map)
  {
    geometry_msgs::msg::Twist cmd_vel;
    const double linear_gain = get_parameter("linear_gain").as_double();
    const double lateral_gain = get_parameter("lateral_gain").as_double();
    const double final_yaw_gain = get_parameter("final_yaw_gain").as_double();
    const bool enable_lateral_motion = get_parameter("enable_lateral_motion").as_bool();
    const double max_linear = get_parameter("max_linear_speed").as_double();
    const double max_lateral = get_parameter("max_lateral_speed").as_double();
    const double max_angular = get_parameter("max_angular_speed").as_double();

    cmd_vel.linear.x = clamp(final_pose_base.pose.position.x * linear_gain, -max_linear, max_linear);
    if (enable_lateral_motion) {
      cmd_vel.linear.y =
        clamp(final_pose_base.pose.position.y * lateral_gain, -max_lateral, max_lateral);
    } else {
      cmd_vel.linear.y = 0.0;
    }
    cmd_vel.linear.x = applyDeadband(cmd_vel.linear.x, get_parameter("linear_deadband").as_double());
    cmd_vel.linear.y = applyDeadband(cmd_vel.linear.y, get_parameter("lateral_deadband").as_double());

    const bool align_final_yaw = get_parameter("align_final_yaw").as_bool();
    double final_yaw_error = 0.0;
    if (align_final_yaw) {
      if (!computeFinalYawErrorXY(final_pose_map, final_yaw_error)) {
        return;
      }
      cmd_vel.angular.z = clamp(final_yaw_error * final_yaw_gain, -max_angular, max_angular);
      cmd_vel.angular.z = applyDeadband(cmd_vel.angular.z, get_parameter("angular_deadband").as_double());
    } else {
      cmd_vel.angular.z = 0.0;
    }

    const bool final_yaw_reached =
      !align_final_yaw || std::abs(final_yaw_error) < get_parameter("goal_yaw_tolerance").as_double();
    const double final_xy_dist = std::hypot(
      final_pose_base.pose.position.x, final_pose_base.pose.position.y);
    const bool final_position_reached =
      final_xy_dist < get_parameter("goal_position_tolerance").as_double();
    if (final_position_reached && final_yaw_reached) {
      finishNavigationAtGoal();
      return;
    }

    publishCmd(cmd_vel);
  }

  void finishNavigationAtGoal()
  {
    pending_plan_.clear();
    clearActivePlan();
    publishZeroBurst();
    RCLCPP_INFO(
      get_logger(),
      "Goal reached within position and yaw tolerances. Navigation finished and controller is idle.");
  }

  bool computeFinalYawErrorXY(
    const geometry_msgs::msg::PoseStamped & final_pose_in,
    double & yaw_error)
  {
    RobotPose2D robot_pose;
    if (!lookupRobotPose2D(robot_pose)) {
      return false;
    }

    const std::string map_frame = get_parameter("map_frame").as_string();
    geometry_msgs::msg::PoseStamped final_pose_map = final_pose_in;
    if (final_pose_map.header.frame_id.empty()) {
      final_pose_map.header.frame_id = map_frame;
    }
    final_pose_map.header.stamp = rclcpp::Time(0);

    try {
      if (final_pose_map.header.frame_id != map_frame) {
        final_pose_map = tf_buffer_.transform(final_pose_map, map_frame, tf2::durationFromSec(0.05));
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Transform final pose yaw %s -> %s failed: %s",
        final_pose_map.header.frame_id.c_str(), map_frame.c_str(), ex.what());
      return false;
    }

    const double goal_yaw = tf2::getYaw(final_pose_map.pose.orientation);
    yaw_error = normalizeAngle(goal_yaw - robot_pose.yaw);
    return true;
  }

  void publishCmd(const geometry_msgs::msg::Twist & cmd_vel)
  {
    last_cmd_vel_ = cmd_vel;
    cmd_pub_->publish(cmd_vel);
  }

  bool transformToBase(
    const geometry_msgs::msg::PoseStamped & pose_in,
    geometry_msgs::msg::PoseStamped & pose_out)
  {
    RobotPose2D unused_pose;
    if (active_base_frame_.empty() && !lookupRobotPose2D(unused_pose)) {
      return false;
    }
    const std::string base_frame =
      active_base_frame_.empty() ? get_parameter("base_frame").as_string() : active_base_frame_;
    const std::string map_frame = get_parameter("map_frame").as_string();

    geometry_msgs::msg::PoseStamped stamped = pose_in;
    if (stamped.header.frame_id.empty()) {
      stamped.header.frame_id = map_frame;
    }
    stamped.header.stamp = rclcpp::Time(0);

    try {
      pose_out = tf_buffer_.transform(stamped, base_frame, tf2::durationFromSec(0.05));
      applyRobotCenterOffsetToRelativePose(base_frame, pose_out);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Transform %s -> %s failed: %s",
        stamped.header.frame_id.c_str(), base_frame.c_str(), ex.what());
      return false;
    }
  }

  std::vector<std::string> getBaseFrameCandidates() const
  {
    std::vector<std::string> candidates;
    const auto add_candidate = [&candidates](const std::string & frame) {
      if (!frame.empty() && std::find(candidates.begin(), candidates.end(), frame) == candidates.end()) {
        candidates.push_back(frame);
      }
    };

    add_candidate(get_parameter("base_frame").as_string());
    for (const auto & frame : splitCsv(get_parameter("base_frame_candidates").as_string())) {
      add_candidate(frame);
    }
    return candidates;
  }

  bool shouldApplyRobotCenterOffset(const std::string & frame) const
  {
    return frame == get_parameter("robot_center_offset_frame").as_string();
  }

  void applyRobotCenterOffset(const std::string & frame, RobotPose2D & robot_pose) const
  {
    if (!shouldApplyRobotCenterOffset(frame)) {
      return;
    }

    const double offset_x = get_parameter("robot_center_offset_x").as_double();
    const double offset_y = get_parameter("robot_center_offset_y").as_double();
    const double offset_z = get_parameter("robot_center_offset_z").as_double();
    const double cos_yaw = std::cos(robot_pose.yaw);
    const double sin_yaw = std::sin(robot_pose.yaw);
    robot_pose.x += cos_yaw * offset_x - sin_yaw * offset_y;
    robot_pose.y += sin_yaw * offset_x + cos_yaw * offset_y;
    robot_pose.z += offset_z;
  }

  void applyRobotCenterOffsetToRelativePose(
    const std::string & frame,
    geometry_msgs::msg::PoseStamped & pose) const
  {
    if (!shouldApplyRobotCenterOffset(frame)) {
      return;
    }

    pose.pose.position.x -= get_parameter("robot_center_offset_x").as_double();
    pose.pose.position.y -= get_parameter("robot_center_offset_y").as_double();
    pose.pose.position.z -= get_parameter("robot_center_offset_z").as_double();
  }

  static std::vector<std::string> splitCsv(const std::string & text)
  {
    std::vector<std::string> parts;
    std::string current;
    for (const char ch : text) {
      if (ch == ',') {
        const auto trimmed = trim(current);
        if (!trimmed.empty()) {
          parts.push_back(trimmed);
        }
        current.clear();
      } else {
        current.push_back(ch);
      }
    }

    const auto trimmed = trim(current);
    if (!trimmed.empty()) {
      parts.push_back(trimmed);
    }
    return parts;
  }

  static std::string trim(const std::string & text)
  {
    std::size_t first = 0;
    while (first < text.size() && std::isspace(static_cast<unsigned char>(text[first]))) {
      ++first;
    }

    std::size_t last = text.size();
    while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1]))) {
      --last;
    }
    return text.substr(first, last - first);
  }

  static double clamp(double value, double min_value, double max_value)
  {
    return std::max(min_value, std::min(max_value, value));
  }

  static double applyDeadband(double value, double deadband)
  {
    return std::abs(value) < deadband ? 0.0 : value;
  }

  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_navigation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_navigation_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tracking_marker_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr debug_view_timer_;

  std::vector<geometry_msgs::msg::PoseStamped> global_plan_;
  std::vector<geometry_msgs::msg::PoseStamped> pending_plan_;
  int target_index_;
  bool pose_adjusting_;
  bool goal_reached_;
  bool debug_view_disabled_{false};
  std::string active_base_frame_;
  geometry_msgs::msg::Twist last_cmd_vel_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<D1ControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
