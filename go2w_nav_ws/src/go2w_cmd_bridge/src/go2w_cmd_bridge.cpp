#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

using namespace std::chrono_literals;

class Go2WCmdBridge : public rclcpp::Node
{
public:
  Go2WCmdBridge()
  : Node("go2w_cmd_bridge")
  {
    this->declare_parameter<std::string>("network_interface", "enp2s0");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("enable_topic", "/go2w_cmd_enable");

    this->declare_parameter<double>("max_vx", 0.10);
    this->declare_parameter<double>("max_vy", 0.05);
    this->declare_parameter<double>("max_vyaw", 0.25);
    this->declare_parameter<double>("cmd_timeout", 0.5);

    this->declare_parameter<bool>("start_enabled", false);
    this->declare_parameter<bool>("send_stopmove_on_disable", true);
    this->declare_parameter<int>("speed_level", -1);  // -1 slow, 0 normal, 1 fast

    network_interface_ = this->get_parameter("network_interface").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();
    enable_topic_ = this->get_parameter("enable_topic").as_string();

    max_vx_ = this->get_parameter("max_vx").as_double();
    max_vy_ = this->get_parameter("max_vy").as_double();
    max_vyaw_ = this->get_parameter("max_vyaw").as_double();
    cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();

    enabled_.store(this->get_parameter("start_enabled").as_bool());
    send_stopmove_on_disable_ = this->get_parameter("send_stopmove_on_disable").as_bool();
    speed_level_ = this->get_parameter("speed_level").as_int();

    RCLCPP_WARN(this->get_logger(), "Initializing Unitree ChannelFactory on interface: %s",
                network_interface_.c_str());

    // 必须先初始化 ChannelFactory，再创建 SportClient。
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);

    sport_client_ = std::make_unique<unitree::robot::go2::SportClient>();
    sport_client_->SetTimeout(1.0f);
    sport_client_->Init();

    // 只设置速度档位，不切换步态。爬楼梯模式由你手动用遥控器/App切换。
    const int32_t ret_speed = sport_client_->SpeedLevel(speed_level_);
    RCLCPP_WARN(this->get_logger(), "SportClient SpeedLevel(%d) return: %d",
                speed_level_, ret_speed);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      10,
      std::bind(&Go2WCmdBridge::cmdVelCallback, this, std::placeholders::_1)
    );

    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      enable_topic_,
      10,
      std::bind(&Go2WCmdBridge::enableCallback, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(
      50ms,
      std::bind(&Go2WCmdBridge::watchdogLoop, this)
    );

    last_cmd_time_ = this->now();

    RCLCPP_WARN(this->get_logger(), "Go2W cmd bridge started.");
    RCLCPP_WARN(this->get_logger(), "Subscribing cmd_vel: %s", cmd_vel_topic_.c_str());
    RCLCPP_WARN(this->get_logger(), "Enable topic: %s", enable_topic_.c_str());
    RCLCPP_WARN(this->get_logger(), "Enabled at startup: %s", enabled_.load() ? "true" : "false");
    RCLCPP_WARN(this->get_logger(), "Safety limits: vx=%.3f, vy=%.3f, vyaw=%.3f",
                max_vx_, max_vy_, max_vyaw_);

    if (!enabled_.load()) {
      RCLCPP_WARN(this->get_logger(),
                  "Bridge is DISABLED. /cmd_vel will be ignored until /go2w_cmd_enable is true.");
    }
  }

  ~Go2WCmdBridge()
  {
    safeStop("node shutdown");
  }

private:
  static double clamp(double x, double lo, double hi)
  {
    return std::max(lo, std::min(hi, x));
  }

  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    enabled_.store(msg->data);

    if (msg->data) {
      RCLCPP_WARN(this->get_logger(), "Go2W command bridge ENABLED. Now forwarding /cmd_vel to Go2-W.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Go2W command bridge DISABLED. Stop sending cmd_vel to Go2-W.");
      safeStop("disabled by /go2w_cmd_enable");
    }
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = this->now();

    const double vx = clamp(msg->linear.x, -max_vx_, max_vx_);
    const double vy = clamp(msg->linear.y, -max_vy_, max_vy_);
    const double vyaw = clamp(msg->angular.z, -max_vyaw_, max_vyaw_);

    latest_vx_ = vx;
    latest_vy_ = vy;
    latest_vyaw_ = vyaw;

    // 核心逻辑：只有 enable=true 时，才把 /cmd_vel 发给狗。
    if (!enabled_.load()) {
      return;
    }

    sendMove(vx, vy, vyaw);
  }

  void watchdogLoop()
  {
    const double dt = (this->now() - last_cmd_time_).seconds();

    // 如果 enable 状态下超过 cmd_timeout 没有收到新的 /cmd_vel，就持续发 0 速度。
    if (enabled_.load() && dt > cmd_timeout_) {
      latest_vx_ = 0.0;
      latest_vy_ = 0.0;
      latest_vyaw_ = 0.0;
      sendMove(0.0, 0.0, 0.0);
    }
  }

  void sendMove(double vx, double vy, double vyaw)
  {
    if (!sport_client_) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "SportClient is not initialized."
      );
      return;
    }

    const int32_t ret = sport_client_->Move(
      static_cast<float>(vx),
      static_cast<float>(vy),
      static_cast<float>(vyaw)
    );

    if (ret != 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "SportClient Move(%.3f, %.3f, %.3f) failed, ret=%d",
        vx, vy, vyaw, ret
      );
    }
  }

  void safeStop(const std::string & reason)
  {
    RCLCPP_WARN(this->get_logger(), "Safe stop: %s", reason.c_str());

    try {
      if (!sport_client_) {
        return;
      }

      sport_client_->Move(0.0f, 0.0f, 0.0f);

      if (send_stopmove_on_disable_) {
        sport_client_->StopMove();
      }
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Exception during safeStop()");
    }
  }

private:
  std::string network_interface_;
  std::string cmd_vel_topic_;
  std::string enable_topic_;

  double max_vx_{0.10};
  double max_vy_{0.05};
  double max_vyaw_{0.25};
  double cmd_timeout_{0.5};

  bool send_stopmove_on_disable_{true};
  int speed_level_{-1};

  std::atomic<bool> enabled_{false};

  double latest_vx_{0.0};
  double latest_vy_{0.0};
  double latest_vyaw_{0.0};

  rclcpp::Time last_cmd_time_;

  std::unique_ptr<unitree::robot::go2::SportClient> sport_client_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<Go2WCmdBridge>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "go2w_cmd_bridge exception: " << e.what() << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
