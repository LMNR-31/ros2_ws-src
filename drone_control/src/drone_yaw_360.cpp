#include <chrono>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "drone_control/msg/yaw_override.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

class DroneYaw360OverrideAngle : public rclcpp::Node
{
public:
  DroneYaw360OverrideAngle()
  : Node("drone_yaw_360")
  {
    this->declare_parameter<std::string>("uav_ns", "/uav1");
    this->declare_parameter<double>("yaw_rate", 1.0);          // rad/s
    this->declare_parameter<double>("yaw_tolerance", 0.05);    // radianos
    this->declare_parameter<double>("yaw_target_delta", 2*M_PI);

    uav_ns_ = this->get_parameter("uav_ns").as_string();
    yaw_rate_ = this->get_parameter("yaw_rate").as_double();
    yaw_tolerance_ = this->get_parameter("yaw_tolerance").as_double();
    yaw_target_delta_ = this->get_parameter("yaw_target_delta").as_double();

    pub_ = this->create_publisher<drone_control::msg::YawOverride>(
      uav_ns_ + "/yaw_override/cmd", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      uav_ns_ + "/mavros/local_position/odom", 10,
      std::bind(&DroneYaw360OverrideAngle::odom_cb, this, std::placeholders::_1)
    );

    timer_ = this->create_wall_timer(
      50ms, std::bind(&DroneYaw360OverrideAngle::timer_cb, this)
    );

    RCLCPP_INFO(this->get_logger(), "Aguardando odometria inicial...");
  }

private:
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    Eigen::Quaterniond q(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z
    );
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    current_yaw_ = euler[2];

    if (!odom_ok_) {
      last_yaw_ = current_yaw_;
      odom_ok_ = true;
    }
  }

  void timer_cb()
  {
    if (!odom_ok_)
      return;

    if (!started_) {
      yaw_initial_ = current_yaw_;
      last_yaw_ = current_yaw_;
      accumulated_yaw_ = 0.0;
      started_ = true;
      publish_enable();
      RCLCPP_INFO(this->get_logger(), "Yaw inicial: %.3f rad (%.1f°), alvo: %.3f rad (%.1f°)",
                  yaw_initial_, yaw_initial_ * 180 / M_PI,
                  yaw_target_delta_, yaw_target_delta_ * 180 / M_PI);
      return;
    }

    double delta = normalize_angle(current_yaw_ - last_yaw_);
    // só soma se o sentido bater com o sentido do comando
    double dir = (yaw_target_delta_ >= 0.0) ? 1.0 : -1.0;
    if (delta * dir < 0) {
      // pulou "para trás" por ruído ou oscilação, ignore esta amostra
      delta = 0;
    }
    accumulated_yaw_ += std::abs(delta);
    last_yaw_ = current_yaw_;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                         "Delta atual: %.3f, yaw acumulado: %.3f/%.3f rad",
                         delta, accumulated_yaw_, std::abs(yaw_target_delta_));

    if (accumulated_yaw_ >= std::abs(yaw_target_delta_) - yaw_tolerance_ && !already_disabled_) {
      publish_disable();
      RCLCPP_INFO(this->get_logger(), 
        "Giro completo! Acumulado: %.3f rad (%.1f°), tolerância: %.3f rad",
        accumulated_yaw_, accumulated_yaw_*180/M_PI, yaw_tolerance_);
    }
  }

  void publish_enable()
  {
    drone_control::msg::YawOverride msg;
    msg.enable = true;
    msg.yaw_rate = (yaw_target_delta_ >= 0.0) ? std::abs(yaw_rate_) : -std::abs(yaw_rate_);
    msg.timeout = 15.0f; // Segurança extra.
    pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "[yaw_override] ENABLE: yaw_rate=%.2f", msg.yaw_rate);
  }

  void publish_disable()
  {
    drone_control::msg::YawOverride msg;
    msg.enable = false;
    msg.yaw_rate = 0.0;
    msg.timeout = 0.0;
    pub_->publish(msg);
    already_disabled_ = true;
    RCLCPP_INFO(this->get_logger(), "[yaw_override] DISABLE!");
    rclcpp::shutdown();
  }

  // Normaliza para [-pi,pi]
  double normalize_angle(double a)
  {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
  }

  std::string uav_ns_;
  double yaw_rate_;
  double yaw_tolerance_;
  double yaw_target_delta_;
  double yaw_initial_{0.0};
  double current_yaw_{0.0};
  double last_yaw_{0.0};
  double accumulated_yaw_{0.0};
  bool started_{false};
  bool odom_ok_{false};
  bool already_disabled_{false};
  rclcpp::Publisher<drone_control::msg::YawOverride>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneYaw360OverrideAngle>());
  rclcpp::shutdown();
  return 0;
}