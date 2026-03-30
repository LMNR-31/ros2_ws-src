#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class TakeoffWaypointPublisher8s : public rclcpp::Node
{
public:
  TakeoffWaypointPublisher8s() : Node("takeoff_waypoint_publisher_8s")
  {
    this->declare_parameter<double>("takeoff_altitude", 2.0);
    this->declare_parameter<double>("success_altitude_threshold", 1.5);
    this->declare_parameter<int>("max_attempts", 5);

    takeoff_altitude_ = this->get_parameter("takeoff_altitude").as_double();
    success_altitude_threshold_ = this->get_parameter("success_altitude_threshold").as_double();
    max_attempts_ = this->get_parameter("max_attempts").as_int();

    waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/mission_waypoints", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/uav1/mavros/local_position/odom", 10,
      std::bind(&TakeoffWaypointPublisher8s::odomCallback, this, _1));
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TakeoffWaypointPublisher8s::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
      "Aguardando primeira odometria... (takeoff_altitude=%.1f, threshold=%.1f, max_attempts=%d)",
      takeoff_altitude_, success_altitude_threshold_, max_attempts_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_pose_ = msg->pose.pose;
    current_z_ = msg->pose.pose.position.z;
    odom_received_ = true;
  }

  void timerCallback()
  {
    if (!odom_received_) {
      return;
    }

    if (!published_) {
      publishTakeoffWaypoint();
      attempt_start_ = this->now();
      published_ = true;
      return;
    }

    // Wait 8 seconds before verifying
    if ((this->now() - attempt_start_).seconds() < 8.0) {
      return;
    }

    if (current_z_ >= success_altitude_threshold_) {
      RCLCPP_INFO(this->get_logger(),
        "Decolagem bem-sucedida: altitude z=%.2f m >= %.2f m. Encerrando nó.",
        current_z_, success_altitude_threshold_);
      rclcpp::shutdown();
    } else {
      attempts_++;
      if (attempts_ >= max_attempts_) {
        RCLCPP_ERROR(this->get_logger(),
          "Falha na decolagem após %d tentativas (z=%.2f m). Encerrando nó com erro.",
          max_attempts_, current_z_);
        rclcpp::shutdown();
        return;
      }
      RCLCPP_WARN(this->get_logger(),
        "Drone não decolou (z=%.2f m após 8s). Tentativa %d/%d...",
        current_z_, attempts_ + 1, max_attempts_);
      published_ = false;
    }
  }

  void publishTakeoffWaypoint()
  {
    geometry_msgs::msg::PoseArray waypoints;
    waypoints.header.frame_id = "map";
    waypoints.header.stamp = this->now();

    geometry_msgs::msg::Pose takeoff_pose;
    takeoff_pose.position.x = current_pose_.position.x;
    takeoff_pose.position.y = current_pose_.position.y;
    takeoff_pose.position.z = takeoff_altitude_;
    takeoff_pose.orientation.w = 1.0;
    takeoff_pose.orientation.x = 0.0;
    takeoff_pose.orientation.y = 0.0;
    takeoff_pose.orientation.z = 0.0;

    waypoints.poses.push_back(takeoff_pose);
    waypoints_pub_->publish(waypoints);

    RCLCPP_INFO(this->get_logger(),
      "Waypoint de decolagem publicado: x=%.2f y=%.2f z=%.2f (tentativa %d/%d)",
      takeoff_pose.position.x, takeoff_pose.position.y, takeoff_pose.position.z,
      attempts_ + 1, max_attempts_);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose current_pose_;
  double current_z_{0.0};
  bool odom_received_{false};
  bool published_{false};
  rclcpp::Time attempt_start_;
  int attempts_{0};
  double takeoff_altitude_{2.0};
  double success_altitude_threshold_{1.5};
  int max_attempts_{5};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffWaypointPublisher8s>());
  rclcpp::shutdown();
  return 0;
}
