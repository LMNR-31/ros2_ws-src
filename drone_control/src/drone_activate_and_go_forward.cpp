#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class TakeoffWaypointPublisher : public rclcpp::Node
{
public:
  TakeoffWaypointPublisher() : Node("takeoff_waypoint_publisher") {
    waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/uav1/mavros/local_position/odom", 10,
      std::bind(&TakeoffWaypointPublisher::odomCallback, this, _1));
    timer_ = this->create_wall_timer(500ms, std::bind(&TakeoffWaypointPublisher::tryPublish, this));
    RCLCPP_INFO(this->get_logger(), "Aguardando primeira odometria para publicar ponto de decolagem...");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_pose_ = msg->pose.pose;
    odom_received_ = true;
  }

  void tryPublish() {
    if (!odom_received_ || published_) return;
    geometry_msgs::msg::PoseArray waypoints;
    waypoints.header.frame_id = "map";
    waypoints.header.stamp = this->now();
    geometry_msgs::msg::Pose takeoff_pose = last_pose_;
    takeoff_pose.position.z = 2.0; // Altitude desejada para decolagem
    waypoints.poses.push_back(takeoff_pose);
    waypoints_pub_->publish(waypoints);

    RCLCPP_INFO(this->get_logger(),
      "Waypoints de decolagem publicados: x=%.2f y=%.2f z=%.2f",
      takeoff_pose.position.x, takeoff_pose.position.y, takeoff_pose.position.z);

    published_ = true;
    // Opcional: Encerra o nó após publicar
    rclcpp::shutdown();
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose last_pose_;
  bool odom_received_{false};
  bool published_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffWaypointPublisher>());
  return 0;
}