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
    timer_ = this->create_wall_timer(500ms, std::bind(&TakeoffWaypointPublisher::mainLoop, this));

    RCLCPP_INFO(this->get_logger(), "Aguardando primeira odometria para publicar ponto de decolagem...");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    last_pose_ = msg->pose.pose;
    odom_z_ = msg->pose.pose.position.z;
    odom_received_ = true;
  }

  void mainLoop() {
    if (!odom_received_) return;

    // Primeiro envio (publica apenas uma vez por tentativa)
    if (!published_) {
      geometry_msgs::msg::PoseArray waypoints;
      waypoints.header.frame_id = "map";
      waypoints.header.stamp = this->now();

      geometry_msgs::msg::Pose takeoff_pose = last_pose_;
      takeoff_pose.position.z = 2.0;
      waypoints.poses.push_back(takeoff_pose);

      waypoints_pub_->publish(waypoints);
      attempt_start_ = this->now();

      RCLCPP_INFO(this->get_logger(),
        "Waypoints de decolagem publicados: x=%.2f y=%.2f z=%.2f (tentativa %d)",
        takeoff_pose.position.x, takeoff_pose.position.y, takeoff_pose.position.z, attempts_+1);

      published_ = true;
      return;
    }

    // Após publicação, aguarda 3s para verificar se subiu
    auto now = this->now();
    if ((now - attempt_start_).seconds() < 7.0) {
      return;
    }

    if (odom_z_ >= 1.5) {
      RCLCPP_INFO(this->get_logger(),
        "Decolagem detectada: Altitude z=%.2f m. Nodo será finalizado.", odom_z_);
      rclcpp::shutdown();
    } else {
      // Não decolou -- tenta novamente
      attempts_++;
      if (attempts_ >= max_attempts_) {
        RCLCPP_ERROR(this->get_logger(), "Falha após %d tentativas. Finalizando nó.", max_attempts_);
        rclcpp::shutdown();
        return;
      }
      RCLCPP_WARN(this->get_logger(),
        "Drone ainda não decolou (z=%.2f m após 5s), tentando novamente... (tentativa %d/%d)",
        odom_z_, attempts_+1, max_attempts_);
      published_ = false;
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose last_pose_;
  bool odom_received_{false};
  bool published_{false};
  double odom_z_{0.0};
  rclcpp::Time attempt_start_;
  int attempts_{0};
  static constexpr int max_attempts_ = 5;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffWaypointPublisher>());
  return 0;
}