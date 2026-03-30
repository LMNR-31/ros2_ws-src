#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
    // Timeout (seconds) to wait for /mission_latch_pose before falling back to odometry
    this->declare_parameter<double>("latch_pose_timeout", 3.0);

    takeoff_altitude_ = this->get_parameter("takeoff_altitude").as_double();
    success_altitude_threshold_ = this->get_parameter("success_altitude_threshold").as_double();
    max_attempts_ = this->get_parameter("max_attempts").as_int();
    latch_pose_timeout_ = this->get_parameter("latch_pose_timeout").as_double();

    waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/mission_waypoints", 10);

    // Primary XY reference: controller-latched waypoint pose published at the moment
    // of reach. Using this avoids the XY=(0,0) drift that can occur with raw odometry
    // after a land/disarm cycle.
    latch_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mission_latch_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        latch_pose_ = msg->pose;
        latch_pose_received_ = true;
        RCLCPP_INFO(this->get_logger(),
          "📌 [latch_pose] Recebida: x=%.2f y=%.2f z=%.2f – será usada como XY de decolagem.",
          msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
      });

    // Fallback XY reference: MAVROS odometry (used only when latch pose is unavailable).
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/uav1/mavros/local_position/odom", 10,
      std::bind(&TakeoffWaypointPublisher8s::odomCallback, this, _1));

    timer_ = this->create_wall_timer(
      500ms, std::bind(&TakeoffWaypointPublisher8s::timerCallback, this));

    node_start_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "🚀 takeoff_waypoint_publisher_8s iniciado. "
      "Aguardando /mission_latch_pose (timeout=%.1fs) ou odometria... "
      "(takeoff_altitude=%.1f, threshold=%.1f, max_attempts=%d)",
      latch_pose_timeout_, takeoff_altitude_, success_altitude_threshold_, max_attempts_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_pose_ = msg->pose.pose;
    current_z_ = msg->pose.pose.position.z;
    odom_received_ = true;
  }

  // Returns true when we have enough information to publish a takeoff waypoint.
  bool hasXyReference() const
  {
    if (latch_pose_received_) { return true; }
    // Fall back to odometry after latch_pose_timeout_ seconds.
    double elapsed = (this->now() - node_start_).seconds();
    return odom_received_ && (elapsed >= latch_pose_timeout_);
  }

  void timerCallback()
  {
    if (!hasXyReference()) {
      return;
    }

    if (!published_) {
      publishTakeoffWaypoint();
      attempt_start_ = this->now();
      published_ = true;
      return;
    }

    // Wait 8 seconds before verifying altitude
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
    if (latch_pose_received_) {
      // PRIMARY: use the XY of the controller-latched waypoint pose.
      takeoff_pose.position.x = latch_pose_.position.x;
      takeoff_pose.position.y = latch_pose_.position.y;
      RCLCPP_INFO(this->get_logger(),
        "📌 [XY fonte: latch_pose] x=%.2f y=%.2f", latch_pose_.position.x, latch_pose_.position.y);
    } else {
      // FALLBACK: use MAVROS odometry XY (may differ from landing point).
      takeoff_pose.position.x = odom_pose_.position.x;
      takeoff_pose.position.y = odom_pose_.position.y;
      RCLCPP_WARN(this->get_logger(),
        "⚠️ [XY fonte: odometria (fallback)] x=%.2f y=%.2f "
        "– /mission_latch_pose não recebida dentro de %.1fs.",
        odom_pose_.position.x, odom_pose_.position.y, latch_pose_timeout_);
    }
    takeoff_pose.position.z = takeoff_altitude_;
    takeoff_pose.orientation.w = 1.0;
    takeoff_pose.orientation.x = 0.0;
    takeoff_pose.orientation.y = 0.0;
    takeoff_pose.orientation.z = 0.0;

    waypoints.poses.push_back(takeoff_pose);
    waypoints_pub_->publish(waypoints);

    RCLCPP_INFO(this->get_logger(),
      "🛫 Waypoint de decolagem publicado em /mission_waypoints: "
      "x=%.2f y=%.2f z=%.2f (tentativa %d/%d)",
      takeoff_pose.position.x, takeoff_pose.position.y, takeoff_pose.position.z,
      attempts_ + 1, max_attempts_);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr latch_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose latch_pose_{};   ///< XY from /mission_latch_pose (primary)
  geometry_msgs::msg::Pose odom_pose_{};    ///< XY from MAVROS odometry (fallback)
  double current_z_{0.0};                   ///< Latest odometry Z for success check
  bool latch_pose_received_{false};
  bool odom_received_{false};
  bool published_{false};
  rclcpp::Time attempt_start_;
  rclcpp::Time node_start_;
  int attempts_{0};
  double takeoff_altitude_{2.0};
  double success_altitude_threshold_{1.5};
  int max_attempts_{5};
  double latch_pose_timeout_{3.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffWaypointPublisher8s>());
  rclcpp::shutdown();
  return 0;
}
