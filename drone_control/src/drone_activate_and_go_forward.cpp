#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

class TakeoffWaypointPublisher : public rclcpp::Node
{
public:
  TakeoffWaypointPublisher() : Node("takeoff_waypoint_publisher") {
    waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/mission_waypoints", 10);

    // Primary XY reference: controller-latched waypoint pose published at the
    // moment of reach. Using this avoids the XY=(0,0) drift that can occur with
    // raw odometry after a land/disarm cycle.
    latch_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mission_latch_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        latch_pose_ = msg->pose;
        latch_pose_received_ = true;
        RCLCPP_INFO(this->get_logger(),
          "📌 [latch_pose] Recebida: x=%.2f y=%.2f – será usada como XY de decolagem.",
          msg->pose.position.x, msg->pose.position.y);
      });

    // Fallback XY reference: MAVROS odometry (used only when latch pose is
    // unavailable after the latch wait window).
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/uav1/mavros/local_position/odom", 10,
      std::bind(&TakeoffWaypointPublisher::odomCallback, this, _1));

    timer_ = this->create_wall_timer(500ms, std::bind(&TakeoffWaypointPublisher::mainLoop, this));
    node_start_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "Aguardando /mission_latch_pose (até %.1fs) ou odometria para publicar ponto de decolagem...",
      latch_pose_timeout_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_pose_ = msg->pose.pose;
    odom_z_ = msg->pose.pose.position.z;
    odom_received_ = true;
  }

  // Returns true when a valid XY reference is available to publish.
  bool hasXyReference() const {
    if (latch_pose_received_) { return true; }
    // Fall back to odometry after the latch wait window expires.
    double elapsed = (this->now() - node_start_).seconds();
    return odom_received_ && (elapsed >= latch_pose_timeout_);
  }

  void publishTakeoffWaypoint() {
    geometry_msgs::msg::PoseArray waypoints;
    waypoints.header.frame_id = "map";
    waypoints.header.stamp = this->now();

    geometry_msgs::msg::Pose takeoff_pose;
    if (latch_pose_received_) {
      // PRIMARY: use XY from /mission_latch_pose so the drone takes off above
      // the landing spot rather than drifting back to the origin.
      takeoff_pose.position.x = latch_pose_.position.x;
      takeoff_pose.position.y = latch_pose_.position.y;
      RCLCPP_INFO(this->get_logger(),
        "📌 [XY fonte: latch_pose] Publicando decolagem: x=%.2f y=%.2f z=2.0 (tentativa %d)",
        takeoff_pose.position.x, takeoff_pose.position.y, attempts_ + 1);
    } else {
      // FALLBACK: use MAVROS odometry XY (may differ from landing point).
      takeoff_pose.position.x = odom_pose_.position.x;
      takeoff_pose.position.y = odom_pose_.position.y;
      RCLCPP_WARN(this->get_logger(),
        "⚠️ [XY fonte: odometria (fallback)] Publicando decolagem: x=%.2f y=%.2f z=2.0 "
        "– /mission_latch_pose não recebida dentro de %.1fs. (tentativa %d)",
        takeoff_pose.position.x, takeoff_pose.position.y, latch_pose_timeout_, attempts_ + 1);
    }
    takeoff_pose.position.z = 2.0;
    takeoff_pose.orientation.w = 1.0;
    takeoff_pose.orientation.x = 0.0;
    takeoff_pose.orientation.y = 0.0;
    takeoff_pose.orientation.z = 0.0;

    waypoints.poses.push_back(takeoff_pose);
    waypoints_pub_->publish(waypoints);
  }

  void mainLoop() {
    if (!hasXyReference()) return;

    // Publish once per attempt.
    if (!published_) {
      publishTakeoffWaypoint();
      attempt_start_ = this->now();
      published_ = true;
      return;
    }

    // After publishing, wait 7s then check altitude.
    auto now = this->now();
    if ((now - attempt_start_).seconds() < 7.0) {
      return;
    }

    if (odom_z_ >= 1.5) {
      RCLCPP_INFO(this->get_logger(),
        "Decolagem detectada: Altitude z=%.2f m. Nodo será finalizado.", odom_z_);
      rclcpp::shutdown();
    } else {
      // Did not take off – retry.
      attempts_++;
      if (attempts_ >= max_attempts_) {
        RCLCPP_ERROR(this->get_logger(), "Falha após %d tentativas. Finalizando nó.", max_attempts_);
        rclcpp::shutdown();
        return;
      }
      RCLCPP_WARN(this->get_logger(),
        "Drone ainda não decolou (z=%.2f m após 7s), tentando novamente... (tentativa %d/%d)",
        odom_z_, attempts_ + 1, max_attempts_);
      published_ = false;
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr latch_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose latch_pose_{};   ///< XY from /mission_latch_pose (primary)
  geometry_msgs::msg::Pose odom_pose_{};    ///< XY from MAVROS odometry (fallback)
  bool latch_pose_received_{false};
  bool odom_received_{false};
  bool published_{false};
  double odom_z_{0.0};
  rclcpp::Time attempt_start_;
  rclcpp::Time node_start_;
  int attempts_{0};
  static constexpr int max_attempts_ = 5;
  static constexpr double latch_pose_timeout_ = 3.0;  ///< seconds to wait for latch_pose before fallback
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TakeoffWaypointPublisher>());
  return 0;
}