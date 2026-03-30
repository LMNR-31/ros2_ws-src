#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <chrono>

using namespace std::chrono_literals;

enum class LandingPhase { WAIT_POSE, PUBLISH, WAIT_DISARM, DONE };

class DronePublishLandingWaypoints : public rclcpp::Node
{
public:
  DronePublishLandingWaypoints() : Node("drone_publish_landing_waypoints"), phase_(LandingPhase::WAIT_POSE)
  {
    waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/mission_waypoints", 10);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/uav1/mavros/local_position/pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
        pose_received_ = true;
      });

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/uav1/mavros/state", 10,
      [this](const mavros_msgs::msg::State::SharedPtr msg) {
        armed_ = msg->armed;
        state_received_ = true;
      });

    // Timer-based state machine — runs at 10 Hz; no blocking or spin_some in constructor
    timer_ = this->create_wall_timer(100ms, std::bind(&DronePublishLandingWaypoints::tick, this));

    RCLCPP_INFO(this->get_logger(), "📡 Nó de publicação de waypoints de pouso iniciado");
    RCLCPP_INFO(this->get_logger(), "⏳ Aguardando posição do drone...");
  }

private:
  void tick()
  {
    switch (phase_) {

      case LandingPhase::WAIT_POSE:
        wait_ticks_++;
        if (pose_received_) {
          RCLCPP_INFO(this->get_logger(), "✅ Pose recebida! X=%.2f, Y=%.2f, Z=%.2f",
            current_pose_.pose.position.x,
            current_pose_.pose.position.y,
            current_pose_.pose.position.z);
          phase_ = LandingPhase::PUBLISH;
        } else if (wait_ticks_ >= 50) {  // 5 s timeout
          RCLCPP_ERROR(this->get_logger(),
            "❌ Timeout aguardando pose: não é possível publicar /mission_waypoints sem posição válida (evitando XY=(0,0)).");
          timer_->cancel();
          rclcpp::shutdown();
        }
        break;

      case LandingPhase::PUBLISH:
        publishLandingWaypoints();
        RCLCPP_INFO(this->get_logger(), "✅ Waypoints de pouso publicados! Aguardando DISARM...");
        disarm_ticks_ = 0;
        phase_ = LandingPhase::WAIT_DISARM;
        break;

      case LandingPhase::WAIT_DISARM:
        if (state_received_ && !armed_) {
          RCLCPP_INFO(this->get_logger(),
            "✅ Pouso confirmado por DISARM (armed=false). Encerrando.");
          phase_ = LandingPhase::DONE;
        } else {
          disarm_ticks_++;
          if (disarm_ticks_ % 20 == 0) {
            RCLCPP_INFO(this->get_logger(),
              "  📍 Aguardando DISARM... Z atual: %.2f m (armed=%s, t=%ds)",
              current_pose_.pose.position.z,
              armed_ ? "true" : "false",
              disarm_ticks_ / 10);
          }
          if (disarm_ticks_ >= 600) {  // 60 s timeout
            RCLCPP_WARN(this->get_logger(),
              "⚠️ Timeout aguardando DISARM (60s)! Encerrando mesmo sem confirmar pouso. Z=%.2f m",
              current_pose_.pose.position.z);
            phase_ = LandingPhase::DONE;
          }
        }
        break;

      case LandingPhase::DONE:
        RCLCPP_INFO(this->get_logger(), "✅ Finalizando nó drone_publish_landing_waypoints");
        timer_->cancel();
        rclcpp::shutdown();
        break;
    }
  }

  void publishLandingWaypoints()
  {
    auto msg = geometry_msgs::msg::PoseArray();
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();

    double current_z = current_pose_.pose.position.z;
    double landing_z = 0.05;
    double delta_z = current_z - landing_z;

    // Descida em 2 waypoints (início e solo)
    for (int i = 0; i <= 1; i++) {
      auto pose = geometry_msgs::msg::Pose();
      pose.position.x = current_pose_.pose.position.x;
      pose.position.y = current_pose_.pose.position.y;

      // Descida linear: começa em current_z, termina em 0.05
      double progress = static_cast<double>(i) / 1.0;  // 0.0 → 1.0
      pose.position.z = current_z - (delta_z * progress);

      pose.orientation.w = 1.0;
      msg.poses.push_back(pose);
    }

    waypoints_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "📍 WAYPOINTS DE POUSO COM DESCIDA PUBLICADOS:");
    RCLCPP_INFO(this->get_logger(),
      "   Posição XY: (%.2f, %.2f)",
      current_pose_.pose.position.x,
      current_pose_.pose.position.y);
    RCLCPP_INFO(this->get_logger(),
      "   Descida: %.2f m → 0.05 m (2 waypoints)",
      current_z);

    for (size_t i = 0; i < msg.poses.size(); i++) {
      RCLCPP_INFO(this->get_logger(),
        "   WP[%zu]: Z=%.2f",
        i,
        msg.poses[i].position.z);
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped current_pose_{};
  bool pose_received_ = false;
  // Initialize armed_ to true so WAIT_DISARM only completes when state_received_ is
  // also true (i.e., after at least one /uav1/mavros/state message is received).
  bool armed_ = true;
  bool state_received_ = false;

  LandingPhase phase_;
  int wait_ticks_ = 0;
  int disarm_ticks_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DronePublishLandingWaypoints>());
  rclcpp::shutdown();
  return 0;
}