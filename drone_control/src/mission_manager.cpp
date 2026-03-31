#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <atomic>

using namespace std::chrono_literals;

class MissionManager : public rclcpp::Node
{
public:
  MissionManager() : Node("mission_manager"), armed_(false), stop_requested_(false)
  {
    this->declare_parameter<int>("waypoint_idx", -1);
    waypoint_idx_ = this->get_parameter("waypoint_idx").as_int();

    interrupt_done_pub_ = this->create_publisher<std_msgs::msg::Int32>("/mission_interrupt_done", 10);

    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/uav1/mavros/state", 10,
      [this](const mavros_msgs::msg::State::SharedPtr msg) {
        armed_.store(msg->armed);
      });

    // Fire once after 2 s, then run the mission in a separate thread so the
    // executor keeps spinning (state callbacks stay live).
    timer_ = this->create_wall_timer(
      2s, std::bind(&MissionManager::startMissionAsync, this));

    RCLCPP_INFO(this->get_logger(),
      "🚦 Mission Manager Iniciado (waypoint_idx=%d)", waypoint_idx_);
  }

  ~MissionManager()
  {
    stop_requested_.store(true);
    if (mission_thread_.joinable()) {
      mission_thread_.join();
    }
  }

private:
  void startMissionAsync()
  {
    timer_->cancel();
    // Run all blocking phases in a separate thread; the executor is free to
    // dispatch state callbacks while the thread sleeps/waits.
    mission_thread_ = std::thread(&MissionManager::runMission, this);
  }

  void runMission()
  {
    RCLCPP_INFO(this->get_logger(),
      "🔄 [MISSION WP%d] FASE 1: PUBLICAR WAYPOINTS DE POUSO", waypoint_idx_);
    int result1 = std::system("ros2 run drone_control drone_publish_landing_waypoints");
    if (result1 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "✅ [MISSION WP%d] Waypoints de pouso publicados!", waypoint_idx_);
    } else {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ [MISSION WP%d] Erro ao publicar waypoints de pouso (codigo: %d)",
        waypoint_idx_, result1);
    }

    RCLCPP_INFO(this->get_logger(),
      "⏳ [MISSION WP%d] FASE 2: AGUARDANDO DISARM", waypoint_idx_);
    static constexpr int kMaxDisarmWaitS = 60;
    bool disarmed = false;
    for (int i = 0; i < kMaxDisarmWaitS * 10; ++i) {
      // armed_ is updated by the executor thread via the state subscription;
      // do NOT call spin_some here — node is already in rclcpp::spin().
      if (stop_requested_.load()) { return; }
      if (!armed_.load()) {
        disarmed = true;
        RCLCPP_INFO(this->get_logger(),
          "✅ [MISSION WP%d] Drone DESARMADO confirmado!", waypoint_idx_);
        break;
      }
      std::this_thread::sleep_for(100ms);
      if (i % 20 == 0) {
        RCLCPP_INFO(this->get_logger(),
          "  ⏳ [MISSION WP%d] Aguardando DISARM... (%d/%d s)",
          waypoint_idx_, i / 10, kMaxDisarmWaitS);
      }
    }
    if (!disarmed) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ [MISSION WP%d] Timeout aguardando DISARM (%d s). Continuando.",
        waypoint_idx_, kMaxDisarmWaitS);
    }

    RCLCPP_INFO(this->get_logger(),
      "😴 [MISSION WP%d] FASE 3: REPOUSO 2 SEGUNDOS", waypoint_idx_);
    for (int i = 2; i > 0; i--) {
      if (stop_requested_.load()) { return; }
      RCLCPP_INFO(this->get_logger(),
        "  ⏱️ [MISSION WP%d] %d segundo(s) restante(s)...", waypoint_idx_, i);
      std::this_thread::sleep_for(1s);
    }
    RCLCPP_INFO(this->get_logger(),
      "✅ [MISSION WP%d] Repouso concluido!", waypoint_idx_);

    RCLCPP_INFO(this->get_logger(),
      "🚀 [MISSION WP%d] FASE 4: ATIVAR DRONE E DECOLAR", waypoint_idx_);
    int result2 = std::system("ros2 run drone_control takeoff_waypoint_publisher_8s");
    if (result2 == 0) {
      RCLCPP_INFO(this->get_logger(),
        "✅ [MISSION WP%d] Drone ativado e decolagem concluida!", waypoint_idx_);
    } else {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ [MISSION WP%d] Erro ao executar decolagem (codigo: %d)", waypoint_idx_, result2);
    }

    RCLCPP_INFO(this->get_logger(),
      "✅ [MISSION WP%d] MISSAO COMPLETA. Sequencia: 1=%s 2=DISARM 3=5s 4=%s",
      waypoint_idx_,
      result1 == 0 ? "OK" : "WARN",
      result2 == 0 ? "OK" : "WARN");

    // Signal to the controller that the full interrupt cycle has finished so
    // it can resume the main trajectory from the next waypoint.
    std_msgs::msg::Int32 done_msg;
    done_msg.data = waypoint_idx_;
    interrupt_done_pub_->publish(done_msg);
    RCLCPP_INFO(this->get_logger(),
      "📢 [MISSION WP%d] /mission_interrupt_done publicado. "
      "Controller retomará trajetória principal.",
      waypoint_idx_);

    // Give the DDS middleware enough time to deliver the message to all
    // subscribers before the node shuts down.  200 ms is sufficient for
    // local-process subscribers and typical RMW implementations.
    static constexpr int kDdsPublishDelayMs = 200;
    std::this_thread::sleep_for(std::chrono::milliseconds(kDdsPublishDelayMs));

    RCLCPP_INFO(this->get_logger(),
      "🔚 [MISSION WP%d] Encerrando Mission Manager...", waypoint_idx_);
    rclcpp::shutdown();
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr interrupt_done_pub_;
  std::atomic<bool> armed_;
  std::atomic<bool> stop_requested_;
  std::thread mission_thread_;
  int waypoint_idx_{-1};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManager>());
  rclcpp::shutdown();
  return 0;
}
