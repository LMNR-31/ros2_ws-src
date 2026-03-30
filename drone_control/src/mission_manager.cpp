#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/state.hpp>
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
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/uav1/mavros/state", 10,
      [this](const mavros_msgs::msg::State::SharedPtr msg) {
        armed_.store(msg->armed);
      });

    // Fire once after 2 s, then run the mission in a separate thread so the
    // executor keeps spinning (state callbacks stay live).
    timer_ = this->create_wall_timer(
      2s, std::bind(&MissionManager::startMissionAsync, this));

    RCLCPP_INFO(this->get_logger(), "Mission Manager Iniciado");
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
    RCLCPP_INFO(this->get_logger(), "FASE 1: PUBLICAR WAYPOINTS DE POUSO");
    int result1 = std::system("ros2 run drone_control drone_publish_landing_waypoints");
    if (result1 == 0) {
      RCLCPP_INFO(this->get_logger(), "Waypoints de pouso publicados!");
    } else {
      RCLCPP_WARN(this->get_logger(), "Erro ao publicar waypoints de pouso (codigo: %d)", result1);
    }

    RCLCPP_INFO(this->get_logger(), "FASE 2: AGUARDANDO DISARM");
    static constexpr int kMaxDisarmWaitS = 60;
    bool disarmed = false;
    for (int i = 0; i < kMaxDisarmWaitS * 10; ++i) {
      // armed_ is updated by the executor thread via the state subscription;
      // do NOT call spin_some here — node is already in rclcpp::spin().
      if (stop_requested_.load()) { return; }
      if (!armed_.load()) {
        disarmed = true;
        RCLCPP_INFO(this->get_logger(), "Drone DESARMADO confirmado!");
        break;
      }
      std::this_thread::sleep_for(100ms);
      if (i % 20 == 0) {
        RCLCPP_INFO(this->get_logger(), "Aguardando DISARM... (%d/%d s)", i / 10, kMaxDisarmWaitS);
      }
    }
    if (!disarmed) {
      RCLCPP_WARN(this->get_logger(),
        "Timeout aguardando DISARM (%d s). Continuando.", kMaxDisarmWaitS);
    }

    RCLCPP_INFO(this->get_logger(), "FASE 3: REPOUSO 5 SEGUNDOS");
    for (int i = 5; i > 0; i--) {
      if (stop_requested_.load()) { return; }
      RCLCPP_INFO(this->get_logger(), "%d segundo(s) restante(s)...", i);
      std::this_thread::sleep_for(1s);
    }
    RCLCPP_INFO(this->get_logger(), "Repouso concluido!");

    RCLCPP_INFO(this->get_logger(), "FASE 4: ATIVAR DRONE E DECOLAR");
    int result2 = std::system("ros2 run drone_control takeoff_waypoint_publisher_8s");
    if (result2 == 0) {
      RCLCPP_INFO(this->get_logger(), "Drone ativado e decolagem concluida!");
    } else {
      RCLCPP_WARN(this->get_logger(), "Erro ao executar decolagem (codigo: %d)", result2);
    }

    RCLCPP_INFO(this->get_logger(), "MISSAO COMPLETA. Sequencia: 1=%s 2=DISARM 3=5s 4=%s",
      result1 == 0 ? "OK" : "WARN",
      result2 == 0 ? "OK" : "WARN");

    RCLCPP_INFO(this->get_logger(), "Encerrando Mission Manager...");
    rclcpp::shutdown();
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  std::atomic<bool> armed_;
  std::atomic<bool> stop_requested_;
  std::thread mission_thread_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManager>());
  rclcpp::shutdown();
  return 0;
}
