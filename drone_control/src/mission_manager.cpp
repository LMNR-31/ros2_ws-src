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
  MissionManager() : Node("mission_manager"), armed_(false)
  {
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
      "/uav1/mavros/state", 10,
      [this](const mavros_msgs::msg::State::SharedPtr msg) {
        armed_.store(msg->armed);
      });

    timer_ = this->create_wall_timer(
      2s, std::bind(&MissionManager::startMission, this));

    RCLCPP_INFO(this->get_logger(), "Mission Manager Iniciado");
  }

private:

  void startMission()
  {
    timer_->cancel();

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
      rclcpp::spin_some(this->get_node_base_interface());
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
      RCLCPP_INFO(this->get_logger(), "%d segundo(s) restante(s)...", i);
      std::this_thread::sleep_for(1s);
    }
    RCLCPP_INFO(this->get_logger(), "Repouso concluido!");

    RCLCPP_INFO(this->get_logger(), "FASE 4: ATIVAR DRONE E DECOLAR");
    int result2 = std::system("ros2 run drone_control drone_activate_and_go_forward");
    if (result2 == 0) {
      RCLCPP_INFO(this->get_logger(), "Drone ativado e decolagem concluida!");
    } else {
      RCLCPP_WARN(this->get_logger(), "Erro ao executar drone (codigo: %d)", result2);
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
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionManager>());
  rclcpp::shutdown();
  return 0;
}
