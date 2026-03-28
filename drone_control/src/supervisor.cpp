#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cstdlib>
#include <memory>

class SupervisorNode : public rclcpp::Node {
public:
    SupervisorNode() : Node("supervisor") {
        progress_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/trajectory_progress", 10,
            std::bind(&SupervisorNode::progress_callback, this, std::placeholders::_1)
        );
        finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/trajectory_finished", 10,
            std::bind(&SupervisorNode::bool_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Supervisor node started! (monitorando Bool e Float32)");
    }

private:
    void progress_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        if (msg->data >= 100.0f) {
            RCLCPP_INFO(this->get_logger(), "Progresso da trajetória: %.1f%% (completo). Iniciando mission_manager...", msg->data);
            launch_mission_manager();
        }
    }

    void bool_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Sinal Booleano de trajetória concluída recebido! Iniciando mission_manager...");
            launch_mission_manager();
        }
    }

    // Função de lançamento protegida para evitar execuções duplicadas (opcional)
    void launch_mission_manager() {
        static bool already_launched = false;
        if (!already_launched) {
            int ret = std::system("ros2 run drone_control mission_manager &");
            if (ret == -1) {
                RCLCPP_ERROR(this->get_logger(), "Falha ao iniciar mission_manager!");
            } else {
                RCLCPP_INFO(this->get_logger(), "mission_manager iniciado.");
            }
            already_launched = true;
            // Se quiser permitir múltiplos lançamentos, comente ou melhore o controle aqui
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr progress_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finished_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SupervisorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}