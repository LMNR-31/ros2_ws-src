#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cstdlib>
#include <memory>

class SupervisorNode : public rclcpp::Node {
public:
    SupervisorNode() : Node("supervisor"), last_launched_waypoint_idx_(-1) {
        progress_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/trajectory_progress", 10,
            std::bind(&SupervisorNode::progress_callback, this, std::placeholders::_1)
        );
        finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/trajectory_finished", 10,
            std::bind(&SupervisorNode::bool_callback, this, std::placeholders::_1)
        );
        waypoint_reached_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/waypoint_reached", 10,
            std::bind(&SupervisorNode::waypoint_reached_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Supervisor node started! (monitorando Bool, Float32 e Int32)");
    }

private:
    void waypoint_reached_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        int wp = msg->data;
        if (wp == last_launched_waypoint_idx_) {
            return;
        }
        last_launched_waypoint_idx_ = wp;
        RCLCPP_INFO(this->get_logger(), "Waypoint %d atingido! Iniciando mission_manager...", wp);
        launch_mission_manager();
    }

    void progress_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        if (msg->data >= 100.0f) {
            RCLCPP_INFO(this->get_logger(), "Progresso da trajetória: %.1f%% (completo). Iniciando mission_manager...", msg->data);
            launch_mission_manager_on_completion();
        }
    }

    void bool_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Sinal Booleano de trajetória concluída recebido! Iniciando mission_manager...");
            launch_mission_manager_on_completion();
        }
    }

    // Per-waypoint launch (called for each /waypoint_reached message).
    void launch_mission_manager() {
        int ret = std::system("ros2 run drone_control mission_manager &");
        if (ret == -1) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao iniciar mission_manager!");
        } else {
            RCLCPP_INFO(this->get_logger(), "mission_manager iniciado.");
        }
    }

    // Launch on trajectory completion (progress>=100 or finished=true); fires only once.
    void launch_mission_manager_on_completion() {
        static bool already_launched = false;
        if (!already_launched) {
            int ret = std::system("ros2 run drone_control mission_manager &");
            if (ret == -1) {
                RCLCPP_ERROR(this->get_logger(), "Falha ao iniciar mission_manager!");
            } else {
                RCLCPP_INFO(this->get_logger(), "mission_manager iniciado.");
            }
            already_launched = true;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr progress_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finished_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr waypoint_reached_sub_;

    int last_launched_waypoint_idx_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SupervisorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}