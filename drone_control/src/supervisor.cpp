#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <memory>
#include <queue>

class SupervisorNode : public rclcpp::Node {
public:
    SupervisorNode()
    : Node("supervisor"),
      last_launched_waypoint_idx_(-1),
      mission_manager_pid_(-1),
      trajectory_done_queued_(false)
    {
        this->declare_parameter<int>("min_waypoint_idx_to_trigger", 1);
        min_waypoint_idx_to_trigger_ = this->get_parameter("min_waypoint_idx_to_trigger").as_int();

        progress_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/trajectory_progress", 10,
            std::bind(&SupervisorNode::progress_callback, this, std::placeholders::_1));
        finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/trajectory_finished", 10,
            std::bind(&SupervisorNode::bool_callback, this, std::placeholders::_1));
        waypoint_reached_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/waypoint_reached", 10,
            std::bind(&SupervisorNode::waypoint_reached_callback, this, std::placeholders::_1));

        // Periodic timer to drain the queue and reap finished mission_manager processes.
        queue_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SupervisorNode::process_queue, this));

        RCLCPP_INFO(this->get_logger(),
            "Supervisor node started! (monitorando /waypoint_reached, /trajectory_progress, /trajectory_finished) | min_waypoint_idx_to_trigger=%d",
            min_waypoint_idx_to_trigger_);
    }

private:
    // ── Called on each /waypoint_reached message ───────────────────────────
    void waypoint_reached_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        int wp = msg->data;
        if (wp < min_waypoint_idx_to_trigger_) {
            RCLCPP_INFO(this->get_logger(),
                "Waypoint %d ignorado — mission_manager só é executado a partir do waypoint %d.",
                wp, min_waypoint_idx_to_trigger_);
            return;  // skip waypoints below threshold: no landing/takeoff cycle
        }
        if (wp == last_launched_waypoint_idx_) {
            return;  // already enqueued for this index
        }
        last_launched_waypoint_idx_ = wp;
        RCLCPP_INFO(this->get_logger(), "Waypoint %d atingido! Enfileirando mission_manager...", wp);
        pending_queue_.push(wp);
    }

    // ── Called when trajectory_progress >= 100 ─────────────────────────────
    void progress_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        if (msg->data >= 100.0f && !trajectory_done_queued_) {
            RCLCPP_INFO(this->get_logger(),
                "Progresso da trajetoria: %.1f%% (completo).", msg->data);
            trajectory_done_queued_ = true;
            // The last waypoint_reached already queued the final cycle, so
            // we only add an extra entry here if nothing was queued yet.
            if (pending_queue_.empty() && mission_manager_pid_ < 0) {
                pending_queue_.push(-1);  // sentinel: end-of-trajectory cycle
            }
        }
    }

    // ── Called when trajectory_finished == true ────────────────────────────
    void bool_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !trajectory_done_queued_) {
            RCLCPP_INFO(this->get_logger(),
                "Sinal booleano de trajetoria concluida recebido!");
            trajectory_done_queued_ = true;
            if (pending_queue_.empty() && mission_manager_pid_ < 0) {
                pending_queue_.push(-1);
            }
        }
    }

    // ── Periodic: reap finished process, launch next from queue ───────────
    void process_queue() {
        // Reap the previous mission_manager if it has finished.
        if (mission_manager_pid_ > 0) {
            int status = 0;
            pid_t result = waitpid(mission_manager_pid_, &status, WNOHANG);
            if (result == mission_manager_pid_) {
                RCLCPP_INFO(this->get_logger(),
                    "mission_manager (PID %d) finalizado.", mission_manager_pid_);
                mission_manager_pid_ = -1;
            } else if (result < 0) {
                // Process no longer exists (already reaped or invalid)
                mission_manager_pid_ = -1;
            } else {
                // Still running; do not launch another instance.
                return;
            }
        }

        // Launch the next pending mission_manager if any.
        if (pending_queue_.empty()) {
            return;
        }

        int wp = pending_queue_.front();
        pending_queue_.pop();

        RCLCPP_INFO(this->get_logger(),
            "Iniciando mission_manager para waypoint %d...", wp);

        pid_t pid = fork();
        if (pid == 0) {
            // Child process: exec mission_manager
            execlp("ros2", "ros2", "run", "drone_control", "mission_manager",
                   (char *)nullptr);
            // exec failed
            _exit(1);
        } else if (pid > 0) {
            mission_manager_pid_ = pid;
            RCLCPP_INFO(this->get_logger(),
                "mission_manager iniciado (PID %d).", pid);
        } else {
            RCLCPP_ERROR(this->get_logger(), "fork() falhou ao iniciar mission_manager!");
        }
    }

    // ── Members ────────────────────────────────────────────────────────────
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr progress_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finished_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr waypoint_reached_sub_;
    rclcpp::TimerBase::SharedPtr queue_timer_;

    int last_launched_waypoint_idx_;
    int min_waypoint_idx_to_trigger_;
    pid_t mission_manager_pid_;
    bool trajectory_done_queued_;
    std::queue<int> pending_queue_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SupervisorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
