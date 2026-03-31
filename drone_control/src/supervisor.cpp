#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cmath>
#include <memory>
#include <queue>

class SupervisorNode : public rclcpp::Node {
public:
    SupervisorNode()
    : Node("supervisor"),
      last_launched_waypoint_idx_(-1),
      mission_manager_pid_(-1),
      trajectory_done_queued_(false),
      trajectory_done_received_(false),
      return_home_active_(false),
      return_home_published_(false),
      landing_launched_(false),
      landing_pid_(-1),
      odom_received_(false),
      current_x_(0.0),
      current_y_(0.0),
      current_z_(0.0)
    {
        this->declare_parameter<int>("min_waypoint_idx_to_trigger", 0);
        min_waypoint_idx_to_trigger_ = this->get_parameter("min_waypoint_idx_to_trigger").as_int();

        this->declare_parameter<double>("home_x", 0.0);
        this->declare_parameter<double>("home_y", 0.0);
        this->declare_parameter<double>("home_z", 1.5);
        this->declare_parameter<double>("xy_tolerance", 0.25);
        this->declare_parameter<double>("z_tolerance", 0.20);
        home_x_ = this->get_parameter("home_x").as_double();
        home_y_ = this->get_parameter("home_y").as_double();
        home_z_ = this->get_parameter("home_z").as_double();
        xy_tolerance_ = this->get_parameter("xy_tolerance").as_double();
        z_tolerance_ = this->get_parameter("z_tolerance").as_double();

        progress_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/trajectory_progress", 10,
            std::bind(&SupervisorNode::progress_callback, this, std::placeholders::_1));
        finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/trajectory_finished", 10,
            std::bind(&SupervisorNode::bool_callback, this, std::placeholders::_1));
        waypoint_reached_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/waypoint_reached", 10,
            std::bind(&SupervisorNode::waypoint_reached_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/uav1/mavros/local_position/odom", 10,
            std::bind(&SupervisorNode::odom_callback, this, std::placeholders::_1));

        return_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/waypoints", 10);

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
    // NOTE: all callbacks and the queue timer run on the same single-threaded
    // executor, so there is no concurrent access to trajectory_done_received_
    // or return_home_published_.
    void progress_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        if (msg->data >= 100.0f && !trajectory_done_received_) {
            trajectory_done_received_ = true;
            trajectory_done_queued_   = true;
            if (pending_queue_.empty() && mission_manager_pid_ < 0) {
                RCLCPP_INFO(this->get_logger(),
                    "Progresso %.1f%% — trajetória concluída, nenhuma missão pendente: "
                    "retorno à origem será publicado em /waypoints no próximo ciclo de fila.",
                    msg->data);
            } else {
                RCLCPP_INFO(this->get_logger(),
                    "Progresso %.1f%% — trajetória concluída, retorno à origem via /waypoints será publicado após missões pendentes "
                    "(fila=%zu, mission_manager_pid=%d).",
                    msg->data, pending_queue_.size(), static_cast<int>(mission_manager_pid_));
            }
        }
    }

    // ── Called when trajectory_finished == true ────────────────────────────
    void bool_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && !trajectory_done_received_) {
            trajectory_done_received_ = true;
            trajectory_done_queued_   = true;
            if (pending_queue_.empty() && mission_manager_pid_ < 0) {
                RCLCPP_INFO(this->get_logger(),
                    "Sinal de trajetória concluída recebido, nenhuma missão pendente: "
                    "retorno à origem via /waypoints será publicado no próximo ciclo de fila.");
            } else {
                RCLCPP_INFO(this->get_logger(),
                    "Sinal de trajetória concluída recebido — retorno à origem via /waypoints será publicado após missões pendentes "
                    "(fila=%zu, mission_manager_pid=%d).",
                    pending_queue_.size(), static_cast<int>(mission_manager_pid_));
            }
        }
    }

    // ── Called on each odometry message from MAVROS ───────────────────────
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
        odom_received_ = true;
    }

    // ── Periodic: reap finished process, launch next from queue ───────────
    void process_queue() {
        // Reap landing publisher process if it has finished.
        if (landing_pid_ > 0) {
            int status = 0;
            pid_t result = waitpid(landing_pid_, &status, WNOHANG);
            if (result == landing_pid_ || result < 0) {
                landing_pid_ = -1;
            }
        }

        // If return-home mode is active, check if we've arrived at the origin.
        if (return_home_active_) {
            if (odom_received_) {
                double dx = current_x_ - home_x_;
                double dy = current_y_ - home_y_;
                double dxy = std::sqrt(dx * dx + dy * dy);
                double dz = std::abs(current_z_ - home_z_);
                if (dxy <= xy_tolerance_ && dz <= z_tolerance_) {
                    RCLCPP_INFO(this->get_logger(),
                        "Chegou à origem (dxy=%.3f m, dz=%.3f m). Iniciando landing publisher...",
                        dxy, dz);
                    if (!landing_launched_) {
                        pid_t pid = fork();
                        if (pid == 0) {
                            // Pass output_topic:=/waypoints so the final landing
                            // is published as a normal trajectory (not
                            // /mission_waypoints) and is accepted regardless of
                            // the controller's mission-cycle phase.
                            execlp("ros2", "ros2", "run", "drone_control",
                                   "drone_publish_landing_waypoints",
                                   "--ros-args", "-p", "output_topic:=/waypoints",
                                   (char *)nullptr);
                            _exit(1);
                        } else if (pid > 0) {
                            landing_launched_ = true;
                            landing_pid_ = pid;
                            RCLCPP_INFO(this->get_logger(),
                                "drone_publish_landing_waypoints iniciado (PID %d).", pid);
                        } else {
                            RCLCPP_ERROR(this->get_logger(),
                                "fork() falhou ao iniciar drone_publish_landing_waypoints!");
                        }
                    }
                    return_home_active_ = false;
                }
            }
            // Do not process further queue items while returning home.
            return;
        }

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
            // All pending cycles are done. If the trajectory finished and we
            // have not yet published the return-to-home waypoint, do it now.
            if (trajectory_done_received_ && !return_home_published_) {
                return_home_published_ = true;
                auto home_msg = geometry_msgs::msg::PoseArray();
                home_msg.header.frame_id = "map";
                home_msg.header.stamp = this->now();
                geometry_msgs::msg::Pose pose;
                pose.position.x = home_x_;
                pose.position.y = home_y_;
                pose.position.z = home_z_;
                pose.orientation.w = 1.0;
                home_msg.poses.push_back(pose);
                return_pub_->publish(home_msg);
                return_home_active_ = true;
                RCLCPP_INFO(this->get_logger(),
                    "🏁 Retorno à origem publicado em /waypoints "
                    "(x=%.2f, y=%.2f, z=%.2f) — trajetória normal.",
                    home_x_, home_y_, home_z_);
            }
            return;
        }

        int wp = pending_queue_.front();
        pending_queue_.pop();

        RCLCPP_INFO(this->get_logger(),
            "Iniciando mission_manager para waypoint %d...", wp);

        // Build the waypoint_idx ROS parameter string before fork so we own
        // the memory in the child (no risk of dangling pointer after exec).
        std::string wp_param = "waypoint_idx:=" + std::to_string(wp);

        pid_t pid = fork();
        if (pid == 0) {
            // Child process: exec mission_manager with the waypoint index so
            // it can publish /mission_interrupt_done with the correct index.
            execlp("ros2", "ros2", "run", "drone_control", "mission_manager",
                   "--ros-args", "-p", wp_param.c_str(),
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
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr return_pub_;
    rclcpp::TimerBase::SharedPtr queue_timer_;

    int last_launched_waypoint_idx_;
    int min_waypoint_idx_to_trigger_;
    pid_t mission_manager_pid_;
    bool trajectory_done_queued_;
    bool trajectory_done_received_;   // set on first trajectory-done event, regardless of queue/pid state
    std::queue<int> pending_queue_;

    // ── Return-to-origin state ─────────────────────────────────────────────
    bool return_home_active_;
    bool return_home_published_;      // idempotence guard: return-home published exactly once
    bool landing_launched_;
    pid_t landing_pid_;
    double home_x_;
    double home_y_;
    double home_z_;
    double xy_tolerance_;
    double z_tolerance_;

    // ── Current odometry ──────────────────────────────────────────────────
    bool odom_received_;
    double current_x_;
    double current_y_;
    double current_z_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SupervisorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
