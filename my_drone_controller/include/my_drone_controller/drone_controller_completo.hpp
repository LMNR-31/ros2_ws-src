#ifndef MY_DRONE_CONTROLLER__DRONE_CONTROLLER_COMPLETO_HPP_
#define MY_DRONE_CONTROLLER__DRONE_CONTROLLER_COMPLETO_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "drone_control/msg/yaw_override.hpp"
#include "drone_control/msg/waypoint4_d.hpp"
#include "drone_control/msg/waypoint4_d_array.hpp"
#include "drone_config.h"
#include "my_drone_controller/command_queue.hpp"
#include "my_drone_controller/waypoint_validation.hpp"

#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace drone_control {

/**
 * @brief Complete drone controller node with command tracking and safety.
 *
 * Implements a 5-state flight state machine:
 *  - State 0: Waiting for waypoint goal
 *  - State 1: Takeoff (OFFBOARD + ARM, climb to hover_altitude)
 *  - State 2: Hover (waiting for trajectory)
 *  - State 3: Executing trajectory
 *  - State 4: Landing — waits for ground detection, then DISARMs and resets
 *              to State 0 to accept a new takeoff command.
 *
 * All drone commands are registered in a CommandQueue so that every
 * operation can be audited and timed out automatically.
 *
 * There is a single universal landing flow: after landing is detected the
 * drone always proceeds to DISARM; once the FCU confirms DISARM all flags
 * are reset and the system returns to State 0, ready for the next takeoff.
 * There are no landing modes (Modo A / Modo B) — only this single flow.
 */
class DroneControllerCompleto : public rclcpp::Node
{
public:
  // type_mask bits for PositionTarget (1 = ignore that field)
  static constexpr uint16_t IGNORE_VX       = (1 << 3);
  static constexpr uint16_t IGNORE_VY       = (1 << 4);
  static constexpr uint16_t IGNORE_VZ       = (1 << 5);
  static constexpr uint16_t IGNORE_AFX      = (1 << 6);
  static constexpr uint16_t IGNORE_AFY      = (1 << 7);
  static constexpr uint16_t IGNORE_AFZ      = (1 << 8);
  static constexpr uint16_t IGNORE_YAW      = (1 << 10);
  static constexpr uint16_t IGNORE_YAW_RATE = (1 << 11);

  // Position + yaw_rate; ignore velocity, acceleration, yaw angle
  static constexpr uint16_t MASK_POS_YAWRATE =
    IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW;

  // Position + absolute yaw; ignore velocity, acceleration, yaw_rate
  static constexpr uint16_t MASK_POS_YAW =
    IGNORE_VX | IGNORE_VY | IGNORE_VZ | IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | IGNORE_YAW_RATE;

  // Position only; ignore velocity, acceleration, yaw angle and yaw_rate
  static constexpr uint16_t MASK_POS_ONLY = MASK_POS_YAWRATE | IGNORE_YAW_RATE;

  /// Minimum number of setpoints to publish before requesting ARM+OFFBOARD.
  /// At 100 Hz this equates to ~200 ms of continuous streaming, which is
  /// enough for the PX4 FCU to accept the ARM command in OFFBOARD mode.
  static constexpr int INITIAL_STREAM_THRESHOLD = 20;

  /// Number of setpoints to publish AFTER the FCU confirms OFFBOARD mode and
  /// BEFORE sending the ARM command.  At 100 Hz (10 ms/iteration) this equals
  /// 1.5 seconds of continuous streaming — equivalent to 30 setpoints at 50 ms
  /// each as recommended by the PX4/MAVROS offboard guide.  The FCU needs to
  /// see a stable setpoint stream in OFFBOARD mode before it will accept ARM.
  static constexpr int POST_OFFBOARD_STREAM_THRESHOLD = 150;

  DroneControllerCompleto();

private:
  // ── Setup ────────────────────────────────────────────────────────────────
  void load_parameters();
  void setup_publishers();
  void setup_subscribers();
  void setup_services();
  void init_variables();

  // ── Landing / activation helpers ────────────────────────────────────────
  void trigger_landing(double z);
  void activate_offboard_arm_if_needed();

  // ── Pre-ARM setpoint streaming ───────────────────────────────────────────
  /**
   * @brief Stream initial position setpoints before requesting ARM+OFFBOARD.
   *
   * PX4/MAVROS will reject an ARM command in OFFBOARD mode unless the FCU
   * has been receiving a continuous stream of setpoints on the
   * setpoint_raw/local topic BEFORE and DURING the ARM/OFFBOARD request.
   * This method publishes INITIAL_STREAM_THRESHOLD setpoints (≥20) at the
   * target hover position so that the FCU accepts the subsequent ARM command.
   *
   * Call from handle_state1_takeoff() before request_arm_and_offboard_activation().
   */
  void stream_initial_setpoints();

  /**
   * @brief Stream position setpoints for 1.5 s AFTER the FCU confirms OFFBOARD.
   *
   * Even after OFFBOARD mode is confirmed, the PX4 FCU may still reject ARM if
   * the setpoint stream has not been running long enough.  Per the PX4/MAVROS
   * offboard guide, the vehicle must receive a continuous stream of setpoints
   * for at least 1.5 seconds in OFFBOARD mode before ARM is accepted.  This method
   * publishes POST_OFFBOARD_STREAM_THRESHOLD setpoints (150 × 10 ms = 1.5 s at
   * 100 Hz, providing the same duration as the PX4/MAVROS recommendation of 30
   * setpoints at 50 ms each) and sets post_offboard_stream_done_ once the
   * threshold is reached.
   *
   * Call from handle_state1_takeoff() between OFFBOARD confirmation (step 3)
   * and the ARM request (step 4).
   */
  void stream_post_offboard_setpoints();

  // ── Parameter handlers ───────────────────────────────────────────────────
  bool apply_enabled_param(const rclcpp::Parameter & p,
                           rcl_interfaces::msg::SetParametersResult & result);
  bool apply_override_active_param(const rclcpp::Parameter & p,
                                   rcl_interfaces::msg::SetParametersResult & result);
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & params);

  // ── FCU service requests ─────────────────────────────────────────────────
  void request_offboard();
  void request_arm();
  void request_disarm();

  // ── Sensor callbacks ─────────────────────────────────────────────────────
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void yaw_override_callback(const drone_control::msg::YawOverride::SharedPtr msg);

  // ── Waypoint callback helpers (3D) ───────────────────────────────────────
  void handle_landing_waypoint_command(double z);
  void handle_single_takeoff_waypoint_command(const geometry_msgs::msg::Pose & pose);
  void log_trajectory_waypoints_3d(const std::vector<geometry_msgs::msg::Pose> & poses);
  void activate_trajectory_in_hover(size_t waypoint_count);

  // ── Waypoint callbacks ───────────────────────────────────────────────────
  void waypoints_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // ── Shared waypoint-goal helpers ─────────────────────────────────────────
  bool check_landing_in_flight(double z);
  bool handle_state4_disarm_reset();
  /// Logs a WARN with all relevant dirty-state flags; call before reset_after_landing().
  void log_dirty_takeoff_state(const char * context);
  /// Logs a DEBUG snapshot of the key FSM flags; call before/after takeoff setup.
  void log_takeoff_debug_flags(const char * tag);
  /// Returns true when the FSM is actively in flight (states 1–3).
  bool is_in_flight() const { return state_voo_ == 1 || state_voo_ == 2 || state_voo_ == 3; }
  /// Returns true when any flag from the previous flight cycle was not reset.
  bool has_dirty_takeoff_state() const {
    return takeoff_cmd_id_.has_value() || activation_confirmed_ ||
           offboard_activated_ || offboard_mode_confirmed_ || arm_requested_ ||
           disarm_requested_ || pouso_em_andamento_ ||
           trajectory_cmd_id_.has_value() || hover_cmd_id_.has_value() ||
           post_offboard_stream_done_ ||
           state_voo_ != 0;
  }

  // ── Waypoint-goal callbacks ──────────────────────────────────────────────
  void waypoint_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void waypoint_goal_4d_callback(const drone_control::msg::Waypoint4D::SharedPtr msg);
  void waypoints_4d_callback(const drone_control::msg::Waypoint4DArray::SharedPtr msg);

  // ── Setpoint publishing ──────────────────────────────────────────────────
  void publishPositionTarget(double x, double y, double z,
                             double yaw_rate, uint16_t type_mask);
  void publishPositionTargetWithYaw(double x, double y, double z, double yaw_rad);

  // ── Main control loop ────────────────────────────────────────────────────
  void control_loop();

  // ── FSM state handlers ───────────────────────────────────────────────────
  void handle_state0_wait_waypoint();

  // State 1 — takeoff layers
  void request_arm_and_offboard_activation();
  /**
   * @brief Wait for FCU to confirm OFFBOARD mode after a SET_MODE OFFBOARD request.
   *
   * Per PX4/MAVROS best practice, ARM must only be sent AFTER the FCU reports
   * mode == "OFFBOARD".  Sending ARM and OFFBOARD simultaneously causes the ARM
   * to be rejected even when the OFFBOARD request succeeds.  This method polls
   * current_state_.mode and sets offboard_mode_confirmed_ once the FCU confirms
   * the transition, or resets offboard_activated_ on timeout so the sequence
   * can be retried cleanly.
   */
  void wait_for_offboard_mode();
  bool wait_for_offboard_arm_confirmation();
  void publish_takeoff_climb_setpoint(double target_alt);
  void finalize_takeoff_on_altitude_reached(double target_alt);
  void handle_state1_takeoff();

  // State 2 — hover
  void handle_state2_hover();

  // State 3 — trajectory sub-routines
  bool detect_and_handle_landing_in_trajectory();
  bool initialize_trajectory();
  double compute_yaw_for_trajectory_waypoint(int idx, bool at_last_wp);
  void publish_trajectory_waypoint_setpoint(int idx);
  void log_trajectory_progress(int idx, double elapsed_time);
  void finalize_trajectory_if_complete(double elapsed_time, double total_time);
  void handle_state3_trajectory();

  // State 4 — landing completers
  void reset_after_landing();
  void complete_landing();
  void handle_state4_landing();

  // ── Configuration ─────────────────────────────────────────────────────────
  DroneConfig config_;

  // ── Publishers ───────────────────────────────────────────────────────────
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr raw_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trajectory_finished_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr progress_publisher_;

  // ── Subscribers ──────────────────────────────────────────────────────────
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr waypoint_goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<drone_control::msg::YawOverride>::SharedPtr yaw_override_sub_;
  rclcpp::Subscription<drone_control::msg::Waypoint4D>::SharedPtr waypoint_goal_4d_sub_;
  rclcpp::Subscription<drone_control::msg::Waypoint4DArray>::SharedPtr waypoints_4d_sub_;

  // ── Service clients ───────────────────────────────────────────────────────
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;

  // ── Timer ────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr timer_;

  // ── FCU state ────────────────────────────────────────────────────────────
  mavros_msgs::msg::State current_state_;

  // ── FSM state variables ───────────────────────────────────────────────────
  int state_voo_;   ///< 0=wait, 1=takeoff, 2=hover, 3=trajectory, 4=landing
  bool controlador_ativo_;
  bool pouso_em_andamento_;
  bool offboard_activated_;
  /// True once FCU confirms mode == "OFFBOARD" following a SET_MODE request.
  /// ARM is sent only after this flag is set, per PX4/MAVROS recommendation.
  bool offboard_mode_confirmed_;
  /// True once request_arm() has been called following OFFBOARD confirmation.
  bool arm_requested_;
  bool activation_confirmed_;
  rclcpp::Time activation_time_;
  int cycle_count_;
  int takeoff_counter_;

  // ── Landing timeout tracking ──────────────────────────────────────────────
  double last_z_;
  rclcpp::Time pouso_start_time_;
  bool pouso_start_time_set_;
  /// true when DISARM has been requested but FCU confirmation is still pending
  bool disarm_requested_;

  // ── Waypoint tracking ────────────────────────────────────────────────────
  geometry_msgs::msg::PoseStamped last_waypoint_goal_;
  bool waypoint_goal_received_;
  double trajectory_setpoint_[3];

  // ── Trajectory state ─────────────────────────────────────────────────────
  std::vector<geometry_msgs::msg::Pose> trajectory_waypoints_;
  rclcpp::Time trajectory_start_time_;
  bool trajectory_started_;
  int current_waypoint_idx_;
  double waypoint_duration_;

  // ── Odometry (NED) ───────────────────────────────────────────────────────
  double current_z_real_;
  double current_x_ned_;
  double current_y_ned_;
  double current_z_ned_;
  double final_waypoint_yaw_{0.0};
  bool at_last_waypoint_yaw_fixed_{false};

  // ── Yaw override ─────────────────────────────────────────────────────────
  bool yaw_override_enabled_;
  double yaw_rate_cmd_;
  double yaw_override_timeout_s_;
  rclcpp::Time last_yaw_cmd_time_;
  bool hold_valid_;
  double hold_x_ned_;
  double hold_y_ned_;
  double hold_z_ned_;

  // ── 4D waypoint support ───────────────────────────────────────────────────
  double current_yaw_rad_;
  double goal_yaw_rad_;
  bool using_4d_goal_;
  bool trajectory_4d_mode_;
  std::vector<double> trajectory_yaws_;

  // ── Control flags ────────────────────────────────────────────────────────
  bool enabled_{true};
  bool override_active_{false};

  // ── Takeoff target altitude (latched) ────────────────────────────────────
  /// Fixed Z target computed ONCE at the start of each takeoff cycle.
  ///
  /// Bug (infinite ascent): if target_altitude is recalculated every control
  /// cycle as std::max(hover_altitude, current_z_real_ + boost), the target
  /// rises together with the drone because current_z_real_ keeps increasing.
  /// The drone never reaches its own target and climbs without bound.
  ///
  /// Fix: compute and latch the value once (when takeoff_counter_ == 0),
  /// then publish it unchanged until the drone arrives at that altitude.
  /// Reset to -1.0 (sentinel) when entering a new takeoff or after landing.
  double takeoff_target_z_{-1.0};

  // ── Pre-ARM setpoint streaming counters ──────────────────────────────────
  /// Number of setpoints published so far in the current pre-ARM stream phase.
  int initial_stream_count_{0};
  /// True once INITIAL_STREAM_THRESHOLD setpoints have been published and
  /// ARM+OFFBOARD can safely be requested from the FSM.
  bool initial_stream_done_{false};

  // ── Post-OFFBOARD setpoint streaming counters ─────────────────────────────
  /// Number of setpoints published since OFFBOARD mode was confirmed by the FCU.
  /// Used to enforce a 1.5-second streaming window before ARM is requested.
  int post_offboard_stream_count_{0};
  /// True once POST_OFFBOARD_STREAM_THRESHOLD setpoints have been published
  /// after OFFBOARD confirmation, so ARM can safely be sent to the FCU.
  bool post_offboard_stream_done_{false};

  // ── Thread safety ────────────────────────────────────────────────────────
  std::mutex mutex_;

  // ── Parameter callback handle ────────────────────────────────────────────
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ── Command queue and IDs ────────────────────────────────────────────────
  CommandQueue cmd_queue_;
  std::optional<uint64_t> offboard_cmd_id_;
  std::optional<uint64_t> arm_cmd_id_;
  std::optional<uint64_t> disarm_cmd_id_;
  std::optional<uint64_t> takeoff_cmd_id_;
  std::optional<uint64_t> hover_cmd_id_;
  std::optional<uint64_t> trajectory_cmd_id_;
  std::optional<uint64_t> land_cmd_id_;
};

}  // namespace drone_control

#endif  // MY_DRONE_CONTROLLER__DRONE_CONTROLLER_COMPLETO_HPP_
