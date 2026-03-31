#include "my_drone_controller/drone_controller_completo.hpp"

#include <cmath>
#include <functional>
#include <string>

using namespace std::chrono_literals;

namespace drone_control {

// ============================================================
// CONSTRUCTOR
// ============================================================

DroneControllerCompleto::DroneControllerCompleto()
: Node("drone_controller_completo")
{
  RCLCPP_INFO(this->get_logger(), "\n");
  RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════════════════════════╗");
  RCLCPP_INFO(this->get_logger(), "║      🚁 CONTROLADOR INTELIGENTE DE DRONE - MODULAR       ║");
  RCLCPP_INFO(this->get_logger(), "║     COM ATIVAÇÃO OFFBOARD + ARM + DETECÇÃO DE POUSO     ║");
  RCLCPP_INFO(this->get_logger(), "║      FSM 5 ESTADOS — FLUXO DE POUSO ÚNICO (DISARM)      ║");
  RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════════════════════════╝\n");

  load_parameters();
  setup_publishers();
  setup_subscribers();
  setup_services();
  init_variables();

  RCLCPP_INFO(this->get_logger(), "\n📊 STATUS INICIAL:");
  RCLCPP_INFO(this->get_logger(), "   Estado: %d (aguardando waypoint)", state_voo_);
  RCLCPP_INFO(this->get_logger(), "   Controlador: %s", controlador_ativo_ ? "ATIVO" : "INATIVO");
  RCLCPP_INFO(this->get_logger(), "   Pouso: %s\n", pouso_em_andamento_ ? "SIM" : "NÃO");
}

// ============================================================
// SETUP
// ============================================================

void DroneControllerCompleto::load_parameters()
{
  this->declare_parameter("hover_altitude",        config_.hover_altitude);
  this->declare_parameter("hover_altitude_margin", config_.hover_altitude_margin);
  this->declare_parameter("max_altitude",          config_.max_altitude);
  this->declare_parameter("min_altitude",          config_.min_altitude);
  this->declare_parameter("waypoint_duration",     config_.waypoint_duration);
  this->declare_parameter("max_waypoint_distance", config_.max_waypoint_distance);
  this->declare_parameter("land_z_threshold",      config_.land_z_threshold);
  this->declare_parameter("activation_timeout",      config_.activation_timeout);
  this->declare_parameter("command_timeout",         config_.command_timeout);
  this->declare_parameter("landing_timeout",         config_.landing_timeout);
  this->declare_parameter("offboard_confirm_timeout", config_.offboard_confirm_timeout);
  this->declare_parameter("takeoff_z_boost",         config_.takeoff_z_boost);
  this->declare_parameter("takeoff_xy_origin_threshold_m", config_.takeoff_xy_origin_threshold_m);
  this->declare_parameter("latch_pose_max_age_s",         config_.latch_pose_max_age_s);

  config_.hover_altitude          = this->get_parameter("hover_altitude").as_double();
  config_.hover_altitude_margin   = this->get_parameter("hover_altitude_margin").as_double();
  config_.max_altitude            = this->get_parameter("max_altitude").as_double();
  config_.min_altitude            = this->get_parameter("min_altitude").as_double();
  config_.waypoint_duration       = this->get_parameter("waypoint_duration").as_double();
  config_.max_waypoint_distance   = this->get_parameter("max_waypoint_distance").as_double();
  config_.land_z_threshold        = this->get_parameter("land_z_threshold").as_double();
  config_.activation_timeout      = this->get_parameter("activation_timeout").as_double();
  config_.command_timeout         = this->get_parameter("command_timeout").as_double();
  config_.landing_timeout         = this->get_parameter("landing_timeout").as_double();
  config_.offboard_confirm_timeout = this->get_parameter("offboard_confirm_timeout").as_double();
  config_.takeoff_z_boost         = this->get_parameter("takeoff_z_boost").as_double();
  config_.takeoff_xy_origin_threshold_m =
    this->get_parameter("takeoff_xy_origin_threshold_m").as_double();
  config_.latch_pose_max_age_s    = this->get_parameter("latch_pose_max_age_s").as_double();

  this->declare_parameter<bool>("enabled", true);
  enabled_ = this->get_parameter("enabled").as_bool();
  RCLCPP_INFO(this->get_logger(), "⚙️  enabled=%s", enabled_ ? "true" : "false");

  this->declare_parameter<bool>("override_active", false);
  override_active_ = this->get_parameter("override_active").as_bool();
  RCLCPP_INFO(this->get_logger(), "⚙️  override_active=%s", override_active_ ? "true" : "false");

  this->declare_parameter<double>("monitor_waypoint_goal_rate_hz", 5.0);
  this->declare_parameter<double>("monitor_waypoints_rate_hz",     1.0);
  this->declare_parameter<bool>  ("monitor_publish_only_when_active", true);
  monitor_waypoint_goal_rate_hz_    = this->get_parameter("monitor_waypoint_goal_rate_hz").as_double();
  monitor_waypoints_rate_hz_        = this->get_parameter("monitor_waypoints_rate_hz").as_double();
  monitor_publish_only_when_active_ = this->get_parameter("monitor_publish_only_when_active").as_bool();
  RCLCPP_INFO(this->get_logger(),
    "⚙️  monitor_waypoint_goal_rate_hz=%.1f  monitor_waypoints_rate_hz=%.1f"
    "  monitor_publish_only_when_active=%s",
    monitor_waypoint_goal_rate_hz_, monitor_waypoints_rate_hz_,
    monitor_publish_only_when_active_ ? "true" : "false");

  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&DroneControllerCompleto::onSetParameters, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "⚙️  Configuração carregada:");
  RCLCPP_INFO(this->get_logger(), "   hover_altitude=%.2f m  margin=%.3f m",
    config_.hover_altitude, config_.hover_altitude_margin);
  RCLCPP_INFO(this->get_logger(), "   waypoint_duration=%.1f s  command_timeout=%.1f s",
    config_.waypoint_duration, config_.command_timeout);
  RCLCPP_INFO(this->get_logger(),
    "⚙️  Configuração de Altitude: Mínima=%.2f m | Pouso detectado: Z < %.2f m | Máxima=%.2f m",
    config_.min_altitude, config_.land_z_threshold, config_.max_altitude);
  RCLCPP_INFO(this->get_logger(),
    "⚙️  Boost de decolagem: takeoff_z_boost=%.2f m "
    "(Z_alvo ≥ Z_real + takeoff_z_boost após ARM, evita auto-disarm do PX4)",
    config_.takeoff_z_boost);
}

void DroneControllerCompleto::setup_publishers()
{
  raw_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>(
    "/uav1/mavros/setpoint_raw/local", 10);
  trajectory_finished_pub_ = this->create_publisher<std_msgs::msg::Bool>("/trajectory_finished", 10);
  progress_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/trajectory_progress", 10);
  waypoint_reached_pub_ = this->create_publisher<std_msgs::msg::Int32>("/waypoint_reached", 10);
  mission_latch_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/mission_latch_pose", 10);
  waypoint_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/waypoint_goal", 10);
  waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints", 10);
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /uav1/mavros/setpoint_raw/local");
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /trajectory_finished");
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /trajectory_progress");
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /waypoint_reached");
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /mission_latch_pose");
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /waypoint_goal");
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /waypoints");
}

void DroneControllerCompleto::setup_subscribers()
{
  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    "/uav1/mavros/state", 10,
    [this](const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; });

  waypoints_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/waypoints", 1,
    std::bind(&DroneControllerCompleto::waypoints_callback, this, std::placeholders::_1));

  mission_waypoints_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/mission_waypoints", 1,
    std::bind(&DroneControllerCompleto::mission_waypoints_callback, this, std::placeholders::_1));

  waypoint_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/waypoint_goal", 1,
    std::bind(&DroneControllerCompleto::waypoint_goal_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/uav1/mavros/local_position/odom", 10,
    std::bind(&DroneControllerCompleto::odometry_callback, this, std::placeholders::_1));

  yaw_override_sub_ = this->create_subscription<drone_control::msg::YawOverride>(
    "/uav1/yaw_override/cmd", 10,
    std::bind(&DroneControllerCompleto::yaw_override_callback, this, std::placeholders::_1));

  waypoint_goal_4d_sub_ = this->create_subscription<drone_control::msg::Waypoint4D>(
    "/waypoint_goal_4d", 1,
    std::bind(&DroneControllerCompleto::waypoint_goal_4d_callback, this, std::placeholders::_1));

  waypoints_4d_sub_ = this->create_subscription<drone_control::msg::Waypoint4DArray>(
    "/waypoints_4d", 1,
    std::bind(&DroneControllerCompleto::waypoints_4d_callback, this, std::placeholders::_1));

  mission_interrupt_done_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/mission_interrupt_done", 10,
    std::bind(&DroneControllerCompleto::mission_interrupt_done_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
    "✓ Subscribers criados: /uav1/mavros/state, /waypoints, /mission_waypoints, /waypoint_goal, "
    "odometria, /uav1/yaw_override/cmd, /waypoint_goal_4d, /waypoints_4d e /mission_interrupt_done");
}

void DroneControllerCompleto::setup_services()
{
  mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/uav1/mavros/set_mode");
  arm_client_  = this->create_client<mavros_msgs::srv::CommandBool>("/uav1/mavros/cmd/arming");
  RCLCPP_INFO(this->get_logger(), "✓ Service Clients criados: set_mode e arming");

  RCLCPP_INFO(this->get_logger(), "⏳ Aguardando serviços MAVROS...");
  while (!mode_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "⏳ Aguardando /uav1/mavros/set_mode...");
  }
  RCLCPP_INFO(this->get_logger(), "✓ Serviço set_mode disponível");

  while (!arm_client_->wait_for_service(1s)) {
    RCLCPP_WARN(this->get_logger(), "⏳ Aguardando /uav1/mavros/cmd/arming...");
  }
  RCLCPP_INFO(this->get_logger(), "✓ Serviço arming disponível\n");

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&DroneControllerCompleto::control_loop, this));
  RCLCPP_INFO(this->get_logger(), "✓ Timer criado: 100 Hz (10ms)");

  // ── Monitoring heartbeat timers ─────────────────────────────────────────
  if (monitor_waypoint_goal_rate_hz_ > 0.0) {
    auto period_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double, std::milli>(1000.0 / monitor_waypoint_goal_rate_hz_));
    monitor_waypoint_goal_timer_ = this->create_wall_timer(
      period_ms,
      std::bind(&DroneControllerCompleto::monitor_waypoint_goal_heartbeat, this));
    RCLCPP_INFO(this->get_logger(),
      "✓ Heartbeat /waypoint_goal: %.1f Hz", monitor_waypoint_goal_rate_hz_);
  }
  if (monitor_waypoints_rate_hz_ > 0.0) {
    auto period_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double, std::milli>(1000.0 / monitor_waypoints_rate_hz_));
    monitor_waypoints_timer_ = this->create_wall_timer(
      period_ms,
      std::bind(&DroneControllerCompleto::monitor_waypoints_heartbeat, this));
    RCLCPP_INFO(this->get_logger(),
      "✓ Heartbeat /waypoints: %.1f Hz", monitor_waypoints_rate_hz_);
  }
}

void DroneControllerCompleto::init_variables()
{
  state_voo_ = 0;
  controlador_ativo_ = false;
  pouso_em_andamento_ = false;
  cycle_count_ = 0;
  offboard_activated_ = false;
  offboard_mode_confirmed_ = false;
  arm_requested_ = false;
  activation_confirmed_ = false;
  takeoff_counter_ = 0;
  takeoff_target_z_ = -1.0;  // -1.0 = sentinel: computed at takeoff-command-receive time
  waypoint_goal_received_ = false;
  last_z_ = 0.0;
  pouso_start_time_set_ = false;
  pouso_start_time_ = this->now();
  disarm_requested_ = false;
  last_waypoint_goal_.pose.position.x = 0.0;
  last_waypoint_goal_.pose.position.y = 0.0;
  last_waypoint_goal_.pose.position.z = config_.hover_altitude;
  trajectory_setpoint_[0] = 0.0;
  trajectory_setpoint_[1] = 0.0;
  trajectory_setpoint_[2] = config_.hover_altitude;
  trajectory_waypoints_.clear();
  trajectory_started_ = false;
  current_waypoint_idx_ = 0;
  waypoint_duration_ = config_.waypoint_duration;
  last_waypoint_reached_idx_ = -1;
  mission_cycle_phase_ = MissionCyclePhase::NONE;
  mission_wp_follow_idx_ = 0;
  last_published_mission_wp_idx_ = -1;
  mission_waypoints_.clear();
  current_z_real_ = 0.0;
  current_x_ned_ = 0.0;
  current_y_ned_ = 0.0;
  current_z_ned_ = 0.0;
  has_latch_pose_ = false;
  yaw_override_enabled_ = false;
  yaw_rate_cmd_ = 0.0;
  yaw_override_timeout_s_ = 0.3;
  hold_valid_ = false;
  hold_x_ned_ = 0.0;
  hold_y_ned_ = 0.0;
  hold_z_ned_ = 0.0;
  last_yaw_cmd_time_ = this->now();
  current_yaw_rad_ = 0.0;
  goal_yaw_rad_ = 0.0;
  using_4d_goal_ = false;
  trajectory_4d_mode_ = false;
  initial_stream_count_ = 0;
  initial_stream_done_ = false;
  post_offboard_stream_count_ = 0;
  post_offboard_stream_done_ = false;
  setpoint_pub_time_initialized_ = false;
}

// ============================================================
// LANDING / ACTIVATION HELPERS
// ============================================================

void DroneControllerCompleto::trigger_landing(double z)
{
  // Note: controlador_ativo_ is intentionally NOT set to false here.
  // Initiating a landing must not also initiate a pause of the controller.
  // The trajectory controller is implicitly deactivated by the state_voo_ = 4
  // transition; the takeoff handler resets controlador_ativo_ before the FSM
  // can re-enter state 2 (hover) on the next flight cycle.
  pouso_em_andamento_ = true;
  state_voo_ = 4;
  land_cmd_id_ = cmd_queue_.enqueue(CommandType::LAND, {{"z", std::to_string(z)}});
}

void DroneControllerCompleto::activate_offboard_arm_if_needed()
{
  if (current_state_.armed && current_state_.mode == "OFFBOARD") {
    RCLCPP_INFO(this->get_logger(),
      "🔋 Já OFFBOARD+ARM — reutilizando ativação existente para levantamento.");
    // Skip the full streaming/OFFBOARD/ARM sequence: the FCU is already in
    // OFFBOARD mode and armed, so all activation flags are set to reflect
    // the confirmed state and the FSM jumps straight to takeoff climb.
    initial_stream_done_ = true;
    offboard_activated_ = true;
    offboard_mode_confirmed_ = true;
    post_offboard_stream_done_ = true;
    arm_requested_ = true;
    return;
  }
  // Reset activation flags so handle_state1_takeoff() re-runs the full
  // streaming → OFFBOARD → wait-for-OFFBOARD → ARM sequence for this new takeoff attempt.
  RCLCPP_INFO(this->get_logger(),
    "🔋 Preparando streaming inicial de setpoints para OFFBOARD+ARM...");
  offboard_activated_ = false;
  offboard_mode_confirmed_ = false;
  arm_requested_ = false;
  activation_confirmed_ = false;
  initial_stream_count_ = 0;
  initial_stream_done_ = false;
  post_offboard_stream_count_ = 0;
  post_offboard_stream_done_ = false;
  // ARM and OFFBOARD are NOT requested here.  They will be triggered by
  // handle_state1_takeoff() only after stream_initial_setpoints() has
  // published INITIAL_STREAM_THRESHOLD setpoints to satisfy the PX4 FCU.
}

// ============================================================
// PARAMETER HANDLERS
// ============================================================

bool DroneControllerCompleto::apply_enabled_param(
  const rclcpp::Parameter & p, rcl_interfaces::msg::SetParametersResult & result)
{
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
    result.successful = false;
    result.reason = "enabled deve ser bool (true ou false)";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  bool old_val = enabled_;
  enabled_ = p.as_bool();
  if (old_val != enabled_) {
    if (enabled_) {
      RCLCPP_INFO(this->get_logger(),
        "✅ enabled atualizado: false → true (publicação de setpoints RETOMADA)");
    } else {
      RCLCPP_INFO(this->get_logger(),
        "🚫 enabled atualizado: true → false (publicação de setpoints PAUSADA)");
    }
  }
  return true;
}

bool DroneControllerCompleto::apply_override_active_param(
  const rclcpp::Parameter & p, rcl_interfaces::msg::SetParametersResult & result)
{
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
    result.successful = false;
    result.reason = "override_active deve ser bool (true ou false)";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  bool old_val = override_active_;
  override_active_ = p.as_bool();
  if (old_val != override_active_) {
    if (override_active_) {
      hold_x_ned_ = current_x_ned_;
      hold_y_ned_ = current_y_ned_;
      hold_z_ned_ = current_z_ned_;
      hold_valid_ = true;
      RCLCPP_INFO(this->get_logger(),
        "🔒 override_active: false → true (override externo ATIVO: FSM congelada, publicando hold setpoint X=%.2f Y=%.2f Z=%.2f)",
        hold_x_ned_, hold_y_ned_, hold_z_ned_);
    } else {
      RCLCPP_INFO(this->get_logger(),
        "🔓 override_active: true → false (override externo DESATIVADO: operação normal RETOMADA)");
    }
  }
  return true;
}

rcl_interfaces::msg::SetParametersResult DroneControllerCompleto::onSetParameters(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : params) {
    if (p.get_name() == "enabled") {
      if (!apply_enabled_param(p, result)) { return result; }
    } else if (p.get_name() == "override_active") {
      if (!apply_override_active_param(p, result)) { return result; }
    }
  }
  return result;
}

// ============================================================
// FCU SERVICE REQUESTS
// ============================================================

void DroneControllerCompleto::request_offboard()
{
  if (!mode_client_->service_is_ready()) { return; }

  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = "OFFBOARD";

  uint64_t cmd_id = cmd_queue_.enqueue(
    CommandType::SET_MODE_OFFBOARD, {{"mode", "OFFBOARD"}});
  offboard_cmd_id_ = cmd_id;

  std::weak_ptr<DroneControllerCompleto> weak_self(
    std::static_pointer_cast<DroneControllerCompleto>(shared_from_this()));

  auto callback = [weak_self, cmd_id](
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
    auto self = weak_self.lock();
    if (!self) { return; }
    auto result = future.get();
    bool mode_set_success = result && result->mode_sent;
    self->cmd_queue_.confirm(cmd_id, mode_set_success);
    if (mode_set_success) {
      RCLCPP_INFO(self->get_logger(),
        "✅ [ID=%lu] SET_MODE OFFBOARD aceito pelo FCU", cmd_id);
    } else {
      RCLCPP_WARN(self->get_logger(),
        "⚠️ [ID=%lu] SET_MODE OFFBOARD rejeitado pelo FCU", cmd_id);
    }
  };

  mode_client_->async_send_request(request, callback);
  RCLCPP_INFO(this->get_logger(), "📡 [ID=%lu] Solicitando OFFBOARD MODE...", cmd_id);
}

void DroneControllerCompleto::request_arm()
{
  if (!arm_client_->service_is_ready()) { return; }

  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = true;

  uint64_t cmd_id = cmd_queue_.enqueue(CommandType::ARM);
  arm_cmd_id_ = cmd_id;

  std::weak_ptr<DroneControllerCompleto> weak_self(
    std::static_pointer_cast<DroneControllerCompleto>(shared_from_this()));

  auto callback = [weak_self, cmd_id](
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
    auto self = weak_self.lock();
    if (!self) { return; }
    auto result = future.get();
    bool arm_confirmed = result && result->success;
    self->cmd_queue_.confirm(cmd_id, arm_confirmed);
    if (arm_confirmed) {
      RCLCPP_INFO(self->get_logger(), "✅ [ID=%lu] ARM confirmado pelo FCU", cmd_id);
    } else {
      RCLCPP_WARN(self->get_logger(), "⚠️ [ID=%lu] ARM rejeitado pelo FCU", cmd_id);
    }
  };

  arm_client_->async_send_request(request, callback);
  RCLCPP_INFO(this->get_logger(), "🔋 [ID=%lu] Solicitando ARM...", cmd_id);
}

void DroneControllerCompleto::request_disarm()
{
  if (!arm_client_->service_is_ready()) { return; }

  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = false;

  uint64_t cmd_id = cmd_queue_.enqueue(CommandType::DISARM);
  disarm_cmd_id_ = cmd_id;

  std::weak_ptr<DroneControllerCompleto> weak_self(
    std::static_pointer_cast<DroneControllerCompleto>(shared_from_this()));

  auto callback = [weak_self, cmd_id](
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
    auto self = weak_self.lock();
    if (!self) { return; }
    auto result = future.get();
    bool disarm_confirmed = result && result->success;
    self->cmd_queue_.confirm(cmd_id, disarm_confirmed);
    if (disarm_confirmed) {
      RCLCPP_INFO(self->get_logger(), "✅ [ID=%lu] DISARM confirmado pelo FCU", cmd_id);
    } else {
      RCLCPP_WARN(self->get_logger(), "⚠️ [ID=%lu] DISARM rejeitado pelo FCU", cmd_id);
    }
  };

  arm_client_->async_send_request(request, callback);
  RCLCPP_INFO(this->get_logger(), "🔴 [ID=%lu] Solicitando DISARM...", cmd_id);
}

// ============================================================
// SENSOR CALLBACKS
// ============================================================

void DroneControllerCompleto::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_x_ned_ = msg->pose.pose.position.x;
  current_y_ned_ = msg->pose.pose.position.y;
  current_z_ned_ = msg->pose.pose.position.z;
  current_z_real_ = std::abs(current_z_ned_);
  const auto & q = msg->pose.pose.orientation;
  current_yaw_rad_ = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void DroneControllerCompleto::yaw_override_callback(
  const drone_control::msg::YawOverride::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->enable) {
    if (!yaw_override_enabled_) {
      hold_x_ned_ = current_x_ned_;
      hold_y_ned_ = current_y_ned_;
      hold_z_ned_ = current_z_ned_;
      hold_valid_ = true;
      RCLCPP_INFO(this->get_logger(),
        "🔒 Yaw override ATIVADO: hold X=%.2f Y=%.2f Z=%.2f (NED), yaw_rate=%.3f rad/s",
        hold_x_ned_, hold_y_ned_, hold_z_ned_, static_cast<double>(msg->yaw_rate));
    }
    yaw_override_enabled_ = true;
    yaw_rate_cmd_ = static_cast<double>(msg->yaw_rate);
    if (msg->timeout > 0.0f) {
      yaw_override_timeout_s_ = static_cast<double>(msg->timeout);
    }
  } else {
    if (yaw_override_enabled_) {
      RCLCPP_INFO(this->get_logger(),
        "🔓 Yaw override DESATIVADO: retomando operação normal da FSM.");
    }
    yaw_override_enabled_ = false;
    yaw_rate_cmd_ = 0.0;
  }
  last_yaw_cmd_time_ = this->now();
}

// ============================================================
// WAYPOINT CALLBACK HELPERS (3D)
// ============================================================

void DroneControllerCompleto::handle_landing_waypoint_command(double z)
{
  RCLCPP_WARN(this->get_logger(), "\n🛬🛬🛬 POUSO DETECTADO! Z_final = %.2f m", z);
  trigger_landing(z);
  RCLCPP_WARN(this->get_logger(), "📋 [ID=%lu] Comando LAND enfileirado", *land_cmd_id_);
  RCLCPP_WARN(this->get_logger(),
    "🛬 POUSANDO — delegando descida ao drone_soft_land\n");
}

geometry_msgs::msg::Pose DroneControllerCompleto::sanitize_takeoff_xy(
  const geometry_msgs::msg::Pose & pose)
{
  double xy_dist = std::hypot(pose.position.x, pose.position.y);
  // last_latch_pose_ is sourced from trajectory_waypoints_[...] which has already
  // passed validate_pose() in the waypoint callbacks, so its coordinates are valid.
  if (xy_dist < config_.takeoff_xy_origin_threshold_m && has_latch_pose_) {
    double age = (this->now() - last_latch_pose_time_).seconds();
    if (age < config_.latch_pose_max_age_s) {
      geometry_msgs::msg::Pose sanitized = pose;
      sanitized.position.x = last_latch_pose_.position.x;
      sanitized.position.y = last_latch_pose_.position.y;
      RCLCPP_WARN(this->get_logger(),
        "🛡️ [TAKEOFF XY SANITIZE] XY solicitado (%.3f, %.3f) está a %.3f m da origem "
        "(limiar=%.2f m) – substituído pelo latch pose (%.3f, %.3f) [age=%.1fs < %.1fs]",
        pose.position.x, pose.position.y, xy_dist,
        config_.takeoff_xy_origin_threshold_m,
        sanitized.position.x, sanitized.position.y,
        age, config_.latch_pose_max_age_s);
      return sanitized;
    }
    RCLCPP_WARN(this->get_logger(),
      "🛡️ [TAKEOFF XY SANITIZE] XY solicitado (%.3f, %.3f) próximo à origem, "
      "mas latch pose está desatualizada (age=%.1fs >= %.1fs) – usando XY solicitado",
      pose.position.x, pose.position.y, age, config_.latch_pose_max_age_s);
  } else {
    RCLCPP_DEBUG(this->get_logger(),
      "🛡️ [TAKEOFF XY] Usando XY solicitado: (%.3f, %.3f) [dist_origem=%.3f m]",
      pose.position.x, pose.position.y, xy_dist);
  }
  return pose;
}

void DroneControllerCompleto::handle_single_takeoff_waypoint_command(
  const geometry_msgs::msg::Pose & pose)
{
  RCLCPP_INFO(this->get_logger(), "\n⬆️ WAYPOINT DE LEVANTAMENTO recebido:");
  RCLCPP_INFO(this->get_logger(), "   Posição: X=%.2f, Y=%.2f, Z=%.2f",
    pose.position.x, pose.position.y, pose.position.z);

  log_takeoff_debug_flags("ANTES");

  // Guard against a near-origin XY that may have been generated by a publisher
  // whose latch pose was not yet available.  sanitize_takeoff_xy() overrides
  // the XY with the last known latch pose when the requested XY is within
  // takeoff_xy_origin_threshold_m of the origin AND the latch pose is fresh.
  const geometry_msgs::msg::Pose safe_pose = sanitize_takeoff_xy(pose);

  // Ensure takeoff_target_z_ is recomputed from the current measured altitude
  // (current_z_real_) every time a new takeoff command is received, whether it
  // is the first takeoff or a subsequent one after a landing cycle.
  // There is NO artificial differentiation between cycles — the same formula
  // always applies:  std::max(hover_altitude, current_z_real_ + takeoff_z_boost).
  // Computing it HERE (once, at command-receive time) and keeping it FIXED for
  // the entire climb prevents the infinite-ascent bug that would occur if the
  // target were recalculated every 10 ms control cycle (it would track the
  // rising drone and never be reached).
  takeoff_target_z_ = std::max(
    config_.hover_altitude,
    current_z_real_ + config_.takeoff_z_boost);
  RCLCPP_INFO(this->get_logger(),
    "⬆️ [TAKEOFF TARGET] Z_real=%.2fm | boost=+%.2fm | hover_alt=%.2fm → Z_alvo=%.2fm (fixo durante subida)",
    current_z_real_, config_.takeoff_z_boost, config_.hover_altitude, takeoff_target_z_);

  last_waypoint_goal_.pose = safe_pose;
  pouso_em_andamento_ = false;
  controlador_ativo_ = false;
  trajectory_started_ = false;
  trajectory_waypoints_.clear();
  trajectory_yaws_.clear();  // cleared defensively to avoid stale 4D yaws if mode changes
  current_waypoint_idx_ = 0;

  activate_offboard_arm_if_needed();

  takeoff_cmd_id_ = cmd_queue_.enqueue(
    CommandType::TAKEOFF,
    {{"x", std::to_string(safe_pose.position.x)},
     {"y", std::to_string(safe_pose.position.y)},
     {"z", std::to_string(config_.hover_altitude)}});
  RCLCPP_INFO(this->get_logger(),
    "📋 [ID=%lu] Comando TAKEOFF enfileirado", *takeoff_cmd_id_);

  // Publish the sanitized goal so /waypoint_goal monitoring is consistent.
  publish_waypoint_goal_status(
    safe_pose.position.x, safe_pose.position.y, safe_pose.position.z);

  state_voo_ = 1;
  takeoff_counter_ = 0;

  log_takeoff_debug_flags("DEPOIS");
}

void DroneControllerCompleto::log_trajectory_waypoints_3d(
  const std::vector<geometry_msgs::msg::Pose> & poses)
{
  RCLCPP_INFO(this->get_logger(),
    "✈️ WAYPOINTS DE TRAJETÓRIA armazenados: %zu pontos", poses.size());
  for (size_t i = 0; i < poses.size(); i++) {
    RCLCPP_INFO(this->get_logger(),
      "   WP[%zu]: X=%.2f, Y=%.2f, Z=%.2f",
      i, poses[i].position.x, poses[i].position.y, poses[i].position.z);
  }
  RCLCPP_INFO(this->get_logger(), " ");
}

void DroneControllerCompleto::activate_trajectory_in_hover(size_t waypoint_count)
{
  RCLCPP_INFO(this->get_logger(), "\n✅ TRAJETÓRIA ACEITA E ATIVADA! Drone em HOVER pronto!\n");
  if (hover_cmd_id_) {
    cmd_queue_.confirm(*hover_cmd_id_, true);
    RCLCPP_INFO(this->get_logger(),
      "✅ [ID=%lu] HOVER confirmado - iniciando trajetória", *hover_cmd_id_);
    hover_cmd_id_.reset();
  }
  trajectory_cmd_id_ = cmd_queue_.enqueue(
    CommandType::TRAJECTORY,
    {{"waypoints", std::to_string(waypoint_count)}});
  RCLCPP_INFO(this->get_logger(),
    "📋 [ID=%lu] Comando TRAJECTORY enfileirado (%zu WPs)",
    *trajectory_cmd_id_, waypoint_count);
  controlador_ativo_ = true;
  pouso_em_andamento_ = false;
  state_voo_ = 3;
  RCLCPP_INFO(this->get_logger(), "✅ Trajetória ativada - Entrando em ESTADO 3\n");
}

// ============================================================
// WAYPOINT CALLBACKS
// ============================================================

void DroneControllerCompleto::waypoints_callback(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Guard against receiving our own published monitoring echo.
  if (skip_self_waypoints_count_ > 0) { skip_self_waypoints_count_--; return; }

  trajectory_4d_mode_ = false;

  if (msg->poses.size() < 1) {
    RCLCPP_WARN(this->get_logger(), "❌ Waypoints insuficientes: %zu", msg->poses.size());
    return;
  }

  for (size_t i = 0; i < msg->poses.size(); ++i) {
    if (!validate_pose(msg->poses[i], config_)) {
      RCLCPP_WARN(this->get_logger(),
        "❌ Waypoint[%zu] inválido (NaN/Inf ou fora dos limites físicos) - mensagem ignorada", i);
      return;
    }
  }

  double last_z = msg->poses.back().position.z;

  if (msg->poses.size() == 1 && last_z < config_.land_z_threshold) {
    handle_landing_waypoint_command(last_z);
    return;
  }

  if (msg->poses.size() == 1 && last_z >= config_.land_z_threshold) {
    // Guard: do not start a new takeoff while the drone is still ARMED in state 4
    // (DISARM is pending confirmation from the FCU). If the drone is already
    // disarmed, handle_state4_disarm_reset() performs the reset and returns false
    // so we fall through and accept the new takeoff command immediately.
    if (handle_state4_disarm_reset()) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ [TAKEOFF] Ignorado: drone ainda armado em estado 4 (DISARM pendente). "
        "Aguardando confirmação de DISARM pelo FCU antes de aceitar novo takeoff.");
      return;
    }
    // Guard: warn if a new takeoff arrives while the drone is already in flight.
    // Hipóteses: operador enviou takeoff durante voo (estado 1/2/3) por engano,
    // ou o sistema de missão reiniciou sem esperar o pouso anterior completar.
    if (is_in_flight()) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ [TAKEOFF] Novo takeoff recebido enquanto drone em ESTADO %d (em voo). "
        "Estado inconsistente — reiniciando ciclo de decolagem. "
        "(Hipótese: missão reenviada antes do pouso ou crash de estado da FSM.)",
        state_voo_);
    }
    handle_single_takeoff_waypoint_command(msg->poses[0]);
    return;
  }

  if (msg->poses.size() >= 2) {
    if (state_voo_ == 4) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ Ignorando waypoints de trajetória durante pouso (estado %d)", state_voo_);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "🔍 Trajetória (2+ waypoints) recebida");
    RCLCPP_INFO(this->get_logger(), "   state_voo_=%d (esperado 2 para ativar)", state_voo_);

    trajectory_waypoints_ = msg->poses;
    trajectory_yaws_.clear();
    current_waypoint_idx_ = 0;
    trajectory_started_ = false;
    last_waypoint_reached_idx_ = -1;
    last_waypoint_goal_.pose = msg->poses[0];
    // Cancel any ongoing mission interrupt: new external trajectory replaces it.
    mission_cycle_phase_ = MissionCyclePhase::NONE;
    mission_waypoints_.clear();
    mission_wp_follow_idx_ = 0;
    last_published_mission_wp_idx_ = -1;

    log_trajectory_waypoints_3d(trajectory_waypoints_);
    publish_waypoints_status();

    if (state_voo_ != 2) {
      RCLCPP_INFO(this->get_logger(),
        "⏸️ Trajetória armazenada - Será ativada quando drone chegar em HOVER (ESTADO 2)");
      controlador_ativo_ = false;
      pouso_em_andamento_ = false;
      return;
    }

    activate_trajectory_in_hover(msg->poses.size());
  }
}

// ============================================================
// MISSION CYCLE PHASE HELPERS
// ============================================================

/// Returns a human-readable name for each MissionCyclePhase value.
static const char * mission_cycle_phase_name(MissionCyclePhase p)
{
  switch (p) {
    case MissionCyclePhase::NONE:            return "NONE";
    case MissionCyclePhase::WAIT_LAND_WP:    return "WAIT_LAND_WP";
    case MissionCyclePhase::FOLLOW_LAND:     return "FOLLOW_LAND";
    case MissionCyclePhase::WAIT_TAKEOFF_WP: return "WAIT_TAKEOFF_WP";
    case MissionCyclePhase::FOLLOW_TAKEOFF:  return "FOLLOW_TAKEOFF";
    default:                                  return "UNKNOWN";
  }
}

// ============================================================
// MISSION WAYPOINTS CALLBACK (/mission_waypoints)
// ============================================================

void DroneControllerCompleto::mission_waypoints_callback(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (msg->poses.empty()) { return; }

  // Takeoff waypoint: single pose with z >= land_z_threshold.
  // Re-arms the drone without clearing the stored main trajectory so that
  // handle_state2_hover() can resume STATE 3 at the saved current_waypoint_idx_.
  if (msg->poses.size() == 1 &&
      msg->poses[0].position.z >= config_.land_z_threshold)
  {
    // Accept takeoff only when the mission cycle is in the WAIT_TAKEOFF_WP phase.
    // This phase is set when DISARM is confirmed after the landing, so the
    // takeoff waypoint is accepted even if the landing portion is already done.
    if (mission_cycle_phase_ != MissionCyclePhase::WAIT_TAKEOFF_WP) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ /mission_waypoints (takeoff) recebido em fase '%s' – ignorado. "
        "Esperado: WAIT_TAKEOFF_WP.",
        mission_cycle_phase_name(mission_cycle_phase_));
      return;
    }

    // Only trigger re-arm when waiting for a new command (state 0).
    // If DISARM is still pending (state 4, armed=true) wait for the FCU.
    if (handle_state4_disarm_reset()) { return; }
    if (state_voo_ != 0) { return; }  // already arming/flying, ignore duplicate

    // Guard against near-origin XY; override with latch pose when fresh.
    const geometry_msgs::msg::Pose safe_pose = sanitize_takeoff_xy(msg->poses[0]);

    takeoff_target_z_ = std::max(
      config_.hover_altitude,
      current_z_real_ + config_.takeoff_z_boost);

    last_waypoint_goal_.pose = safe_pose;
    pouso_em_andamento_ = false;
    controlador_ativo_ = false;
    // Intentionally do NOT clear trajectory_waypoints_ / trajectory_started_ /
    // current_waypoint_idx_ — the main trajectory must be preserved so that
    // handle_state2_hover() can resume it after the drone reaches altitude.

    // Advance phase: drone is now climbing; trajectory resumes at z >= MISSION_RESUME_ALTITUDE_M.
    mission_cycle_phase_ = MissionCyclePhase::FOLLOW_TAKEOFF;

    activate_offboard_arm_if_needed();

    takeoff_cmd_id_ = cmd_queue_.enqueue(
      CommandType::TAKEOFF,
      {{"x", std::to_string(safe_pose.position.x)},
       {"y", std::to_string(safe_pose.position.y)},
       {"z", std::to_string(config_.hover_altitude)}});

    // Publish the sanitized goal so /waypoint_goal monitoring is consistent.
    publish_waypoint_goal_status(
      safe_pose.position.x, safe_pose.position.y, safe_pose.position.z);

    state_voo_ = 1;
    takeoff_counter_ = 0;
    RCLCPP_INFO(this->get_logger(),
      "🚀 [MISSION FOLLOW_TAKEOFF] Re-arm/decolagem iniciada (z_alvo=%.2f m). "
      "Retomará trajetória em WP[%d] após z >= %.1f m.",
      takeoff_target_z_, current_waypoint_idx_, MISSION_RESUME_ALTITUDE_M);
    return;
  }

  // Landing waypoints: store for use by handle_mission_interrupt_in_state3().
  // Accept only when the mission cycle is waiting for landing waypoints.
  if (mission_cycle_phase_ != MissionCyclePhase::WAIT_LAND_WP) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ /mission_waypoints (pouso) recebido em fase '%s' – ignorado. "
      "Esperado: WAIT_LAND_WP.",
      mission_cycle_phase_name(mission_cycle_phase_));
    return;
  }

  mission_waypoints_ = msg->poses;
  mission_wp_follow_idx_ = 0;
  mission_cycle_phase_ = MissionCyclePhase::FOLLOW_LAND;
  RCLCPP_INFO(this->get_logger(),
    "📍 [MISSION FOLLOW_LAND] %zu waypoints de pouso recebidos em /mission_waypoints.",
    msg->poses.size());
}

// ============================================================
// SHARED WAYPOINT-GOAL HELPERS
// ============================================================

bool DroneControllerCompleto::check_landing_in_flight(double z)
{
  if ((state_voo_ == 2 || state_voo_ == 3) && z < config_.land_z_threshold) {
    trigger_landing(z);
    RCLCPP_WARN(this->get_logger(),
      "🛬 [ID=%lu] POUSO DETECTADO! Z = %.2f m - Comando LAND enfileirado",
      *land_cmd_id_, z);
    return true;
  }
  return false;
}

bool DroneControllerCompleto::handle_state4_disarm_reset()
{
  if (state_voo_ != 4) { return false; }

  if (!current_state_.armed) {
    // Drone is already disarmed while the FSM is still in state 4 (this can
    // happen when a new command arrives between the FCU confirming DISARM and
    // the control-loop's handle_state4_landing() transitioning the state).
    // Transition to state 0 here and return FALSE so that the caller continues
    // to process the incoming takeoff command — without this the command would
    // always be silently dropped, requiring the operator to re-send it.
    RCLCPP_INFO(this->get_logger(),
      "✅ DRONE DESARMADO em estado 4 — transicionando para aguardar novo comando.");
    disarm_requested_ = false;
    state_voo_ = 0;
    return false;  // let the caller proceed with the new takeoff command
  }

  if (disarm_requested_) {
    // Drone is still armed and we are waiting for the FCU to confirm DISARM.
    // The new command must be dropped — accepting a takeoff while DISARM is
    // pending would conflict with the ongoing landing sequence.
    RCLCPP_INFO(this->get_logger(),
      "[DISARM] Aguardando confirmação de DISARM pelo FCU (armed=1); ignorando waypoint recebido.");
  }
  // State 4 and drone still armed: signal to caller that this was a landing
  // state and the command should not be processed yet.
  return true;
}

void DroneControllerCompleto::log_takeoff_debug_flags(const char * tag)
{
  RCLCPP_INFO(this->get_logger(),
    "🔍 DEBUG FLAGS %s: state_voo_=%d offboard_activated_=%d "
    "offboard_mode_confirmed_=%d arm_requested_=%d activation_confirmed_=%d "
    "disarm_requested_=%d takeoff_cmd_id_=%s takeoff_counter_=%d current_waypoint_idx_=%d",
    tag, state_voo_,
    static_cast<int>(offboard_activated_),
    static_cast<int>(offboard_mode_confirmed_),
    static_cast<int>(arm_requested_),
    static_cast<int>(activation_confirmed_),
    static_cast<int>(disarm_requested_),
    takeoff_cmd_id_ ? "set" : "empty",
    takeoff_counter_, current_waypoint_idx_);
}

// ============================================================
// WAYPOINT-GOAL CALLBACKS
// ============================================================

void DroneControllerCompleto::waypoint_goal_callback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Guard against receiving our own published monitoring echo.
  if (skip_self_waypoint_goal_count_ > 0) { skip_self_waypoint_goal_count_--; return; }

  using_4d_goal_ = false;

  if (!validate_waypoint(*msg, config_)) {
    RCLCPP_WARN(this->get_logger(),
      "❌ waypoint_goal inválido (NaN/Inf ou fora dos limites físicos) - ignorado");
    return;
  }

  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  double z = msg->pose.position.z;
  last_z_ = z;

  if (check_landing_in_flight(z)) { return; }
  if (handle_state4_disarm_reset()) { return; }

  if (state_voo_ == 3) {
    trajectory_setpoint_[0] = x;
    trajectory_setpoint_[1] = y;
    trajectory_setpoint_[2] = z;
    controlador_ativo_ = true;
    pouso_em_andamento_ = false;
    publish_waypoint_goal_status(x, y, z);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "\n📍 NOVO WAYPOINT RECEBIDO: X=%.2f, Y=%.2f, Z=%.2f", x, y, z);

  last_waypoint_goal_ = *msg;
  waypoint_goal_received_ = true;
  controlador_ativo_ = true;
  pouso_em_andamento_ = false;
  publish_waypoint_goal_status(x, y, z);
}

void DroneControllerCompleto::waypoint_goal_4d_callback(
  const drone_control::msg::Waypoint4D::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  geometry_msgs::msg::PoseStamped ps;
  ps.pose = msg->pose;
  if (!validate_waypoint(ps, config_)) {
    RCLCPP_WARN(this->get_logger(),
      "❌ waypoint_goal_4d inválido (NaN/Inf ou fora dos limites físicos) - ignorado");
    return;
  }

  if (std::isnan(static_cast<double>(msg->yaw))) {
    goal_yaw_rad_ = current_yaw_rad_;
    RCLCPP_INFO(this->get_logger(),
      "📍 4D Waypoint: yaw=NaN → usando yaw atual (%.3f rad)", goal_yaw_rad_);
  } else {
    double raw_yaw = static_cast<double>(msg->yaw);
    goal_yaw_rad_ = std::atan2(std::sin(raw_yaw), std::cos(raw_yaw));
    RCLCPP_INFO(this->get_logger(),
      "📍 4D Waypoint: yaw=%.3f rad (normalizado para %.3f rad)", raw_yaw, goal_yaw_rad_);
  }

  using_4d_goal_ = true;
  trajectory_4d_mode_ = false;

  auto pose_stamped = std::make_shared<geometry_msgs::msg::PoseStamped>();
  pose_stamped->pose = msg->pose;
  pose_stamped->header.stamp = this->now();
  pose_stamped->header.frame_id = "map";

  double x = pose_stamped->pose.position.x;
  double y = pose_stamped->pose.position.y;
  double z = pose_stamped->pose.position.z;
  last_z_ = z;

  if (check_landing_in_flight(z)) { return; }

  if (handle_state4_disarm_reset()) { return; }

  if (state_voo_ == 3) {
    trajectory_setpoint_[0] = x;
    trajectory_setpoint_[1] = y;
    trajectory_setpoint_[2] = z;
    controlador_ativo_ = true;
    pouso_em_andamento_ = false;
    publish_waypoint_goal_status(x, y, z);
    return;
  }

  RCLCPP_INFO(this->get_logger(),
    "\n📍 4D WAYPOINT RECEBIDO: X=%.2f, Y=%.2f, Z=%.2f, yaw=%.3f rad",
    x, y, z, goal_yaw_rad_);

  last_waypoint_goal_ = *pose_stamped;
  waypoint_goal_received_ = true;
  controlador_ativo_ = true;
  pouso_em_andamento_ = false;
  publish_waypoint_goal_status(x, y, z);
}

void DroneControllerCompleto::waypoints_4d_callback(
  const drone_control::msg::Waypoint4DArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (msg->waypoints.size() < 1) {
    RCLCPP_WARN(this->get_logger(),
      "❌ Waypoints4D insuficientes: %zu", msg->waypoints.size());
    return;
  }

  for (size_t i = 0; i < msg->waypoints.size(); ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose = msg->waypoints[i].pose;
    if (!validate_waypoint(ps, config_)) {
      RCLCPP_WARN(this->get_logger(),
        "❌ Waypoint4D[%zu] inválido (NaN/Inf ou fora dos limites físicos) - mensagem ignorada", i);
      return;
    }
  }

  std::vector<geometry_msgs::msg::Pose> poses;
  std::vector<double> yaws;
  poses.reserve(msg->waypoints.size());
  yaws.reserve(msg->waypoints.size());
  for (const auto & wp : msg->waypoints) {
    poses.push_back(wp.pose);
    if (std::isnan(static_cast<double>(wp.yaw))) {
      yaws.push_back(current_yaw_rad_);
    } else {
      double raw_yaw = static_cast<double>(wp.yaw);
      yaws.push_back(std::atan2(std::sin(raw_yaw), std::cos(raw_yaw)));
    }
  }

  double last_z = poses.back().position.z;

  if (msg->waypoints.size() == 1 && last_z < config_.land_z_threshold) {
    RCLCPP_WARN(this->get_logger(), "\n🛬🛬🛬 4D POUSO DETECTADO! Z_final = %.2f m", last_z);
    trigger_landing(last_z);
    trajectory_4d_mode_ = false;
    return;
  }

  if (msg->waypoints.size() == 1) {
    // Guard: do not start a new 4D takeoff while the drone is still ARMED in
    // state 4 (DISARM pending). If the drone is already disarmed,
    // handle_state4_disarm_reset() performs the reset and returns false so we
    // fall through and accept the new takeoff command immediately.
    if (handle_state4_disarm_reset()) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ [4D TAKEOFF] Ignorado: drone ainda armado em estado 4 (DISARM pendente). "
        "Aguardando confirmação de DISARM pelo FCU antes de aceitar novo takeoff.");
      return;
    }
    // Guard: warn if a new takeoff arrives while the drone is already in flight.
    // Hipóteses: operador enviou takeoff durante voo (estado 1/2/3) por engano,
    // ou o sistema de missão reiniciou sem esperar o pouso anterior completar.
    if (is_in_flight()) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ [4D TAKEOFF] Novo takeoff recebido enquanto drone em ESTADO %d (em voo). "
        "Estado inconsistente — reiniciando ciclo de decolagem. "
        "(Hipótese: missão reenviada antes do pouso ou crash de estado da FSM.)",
        state_voo_);
    }

    RCLCPP_INFO(this->get_logger(),
      "\n⬆️ 4D WAYPOINT DE LEVANTAMENTO recebido: X=%.2f Y=%.2f Z=%.2f yaw=%.3f rad",
      poses[0].position.x, poses[0].position.y, poses[0].position.z, yaws[0]);

    // Compute and fix the takeoff target altitude at command-receive time.
    // No differentiation between the first takeoff and subsequent cycles —
    // the same formula always applies (see handle_single_takeoff_waypoint_command
    // for the full rationale).  The value is kept fixed throughout the climb.
    takeoff_target_z_ = std::max(
      config_.hover_altitude,
      current_z_real_ + config_.takeoff_z_boost);
    RCLCPP_INFO(this->get_logger(),
      "⬆️ [4D TAKEOFF TARGET] Z_real=%.2fm | boost=+%.2fm | hover_alt=%.2fm → Z_alvo=%.2fm (fixo durante subida)",
      current_z_real_, config_.takeoff_z_boost, config_.hover_altitude, takeoff_target_z_);

    goal_yaw_rad_ = yaws[0];
    using_4d_goal_ = true;
    trajectory_4d_mode_ = false;

    // Guard against near-origin XY; override with latch pose when fresh.
    geometry_msgs::msg::Pose safe_pose_4d = sanitize_takeoff_xy(poses[0]);

    geometry_msgs::msg::PoseStamped ps;
    ps.pose = safe_pose_4d;
    last_waypoint_goal_ = ps;

    pouso_em_andamento_ = false;
    controlador_ativo_ = false;
    trajectory_started_ = false;
    trajectory_waypoints_.clear();
    trajectory_yaws_.clear();
    current_waypoint_idx_ = 0;

    activate_offboard_arm_if_needed();
    takeoff_cmd_id_ = cmd_queue_.enqueue(
      CommandType::TAKEOFF,
      {{"x", std::to_string(safe_pose_4d.position.x)},
       {"y", std::to_string(safe_pose_4d.position.y)},
       {"z", std::to_string(config_.hover_altitude)}});
    state_voo_ = 1;
    takeoff_counter_ = 0;
    return;
  }

  if (state_voo_ == 4) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ Ignorando trajetória 4D durante pouso (estado %d)", state_voo_);
    return;
  }

  trajectory_waypoints_ = poses;
  trajectory_yaws_ = yaws;
  trajectory_4d_mode_ = true;
  using_4d_goal_ = false;
  current_waypoint_idx_ = 0;
  trajectory_started_ = false;
  last_waypoint_reached_idx_ = -1;

  geometry_msgs::msg::PoseStamped ps;
  ps.pose = poses[0];
  last_waypoint_goal_ = ps;

  RCLCPP_INFO(this->get_logger(),
    "✈️ 4D WAYPOINTS DE TRAJETÓRIA armazenados: %zu pontos", msg->waypoints.size());
  for (size_t i = 0; i < msg->waypoints.size(); i++) {
    RCLCPP_INFO(this->get_logger(),
      "   WP4D[%zu]: X=%.2f Y=%.2f Z=%.2f yaw=%.3f rad",
      i, poses[i].position.x, poses[i].position.y, poses[i].position.z, yaws[i]);
  }

  publish_waypoints_status();

  if (state_voo_ != 2) {
    RCLCPP_INFO(this->get_logger(),
      "⏸️ Trajetória 4D armazenada - Será ativada quando drone chegar em HOVER (ESTADO 2)");
    controlador_ativo_ = false;
    pouso_em_andamento_ = false;
    return;
  }

  activate_trajectory_in_hover(msg->waypoints.size());
}

// ============================================================
// MONITORING PUBLISHERS (/waypoint_goal, /waypoints)
// ============================================================

void DroneControllerCompleto::publish_waypoint_goal_status(double x, double y, double z)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  skip_self_waypoint_goal_count_++;
  waypoint_goal_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
    "📢 [/waypoint_goal] Publicado: X=%.2f, Y=%.2f, Z=%.2f", x, y, z);
}

void DroneControllerCompleto::publish_waypoints_status()
{
  if (trajectory_waypoints_.empty()) { return; }
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.poses = trajectory_waypoints_;
  skip_self_waypoints_count_++;
  waypoints_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
    "📢 [/waypoints] Publicando %zu waypoints de trajetória",
    trajectory_waypoints_.size());
}

void DroneControllerCompleto::monitor_waypoint_goal_heartbeat()
{
  std::lock_guard<std::mutex> lock(mutex_);

  // If configured to publish only when active, suppress when there is no active goal/flight.
  if (monitor_publish_only_when_active_ && !waypoint_goal_received_) { return; }

  double x = 0.0, y = 0.0, z = 0.0;

  // Source priority:
  // 1) FOLLOW_LAND phase with mission waypoints → current mission landing waypoint
  if (mission_cycle_phase_ == MissionCyclePhase::FOLLOW_LAND && !mission_waypoints_.empty()) {
    int idx = mission_wp_follow_idx_;
    if (idx < 0) { idx = 0; }
    if (static_cast<size_t>(idx) >= mission_waypoints_.size()) {
      idx = static_cast<int>(mission_waypoints_.size()) - 1;
    }
    const auto & p = mission_waypoints_[idx].position;
    x = p.x; y = p.y; z = p.z;
  }
  // 2) Trajectory active with waypoints → current trajectory waypoint (clamped)
  else if (state_voo_ == 3 && !trajectory_waypoints_.empty()) {
    int idx = current_waypoint_idx_;
    if (idx < 0) { idx = 0; }
    if (static_cast<size_t>(idx) >= trajectory_waypoints_.size()) {
      idx = static_cast<int>(trajectory_waypoints_.size()) - 1;
    }
    const auto & p = trajectory_waypoints_[idx].position;
    x = p.x; y = p.y; z = p.z;
  }
  // 3) Fall back to the last received waypoint goal
  else {
    const auto & p = last_waypoint_goal_.pose.position;
    x = p.x; y = p.y; z = p.z;
  }

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.pose.position.x = x;
  msg.pose.position.y = y;
  msg.pose.position.z = z;
  skip_self_waypoint_goal_count_++;
  waypoint_goal_pub_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(),
    "[heartbeat /waypoint_goal] X=%.2f Y=%.2f Z=%.2f", x, y, z);
}

void DroneControllerCompleto::monitor_waypoints_heartbeat()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (trajectory_waypoints_.empty()) { return; }
  if (monitor_publish_only_when_active_ && !is_in_flight()) { return; }

  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.poses = trajectory_waypoints_;
  skip_self_waypoints_count_++;
  waypoints_pub_->publish(msg);
  RCLCPP_DEBUG(this->get_logger(),
    "[heartbeat /waypoints] %zu waypoints", trajectory_waypoints_.size());
}

// ============================================================
// SETPOINT PUBLISHING
// ============================================================

void DroneControllerCompleto::publishPositionTarget(
  double x, double y, double z, double yaw_rate, uint16_t type_mask)
{
  mavros_msgs::msg::PositionTarget pt;
  pt.header.stamp = this->now();
  pt.header.frame_id = "map";
  pt.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  pt.type_mask = type_mask;
  pt.position.x = static_cast<float>(x);
  pt.position.y = static_cast<float>(y);
  pt.position.z = static_cast<float>(z);
  pt.yaw_rate = static_cast<float>(yaw_rate);
  raw_pub_->publish(pt);
  // Watchdog: record time of last real publish
  last_setpoint_pub_time_ = pt.header.stamp;
  setpoint_pub_time_initialized_ = true;
}

void DroneControllerCompleto::publishPositionTargetWithYaw(
  double x, double y, double z, double yaw_rad)
{
  mavros_msgs::msg::PositionTarget pt;
  pt.header.stamp = this->now();
  pt.header.frame_id = "map";
  pt.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  pt.type_mask = MASK_POS_YAW;
  pt.position.x = static_cast<float>(x);
  pt.position.y = static_cast<float>(y);
  pt.position.z = static_cast<float>(z);
  pt.yaw = static_cast<float>(yaw_rad);
  raw_pub_->publish(pt);
  // Watchdog: record time of last real publish
  last_setpoint_pub_time_ = pt.header.stamp;
  setpoint_pub_time_initialized_ = true;
}

// ============================================================
// WATCHDOG HOLD SETPOINT
// ============================================================

void DroneControllerCompleto::publish_hold_setpoint()
{
  // Use captured hold position if valid; otherwise fall back to current odometry.
  if (hold_valid_) {
    publishPositionTarget(hold_x_ned_, hold_y_ned_, hold_z_ned_, 0.0, MASK_POS_ONLY);
  } else {
    publishPositionTarget(current_x_ned_, current_y_ned_, current_z_ned_, 0.0, MASK_POS_ONLY);
  }
}

// ============================================================
// MAIN CONTROL LOOP
// ============================================================

void DroneControllerCompleto::control_loop()
{
  std::lock_guard<std::mutex> lock(mutex_);

  cycle_count_++;

  const auto now = this->now();

  // Initialize watchdog timestamp on first iteration so it doesn't fire immediately.
  if (!setpoint_pub_time_initialized_) {
    last_setpoint_pub_time_ = now;
    setpoint_pub_time_initialized_ = true;
  }

  if (!enabled_) {
    // FSM is paused, but we must keep the MAVROS setpoint stream alive (≥20 Hz)
    // to prevent PX4 from dropping OFFBOARD mode.
    if ((now - last_setpoint_pub_time_).seconds() >= MAX_SETPOINT_SILENCE_S) {
      publish_hold_setpoint();
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "⏱️ Watchdog: enabled=false, publicando hold setpoint (>=20 Hz)");
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "🚫 Controller desabilitado (enabled=false): FSM pausada, mantendo stream mínimo (>=20Hz)");
    return;
  }

  // Watchdog: yaw override timeout
  if (yaw_override_enabled_) {
    double since_last = (this->now() - last_yaw_cmd_time_).seconds();
    if (since_last > yaw_override_timeout_s_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "⚠️ Yaw override timeout (%.2fs sem cmd): desativando override e zerando yaw_rate.",
        since_last);
      yaw_override_enabled_ = false;
      yaw_rate_cmd_ = 0.0;
    }
  }

  // Yaw override ativo: FSM congelada, publica hold + yaw_rate
  if (yaw_override_enabled_ && hold_valid_) {
    publishPositionTarget(hold_x_ned_, hold_y_ned_, hold_z_ned_, yaw_rate_cmd_, MASK_POS_YAWRATE);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "🔒 Yaw override ativo: hold X=%.2f Y=%.2f Z=%.2f  yaw_rate=%.3f rad/s",
      hold_x_ned_, hold_y_ned_, hold_z_ned_, yaw_rate_cmd_);
    return;
  }

  // Override externo ativo: FSM congelada, mas continua publicando hold setpoint
  if (override_active_) {
    if (hold_valid_) {
      publishPositionTarget(hold_x_ned_, hold_y_ned_, hold_z_ned_, 0.0, MASK_POS_ONLY);
    } else {
      publishPositionTarget(current_x_ned_, current_y_ned_, current_z_ned_, 0.0, MASK_POS_ONLY);
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "🔒 Override externo ativo (override_active=true): FSM congelada, publicando hold setpoint");
    return;
  }

  // Verificação periódica de timeouts e salvamento de log (~10 segundos)
  if (cycle_count_ % 1000 == 0) {
    auto timed_out = cmd_queue_.check_timeouts(config_.command_timeout);
    for (auto id : timed_out) {
      RCLCPP_WARN(this->get_logger(),
        "⏰ [ID=%lu] Comando TIMEOUT! (>%.0f s sem confirmação)", id, config_.command_timeout);
    }
    cmd_queue_.save_log("/tmp/drone_commands.log");
  }

  // Despacha para o método do estado atual
  switch (state_voo_) {
    case 0: handle_state0_wait_waypoint(); break;
    case 1: handle_state1_takeoff();       break;
    case 2: handle_state2_hover();         break;
    case 3: handle_state3_trajectory();    break;
    case 4: handle_state4_landing();       break;
    default:
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "❌ Estado FSM inválido: %d", state_voo_);
      break;
  }
}

// ============================================================
// FSM STATE 0 — AGUARDANDO WAYPOINT
// ============================================================

void DroneControllerCompleto::handle_state0_wait_waypoint()
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
    "⏳ Aguardando novo comando de waypoint para decolar...");
}

// ============================================================
// FSM STATE 1 — DECOLAGEM (3-layer sequence)
// ============================================================

void DroneControllerCompleto::request_arm_and_offboard_activation()
{
  // Per PX4/MAVROS best practice, OFFBOARD and ARM must be sent SEPARATELY.
  // Sending ARM immediately after (or simultaneously with) the OFFBOARD request
  // causes the FCU to reject ARM even when OFFBOARD is accepted, because PX4
  // enforces that the mode transition must be complete before it honours ARM.
  //
  // This function therefore requests ONLY the OFFBOARD mode change.  The ARM
  // command is sent later by handle_state1_takeoff(), after wait_for_offboard_mode()
  // confirms that the FCU has actually switched to OFFBOARD.
  RCLCPP_INFO(this->get_logger(),
    "📡 Solicitando modo OFFBOARD (ARM aguardará confirmação do FCU)...");
  request_offboard();
  offboard_activated_ = true;
  activation_time_ = this->now();
}

// ============================================================
// PRE-ARM SETPOINT STREAMING
// ============================================================

void DroneControllerCompleto::stream_initial_setpoints()
{
  // PX4/MAVROS requires a continuous stream of position setpoints to be
  // published on /uav1/mavros/setpoint_raw/local BEFORE the FCU will accept
  // an ARM command in OFFBOARD mode.  Without this pre-stream the FCU replies
  // "ARM rejected" immediately even though it successfully enters OFFBOARD.
  //
  // We publish INITIAL_STREAM_THRESHOLD setpoints (~200 ms at 100 Hz) at the
  // target takeoff position before triggering the OFFBOARD+ARM sequence.
  // After this point the control-loop timer keeps setpoints flowing, so ARM
  // will not be rejected on subsequent retry attempts either.
  //
  // Invariant: last_waypoint_goal_ is always set before state_voo_ is moved
  // to 1 (see handle_single_takeoff_waypoint_command / waypoints_4d_callback),
  // and init_variables() initialises it to (0, 0, hover_altitude) as a safe
  // fallback, so the published position is always well-defined.
  publishPositionTarget(
    last_waypoint_goal_.pose.position.x,
    last_waypoint_goal_.pose.position.y,
    last_waypoint_goal_.pose.position.z,
    0.0, MASK_POS_ONLY);

  initial_stream_count_++;

  if (initial_stream_count_ == 1) {
    RCLCPP_INFO(this->get_logger(),
      "📡 [STREAM] Iniciando streaming inicial de setpoints antes de ARM+OFFBOARD "
      "(meta: %d mensagens)...", INITIAL_STREAM_THRESHOLD);
  }

  if (initial_stream_count_ >= INITIAL_STREAM_THRESHOLD) {
    initial_stream_done_ = true;
    RCLCPP_INFO(this->get_logger(),
      "✅ [STREAM] Streaming inicial concluído (%d setpoints publicados). "
      "Prosseguindo com solicitação de OFFBOARD+ARM.", initial_stream_count_);
  }
}

void DroneControllerCompleto::stream_post_offboard_setpoints()
{
  // PX4/MAVROS rejects ARM even after accepting OFFBOARD mode if the setpoint
  // stream has not been active long enough.  The FCU requires at least 1.5 seconds
  // of continuous setpoints in OFFBOARD mode before it will honour an ARM request.
  //
  // This function is called every control-loop iteration (100 Hz / 10 ms) AFTER
  // the FCU confirms OFFBOARD and BEFORE ARM is requested.  It accumulates
  // POST_OFFBOARD_STREAM_THRESHOLD = 150 setpoints at 10 ms per call — 1.5 s
  // total at 100 Hz, providing the same duration as the PX4/MAVROS recommendation
  // of 30 setpoints at 50 ms each — then sets post_offboard_stream_done_ to allow
  // handle_state1_takeoff() to proceed to the ARM step.

  publishPositionTarget(
    last_waypoint_goal_.pose.position.x,
    last_waypoint_goal_.pose.position.y,
    last_waypoint_goal_.pose.position.z,
    0.0, MASK_POS_ONLY);

  post_offboard_stream_count_++;

  if (post_offboard_stream_count_ == 1) {
    RCLCPP_INFO(this->get_logger(),
      "📡 [STREAM] OFFBOARD confirmado — iniciando streaming prolongado de setpoints "
      "antes do ARM (%d setpoints × 10 ms = %.1f s)...",
      POST_OFFBOARD_STREAM_THRESHOLD,
      POST_OFFBOARD_STREAM_THRESHOLD * 0.01);
  }

  if (post_offboard_stream_count_ >= POST_OFFBOARD_STREAM_THRESHOLD) {
    post_offboard_stream_done_ = true;
    RCLCPP_INFO(this->get_logger(),
      "✅ [STREAM] Streaming pós-OFFBOARD concluído (%d setpoints / %.1f s). "
      "Enviando ARM agora...",
      post_offboard_stream_count_,
      post_offboard_stream_count_ * 0.01);
  }
}

void DroneControllerCompleto::wait_for_offboard_mode()
{
  // PX4/MAVROS requires that ARM be sent AFTER the FCU has confirmed the mode
  // change to OFFBOARD.  This function polls current_state_.mode and, once the
  // FCU reports "OFFBOARD", sets offboard_mode_confirmed_ so that
  // handle_state1_takeoff() can proceed to call request_arm() in the next cycle.
  // If the FCU does not confirm within offboard_confirm_timeout seconds the
  // OFFBOARD request is retried from scratch.

  if (current_state_.mode == "OFFBOARD") {
    RCLCPP_INFO(this->get_logger(),
      "✅ FCU confirmou modo OFFBOARD — aguardando ARM na próxima etapa...");
    offboard_mode_confirmed_ = true;
    // Reset activation_time_ so the ARM-wait timeout (activation_timeout) starts
    // counting only from the moment OFFBOARD was actually confirmed.
    activation_time_ = this->now();
    return;
  }

  if ((this->now() - activation_time_).seconds() > config_.offboard_confirm_timeout) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ Timeout esperando confirmação de modo OFFBOARD (%.0f s)! Tentando novamente...",
      config_.offboard_confirm_timeout);
    RCLCPP_WARN(this->get_logger(), "   Estado atual: mode=%s",
      current_state_.mode.c_str());
    // Reset so the next cycle re-requests OFFBOARD and starts fresh.
    offboard_activated_ = false;
    offboard_mode_confirmed_ = false;
    arm_requested_ = false;
    post_offboard_stream_count_ = 0;
    post_offboard_stream_done_ = false;
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "⏳ Aguardando FCU confirmar modo OFFBOARD... | mode=%s",
    current_state_.mode.c_str());
}

bool DroneControllerCompleto::wait_for_offboard_arm_confirmation()
{
  if (current_state_.armed && current_state_.mode == "OFFBOARD") {
    RCLCPP_INFO(this->get_logger(), "✅ OFFBOARD+ARM CONFIRMADOS! Iniciando decolagem...");
    activation_confirmed_ = true;
    takeoff_counter_ = 0;

    if (!takeoff_cmd_id_) {
      takeoff_cmd_id_ = cmd_queue_.enqueue(
        CommandType::TAKEOFF,
        {{"x", std::to_string(last_waypoint_goal_.pose.position.x)},
         {"y", std::to_string(last_waypoint_goal_.pose.position.y)},
         {"z", std::to_string(config_.hover_altitude)}});
      RCLCPP_INFO(this->get_logger(),
        "📋 [ID=%lu] Comando TAKEOFF enfileirado após ARM+OFFBOARD confirmado!", *takeoff_cmd_id_);
    }
    return true;
  }

  if ((this->now() - activation_time_).seconds() > config_.activation_timeout) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ Timeout ativação OFFBOARD+ARM (%.0f s)! Tentando novamente...",
      config_.activation_timeout);
    RCLCPP_WARN(this->get_logger(), "   Estado: armed=%d | mode=%s",
      current_state_.armed, current_state_.mode.c_str());
    // Reset all activation flags so the full sequence restarts from OFFBOARD request.
    offboard_activated_ = false;
    offboard_mode_confirmed_ = false;
    arm_requested_ = false;
    activation_confirmed_ = false;
    post_offboard_stream_count_ = 0;
    post_offboard_stream_done_ = false;
    takeoff_counter_ = 0;
    return false;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "⏳ Aguardando ARM... | armed=%d | mode=%s",
    current_state_.armed, current_state_.mode.c_str());
  takeoff_counter_++;
  return false;
}

void DroneControllerCompleto::publish_takeoff_climb_setpoint(double target_alt)
{
  if (using_4d_goal_) {
    publishPositionTargetWithYaw(
      last_waypoint_goal_.pose.position.x,
      last_waypoint_goal_.pose.position.y,
      target_alt, goal_yaw_rad_);
  } else {
    publishPositionTarget(
      last_waypoint_goal_.pose.position.x,
      last_waypoint_goal_.pose.position.y,
      target_alt, 0.0, MASK_POS_ONLY);
  }

  takeoff_counter_++;

  if (takeoff_counter_ == 1) {
    RCLCPP_INFO(this->get_logger(), "⬆️ Decolando para %.1f metros...", target_alt);
    RCLCPP_INFO(this->get_logger(), "   Posição: X=%.2f, Y=%.2f, Z=%.1f",
      last_waypoint_goal_.pose.position.x,
      last_waypoint_goal_.pose.position.y, target_alt);
  }

  if (takeoff_counter_ % 100 == 0) {
    RCLCPP_INFO(this->get_logger(),
      "📈 Decolando... Z_alvo=%.1fm | Z_real=%.2fm | Tempo=%.1fs",
      target_alt, current_z_real_, (double)takeoff_counter_ / 100.0);
  }
}

void DroneControllerCompleto::finalize_takeoff_on_altitude_reached(double target_alt)
{
  double arrival_threshold = target_alt - config_.hover_altitude_margin;
  if (current_z_real_ < arrival_threshold) { return; }

  RCLCPP_INFO(this->get_logger(), "✅ Decolagem concluída! Altitude = %.2fm\n", current_z_real_);

  if (takeoff_cmd_id_) {
    cmd_queue_.confirm(*takeoff_cmd_id_, true);
    RCLCPP_INFO(this->get_logger(),
      "✅ [ID=%lu] TAKEOFF confirmado! Altitude=%.2fm", *takeoff_cmd_id_, current_z_real_);
    takeoff_cmd_id_.reset();
  }

  hover_cmd_id_ = cmd_queue_.enqueue(
    CommandType::HOVER,
    {{"x", std::to_string(last_waypoint_goal_.pose.position.x)},
     {"y", std::to_string(last_waypoint_goal_.pose.position.y)},
     {"z", std::to_string(target_alt)}});
  RCLCPP_INFO(this->get_logger(), "📋 [ID=%lu] Comando HOVER enfileirado", *hover_cmd_id_);

  state_voo_ = 2;
  takeoff_counter_ = 0;
}

void DroneControllerCompleto::handle_state1_takeoff()
{
  // Step 1 — Pre-ARM streaming: publish setpoints before requesting OFFBOARD.
  // PX4 rejects ARM in OFFBOARD mode when no setpoints have been streamed yet.
  // stream_initial_setpoints() accumulates INITIAL_STREAM_THRESHOLD messages
  // (~200 ms at 100 Hz) and sets initial_stream_done_ when the threshold is met.
  if (!initial_stream_done_) {
    stream_initial_setpoints();
    return;
  }

  // Step 2 — Request OFFBOARD mode only (ARM is withheld until OFFBOARD is confirmed).
  // Sending ARM simultaneously with OFFBOARD is rejected by PX4 because the mode
  // transition has not completed yet.  See PX4 MAVROS offboard guide and the
  // wait_for_offboard_mode() implementation above for full rationale.
  if (!offboard_activated_) {
    request_arm_and_offboard_activation();
    return;
  }

  // Step 3 — Wait for the FCU to report mode == "OFFBOARD" before arming.
  // offboard_mode_confirmed_ is set by wait_for_offboard_mode() once the FCU
  // confirms the transition.  Only then does the sequence proceed to ARM.
  if (!offboard_mode_confirmed_) {
    wait_for_offboard_mode();
    return;
  }

  // Step 3.5 — Post-OFFBOARD extended streaming: publish setpoints for 1.5 s
  // after OFFBOARD is confirmed, BEFORE sending ARM.
  //
  // Rationale (PX4/MAVROS offboard guide): even after the FCU accepts the
  // OFFBOARD mode change, it may still reject ARM if the setpoint stream has
  // not been running long enough inside OFFBOARD mode.  Publishing at least
  // 30 setpoints at 50 ms intervals (= 1.5 s) gives the FCU enough time to
  // stabilise the mode and accept the subsequent ARM command.
  //
  // stream_post_offboard_setpoints() counts calls and sets
  // post_offboard_stream_done_ once POST_OFFBOARD_STREAM_THRESHOLD is reached.
  if (!post_offboard_stream_done_) {
    stream_post_offboard_setpoints();
    return;
  }

  // Step 4 — ARM the vehicle, now that OFFBOARD mode is confirmed by the FCU
  // AND the 1.5-second post-OFFBOARD streaming window has elapsed.
  // Sending ARM only after OFFBOARD is confirmed AND the stream has been
  // running long enough avoids the FCU rejecting ARM.
  if (!arm_requested_) {
    RCLCPP_INFO(this->get_logger(),
      "🔋 Modo OFFBOARD confirmado pelo FCU — solicitando ARM agora...");
    request_arm();
    arm_requested_ = true;
    return;
  }

  // Step 5 — Wait for ARM confirmation from the FCU.
  if (!activation_confirmed_) {
    if (!wait_for_offboard_arm_confirmation()) { return; }
  }

  // The takeoff target altitude was computed and fixed at command-receive time
  // (in handle_single_takeoff_waypoint_command() or the 4D equivalent) as:
  //   takeoff_target_z_ = std::max(hover_altitude, current_z_real_ + takeoff_z_boost)
  //
  // There is NO artificial differentiation between the first takeoff and any
  // subsequent cycle after a landing — the same formula always runs, always
  // from the actual ground altitude measured at the moment the command arrived.
  // Publishing a fixed target (not recomputed on every 10 ms cycle) prevents
  // the infinite-ascent bug: if the target tracked current_z_real_ every cycle
  // it would rise together with the drone and never be reached.
  //
  // Safety guard: takeoff_target_z_ should never still be -1.0 here in normal
  // operation.  If it is, fall back to a safe computation from current_z_real_.
  if (takeoff_target_z_ < 0.0) {
    RCLCPP_ERROR(this->get_logger(),
      "⚠️ [TAKEOFF] takeoff_target_z_ não foi inicializado antes de handle_state1_takeoff() "
      "(sentinel -1.0 inesperado). Calculando fallback a partir de current_z_real_=%.2fm.",
      current_z_real_);
    takeoff_target_z_ = std::max(
      config_.hover_altitude,
      current_z_real_ + config_.takeoff_z_boost);
  }

  // Use the fixed value for every control cycle until the drone reaches the target.
  const double target_altitude = takeoff_target_z_;

  publish_takeoff_climb_setpoint(target_altitude);
  finalize_takeoff_on_altitude_reached(target_altitude);
}

// ============================================================
// FSM STATE 2 — HOVER
// ============================================================

void DroneControllerCompleto::handle_state2_hover()
{
  if (using_4d_goal_) {
    publishPositionTargetWithYaw(
      last_waypoint_goal_.pose.position.x,
      last_waypoint_goal_.pose.position.y,
      last_waypoint_goal_.pose.position.z,
      goal_yaw_rad_);
  } else {
    publishPositionTarget(
      last_waypoint_goal_.pose.position.x,
      last_waypoint_goal_.pose.position.y,
      last_waypoint_goal_.pose.position.z,
      0.0, MASK_POS_ONLY);
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
    "🛸 Em HOVER (%.1fm) | Posição: X=%.2f, Y=%.2f | Aguardando waypoints... Controlador: %s",
    last_waypoint_goal_.pose.position.z,
    last_waypoint_goal_.pose.position.x,
    last_waypoint_goal_.pose.position.y,
    controlador_ativo_ ? "ATIVO" : "INATIVO");

  // After a mission interrupt re-arm, resume the main trajectory when the
  // drone has reached a safe altitude (z >= MISSION_RESUME_ALTITUDE_M).
  if (mission_cycle_phase_ == MissionCyclePhase::FOLLOW_TAKEOFF &&
      current_z_real_ >= MISSION_RESUME_ALTITUDE_M)
  {
    RCLCPP_INFO(this->get_logger(),
      "✅ [MISSION FOLLOW_TAKEOFF] Altitude %.2f m >= %.1f m após re-arm. "
      "Retomando trajetória principal em WP[%d]. Fase: NONE.",
      current_z_real_, MISSION_RESUME_ALTITUDE_M, current_waypoint_idx_);
    mission_cycle_phase_ = MissionCyclePhase::NONE;
    mission_waypoints_.clear();
    mission_wp_follow_idx_ = 0;
    last_published_mission_wp_idx_ = -1;
    // Only resume trajectory if there are remaining waypoints.
    if (!trajectory_waypoints_.empty() &&
        current_waypoint_idx_ < static_cast<int>(trajectory_waypoints_.size()))
    {
      controlador_ativo_ = true;
    }
    return;
  }

  if (controlador_ativo_) {
    state_voo_ = 3;
    RCLCPP_INFO(this->get_logger(), "✈️ Iniciando execução de trajetória...\n");
  }

  if (pouso_em_andamento_) {
    RCLCPP_WARN(this->get_logger(), "🛬 POUSO DETECTADO NO HOVER - DESLIGANDO!");
    state_voo_ = 4;
  }
}

// ============================================================
// FSM STATE 3 — TRAJETÓRIA (sub-routines)
// ============================================================

bool DroneControllerCompleto::detect_and_handle_landing_in_trajectory()
{
  if (current_z_real_ >= config_.land_z_threshold) { return false; }

  RCLCPP_WARN(this->get_logger(),
    "\n🛬🛬🛬 POUSO DETECTADO DURANTE TRAJETÓRIA! Z = %.2f m", current_z_real_);

  if (trajectory_cmd_id_) {
    cmd_queue_.confirm(*trajectory_cmd_id_, false);
    RCLCPP_WARN(this->get_logger(),
      "⚠️ [ID=%lu] TRAJECTORY interrompida por pouso", *trajectory_cmd_id_);
    trajectory_cmd_id_.reset();
  }

  if (!land_cmd_id_) {
    land_cmd_id_ = cmd_queue_.enqueue(
      CommandType::LAND, {{"z", std::to_string(current_z_real_)}});
    RCLCPP_WARN(this->get_logger(),
      "📋 [ID=%lu] Comando LAND enfileirado (pouso durante trajetória)", *land_cmd_id_);
  }

  pouso_em_andamento_ = true;
  // Note: controlador_ativo_ is intentionally NOT set to false here.
  // Landing must not also pause the controller; the trajectory controller is
  // implicitly stopped by the state_voo_ = 4 transition below.
  state_voo_ = 4;
  return true;
}

bool DroneControllerCompleto::initialize_trajectory()
{
  if (trajectory_started_) { return true; }

  if (trajectory_waypoints_.empty()) {
    RCLCPP_WARN(this->get_logger(), "⚠️ Nenhum waypoint armazenado, voltando para ESTADO 2");
    state_voo_ = 2;
    return false;
  }

  trajectory_start_time_ = this->now();
  trajectory_started_ = true;
  current_waypoint_idx_ = 0;

  RCLCPP_INFO(this->get_logger(),
    "✈️ Trajetória iniciada! %zu waypoints | %.1f segundos cada",
    trajectory_waypoints_.size(), waypoint_duration_);

  // Publish the first waypoint as the active goal for monitoring.
  const auto & first_wp = trajectory_waypoints_[0];
  publish_waypoint_goal_status(first_wp.position.x, first_wp.position.y, first_wp.position.z);

  return true;
}

double DroneControllerCompleto::compute_yaw_for_trajectory_waypoint(int idx, bool at_last_wp)
{
  const auto & wp = trajectory_waypoints_[idx];
  double yaw_follow = 0.0;

  if (at_last_wp) {
    if (!at_last_waypoint_yaw_fixed_) {
      double dx = wp.position.x - current_x_ned_;
      double dy = wp.position.y - current_y_ned_;
      yaw_follow = std::atan2(dy, dx);
      final_waypoint_yaw_ = yaw_follow;
      at_last_waypoint_yaw_fixed_ = true;
    } else {
      yaw_follow = final_waypoint_yaw_;
    }
  } else {
    double dx = wp.position.x - current_x_ned_;
    double dy = wp.position.y - current_y_ned_;
    yaw_follow = std::atan2(dy, dx);
    final_waypoint_yaw_ = yaw_follow;
    at_last_waypoint_yaw_fixed_ = false;
  }

  return yaw_follow;
}

void DroneControllerCompleto::publish_trajectory_waypoint_setpoint(int idx)
{
  const auto & wp = trajectory_waypoints_[idx];
  size_t last_idx = trajectory_waypoints_.size() - 1;
  bool at_last_wp = (static_cast<size_t>(idx) == last_idx);

  if (trajectory_4d_mode_ && at_last_wp &&
      static_cast<size_t>(idx) < trajectory_yaws_.size()) {
    publishPositionTargetWithYaw(
      wp.position.x, wp.position.y, wp.position.z, trajectory_yaws_[idx]);
  } else {
    double yaw_follow = compute_yaw_for_trajectory_waypoint(idx, at_last_wp);
    publishPositionTargetWithYaw(wp.position.x, wp.position.y, wp.position.z, yaw_follow);
  }
}

void DroneControllerCompleto::log_trajectory_progress(int idx)
{
  const auto & wp = trajectory_waypoints_[idx];
  double progress_pct = (trajectory_waypoints_.size() > 1)
    ? (static_cast<double>(idx) / static_cast<double>(trajectory_waypoints_.size() - 1)) * 100.0
    : 100.0;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
    "📡 Trajetória em execução | WP[%d/%zu]: X=%.2fm, Y=%.2fm, Z=%.2fm (Z_real=%.2f) | %.1f%% concluído",
    idx, trajectory_waypoints_.size() - 1,
    wp.position.x, wp.position.y, wp.position.z,
    current_z_real_, progress_pct);
}

void DroneControllerCompleto::finalize_trajectory_complete()
{
  if (!trajectory_cmd_id_) { return; }

  cmd_queue_.confirm(*trajectory_cmd_id_, true);
  RCLCPP_INFO(this->get_logger(),
    "✅ [ID=%lu] TRAJECTORY confirmada - todos os waypoints visitados", *trajectory_cmd_id_);
  trajectory_cmd_id_.reset();

  std_msgs::msg::Bool done_msg;
  done_msg.data = true;
  trajectory_finished_pub_->publish(done_msg);

  std_msgs::msg::Float32 progress_msg;
  progress_msg.data = 100.0;
  progress_publisher_->publish(progress_msg);

  RCLCPP_INFO(this->get_logger(),
    "📢 Trajetória terminada! Publicado /trajectory_finished = true");
}

void DroneControllerCompleto::handle_mission_interrupt_in_state3()
{
  // Hold at the waypoint that was just reached while waiting for mission waypoints.
  if (mission_waypoints_.empty()) {
    int hold_idx = current_waypoint_idx_ - 1;
    if (hold_idx >= 0 && static_cast<size_t>(hold_idx) < trajectory_waypoints_.size()) {
      publish_trajectory_waypoint_setpoint(hold_idx);
    }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "⏳ [MISSION] Aguardando /mission_waypoints (waypoints de pouso)...");
    return;
  }

  // Clamp index.
  if (mission_wp_follow_idx_ < 0 ||
      static_cast<size_t>(mission_wp_follow_idx_) >= mission_waypoints_.size())
  {
    mission_wp_follow_idx_ = static_cast<int>(mission_waypoints_.size()) - 1;
  }

  const auto & mwp = mission_waypoints_[mission_wp_follow_idx_];
  publishPositionTargetWithYaw(mwp.position.x, mwp.position.y, mwp.position.z, 0.0);

  // Publish current mission waypoint to /waypoint_goal when the active index changes.
  if (last_published_mission_wp_idx_ != mission_wp_follow_idx_) {
    last_published_mission_wp_idx_ = mission_wp_follow_idx_;
    publish_waypoint_goal_status(mwp.position.x, mwp.position.y, mwp.position.z);
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
    "🔄 [MISSION] Seguindo waypoint de missão [%d/%zu]: X=%.2f, Y=%.2f, Z=%.2f",
    mission_wp_follow_idx_,
    mission_waypoints_.size() - 1,
    mwp.position.x, mwp.position.y, mwp.position.z);

  // Advance to the next mission waypoint when the current one is reached.
  // Slightly larger tolerances than the main trajectory (XY=0.15, Z=0.25 vs
  // XY=0.10, Z=0.15) because mission waypoints (landing approach steps) are
  // intermediate altitude targets — the existing landing-detection logic will
  // trigger STATE 4 once the drone actually reaches ground level.
  if (mission_wp_follow_idx_ < static_cast<int>(mission_waypoints_.size()) - 1) {
    constexpr double XY_TOL = 0.15;
    constexpr double Z_TOL  = 0.25;
    double dx = mwp.position.x - current_x_ned_;
    double dy = mwp.position.y - current_y_ned_;
    double dz = std::abs(mwp.position.z - current_z_real_);
    double dist_xy = std::sqrt(dx * dx + dy * dy);
    if (dist_xy <= XY_TOL && dz <= Z_TOL) {
      mission_wp_follow_idx_++;
    }
  }
  // When at the last mission waypoint the existing landing-detection logic in
  // detect_and_handle_landing_in_trajectory() will fire once z < land_z_threshold.
}

void DroneControllerCompleto::handle_state3_trajectory()
{
  if (detect_and_handle_landing_in_trajectory()) { return; }

  if (pouso_em_andamento_ && !controlador_ativo_) {
    RCLCPP_WARN(this->get_logger(), "🛬 POUSO DETECTADO EM TRAJETÓRIA - PARANDO IMEDIATAMENTE!");
    state_voo_ = 4;
    return;
  }

  if (!initialize_trajectory()) { return; }

  // Mission interrupt: follow mission_waypoints instead of main trajectory.
  // Only handle the landing phases (WAIT_LAND_WP / FOLLOW_LAND) here.
  // The WAIT_TAKEOFF_WP and FOLLOW_TAKEOFF phases are handled by state 0 and
  // handle_state2_hover() respectively, after the drone is on the ground.
  if (mission_cycle_phase_ == MissionCyclePhase::WAIT_LAND_WP ||
      mission_cycle_phase_ == MissionCyclePhase::FOLLOW_LAND)
  {
    handle_mission_interrupt_in_state3();
    return;
  }

  // Index bounds check (trajectory_waypoints_ is non-empty — guaranteed by
  // initialize_trajectory() which returns false when the list is empty).
  if (current_waypoint_idx_ < 0 ||
      static_cast<size_t>(current_waypoint_idx_) >= trajectory_waypoints_.size())
  {
    RCLCPP_ERROR(this->get_logger(),
      "❌ Índice de waypoint inválido: %d (tamanho=%zu)",
      current_waypoint_idx_, trajectory_waypoints_.size());
    state_voo_ = 2;
    return;
  }

  // Publish current waypoint setpoint (hold until reach condition fires).
  publish_trajectory_waypoint_setpoint(current_waypoint_idx_);
  log_trajectory_progress(current_waypoint_idx_);

  // Detect waypoint reach by position and publish /waypoint_reached (once per index).
  {
    constexpr double XY_TOL = 0.10;  // radial XY tolerance [m]
    constexpr double Z_TOL  = 0.15;  // absolute Z tolerance [m]
    const auto & wp = trajectory_waypoints_[current_waypoint_idx_];
    double dx = wp.position.x - current_x_ned_;
    double dy = wp.position.y - current_y_ned_;
    double dz = std::abs(wp.position.z - current_z_real_);
    double dist_xy = std::sqrt(dx * dx + dy * dy);
    if (dist_xy <= XY_TOL && dz <= Z_TOL &&
        current_waypoint_idx_ != last_waypoint_reached_idx_)
    {
      last_waypoint_reached_idx_ = current_waypoint_idx_;

      std_msgs::msg::Int32 reached_msg;
      reached_msg.data = current_waypoint_idx_;
      waypoint_reached_pub_->publish(reached_msg);
      RCLCPP_INFO(this->get_logger(),
        "📍 Waypoint %d atingido (dist_xy=%.3fm, dz=%.3fm) → /waypoint_reached",
        current_waypoint_idx_, dist_xy, dz);

      // Publish the exact waypoint position as latch pose so that
      // drone_publish_landing_waypoints can use it as the XY reference for
      // landing (avoids any XY=(0,0) fallback from delayed MAVROS pose).
      {
        geometry_msgs::msg::PoseStamped latch_msg;
        latch_msg.header.stamp = this->now();
        latch_msg.header.frame_id = "map";
        latch_msg.pose = trajectory_waypoints_[last_waypoint_reached_idx_];
        mission_latch_pose_pub_->publish(latch_msg);
        // Mirror locally so sanitize_takeoff_xy() can use it without a self-subscriber.
        last_latch_pose_      = latch_msg.pose;
        last_latch_pose_time_ = latch_msg.header.stamp;
        has_latch_pose_       = true;
        RCLCPP_INFO(this->get_logger(),
          "📌 [MISSION] Latch pose publicada para WP[%d]: X=%.2f, Y=%.2f, Z=%.2f",
          last_waypoint_reached_idx_,
          latch_msg.pose.position.x, latch_msg.pose.position.y, latch_msg.pose.position.z);
      }

      // Advance to the next waypoint index.
      current_waypoint_idx_++;

      // Publish the next active trajectory waypoint to /waypoint_goal for monitoring.
      if (current_waypoint_idx_ < static_cast<int>(trajectory_waypoints_.size())) {
        const auto & next_wp = trajectory_waypoints_[current_waypoint_idx_];
        publish_waypoint_goal_status(
          next_wp.position.x, next_wp.position.y, next_wp.position.z);
      }

      // Check whether the trajectory is now complete.
      if (current_waypoint_idx_ >= static_cast<int>(trajectory_waypoints_.size())) {
        finalize_trajectory_complete();
        // Enter mission interrupt so mission_manager runs its land/disarm/rearm/takeoff
        // cycle for the last reached waypoint (including WP0).
        RCLCPP_INFO(this->get_logger(),
          "🔄 [MISSION WAIT_LAND_WP] Fim de trajetória — iniciando ciclo de missão para WP%d. "
          "Aguardando /mission_waypoints de pouso.",
          last_waypoint_reached_idx_);
        mission_cycle_phase_ = MissionCyclePhase::WAIT_LAND_WP;
        mission_waypoints_.clear();
        mission_wp_follow_idx_ = 0;
        last_published_mission_wp_idx_ = -1;
        return;
      }

      // Enter mission interrupt mode: wait for /mission_waypoints before
      // continuing to the next main waypoint (including WP0).
      RCLCPP_INFO(this->get_logger(),
        "🔄 [MISSION WAIT_LAND_WP] Iniciando ciclo de missão para WP%d. "
        "Trajetória pausada; aguardando /mission_waypoints de pouso.",
        last_waypoint_reached_idx_);
      mission_cycle_phase_ = MissionCyclePhase::WAIT_LAND_WP;
      mission_waypoints_.clear();
      mission_wp_follow_idx_ = 0;
      last_published_mission_wp_idx_ = -1;
    }
  }
}

// ============================================================
// MISSION INTERRUPT DONE CALLBACK (/mission_interrupt_done)
// ============================================================

void DroneControllerCompleto::mission_interrupt_done_callback(
  const std_msgs::msg::Int32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  RCLCPP_INFO(this->get_logger(),
    "📩 [MISSION] /mission_interrupt_done recebido (WP[%d]). "
    "mission_cycle_phase_=%s, state_voo_=%d",
    msg->data,
    mission_cycle_phase_name(mission_cycle_phase_),
    state_voo_);

  if (mission_cycle_phase_ == MissionCyclePhase::NONE) {
    // Already resumed via the altitude-based check in handle_state2_hover(); no-op.
    RCLCPP_INFO(this->get_logger(),
      "ℹ️ [MISSION] Trajetória já retomada (phase=NONE). Sinal ignorado.");
    return;
  }

  // Fallback resume: altitude-based check in handle_state2_hover() may not have
  // fired yet (e.g. drone still climbing).  Set controlador_ativo_ and clear
  // mission state so the FSM resumes trajectory as soon as state_voo_ reaches 2.
  RCLCPP_INFO(this->get_logger(),
    "✅ [MISSION INTERRUPT DONE] Retomando trajetória em WP[%d] (fase anterior: %s).",
    current_waypoint_idx_, mission_cycle_phase_name(mission_cycle_phase_));

  mission_cycle_phase_ = MissionCyclePhase::NONE;
  mission_waypoints_.clear();
  mission_wp_follow_idx_ = 0;
  last_published_mission_wp_idx_ = -1;

  if (!trajectory_waypoints_.empty() &&
      current_waypoint_idx_ < static_cast<int>(trajectory_waypoints_.size()))
  {
    controlador_ativo_ = true;
    RCLCPP_INFO(this->get_logger(),
      "▶️ [MISSION] controlador_ativo_=true — trajetória retomará em WP[%d] ao atingir HOVER.",
      current_waypoint_idx_);
    // Republish the stored trajectory waypoints so observers can see the active set.
    publish_waypoints_status();
  } else {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ [MISSION] Nenhum waypoint restante em current_waypoint_idx_=%d (tamanho=%zu). "
      "Trajetória concluída.",
      current_waypoint_idx_, trajectory_waypoints_.size());
  }
}

// ============================================================
// FSM STATE 4 — POUSO
// ============================================================

void DroneControllerCompleto::complete_landing()
{
  if (land_cmd_id_) {
    cmd_queue_.confirm(*land_cmd_id_, true);
    RCLCPP_WARN(this->get_logger(),
      "✅ [ID=%lu] LAND confirmado — iniciando DISARM", *land_cmd_id_);
  }

  cmd_queue_.save_log("/tmp/drone_commands.log");
  RCLCPP_INFO(this->get_logger(),
    "💾 Histórico de comandos salvo em /tmp/drone_commands.log");

  RCLCPP_WARN(this->get_logger(),
    "\n✅ POUSO CONCLUÍDO! Solicitando DISARM e aguardando confirmação do FCU...\n");

  // Mark that we are waiting for DISARM confirmation before transitioning state.
  // The state transition to 0 happens only after current_state_.armed becomes false.
  disarm_requested_ = true;
  // Clear these early so handle_state4_landing() does NOT re-enter the
  // landing-timeout block and re-call complete_landing() while we are still
  // waiting for the FCU to confirm DISARM.
  pouso_em_andamento_ = false;
  pouso_start_time_set_ = false;
}

void DroneControllerCompleto::handle_state4_landing()
{
  // Wait for FCU to confirm DISARM before transitioning state.
  if (disarm_requested_) {
    if (!current_state_.armed) {
      RCLCPP_INFO(this->get_logger(),
        "✅ DISARM confirmado pelo FCU — transicionando para aguardar novo comando.");
      disarm_requested_ = false;

      // If this DISARM is part of a mission interrupt cycle, advance to
      // WAIT_TAKEOFF_WP so the takeoff waypoint from mission_manager is accepted
      // even if the WAIT_LAND_WP / FOLLOW_LAND phase has already been processed.
      // This is the key state transition that prevents the "takeoff recebido sem
      // missão ativa" bug: the phase is now WAIT_TAKEOFF_WP, not NONE.
      if (mission_cycle_phase_ == MissionCyclePhase::FOLLOW_LAND ||
          mission_cycle_phase_ == MissionCyclePhase::WAIT_LAND_WP)
      {
        mission_cycle_phase_ = MissionCyclePhase::WAIT_TAKEOFF_WP;
        RCLCPP_INFO(this->get_logger(),
          "🔄 [MISSION WAIT_TAKEOFF_WP] DISARM confirmado durante ciclo de missão. "
          "Aguardando waypoint de decolagem em /mission_waypoints...");
      }

      state_voo_ = 0;
      RCLCPP_WARN(this->get_logger(),
        "⏳ Aguardando novo comando de waypoint para decolar novamente... "
        "(fase: %s)",
        mission_cycle_phase_name(mission_cycle_phase_));
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "⏳ [DISARM] Aguardando confirmação de DISARM pelo FCU...");
    }
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
    "⬇️ POUSANDO — aguardando confirmação de aterrissagem...");

  if (pouso_em_andamento_) {
    if (!pouso_start_time_set_) {
      pouso_start_time_ = this->now();
      pouso_start_time_set_ = true;
      RCLCPP_INFO(this->get_logger(),
        "⏱️ Iniciando contagem de pouso (%.0f s para confirmar)...", config_.landing_timeout);
    }

    if ((this->now() - pouso_start_time_).seconds() > config_.landing_timeout) {
      complete_landing();
      return;
    }
  }
}

}  // namespace drone_control
