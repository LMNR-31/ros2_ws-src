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
  RCLCPP_INFO(this->get_logger(), "║    FSM 5 ESTADOS — FLUXO DE POUSO ÚNICO (DISARM+RESET)  ║");
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

  this->declare_parameter<bool>("enabled", true);
  enabled_ = this->get_parameter("enabled").as_bool();
  RCLCPP_INFO(this->get_logger(), "⚙️  enabled=%s", enabled_ ? "true" : "false");

  this->declare_parameter<bool>("override_active", false);
  override_active_ = this->get_parameter("override_active").as_bool();
  RCLCPP_INFO(this->get_logger(), "⚙️  override_active=%s", override_active_ ? "true" : "false");

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
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /uav1/mavros/setpoint_raw/local");
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /trajectory_finished");
  RCLCPP_INFO(this->get_logger(), "✓ Publisher criado: /trajectory_progress");
}

void DroneControllerCompleto::setup_subscribers()
{
  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    "/uav1/mavros/state", 10,
    [this](const mavros_msgs::msg::State::SharedPtr msg) { current_state_ = *msg; });

  waypoints_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/waypoints", 1,
    std::bind(&DroneControllerCompleto::waypoints_callback, this, std::placeholders::_1));

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

  RCLCPP_INFO(this->get_logger(),
    "✓ Subscribers criados: /uav1/mavros/state, /waypoints, /waypoint_goal, "
    "odometria, /uav1/yaw_override/cmd, /waypoint_goal_4d e /waypoints_4d");
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
  takeoff_target_z_ = -1.0;  // -1.0 = sentinel: not yet latched for this takeoff cycle
  first_takeoff_cycle_ = true;  // will compute takeoff_target_z_ on first handle_state1_takeoff() call
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
  current_z_real_ = 0.0;
  current_x_ned_ = 0.0;
  current_y_ned_ = 0.0;
  current_z_ned_ = 0.0;
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
}

// ============================================================
// LANDING / ACTIVATION HELPERS
// ============================================================

void DroneControllerCompleto::trigger_landing(double z)
{
  pouso_em_andamento_ = true;
  controlador_ativo_ = false;
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
    "🛬 CONTROLADOR DESLIGADO - DEIXANDO drone_soft_land POUSAR\n");
}

void DroneControllerCompleto::handle_single_takeoff_waypoint_command(
  const geometry_msgs::msg::Pose & pose)
{
  RCLCPP_INFO(this->get_logger(), "\n⬆️ WAYPOINT DE LEVANTAMENTO recebido:");
  RCLCPP_INFO(this->get_logger(), "   Posição: X=%.2f, Y=%.2f, Z=%.2f",
    pose.position.x, pose.position.y, pose.position.z);

  log_takeoff_debug_flags("ANTES");

  if (has_dirty_takeoff_state()) {
    log_dirty_takeoff_state("TAKEOFF");
    reset_after_landing();  // reset completo: flags + IDs + fila de comandos
  }

  // Ensure takeoff_target_z_ is recomputed from the current measured altitude
  // (current_z_real_) at the start of the new climb.  reset_after_landing()
  // sets this flag when called above; set it explicitly here too so the flag
  // is always true at this point even when no dirty state was detected (e.g.
  // clean re-takeoff from state 0 after a normal landing cycle).
  first_takeoff_cycle_ = true;
  takeoff_target_z_ = -1.0;  // companion sentinel — belt-and-suspenders safety

  last_waypoint_goal_.pose = pose;
  pouso_em_andamento_ = false;
  controlador_ativo_ = false;
  trajectory_started_ = false;
  trajectory_waypoints_.clear();
  trajectory_yaws_.clear();  // cleared defensively to avoid stale 4D yaws if mode changes
  current_waypoint_idx_ = 0;

  activate_offboard_arm_if_needed();

  takeoff_cmd_id_ = cmd_queue_.enqueue(
    CommandType::TAKEOFF,
    {{"x", std::to_string(pose.position.x)},
     {"y", std::to_string(pose.position.y)},
     {"z", std::to_string(config_.hover_altitude)}});
  RCLCPP_INFO(this->get_logger(),
    "📋 [ID=%lu] Comando TAKEOFF enfileirado", *takeoff_cmd_id_);

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
    last_waypoint_goal_.pose = msg->poses[0];

    log_trajectory_waypoints_3d(trajectory_waypoints_);

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
    // the control-loop's handle_state4_landing() executing the reset).
    // Perform the reset here and return FALSE so that the caller continues to
    // process the incoming takeoff command — without this the command would
    // always be silently dropped, requiring the operator to re-send it.
    RCLCPP_INFO(this->get_logger(),
      "✅ [RESET] DRONE DESARMADO em estado 4 — realizando reset imediato para aceitar novo comando.");
    disarm_requested_ = false;
    reset_after_landing();
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

void DroneControllerCompleto::log_dirty_takeoff_state(const char * context)
{
  RCLCPP_WARN(this->get_logger(),
    "[%s] Detectado estado sujo de ciclo anterior — limpando antes de novo takeoff. "
    "takeoff_cmd_id_=%s | activation_confirmed_=%d | offboard_activated_=%d "
    "| offboard_mode_confirmed_=%d | arm_requested_=%d "
    "| post_offboard_stream_done_=%d "
    "| disarm_requested_=%d | pouso_em_andamento_=%d | state_voo_=%d "
    "| trajectory_cmd_id_=%s | hover_cmd_id_=%s",
    context,
    takeoff_cmd_id_ ? "set" : "empty",
    static_cast<int>(activation_confirmed_),
    static_cast<int>(offboard_activated_),
    static_cast<int>(offboard_mode_confirmed_),
    static_cast<int>(arm_requested_),
    static_cast<int>(post_offboard_stream_done_),
    static_cast<int>(disarm_requested_),
    static_cast<int>(pouso_em_andamento_),
    state_voo_,
    trajectory_cmd_id_ ? "set" : "empty",
    hover_cmd_id_ ? "set" : "empty");
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
    return;
  }

  RCLCPP_INFO(this->get_logger(), "\n📍 NOVO WAYPOINT RECEBIDO: X=%.2f, Y=%.2f, Z=%.2f", x, y, z);

  last_waypoint_goal_ = *msg;
  waypoint_goal_received_ = true;
  controlador_ativo_ = true;
  pouso_em_andamento_ = false;
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
    return;
  }

  RCLCPP_INFO(this->get_logger(),
    "\n📍 4D WAYPOINT RECEBIDO: X=%.2f, Y=%.2f, Z=%.2f, yaw=%.3f rad",
    x, y, z, goal_yaw_rad_);

  last_waypoint_goal_ = *pose_stamped;
  waypoint_goal_received_ = true;
  controlador_ativo_ = true;
  pouso_em_andamento_ = false;
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

    if (has_dirty_takeoff_state()) {
      log_dirty_takeoff_state("4D TAKEOFF");
      reset_after_landing();  // reset completo: flags + IDs + fila de comandos
    }

    // Ensure takeoff_target_z_ is recomputed from current_z_real_ for this
    // new takeoff cycle (same rationale as handle_single_takeoff_waypoint_command).
    first_takeoff_cycle_ = true;
    takeoff_target_z_ = -1.0;

    goal_yaw_rad_ = yaws[0];
    using_4d_goal_ = true;
    trajectory_4d_mode_ = false;

    geometry_msgs::msg::PoseStamped ps;
    ps.pose = poses[0];
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
      {{"x", std::to_string(poses[0].position.x)},
       {"y", std::to_string(poses[0].position.y)},
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
}

// ============================================================
// MAIN CONTROL LOOP
// ============================================================

void DroneControllerCompleto::control_loop()
{
  std::lock_guard<std::mutex> lock(mutex_);

  cycle_count_++;

  if (!enabled_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "🚫 Controller desabilitado (enabled=false): setpoints pausados");
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

  // Compute (and latch) the takeoff target altitude.
  //
  // BUG (infinite ascent): if target_altitude is recomputed every control cycle
  // as std::max(hover_altitude, current_z_real_ + takeoff_z_boost), the target
  // climbs together with the drone because current_z_real_ keeps increasing as
  // the drone ascends.  The drone therefore never reaches its own target and
  // climbs without bound.
  //
  // FIX: compute the target exactly ONCE per takeoff cycle, using the
  // first_takeoff_cycle_ flag (set to true in init_variables(), reset_after_landing(),
  // and every takeoff initiator).  Store the result in takeoff_target_z_ and reuse
  // that fixed value for every subsequent control cycle until the drone arrives or
  // the FSM transitions away.  A new takeoff cycle always has first_takeoff_cycle_
  // = true so it recomputes from the current measured altitude (current_z_real_).
  // The sentinel takeoff_target_z_ < 0.0 serves as a belt-and-suspenders guard.
  if (first_takeoff_cycle_ || takeoff_target_z_ < 0.0) {
    // First cycle of this takeoff: latch the target from the current altitude.
    // Never recalculate it again during this climb to avoid the infinite-ascent bug.
    takeoff_target_z_ = std::max(
      config_.hover_altitude,
      current_z_real_ + config_.takeoff_z_boost);
    first_takeoff_cycle_ = false;  // latch: do not recalculate for the rest of this cycle

    RCLCPP_INFO(this->get_logger(),
      "⬆️ [TAKEOFF BOOST] Z_real=%.2fm | boost=+%.2fm | hover_alt=%.2fm "
      "→ Z_alvo=%.2fm (fixo até atingir a altitude — evita subida infinita e auto-disarm do PX4)",
      current_z_real_, config_.takeoff_z_boost,
      config_.hover_altitude, takeoff_target_z_);
  }

  // Use the latched value for every cycle (takeoff_target_z_ never changes during climb).
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
  controlador_ativo_ = false;
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

void DroneControllerCompleto::log_trajectory_progress(int idx, double elapsed_time)
{
  const auto & wp = trajectory_waypoints_[idx];
  double total_time = waypoint_duration_ * (double)trajectory_waypoints_.size();
  double progress_pct = (elapsed_time / total_time) * 100.0;

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
    "📡 Trajetória em execução | WP[%d/%zu]: X=%.2fm, Y=%.2fm, Z=%.2fm (Z_real=%.2f) | %.1f%% concluído",
    idx, trajectory_waypoints_.size() - 1,
    wp.position.x, wp.position.y, wp.position.z,
    current_z_real_, progress_pct);
}

void DroneControllerCompleto::finalize_trajectory_if_complete(
  double elapsed_time, double total_time)
{
  if (elapsed_time < total_time || !trajectory_cmd_id_) { return; }

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

void DroneControllerCompleto::handle_state3_trajectory()
{
  if (detect_and_handle_landing_in_trajectory()) { return; }

  if (pouso_em_andamento_ && !controlador_ativo_) {
    RCLCPP_WARN(this->get_logger(), "🛬 POUSO DETECTADO EM TRAJETÓRIA - PARANDO IMEDIATAMENTE!");
    state_voo_ = 4;
    return;
  }

  if (!initialize_trajectory()) { return; }

  double elapsed_time = (this->now() - trajectory_start_time_).seconds();
  int computed_idx = static_cast<int>(elapsed_time / waypoint_duration_);

  // trajectory_waypoints_ is guaranteed non-empty at this point:
  // initialize_trajectory() returns false (and transitions to state 2)
  // when the list is empty, so execution only reaches here with valid data.
  current_waypoint_idx_ = std::min(
    computed_idx,
    static_cast<int>(trajectory_waypoints_.size()) - 1);

  if (current_waypoint_idx_ < 0 ||
      static_cast<size_t>(current_waypoint_idx_) >= trajectory_waypoints_.size()) {
    RCLCPP_ERROR(this->get_logger(),
      "❌ Índice de waypoint inválido: %d (tamanho=%zu)",
      current_waypoint_idx_, trajectory_waypoints_.size());
    state_voo_ = 2;
    return;
  }

  publish_trajectory_waypoint_setpoint(current_waypoint_idx_);
  log_trajectory_progress(current_waypoint_idx_, elapsed_time);

  double total_time = waypoint_duration_ * (double)trajectory_waypoints_.size();
  finalize_trajectory_if_complete(elapsed_time, total_time);
}

// ============================================================
// FSM STATE 4 — POUSO / PAUSADO
// ============================================================

void DroneControllerCompleto::reset_after_landing()
{
  // Called after DISARM is confirmed by the FCU (or when the drone is already
  // disarmed when a new waypoint arrives). Resets all flight-cycle flags so
  // the system is ready for the next takeoff.
  RCLCPP_WARN(this->get_logger(),
    "[RESET] reset_after_landing() chamado — zerando todas as flags e comandos do sistema de pouso.");

  // FSM state — must be 0 so the next takeoff command is accepted by
  // handle_single_takeoff_waypoint_command() / waypoints_callback().
  state_voo_ = 0;
  pouso_em_andamento_ = false;
  controlador_ativo_ = false;
  trajectory_started_ = false;
  pouso_start_time_set_ = false;

  // OFFBOARD/ARM activation — must be zero so the next takeoff activates fresh
  // (offboard_activated_ still true would skip the streaming + OFFBOARD steps,
  //  arm_requested_ still true would skip the ARM step, etc.).
  offboard_activated_ = false;
  offboard_mode_confirmed_ = false;
  arm_requested_ = false;
  activation_confirmed_ = false;

  // DISARM already completed
  disarm_requested_ = false;

  // Counters and indices — takeoff_counter_ still > 0 would confuse the
  // finalize_takeoff_on_altitude_reached() logic; current_waypoint_idx_ still
  // non-zero would skip waypoints at the start of the next trajectory.
  takeoff_counter_ = 0;

  // Reset the latched takeoff target so the next cycle recomputes it from
  // current_z_real_ at the moment of the new takeoff command.  Without this
  // the drone would target a stale altitude from the previous flight.
  takeoff_target_z_ = -1.0;

  // first_takeoff_cycle_ = true ensures handle_state1_takeoff() will recompute
  // takeoff_target_z_ exactly once at the beginning of the next climb and then
  // latch it — preventing both the infinite-ascent bug and the stale-target bug.
  first_takeoff_cycle_ = true;

  current_waypoint_idx_ = 0;

  // Waypoints and trajectory
  trajectory_waypoints_.clear();
  trajectory_yaws_.clear();
  waypoint_goal_received_ = false;

  // 4D mode flags — stale 4D state from a previous flight must not bleed into
  // the next flight cycle (e.g. using_4d_goal_ true would publish yaw setpoints
  // when the operator sends a plain 3D takeoff command).
  using_4d_goal_ = false;
  trajectory_4d_mode_ = false;
  at_last_waypoint_yaw_fixed_ = false;
  // final_waypoint_yaw_ is recalculated when at_last_waypoint_yaw_fixed_ is false,
  // but reset it to a neutral value to avoid confusing log output between flights.
  final_waypoint_yaw_ = 0.0;

  // Yaw override — a stale yaw-rate override from the previous flight would
  // rotate the drone immediately after it lifts off in the next cycle.
  yaw_override_enabled_ = false;
  yaw_rate_cmd_ = 0.0;

  // Position hold — a stale hold position from the previous flight (hold_valid_
  // true) would make the override-active path publish wrong setpoints if
  // override_active_ is set before the odometry callback updates the hold.
  hold_valid_ = false;

  // Pre-ARM setpoint streaming state
  initial_stream_count_ = 0;
  initial_stream_done_ = false;

  // Post-OFFBOARD setpoint streaming state (1.5 s window before ARM)
  post_offboard_stream_count_ = 0;
  post_offboard_stream_done_ = false;

  // Cancel pending commands and reset all command IDs.
  // cancel_all_pending() marks leftover queue entries as FAILED so they don't
  // appear as TIMEOUT in the next check_timeouts() pass.
  cmd_queue_.cancel_all_pending();
  takeoff_cmd_id_.reset();
  hover_cmd_id_.reset();
  trajectory_cmd_id_.reset();
  land_cmd_id_.reset();
  offboard_cmd_id_.reset();
  arm_cmd_id_.reset();
  disarm_cmd_id_.reset();

  RCLCPP_WARN(this->get_logger(),
    "[RESET] Sistema pronto para novo ciclo de voo (takeoff → voo → pouso).");
}

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

  // Mark that we are waiting for DISARM confirmation before resetting flags.
  // The reset is performed only after current_state_.armed becomes false.
  disarm_requested_ = true;
  // Clear these early so handle_state4_landing() does NOT re-enter the
  // landing-timeout block and re-call complete_landing() while we are still
  // waiting for the FCU to confirm DISARM. All other flags are cleaned up
  // inside reset_after_landing() once DISARM is confirmed.
  pouso_em_andamento_ = false;
  pouso_start_time_set_ = false;

  if (current_state_.armed) {
    RCLCPP_WARN(this->get_logger(), "🔌 Solicitando DISARM...");
    request_disarm();
  } else {
    RCLCPP_WARN(this->get_logger(),
      "[DISARM] Drone já está desarmado — reset imediato sem enviar DISARM ao FCU.");
    disarm_requested_ = false;
    reset_after_landing();
  }
}

void DroneControllerCompleto::handle_state4_landing()
{
  // Wait for FCU to confirm DISARM before resetting flags.
  // reset_after_landing() must only run after armed==false is observed.
  if (disarm_requested_) {
    if (!current_state_.armed) {
      RCLCPP_INFO(this->get_logger(),
        "✅ DISARM confirmado pelo FCU — realizando reset do sistema de pouso.");
      disarm_requested_ = false;
      reset_after_landing();
      RCLCPP_WARN(this->get_logger(),
        "⏳ Aguardando novo comando de waypoint para decolar novamente...");
    } else {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "⏳ [DISARM] Aguardando confirmação de DISARM pelo FCU...");
    }
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
    "🛑 CONTROLADOR PAUSADO | drone_soft_land fazendo pouso...");

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
