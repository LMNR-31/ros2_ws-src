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
  RCLCPP_INFO(this->get_logger(), "║         FSM 6 ESTADOS — CÓDIGO LIMPO E MODULARIZADO      ║");
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
  this->declare_parameter("activation_timeout",    config_.activation_timeout);
  this->declare_parameter("command_timeout",       config_.command_timeout);
  this->declare_parameter("landing_timeout",       config_.landing_timeout);

  config_.hover_altitude        = this->get_parameter("hover_altitude").as_double();
  config_.hover_altitude_margin = this->get_parameter("hover_altitude_margin").as_double();
  config_.max_altitude          = this->get_parameter("max_altitude").as_double();
  config_.min_altitude          = this->get_parameter("min_altitude").as_double();
  config_.waypoint_duration     = this->get_parameter("waypoint_duration").as_double();
  config_.max_waypoint_distance = this->get_parameter("max_waypoint_distance").as_double();
  config_.land_z_threshold      = this->get_parameter("land_z_threshold").as_double();
  config_.activation_timeout    = this->get_parameter("activation_timeout").as_double();
  config_.command_timeout       = this->get_parameter("command_timeout").as_double();
  config_.landing_timeout       = this->get_parameter("landing_timeout").as_double();

  this->declare_parameter<int>("landing_mode", 1);
  landing_mode_ = this->get_parameter("landing_mode").as_int();
  if (landing_mode_ != 0 && landing_mode_ != 1) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️  landing_mode=%d inválido; usando 1 (DISARM)", landing_mode_);
    landing_mode_ = 1;
  }
  RCLCPP_INFO(this->get_logger(),
    "⚙️  landing_mode=%d (%s)", landing_mode_,
    landing_mode_ == 0 ? "Modo A - standby no chão" : "Modo B - DISARM");

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
  activation_confirmed_ = false;
  takeoff_counter_ = 0;
  waypoint_goal_received_ = false;
  last_z_ = 0.0;
  pouso_start_time_set_ = false;
  pouso_start_time_ = this->now();
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
  ground_hold_x_ = 0.0;
  ground_hold_y_ = 0.0;
  ground_hold_z_ = std::max(0.01, config_.land_z_threshold);
  state5_recovery_time_ = this->now();
  current_yaw_rad_ = 0.0;
  goal_yaw_rad_ = 0.0;
  using_4d_goal_ = false;
  trajectory_4d_mode_ = false;
}

// ============================================================
// LANDING / ACTIVATION HELPERS
// ============================================================

void DroneControllerCompleto::trigger_landing(double x, double y, double z)
{
  pouso_em_andamento_ = true;
  controlador_ativo_ = false;
  state_voo_ = 4;
  land_cmd_id_ = cmd_queue_.enqueue(CommandType::LAND, {{"z", std::to_string(z)}});
  ground_hold_x_ = x;
  ground_hold_y_ = y;
  ground_hold_z_ = std::max(0.01, config_.land_z_threshold);
}

void DroneControllerCompleto::activate_offboard_arm_if_needed()
{
  if (current_state_.armed && current_state_.mode == "OFFBOARD") {
    return;  // Já ativo — reutiliza ativação existente
  }
  offboard_activated_ = false;
  activation_confirmed_ = false;
  request_offboard();
  request_arm();
  offboard_activated_ = true;
  activation_time_ = this->now();
}

// ============================================================
// PARAMETER HANDLERS
// ============================================================

bool DroneControllerCompleto::apply_landing_mode_param(
  const rclcpp::Parameter & p, rcl_interfaces::msg::SetParametersResult & result)
{
  if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
    result.successful = false;
    result.reason = "landing_mode deve ser inteiro (0 ou 1)";
    return false;
  }
  int64_t new_val = p.as_int();
  if (new_val != 0 && new_val != 1) {
    result.successful = false;
    result.reason = "landing_mode deve ser 0 (Modo A - standby no chão) ou 1 (Modo B - DISARM)";
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  int old_val = landing_mode_;
  landing_mode_ = static_cast<int>(new_val);
  RCLCPP_INFO(this->get_logger(),
    "🔄 landing_mode atualizado: %d → %d (%s)",
    old_val, landing_mode_,
    landing_mode_ == 0 ? "Modo A - standby no chão" : "Modo B - DISARM");
  return true;
}

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
    if (p.get_name() == "landing_mode") {
      if (!apply_landing_mode_param(p, result)) { return result; }
    } else if (p.get_name() == "enabled") {
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

void DroneControllerCompleto::handle_landing_waypoint_command(double x, double y, double z)
{
  if (state_voo_ == 5) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "ℹ️ Comando de pouso ignorado: drone já em standby no chão (estado 5)");
    return;
  }
  RCLCPP_WARN(this->get_logger(), "\n🛬🛬🛬 POUSO DETECTADO! Z_final = %.2f m", z);
  trigger_landing(x, y, z);
  RCLCPP_WARN(this->get_logger(), "📋 [ID=%lu] Comando LAND enfileirado", *land_cmd_id_);
  RCLCPP_WARN(this->get_logger(),
    "📍 Ancoragem de pouso definida: X=%.2f, Y=%.2f, Z=%.3f",
    ground_hold_x_, ground_hold_y_, ground_hold_z_);
  RCLCPP_WARN(this->get_logger(),
    "🛬 CONTROLADOR DESLIGADO - DEIXANDO drone_soft_land POUSAR\n");
}

void DroneControllerCompleto::handle_single_takeoff_waypoint_command(
  const geometry_msgs::msg::Pose & pose)
{
  RCLCPP_INFO(this->get_logger(), "\n⬆️ WAYPOINT DE LEVANTAMENTO recebido:");
  RCLCPP_INFO(this->get_logger(), "   Posição: X=%.2f, Y=%.2f, Z=%.2f",
    pose.position.x, pose.position.y, pose.position.z);

  last_waypoint_goal_.pose = pose;
  pouso_em_andamento_ = false;
  controlador_ativo_ = false;
  trajectory_started_ = false;
  trajectory_waypoints_.clear();
  current_waypoint_idx_ = 0;

  RCLCPP_INFO(this->get_logger(), "🔍 DEBUG FLAGS ANTES:");
  RCLCPP_INFO(this->get_logger(), "   offboard_activated_=%d", offboard_activated_);
  RCLCPP_INFO(this->get_logger(), "   state_voo_=%d", state_voo_);
  RCLCPP_INFO(this->get_logger(), "   activation_confirmed_=%d", activation_confirmed_);

  if (state_voo_ == 5 && current_state_.armed && current_state_.mode == "OFFBOARD") {
    RCLCPP_INFO(this->get_logger(),
      "🔋 Estado 5: já OFFBOARD+ARM – reutilizando ativação para levantamento...\n");
  } else {
    RCLCPP_INFO(this->get_logger(), "🔋 Ativando OFFBOARD+ARM para levantamento...\n");
    activate_offboard_arm_if_needed();
  }

  takeoff_cmd_id_ = cmd_queue_.enqueue(
    CommandType::TAKEOFF,
    {{"x", std::to_string(pose.position.x)},
     {"y", std::to_string(pose.position.y)},
     {"z", std::to_string(config_.hover_altitude)}});
  RCLCPP_INFO(this->get_logger(),
    "📋 [ID=%lu] Comando TAKEOFF enfileirado", *takeoff_cmd_id_);

  state_voo_ = 1;
  takeoff_counter_ = 0;

  RCLCPP_INFO(this->get_logger(), "🔍 DEBUG FLAGS DEPOIS:");
  RCLCPP_INFO(this->get_logger(), "   offboard_activated_=%d", offboard_activated_);
  RCLCPP_INFO(this->get_logger(), "   state_voo_=%d\n", state_voo_);
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
    handle_landing_waypoint_command(
      msg->poses[0].position.x, msg->poses[0].position.y, last_z);
    return;
  }

  if (msg->poses.size() == 1 && last_z >= config_.land_z_threshold) {
    handle_single_takeoff_waypoint_command(msg->poses[0]);
    return;
  }

  if (msg->poses.size() >= 2) {
    if (state_voo_ == 4 || state_voo_ == 5) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ Ignorando waypoints de trajetória durante pouso/standby (estado %d)", state_voo_);
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

bool DroneControllerCompleto::check_landing_in_flight(double x, double y, double z)
{
  if ((state_voo_ == 2 || state_voo_ == 3) && z < config_.land_z_threshold) {
    trigger_landing(x, y, z);
    RCLCPP_WARN(this->get_logger(),
      "🛬 [ID=%lu] POUSO DETECTADO! Z = %.2f m - Comando LAND enfileirado",
      *land_cmd_id_, z);
    RCLCPP_WARN(this->get_logger(),
      "📍 Ancoragem de pouso definida: X=%.2f, Y=%.2f, Z=%.3f",
      ground_hold_x_, ground_hold_y_, ground_hold_z_);
    return true;
  }
  return false;
}

bool DroneControllerCompleto::handle_state4_disarm_reset()
{
  if (state_voo_ != 4) { return false; }
  if (!current_state_.armed) {
    RCLCPP_INFO(this->get_logger(), "✅ DRONE DESARMADO! Pronto para novo ciclo");
    offboard_activated_ = false;
    activation_confirmed_ = false;
    state_voo_ = 0;
    takeoff_counter_ = 0;
    pouso_em_andamento_ = false;
  }
  return true;
}

bool DroneControllerCompleto::handle_state5_flight_waypoint(
  const std::string & label, double x, double y, double z,
  const geometry_msgs::msg::PoseStamped & ps)
{
  if (z >= config_.land_z_threshold) {
    RCLCPP_INFO(this->get_logger(),
      "\n📍 [ESTADO 5] %sWAYPOINT DE VÔO RECEBIDO: X=%.2f, Y=%.2f, Z=%.2f – saindo do standby\n",
      label.c_str(), x, y, z);
    last_waypoint_goal_ = ps;
    waypoint_goal_received_ = true;
    pouso_em_andamento_ = false;
    controlador_ativo_ = false;
    trajectory_started_ = false;
    trajectory_waypoints_.clear();
    current_waypoint_idx_ = 0;

    if (current_state_.armed && current_state_.mode == "OFFBOARD") {
      RCLCPP_INFO(this->get_logger(),
        "🔋 [ESTADO 5] Já OFFBOARD+ARM – iniciando decolagem diretamente...");
    } else {
      RCLCPP_INFO(this->get_logger(),
        "🔋 [ESTADO 5] Reativando OFFBOARD+ARM para decolagem...");
      activate_offboard_arm_if_needed();
    }

    takeoff_cmd_id_ = cmd_queue_.enqueue(
      CommandType::TAKEOFF,
      {{"x", std::to_string(x)},
       {"y", std::to_string(y)},
       {"z", std::to_string(config_.hover_altitude)}});
    RCLCPP_INFO(this->get_logger(),
      "📋 [ID=%lu] Comando TAKEOFF enfileirado (saída do estado 5)", *takeoff_cmd_id_);

    state_voo_ = 1;
    takeoff_counter_ = 0;
  } else {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "ℹ️ [ESTADO 5] %sWaypoint de pouso ignorado (drone já no chão)", label.c_str());
  }
  return true;
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

  if (check_landing_in_flight(x, y, z)) { return; }
  if (handle_state4_disarm_reset()) { return; }

  if (state_voo_ == 5) {
    handle_state5_flight_waypoint("", x, y, z, *msg);
    return;
  }

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

  if (check_landing_in_flight(x, y, z)) {
    RCLCPP_WARN(this->get_logger(),
      "🛬 [ID=%lu] 4D POUSO DETECTADO! Z = %.2f m - Comando LAND enfileirado",
      *land_cmd_id_, z);
    return;
  }

  if (handle_state4_disarm_reset()) { return; }

  if (state_voo_ == 5) {
    handle_state5_flight_waypoint("4D ", x, y, z, *pose_stamped);
    return;
  }

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
    if (state_voo_ == 5) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "ℹ️ 4D Comando de pouso ignorado: drone já em standby no chão (estado 5)");
      return;
    }
    RCLCPP_WARN(this->get_logger(), "\n🛬🛬🛬 4D POUSO DETECTADO! Z_final = %.2f m", last_z);
    trigger_landing(poses[0].position.x, poses[0].position.y, last_z);
    trajectory_4d_mode_ = false;
    return;
  }

  if (msg->waypoints.size() == 1) {
    RCLCPP_INFO(this->get_logger(),
      "\n⬆️ 4D WAYPOINT DE LEVANTAMENTO recebido: X=%.2f Y=%.2f Z=%.2f yaw=%.3f rad",
      poses[0].position.x, poses[0].position.y, poses[0].position.z, yaws[0]);

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

  if (state_voo_ == 4 || state_voo_ == 5) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️ Ignorando trajetória 4D durante pouso/standby (estado %d)", state_voo_);
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

  RCLCPP_INFO(this->get_logger(), "\n✅ TRAJETÓRIA 4D ACEITA E ATIVADA! Drone em HOVER pronto!\n");
  if (hover_cmd_id_) {
    cmd_queue_.confirm(*hover_cmd_id_, true);
    hover_cmd_id_.reset();
  }
  trajectory_cmd_id_ = cmd_queue_.enqueue(
    CommandType::TRAJECTORY, {{"waypoints", std::to_string(msg->waypoints.size())}});
  controlador_ativo_ = true;
  pouso_em_andamento_ = false;
  state_voo_ = 3;
  RCLCPP_INFO(this->get_logger(), "✅ Trajetória 4D ativada - Entrando em ESTADO 3\n");
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
  if      (state_voo_ == 0) { handle_state0_wait_waypoint(); }
  else if (state_voo_ == 1) { handle_state1_takeoff(); }
  else if (state_voo_ == 2) { handle_state2_hover(); }
  else if (state_voo_ == 3) { handle_state3_trajectory(); }
  else if (state_voo_ == 4) { handle_state4_landing(); }
  else if (state_voo_ == 5) { handle_state5_standby(); }
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
  RCLCPP_INFO(this->get_logger(), "📡 Solicitando OFFBOARD+ARM...");
  request_offboard();
  request_arm();
  offboard_activated_ = true;
  activation_time_ = this->now();
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
    offboard_activated_ = false;
    activation_confirmed_ = false;
    takeoff_counter_ = 0;
    return false;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
    "⏳ Aguardando OFFBOARD+ARM... | armed=%d | mode=%s",
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
  if (!offboard_activated_) {
    request_arm_and_offboard_activation();
    return;
  }

  if (!activation_confirmed_) {
    if (!wait_for_offboard_arm_confirmation()) { return; }
  }

  double target_altitude = last_waypoint_goal_.pose.position.z;
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

  if (cycle_count_ % 500 == 0) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
      "🛸 Em HOVER (%.1fm) | Posição: X=%.2f, Y=%.2f | Aguardando waypoints... Controlador: %s",
      last_waypoint_goal_.pose.position.z,
      last_waypoint_goal_.pose.position.x,
      last_waypoint_goal_.pose.position.y,
      controlador_ativo_ ? "ATIVO" : "INATIVO");
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
  if (cycle_count_ % 500 != 0) { return; }

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

void DroneControllerCompleto::complete_landing_mode_a()
{
  if (land_cmd_id_) {
    cmd_queue_.confirm(*land_cmd_id_, true);
    RCLCPP_WARN(this->get_logger(),
      "✅ [ID=%lu] LAND confirmado (Modo A - sem DISARM)", *land_cmd_id_);
    land_cmd_id_.reset();
  }

  cmd_queue_.save_log("/tmp/drone_commands.log");
  RCLCPP_INFO(this->get_logger(),
    "💾 Histórico de comandos salvo em /tmp/drone_commands.log");

  RCLCPP_WARN(this->get_logger(),
    "\n✅ POUSO CONCLUÍDO (MODO A): mantendo OFFBOARD+ARM e segurando no chão.\n"
    "   Drone permanece armado em estado 5 (standby no chão).\n");

  ground_hold_z_ = std::max(0.01, config_.land_z_threshold);

  pouso_em_andamento_ = false;
  controlador_ativo_ = false;
  trajectory_started_ = false;
  pouso_start_time_set_ = false;
  takeoff_counter_ = 0;
  trajectory_waypoints_.clear();
  current_waypoint_idx_ = 0;
  takeoff_cmd_id_.reset();
  hover_cmd_id_.reset();
  trajectory_cmd_id_.reset();

  state_voo_ = 5;

  RCLCPP_WARN(this->get_logger(), "🔍 DEBUG MODO A (estado 5):");
  RCLCPP_WARN(this->get_logger(), "   offboard_activated_=%d (mantido)", offboard_activated_);
  RCLCPP_WARN(this->get_logger(), "   activation_confirmed_=%d (mantido)", activation_confirmed_);
  RCLCPP_WARN(this->get_logger(), "   ground_hold=(%.2f, %.2f, %.2f)",
    ground_hold_x_, ground_hold_y_, ground_hold_z_);
}

void DroneControllerCompleto::complete_landing_mode_b()
{
  RCLCPP_WARN(this->get_logger(), "🔌 Solicitando DISARM...");
  request_disarm();

  if (land_cmd_id_) {
    cmd_queue_.confirm(*land_cmd_id_, true);
    RCLCPP_WARN(this->get_logger(),
      "✅ [ID=%lu] LAND confirmado - pouso concluído", *land_cmd_id_);
    land_cmd_id_.reset();
  }

  cmd_queue_.save_log("/tmp/drone_commands.log");
  RCLCPP_INFO(this->get_logger(),
    "💾 Histórico de comandos salvo em /tmp/drone_commands.log");

  RCLCPP_WARN(this->get_logger(),
    "\n✅ POUSO CONCLUÍDO! Aguardando novo comando de waypoint para decolar novamente...\n");

  state_voo_ = 0;
  pouso_em_andamento_ = false;
  controlador_ativo_ = false;
  trajectory_started_ = false;
  pouso_start_time_set_ = false;
  offboard_activated_ = false;
  activation_confirmed_ = false;
  takeoff_counter_ = 0;
  trajectory_waypoints_.clear();
  current_waypoint_idx_ = 0;
  takeoff_cmd_id_.reset();
  hover_cmd_id_.reset();
  trajectory_cmd_id_.reset();

  RCLCPP_WARN(this->get_logger(), "🔍 DEBUG RESET EM ESTADO 4:");
  RCLCPP_WARN(this->get_logger(), "   offboard_activated_=%d (deve ser 0)", offboard_activated_);
  RCLCPP_WARN(this->get_logger(), "   state_voo_=%d (deve ser 0)", state_voo_);
}

void DroneControllerCompleto::handle_state4_landing()
{
  if (cycle_count_ % 500 == 0) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
      "🛑 CONTROLADOR PAUSADO | drone_soft_land fazendo pouso...");
  }

  if (pouso_em_andamento_) {
    if (!pouso_start_time_set_) {
      pouso_start_time_ = this->now();
      pouso_start_time_set_ = true;
      RCLCPP_INFO(this->get_logger(),
        "⏱️ Iniciando contagem de pouso (%.0f s para confirmar)...", config_.landing_timeout);
    }

    if ((this->now() - pouso_start_time_).seconds() > config_.landing_timeout) {
      if (landing_mode_ == 0) {
        complete_landing_mode_a();
      } else {
        complete_landing_mode_b();
      }
      return;
    }
  }

  // Modo A: publica setpoint de ancoragem para manter OFFBOARD ativo durante pouso
  if (landing_mode_ == 0) {
    publishPositionTarget(ground_hold_x_, ground_hold_y_, ground_hold_z_, 0.0, MASK_POS_ONLY);
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "🛬 [MODO A] Publicando setpoint de ancoragem durante pouso: X=%.2f, Y=%.2f, Z=%.3f",
      ground_hold_x_, ground_hold_y_, ground_hold_z_);
  }
  // Modo B: não publica nada durante pouso
}

// ============================================================
// FSM STATE 5 — STANDBY NO CHÃO (MODO A)
// ============================================================

void DroneControllerCompleto::handle_state5_standby()
{
  if (!current_state_.armed || current_state_.mode != "OFFBOARD") {
    bool should_request =
      (this->now() - state5_recovery_time_).seconds() > config_.activation_timeout;
    if (should_request) {
      RCLCPP_WARN(this->get_logger(),
        "⚠️ Estado 5: não está armado/OFFBOARD (armed=%d mode=%s). Tentando reativar...",
        current_state_.armed, current_state_.mode.c_str());
      request_offboard();
      request_arm();
      state5_recovery_time_ = this->now();
    }
    return;
  }

  publishPositionTarget(ground_hold_x_, ground_hold_y_, ground_hold_z_, 0.0, MASK_POS_ONLY);

  if (cycle_count_ % 500 == 0) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
      "🟢 STANDBY NO CHÃO | Setpoint: X=%.2f, Y=%.2f, Z=%.3f | Aguardando novo waypoint...",
      ground_hold_x_, ground_hold_y_, ground_hold_z_);
  }
}

}  // namespace drone_control
