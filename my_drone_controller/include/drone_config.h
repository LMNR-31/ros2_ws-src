#ifndef DRONE_CONFIG_H
#define DRONE_CONFIG_H

namespace drone_control {

/**
 * @brief Centralised configuration constants for the drone controller.
 *
 * All magic numbers that were previously hard-coded throughout
 * main_codegen.cpp are collected here as named fields with sensible
 * defaults.  At runtime the node loads each field from a ROS 2 parameter
 * so the values can be overridden via drone_config.yaml without
 * recompilation.
 */
struct DroneConfig {
  // ── Altitude ──────────────────────────────────────────────────────────
  /// Target hover altitude [m]
  double hover_altitude{2.0};
  /// Hysteresis margin below hover_altitude used to detect arrival [m]
  double hover_altitude_margin{0.05};
  /// Maximum permitted altitude for incoming waypoints [m]
  double max_altitude{500.0};
  /// Time spent publishing each waypoint during trajectory execution [s]
  double waypoint_duration{4.0};

  // ── Limites de Segurança ───────────────────────────────────────────────
  /// Altitude mínima permitida para voo (drone não desce abaixo durante navegação) [m]
  double min_altitude{0.2};
  /// Z threshold abaixo do qual o drone é considerado pousado [m]
  double land_z_threshold{0.1};

  // ── Validation limits ─────────────────────────────────────────────────
  /// Maximum allowed |X| or |Y| distance for incoming waypoints [m]
  double max_waypoint_distance{100.0};

  // ── Takeoff boost ─────────────────────────────────────────────────────
  /// Minimum Z increment above current odometry altitude applied to the
  /// takeoff setpoint immediately after arming [m].
  ///
  /// After the FCU accepts ARM in OFFBOARD mode, PX4 starts an auto-disarm
  /// timer if no upward movement is detected within a few seconds.  Publishing
  /// a Z target of at least (current_z_real + takeoff_z_boost) guarantees that
  /// the FCU sees a clear climb intent and does not trigger auto-disarm.
  /// The effective target is: max(hover_altitude, current_z_real + takeoff_z_boost).
  double takeoff_z_boost{0.7};

  // ── Timeouts ──────────────────────────────────────────────────────────
  /// Timeout waiting for OFFBOARD + ARM confirmation from FCU [s]
  double activation_timeout{5.0};
  /// Timeout before a pending command is automatically marked TIMEOUT [s]
  double command_timeout{15.0};
  /// Time to wait after landing is detected before requesting DISARM [s]
  double landing_timeout{3.0};
  /// Timeout waiting for FCU to confirm OFFBOARD mode before sending ARM [s].
  /// ARM is only sent once the FCU reports mode == "OFFBOARD"; this parameter
  /// controls how long we wait for that confirmation before retrying OFFBOARD.
  double offboard_confirm_timeout{5.0};
};

}  // namespace drone_control

#endif  // DRONE_CONFIG_H
