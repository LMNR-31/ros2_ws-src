#include "my_drone_controller/waypoint_validation.hpp"

#include "rclcpp/rclcpp.hpp"

#include <cmath>

namespace drone_control {

bool validate_waypoint(const geometry_msgs::msg::PoseStamped & msg,
                       const DroneConfig & config)
{
  const auto & pos = msg.pose.position;

  if (std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.z)) return false;
  if (std::isinf(pos.x) || std::isinf(pos.y) || std::isinf(pos.z)) return false;

  // Z < land_z_threshold: landing intent — always accepted regardless of min_altitude.
  // Skip altitude range checks and fall through to XY validation.
  if (pos.z >= config.land_z_threshold) {
    if (pos.z < config.min_altitude) {
      RCLCPP_WARN(rclcpp::get_logger("validate_waypoint"),
        "❌ Waypoint Z=%.2fm REJEITADO: abaixo da altitude mínima de voo (%.2fm)",
        pos.z, config.min_altitude);
      return false;
    }
    if (pos.z > config.max_altitude) {
      RCLCPP_WARN(rclcpp::get_logger("validate_waypoint"),
        "❌ Waypoint Z=%.2fm REJEITADO: acima da altitude máxima (%.2fm)",
        pos.z, config.max_altitude);
      return false;
    }
  }

  if (std::abs(pos.x) > config.max_waypoint_distance) return false;
  if (std::abs(pos.y) > config.max_waypoint_distance) return false;

  return true;
}

bool validate_pose(const geometry_msgs::msg::Pose & pose,
                   const DroneConfig & config)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.pose = pose;
  return validate_waypoint(ps, config);
}

}  // namespace drone_control
