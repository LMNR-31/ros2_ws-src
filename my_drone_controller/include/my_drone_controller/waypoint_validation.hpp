#ifndef MY_DRONE_CONTROLLER__WAYPOINT_VALIDATION_HPP_
#define MY_DRONE_CONTROLLER__WAYPOINT_VALIDATION_HPP_

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "drone_config.h"

namespace drone_control {

/**
 * @brief Validate a PoseStamped waypoint against physical limits.
 *
 * Checks for NaN/Inf coordinates and enforces the XY distance and altitude
 * limits defined in @p config.
 *
 * @return true if the waypoint is safe to use.
 */
bool validate_waypoint(const geometry_msgs::msg::PoseStamped & msg,
                       const DroneConfig & config);

/**
 * @brief Validate a plain Pose (without header) against physical limits.
 *
 * Convenience wrapper around validate_waypoint().
 *
 * @return true if the pose is safe to use.
 */
bool validate_pose(const geometry_msgs::msg::Pose & pose,
                   const DroneConfig & config);

}  // namespace drone_control

#endif  // MY_DRONE_CONTROLLER__WAYPOINT_VALIDATION_HPP_
