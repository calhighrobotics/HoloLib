#include "chassis.h"
#include <algorithm>
#include <cmath>

/**
 * @brief Sets the robot dimensions (from WHEEL TO WHEEL)
 *
 * @param width The width of the robot (from wheel to wheel) in inches.
 * @param length The length of the robot (from wheel to wheel) in inches.
 *
 * @return void
 *
 * @note What you classify as width and length is dependant on your
 * interpretation of front in your robot's local reference frame.
 */
void ObstacleManager::setRobotDimensions(float width, float length) {
  robot_width = width;
  robot_length = length;
}

/**
 *@brief Adds an obstacle to the obstacle manager.
 *@param x The x-coordinate of the center of the obstacle in meters.
 *@param y The y-coordinate of the center of the obstacle in meters.
 *@param radius The radius of the obstacle in meters.
 *@return void
 *@note It may be wise to combine circles together to create more complex shapes
 */
void ObstacleManager::addObstacle(float x, float y, float radius) {
  obstacles.push_back({Eigen::Vector2f(x, y), radius});
}

/**
 *@brief Removes an obstacle from the obstacle manager.
 *@param index The index of the obstacle to remove.
 *@return void
 */
void ObstacleManager::removeObstacle(size_t index) {
  if (index < obstacles.size()) {
    obstacles.erase(obstacles.begin() + index);
  }
}

/**
 *@brief Clears all obstacles from the obstacle manager.
 *@return void
 */
void ObstacleManager::clearObstacles() { obstacles.clear(); }

/**
 *@brief Gets the obstacles from the obstacle manager.
 *@return const std::vector<Obstacle>& The obstacles.
 */
const std::vector<Obstacle> &ObstacleManager::getObstacles() const {
  return obstacles;
}

/**
 *@brief Checks if there is an intersection between the robot and an obstacle.
 *@param start The starting point of the robot.
 *@param end The ending point of the robot.
 *@param safety_margin The safety margin around the obstacle.
 *@param out_obstacle The obstacle that is intersected.
 *@param out_closest The closest point on the obstacle to the robot.
 *@return Returns whether or not an object is found at the intersection.
 */
bool ObstacleManager::checkIntersection(const Eigen::Vector2f &start,
                                        const Eigen::Vector2f &end,
                                        float safety_margin,
                                        Obstacle &out_obstacle,
                                        Eigen::Vector2f &out_closest) const {
  Eigen::Vector2f ab = end - start;
  float ab_len_sq = ab.squaredNorm();
  if (ab_len_sq < 1e-6f) {
    for (const auto &obs : obstacles) {
      float dist = (start - obs.position).norm();
      if (dist < obs.radius + safety_margin) {
        out_obstacle = obs;
        out_closest = start;
        return true;
      }
    }
    return false;
  }

  for (const auto &obs : obstacles) {
    Eigen::Vector2f ac = obs.position - start;
    float t = ac.dot(ab) / ab_len_sq;
    t = std::clamp(t, 0.0f, 1.0f);

    Eigen::Vector2f closest = start + t * ab;
    float dist = (closest - obs.position).norm();

    if (dist < obs.radius + safety_margin) {
      out_obstacle = obs;
      out_closest = closest;
      return true;
    }
  }
  return false;
}

/**
 *@brief Gets the avoidance target for the robot.
 *@param robot_pos The position of the robot.
 *@param target_pos The target position of the robot.
 *@param safety_margin The safety margin around the obstacle.
 *@param clearance The clearance around the obstacle.
 *@param robot_heading_rad The heading of the robot in radians.
 *@param recursion_depth The recursion depth.
 *@return Eigen::Vector2f The avoidance target.
 */
Eigen::Vector2f ObstacleManager::getAvoidanceTarget(
    const Eigen::Vector2f &robot_pos, const Eigen::Vector2f &target_pos,
    float safety_margin, float clearance, float robot_heading_rad,
    int recursion_depth) const {
  if (recursion_depth > 3) {
    return target_pos;
  }

  Obstacle obs;
  Eigen::Vector2f closest;
  if (checkIntersection(robot_pos, target_pos, safety_margin, obs, closest)) {
    Eigen::Vector2f to_obs = obs.position - robot_pos;
    Eigen::Vector2f perp(-to_obs.y(), to_obs.x());
    if (perp.squaredNorm() < 1e-6f) {
      return target_pos;
    }
    perp.normalize();
    float abs_obs_angle = std::atan2(to_obs.x(), to_obs.y());
    float alpha = abs_obs_angle - robot_heading_rad;
    float r_width = (robot_width / 2.0f) / (std::abs(std::sin(alpha)) + 1e-6f);
    float r_length =
        (robot_length / 2.0f) / (std::abs(std::cos(alpha)) + 1e-6f);
    float dynamic_clearance = std::min(r_width, r_length) + clearance;

    Eigen::Vector2f w1 = obs.position + perp * (obs.radius + dynamic_clearance);
    Eigen::Vector2f w2 = obs.position - perp * (obs.radius + dynamic_clearance);

    float d1 = (w1 - robot_pos).norm() + (target_pos - w1).norm();
    float d2 = (w2 - robot_pos).norm() + (target_pos - w2).norm();
    Eigen::Vector2f best_waypoint = (d1 < d2) ? w1 : w2;

    return getAvoidanceTarget(robot_pos, best_waypoint, safety_margin,
                              clearance, robot_heading_rad,
                              recursion_depth + 1);
  }

  return target_pos;
}

/**
 *@brief Gets the potential field target for the robot.
 *@param robot_pos The position of the robot.
 *@param target_pos The target position of the robot.
 *@param ka The attractive gain.
 *@param kr The repulsive gain.
 *@param influence_radius The influence radius.
 *@param robot_heading_rad The heading of the robot in radians.
 *@return Eigen::Vector2f The potential field target.
 */
Eigen::Vector2f ObstacleManager::getPotentialFieldTarget(
    const Eigen::Vector2f &robot_pos, const Eigen::Vector2f &target_pos,
    float ka, float kr, float influence_radius, float robot_heading_rad) const {
  Eigen::Vector2f to_target = target_pos - robot_pos;
  float dist_to_target = to_target.norm();
  if (dist_to_target < 1e-4f) {
    return target_pos;
  }

  Eigen::Vector2f F_attractive = (to_target / dist_to_target) * ka;
  Eigen::Vector2f F_repulsive = Eigen::Vector2f::Zero();

  for (const auto &obs : obstacles) {
    Eigen::Vector2f to_obs = robot_pos - obs.position;
    float dist_to_center = to_obs.norm();

    float abs_obs_angle = std::atan2(to_obs.x(), to_obs.y());
    float alpha = abs_obs_angle - robot_heading_rad;

    float sin_alpha = std::abs(std::sin(alpha));
    float cos_alpha = std::abs(std::cos(alpha));

    float r_width = (robot_width / 2.0f) / (sin_alpha + 1e-6f);
    float r_length = (robot_length / 2.0f) / (cos_alpha + 1e-6f);
    float dynamic_clearance = std::min(r_width, r_length);

    float effective_radius = obs.radius + dynamic_clearance + 1.0f;
    float dist_to_boundary = dist_to_center - effective_radius;

    if (dist_to_boundary <= 0.0f) {
      Eigen::Vector2f dir = (dist_to_center > 1e-4f)
                                ? to_obs / dist_to_center
                                : Eigen::Vector2f(1.0f, 0.0f);
      F_repulsive += dir * kr * 5.0f;
    } else if (dist_to_boundary <= influence_radius) {
      Eigen::Vector2f dir = to_obs / dist_to_center;
      float factor = 1.0f - (dist_to_boundary / influence_radius);

      float force_mag = kr * factor / (dist_to_boundary + 0.1f);

      Eigen::Vector2f direct_repulse = dir * force_mag;
      Eigen::Vector2f tangent(-dir.y(), dir.x());
      if (tangent.dot(to_target) < 0) {
        tangent = Eigen::Vector2f(dir.y(), -dir.x());
      }

      Eigen::Vector2f tangential_bypass = tangent * (force_mag * 0.85f);
      F_repulsive += direct_repulse + tangential_bypass;
    }
  }

  Eigen::Vector2f F_total = F_attractive + F_repulsive;
  if (F_total.norm() < 1e-3f) {
    F_total = F_attractive +
              Eigen::Vector2f(-F_attractive.y(), F_attractive.x()) * 0.5f;
  }
  float step_size = std::min(10.0f, dist_to_target);
  return robot_pos + F_total.normalized() * step_size;
}

/**
 *@brief Adds an obstacle to the obstacle manager.
 *@param x The x-coordinate of the center of the obstacle in meters.
 *@param y The y-coordinate of the center of the obstacle in meters.
 *@param radius The radius of the obstacle in meters.
 *@return void
 */
void Chassis::addObstacle(float x, float y, float radius) {
  obstacles.addObstacle(x, y, radius);
}

/**
 *@brief Removes an obstacle from the obstacle manager.
 *@param index The index of the obstacle to remove.
 *@return void
 */
void Chassis::removeObstacle(size_t index) { obstacles.removeObstacle(index); }

/**
 *@brief Clears all obstacles from the obstacle manager.
 *@return void
 */
void Chassis::clearObstacles() { obstacles.clearObstacles(); }

/**
 *@brief Sets the avoidance mode.
 *@param mode The avoidance mode.
 *@return void
 */
void Chassis::setAvoidanceMode(AvoidanceMode mode) { avoidanceMode = mode; }

/**
 *@brief Sets the avoidance parameters.
 *@param safetyMargin The safety margin around the obstacle.
 *@param clearance The clearance around the obstacle.
 *@return void
 */
void Chassis::setAvoidanceParams(float safetyMargin, float clearance) {
  avoidanceSafetyMargin = safetyMargin;
  avoidanceClearance = clearance;
}

/**
 *@brief Sets the potential field parameters.
 *@param ka The attractive gain.
 *@param kr The repulsive gain.
 *@param influenceRadius The influence radius.
 *@return void
 */
void Chassis::setPotentialFieldParams(float ka, float kr,
                                      float influenceRadius) {
  pf_ka = ka;
  pf_kr = kr;
  pf_influence_radius = influenceRadius;
}
