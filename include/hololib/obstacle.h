#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include <cstddef>
#include <vector>

/**
 * @brief Represents a physical obstacle on the field.
 */
struct Obstacle {
  Eigen::Vector2f position; /**< 2D coordinate position of the obstacle */
  float radius;             /**< Radius of the obstacle used for avoidance */
};

/**
 * @brief Manages a collection of obstacles and provides avoidance computations.
 *
 * Provides methods for checking path intersections and generating vectors to
 * steer away from potential collisions.
 */
class ObstacleManager {
public:
  /**
   * @brief Default constructor for ObstacleManager.
   */
  ObstacleManager() = default;

  /**
   * @brief Set the dimensions of the robot to define its bounding box for
   * collisions.
   *
   * @param width Width of the robot.
   * @param length Length of the robot.
   */
  void setRobotDimensions(float width, float length);

  /**
   * @brief Add a circular obstacle to the field representation.
   *
   * @param x X coordinate of the obstacle.
   * @param y Y coordinate of the obstacle.
   * @param radius Radius of the obstacle.
   */
  void addObstacle(float x, float y, float radius);

  /**
   * @brief Remove an obstacle by its index.
   *
   * @param index The index of the obstacle in the internal vector.
   */
  void removeObstacle(size_t index);

  /**
   * @brief Clear all tracked obstacles.
   */
  void clearObstacles();

  /**
   * @brief Get the list of currently tracked obstacles.
   *
   * @return const std::vector<Obstacle>& Reference to the obstacle vector.
   */
  const std::vector<Obstacle> &getObstacles() const;

  /**
   * @brief Check if a path segment intersects with any obstacle.
   *
   * @param start Start point of the segment.
   * @param end End point of the segment.
   * @param safety_margin Extra margin to add to the robot/obstacle bounds.
   * @param out_obstacle Output parameter for the intersected obstacle.
   * @param out_closest Output parameter for the closest point of intersection.
   * @return true if an intersection occurs, false otherwise.
   */
  bool checkIntersection(const Eigen::Vector2f &start,
                         const Eigen::Vector2f &end, float safety_margin,
                         Obstacle &out_obstacle,
                         Eigen::Vector2f &out_closest) const;

  /**
   * @brief Calculate a target vector that avoids collisions while navigating
   * towards a goal.
   *
   * @param robot_pos Current robot position.
   * @param target_pos Desired goal position.
   * @param safety_margin Extra margin around obstacles.
   * @param clearance Desired clearance distance from obstacles.
   * @param robot_heading_rad Current robot heading in radians.
   * @param recursion_depth Internal use for recursive path-finding limitations.
   * @return Eigen::Vector2f The corrected target point to aim for.
   */
  Eigen::Vector2f getAvoidanceTarget(const Eigen::Vector2f &robot_pos,
                                     const Eigen::Vector2f &target_pos,
                                     float safety_margin, float clearance,
                                     float robot_heading_rad,
                                     int recursion_depth = 0) const;

  /**
   * @brief Get an artificial potential field target vector.
   *
   * @param robot_pos Current robot position.
   * @param target_pos Desired goal position.
   * @param ka Attractive force constant.
   * @param kr Repulsive force constant.
   * @param influence_radius Distance at which obstacles begin exerting
   * repulsive force.
   * @param robot_heading_rad Current robot heading in radians.
   * @return Eigen::Vector2f The resultant vector from potential fields.
   */
  Eigen::Vector2f getPotentialFieldTarget(const Eigen::Vector2f &robot_pos,
                                          const Eigen::Vector2f &target_pos,
                                          float ka, float kr,
                                          float influence_radius,
                                          float robot_heading_rad) const;

private:
  std::vector<Obstacle> obstacles;
  float robot_width = 18.0f;
  float robot_length = 18.0f;
};
