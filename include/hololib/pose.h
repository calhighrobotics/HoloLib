#pragma once

#include <cmath>

/**
 * @brief Contains internal robot velocities.
 */
struct VelocityComponents {
  float vx, vy, w; /**< Velocities in X, Y, and Rotational domains */
};

/**
 * @brief Contains global coordinates and velocity.
 */
struct Pose {
  float x = 0;                          /**< Global X coordinate */
  float y = 0;                          /**< Global Y coordinate */
  float theta = 0;                      /**< Global heading angle */
  VelocityComponents velocity{0, 0, 0}; /**< Current velocity components */
};

/**
 * @brief Holonomic target voltages for a 4-motor X-drive or mecanum base.
 */
struct XDriveVoltages {
  float fl, fr, bl,
      br; /**< Front-left, Front-right, Back-left, Back-right voltages */
};

/**
 * @brief Helper utility to calculate the shortest path between two angles.
 *
 * @param target The desired angle in degrees.
 * @param current The current angle in degrees.
 * @return float The minimal error difference [-180, 180].
 */
inline float getAngleError(float target, float current) {
  return std::remainder(target - current, 360.0f);
}
