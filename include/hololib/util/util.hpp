#pragma once

#include <cmath>
namespace hololib {
/**
 * @brief Helper utility to calculate the shortest path between two angles.
 *
 * @param target The desired angle in degrees.
 * @param current The current angle in degrees.
 * @return float The minimal error difference [-180, 180].
 */
float getAngleError(float target, float current); 
}; // namespace hololib