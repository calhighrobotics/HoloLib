#pragma once

#include <string>
#include <vector>

/**
 * @brief Represents a single point along an autonomous path.
 */
struct PathPoint {
  float x = 0;     /**< Target X coordinate */
  float y = 0;     /**< Target Y coordinate */
  float theta = 0; /**< Target heading at this point */
};

/**
 * @brief Specifies the heading behavior during path following.
 */
enum class HeadingMode { FollowPath, HoldAngle, CustomAngles };

/**
 * @brief Parses path point data from a file or string.
 *
 * @param input_source The source identifier (e.g. filename).
 * @param convertFromMeters True to scale the values from meters to inches.
 * @return std::vector<PathPoint> The parsed vector of path points.
 */
std::vector<PathPoint> parsePathData(const std::string &input_source,
                                     bool convertFromMeters = false);
