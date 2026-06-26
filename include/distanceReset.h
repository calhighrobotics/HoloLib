#pragma once

#include "chassis.h"
#include "pros/distance.hpp"
#include <vector>

/**
 * @brief Configuration for a single distance sensor used in odometry reset.
 */
struct DistanceSensor {
  pros::Distance *device; /**< Pointer to the PROS Distance sensor object */
  double forward_offset;  /**< Offset of the sensor from the center of rotation (forward direction) */
  double strafe_offset;   /**< Offset of the sensor from the center of rotation (strafe direction) */
  double mounting_angle;  /**< Mounting angle of the sensor in degrees relative to the robot heading */
};

/**
 * @brief The resultant position calculated by distance sensors.
 */
struct distancePose {
  double x;             /**< X coordinate of the robot */
  double y;             /**< Y coordinate of the robot */
  bool using_odom_x;    /**< True if the X coordinate is from odometry (sensor couldn't read X) */
  bool using_odom_y;    /**< True if the Y coordinate is from odometry (sensor couldn't read Y) */
};

/**
 * @brief Handles resetting the robot's odometry using distance sensors.
 * 
 * It calculates the absolute position on the field using distance to walls.
 */
class DistanceReset {
private:
  static constexpr double MM_TO_IN = 0.0393701;
  static constexpr double FIELD_WIDTH = 3566.668 * MM_TO_IN;
  static constexpr double FIELD_HEIGHT = 3566.668 * MM_TO_IN;
  static constexpr double HALF_WIDTH = FIELD_WIDTH / 2.0;
  static constexpr double HALF_HEIGHT = FIELD_HEIGHT / 2.0;
  static constexpr double MAX_SENSOR_RANGE = 2000 * MM_TO_IN;
  static constexpr double MIN_SENSOR_RANGE = 0 * MM_TO_IN;

  Chassis *chassis;
  std::vector<DistanceSensor> sensors;
  float default_heading_tolerance;
  float default_filter_range;

  distancePose
  calculateGlobalPosition(const std::vector<DistanceSensor> &active_sensors,
                          double heading_deg, double current_x,
                          double current_y);

public:
  /**
   * @brief Construct a DistanceReset handler.
   * 
   * @param robot_chassis Pointer to the chassis object.
   * @param robot_sensors Vector of DistanceSensor configurations.
   * @param heading_tolerance Tolerance in degrees for the sensor to be considered perpendicular to the wall.
   * @param filter_range Maximum acceptable deviation from the current odometry reading.
   */
  DistanceReset(Chassis *robot_chassis,
                const std::vector<DistanceSensor> &robot_sensors,
                float heading_tolerance = 40.0, float filter_range = 3.5);

  /**
   * @brief Updates position using all configured sensors.
   * 
   * @param setPose If true, automatically updates the chassis odometry with the calculated position.
   * @param filter If true, applies filtering based on `filter_range`.
   * @return distancePose The newly calculated position.
   */
  distancePose update(bool setPose = false, bool filter = true);

  /**
   * @brief Updates position using a specific set of sensors by flags.
   * 
   * @param use_flags Boolean flags corresponding to each configured sensor (true to use).
   * @param setPose If true, automatically updates the chassis odometry.
   * @param filter If true, applies filtering.
   * @return distancePose The newly calculated position.
   */
  distancePose update(const std::vector<bool> &use_flags, bool setPose = false,
                      bool filter = true);

  /**
   * @brief Updates position using an explicitly provided list of active sensors.
   * 
   * @param active_sensors Vector of sensors to use for this update.
   * @param setPose If true, automatically updates the chassis odometry.
   * @param filter If true, applies filtering.
   * @return distancePose The newly calculated position.
   */
  distancePose updateSpecific(const std::vector<DistanceSensor> &active_sensors,
                              bool setPose, bool filter);
};
