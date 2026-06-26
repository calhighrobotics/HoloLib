#pragma once

#include "PID.h"
#include <vector>

/**
 * @brief A mapping of error threshold to specific PID gains.
 */
struct ScheduledGain {
  float threshold; /**< The error bound for these gains */
  PIDGains
      gains; /**< The PID gains applied when error is below this threshold */
};

/**
 * @brief A scheduler that interpolates and selects PID gains based on the
 * current error.
 */
class GainScheduler {
public:
  /**
   * @brief Add a scheduling step.
   *
   * @param threshold Max error for this step.
   * @param kP Proportional gain.
   * @param kI Integral gain.
   * @param kD Derivative gain.
   * @param slew Slew rate.
   */
  void addStep(float threshold, float kP, float kI, float kD,
               float slew = 0.0f);

  /**
   * @brief Compute the correct gains based on the current error distance.
   *
   * @param error The current positional or rotational error.
   * @return PIDGains Interpolated gains for this specific error level.
   */
  PIDGains getGains(float error) const;

  /**
   * @brief Clears the current schedules.
   */
  void clear();

private:
  std::vector<ScheduledGain> schedules;
};
