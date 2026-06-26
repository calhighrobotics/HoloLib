#include "hololib/gain_scheduler.h"
#include <algorithm>
#include <cmath>

/**
 *@brief Adds a gain step to the gain scheduler.
 *@param threshold The error threshold for the gain step.
 *@param kP The proportional gain for the gain step.
 *@param kI The integral gain for the gain step.
 *@param kD The derivative gain for the gain step.
 *@param slew The slew rate for the gain step.
 *@return void
 */
void GainScheduler::addStep(float threshold, float kP, float kI, float kD,
                            float slew) {
  schedules.push_back({threshold, {kP, kI, kD, 0.0f, slew}});
  std::sort(schedules.begin(), schedules.end(),
            [](const ScheduledGain &a, const ScheduledGain &b) {
              return a.threshold < b.threshold;
            });
}

/**
 *@brief Gets the gains for a given error.
 *@param error The error to get the gains for.
 *@return PIDGains The gains for the given error.
 */
PIDGains GainScheduler::getGains(float error) const {
  if (schedules.empty())
    return {0, 0, 0, 0, 0};

  float absErr = std::abs(error);

  if (absErr <= schedules.front().threshold)
    return schedules.front().gains;

  if (absErr >= schedules.back().threshold)
    return schedules.back().gains;

  for (size_t i = 0; i < schedules.size() - 1; ++i) {
    const auto &lo = schedules[i];
    const auto &hi = schedules[i + 1];
    if (absErr >= lo.threshold && absErr <= hi.threshold) {
      float t = (absErr - lo.threshold) / (hi.threshold - lo.threshold);
      return {lo.gains.kP + t * (hi.gains.kP - lo.gains.kP),
              lo.gains.kI + t * (hi.gains.kI - lo.gains.kI),
              lo.gains.kD + t * (hi.gains.kD - lo.gains.kD), 0.0f,
              lo.gains.slew + t * (hi.gains.slew - lo.gains.slew)};
    }
  }

  return schedules.back().gains;
}

/**
 *@brief Clears all gain steps from the gain scheduler.
 *@return void
 */
void GainScheduler::clear() { schedules.clear(); }
