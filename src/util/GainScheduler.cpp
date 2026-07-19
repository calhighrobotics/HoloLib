#include "hololib/util/GainScheduler.hpp"
#include <algorithm>
#include <cmath>

namespace hololib {
void GainScheduler::addStep(float threshold, float kP, float kI, float kD, float slew) {
    schedules.push_back({
        threshold, {kP, kI, kD, 0.0f, slew}
    });
    std::sort(
        schedules.begin(), schedules.end(),
        [](const ScheduledGain& a, const ScheduledGain& b) { return a.threshold < b.threshold; });
}

PIDGains GainScheduler::getGains(float error) const {
    if (schedules.empty())
        return {0, 0, 0, 0, 0};

    float absErr = std::abs(error);

    if (absErr <= schedules.front().threshold)
        return schedules.front().gains;

    if (absErr >= schedules.back().threshold)
        return schedules.back().gains;

    for (size_t i = 0; i < schedules.size() - 1; ++i) {
        const auto& lo = schedules[i];
        const auto& hi = schedules[i + 1];
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

void GainScheduler::clear() { schedules.clear(); }

void GainScheduler::setGains(std::vector<ScheduledGain> newSchedules) {
    clear();
    for (const auto& step : newSchedules) {
        this->addStep(step.threshold, step.gains.kP, step.gains.kI, step.gains.kD, step.gains.slew);
    }
}
} // namespace hololib