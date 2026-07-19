#include "hololib/motions/motion_cancel_helper.hpp"
#include "hololib/motions/motions.hpp"
#include "hololib/util/PID.hpp"
#include "hololib/util/Timer.hpp"
#include "hololib/util/util.hpp"
#include <algorithm>
#include <cmath>

namespace hololib {
void turnToPoint(float tx, float ty, MoveParams params, MoveSettings settings) {
    uint32_t settleStart = 0;
    constexpr uint32_t settleTime = 100;

    PID tPID(0, 0, 0, 0);
    float prevError = 0.0f;
    Timer timeoutTimer(params.timeout);
    MotionCancelHelper helper(10);

    while (helper.wait() && !timeoutTimer.isDone()) {
        Pose curr = settings.poseGetter(false);
        float ex = tx - curr.x;
        float ey = ty - curr.y;
        float targetDeg = std::atan2(ex, ey) * RAD2DEG;
        float error = getAngleError(targetDeg, curr.theta);

        if (params.earlyExitRange > 0.0f && std::abs(error) <= params.earlyExitRange)
            return;

        if (std::abs(error) < params.exitRange) {
            if (settleStart == 0)
                settleStart = pros::millis();
            float vel = (error - prevError) / 0.01f;
            if (pros::millis() - settleStart >= settleTime && std::abs(vel) < 0.5f)
                break;
        } else {
            settleStart = 0;
        }

        tPID.setGains(settings.thetaSched.getGains(error));
        float output = static_cast<float>(tPID.update(error));

        if (std::abs(output) > 1e-3f && std::abs(output) < params.minSpeed)
            output = std::copysign(params.minSpeed, output);
        output = std::clamp(output, -params.maxRotationSpeed, params.maxRotationSpeed);

        settings.chassis.xdrive(0.0f, 0.0f, output, curr.theta);
        prevError = error;
        pros::delay(10);
    }

    settings.chassis.brake();
}
} // namespace hololib
