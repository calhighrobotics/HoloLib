#include "hololib/motions/motion_cancel_helper.hpp"
#include "hololib/motions/motions.hpp"
#include "hololib/util/PID.hpp"
#include "hololib/util/Timer.hpp"
#include "hololib/util/util.hpp"
#include <algorithm>
#include <cmath>

namespace hololib {
void moveToPose(float tx, float ty, float targetThetaDeg, MoveParams params, MoveSettings settings) {
    uint32_t settleStart = 0;
    constexpr uint32_t settleTime = 120;
    constexpr float angleExitDeg = 2.0f;

    PID xPID(0, 0, 0, 0), yPID(0, 0, 0, 0), tPID(0, 0, 0, 0);
    Timer timeoutTimer(params.timeout);
    MotionCancelHelper helper(10);

    while (helper.wait() && !timeoutTimer.isDone()) {
        Pose curr = settings.poseGetter(false);
        float ex = tx - curr.x;
        float ey = ty - curr.y;
        float distErr = std::hypot(ex, ey);
        float angleError = getAngleError(targetThetaDeg, curr.theta);

        if (params.earlyExitRange > 0.0f && distErr <= params.earlyExitRange)
            return;

        bool posSettled = distErr < params.exitRange;
        bool angleSettled = std::abs(angleError) < angleExitDeg;
        if (posSettled && angleSettled) {
            if (settleStart == 0)
                settleStart = pros::millis();
            if (pros::millis() - settleStart >= settleTime)
                break;
        } else {
            settleStart = 0;
        }

        xPID.setGains(settings.xSched.getGains(ex));
        yPID.setGains(settings.ySched.getGains(ey));
        tPID.setGains(settings.thetaSched.getGains(angleError));

        float outX_g = static_cast<float>(xPID.update(ex));
        float outY_g = static_cast<float>(yPID.update(ey));
        float outT = static_cast<float>(tPID.update(angleError));

        float mag = std::hypot(outX_g, outY_g);
        if (!posSettled && mag > 1e-3f && mag < params.minSpeed) {
            float s = params.minSpeed / mag;
            outX_g *= s;
            outY_g *= s;
        }
        if (mag > params.maxTranslationSpeed) {
            float s = params.maxTranslationSpeed / mag;
            outX_g *= s;
            outY_g *= s;
        }
        outT = std::clamp(outT, -params.maxRotationSpeed, params.maxRotationSpeed);

        settings.chassis.xdrive(outX_g, outY_g, outT, curr.theta);
        pros::delay(10);
    }

    settings.chassis.brake();
}
} // namespace hololib
