#include "hololib/motions/motion_cancel_helper.hpp"
#include "hololib/motions/motions.hpp"
#include "hololib/util/PID.hpp"
#include "hololib/util/Timer.hpp"
#include "hololib/util/util.hpp"
#include <algorithm>
#include <cmath>

namespace hololib {
void swingTurn(float targetThetaDeg, SwingSide lockedSide, MoveParams params, MoveSettings settings) {
    uint32_t settleStart = 0;
    constexpr uint32_t settleTime = 120;
    constexpr float angleExitDeg = 2.0f;

    PID tPID(0, 0, 0, 0);
    float maxRotation = std::min(params.maxRotationSpeed, 127.0f);
    Timer timeoutTimer(params.timeout);
    MotionCancelHelper helper(10);

    while (helper.wait() && !timeoutTimer.isDone()) {
        Pose curr = settings.poseGetter(false);
        float angleError = getAngleError(targetThetaDeg, curr.theta);

        bool angleSettled = std::abs(angleError) < angleExitDeg;
        if (angleSettled) {
            if (settleStart == 0)
                settleStart = pros::millis();
            if (pros::millis() - settleStart >= settleTime)
                break;
        } else {
            settleStart = 0;
        }

        tPID.setGains(settings.thetaSched.getGains(angleError));
        float outT = static_cast<float>(tPID.update(angleError));
        if (!angleSettled && std::abs(outT) < params.minSpeed) {
            outT = std::copysign(params.minSpeed, outT);
        }
        outT = std::clamp(outT, -maxRotation, maxRotation);

        float outX_local = 0.0f;
        float outY_local = (lockedSide == SwingSide::Left) ? -outT : outT;

        float rad = curr.theta * DEG2RAD;
        float cosH = std::cos(rad);
        float sinH = std::sin(rad);

        float outX_global = outX_local * cosH + outY_local * sinH;
        float outY_global = -outX_local * sinH + outY_local * cosH;

        settings.chassis.xdrive(outX_global, outY_global, outT, curr.theta);
        pros::delay(10);
    }

    settings.chassis.brake();
}
} // namespace hololib
