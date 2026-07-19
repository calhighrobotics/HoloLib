#include "hololib/motions/motion_cancel_helper.hpp"
#include "hololib/motions/motions.hpp"
#include "hololib/util/PID.hpp"
#include "hololib/util/Timer.hpp"
#include "hololib/util/util.hpp"
#include "hololib/util/obstacle_manager.hpp"
#include <cstdint>

namespace hololib {
void moveToPoint(float tx, float ty, MoveParams params, MoveSettings settings) {

    uint32_t settleStart = 0;
    constexpr uint32_t settleTime = 100;

    PID xPID(0, 0, 0, 0), yPID(0, 0, 0, 0), tPID(0, 0, 0, 0);
    Timer timeoutTimer(params.timeout);
    float holdHeading = settings.poseGetter(false).theta;

    MotionCancelHelper helper(10);

    while (helper.wait() && !timeoutTimer.isDone()) {
        Pose curr = settings.poseGetter(false);

        float distErr = std::hypot(tx - curr.x, ty - curr.y);

        if (params.earlyExitRange > 0.0f && distErr <= params.earlyExitRange)
            return;

        if (distErr < params.exitRange) {
            if (settleStart == 0)
                settleStart = pros::millis();
            if (pros::millis() - settleStart >= settleTime)
                break;
        } else {
            settleStart = 0;
        }

        Eigen::Vector2f robotPos(curr.x, curr.y);
        Eigen::Vector2f targetPos(tx, ty);
        Eigen::Vector2f activeTarget = targetPos;

        if (settings.obstacles.getAvoidanceMode() == ObstacleManager::AvoidanceMode::On) {
            activeTarget = settings.obstacles.getPotentialFieldTarget(robotPos, targetPos,
                                                             settings.poseGetter(true).theta);
        }

        float ex_g = activeTarget.x() - curr.x;
        float ey_g = activeTarget.y() - curr.y;

        float targetHeading = params.angleCorrection ? std::atan2(ex_g, ey_g) * RAD2DEG : holdHeading;
        float angleError = getAngleError(targetHeading, curr.theta);

        xPID.setGains(settings.xSched.getGains(std::abs(ex_g)));
        yPID.setGains(settings.ySched.getGains(std::abs(ey_g)));
        tPID.setGains(settings.thetaSched.getGains(angleError));

        float outX_global = (float)xPID.update(ex_g);
        float outY_global = (float)yPID.update(ey_g);
        float outT = params.angleCorrection ? (float)tPID.update(angleError) : 0.0f;

        float mag = std::hypot(outX_global, outY_global);
        if (mag > 1e-3f && mag < params.minSpeed) {
            float s = params.minSpeed / mag;
            outX_global *= s;
            outY_global *= s;
        }
        if (mag > params.maxTranslationSpeed) {
            float s = params.maxTranslationSpeed / mag;
            outX_global *= s;
            outY_global *= s;
        }

        if (distErr < 2.0f)
            outT = 0.0f;
        outT = std::clamp(outT, -params.maxRotationSpeed, params.maxRotationSpeed);

        settings.chassis.xdrive(outX_global, outY_global, outT, curr.theta);

        pros::delay(10);
    }
    settings.chassis.brake();
}
} // namespace hololib