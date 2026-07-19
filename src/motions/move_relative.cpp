#include "hololib/motions/motion_cancel_helper.hpp"
#include "hololib/motions/motions.hpp"
#include "hololib/util/PID.hpp"
#include "hololib/util/Timer.hpp"
#include "hololib/util/util.hpp"
#include <algorithm>
#include <cmath>

namespace hololib {
void moveRelative(float forward, float sideways, bool holdHeading, MoveParams params, MoveSettings settings) {
    Pose start = settings.poseGetter(false);
    float headingRad = start.theta * DEG2RAD;

    float targetX = start.x + forward * std::sin(headingRad) + sideways * std::cos(headingRad);
    float targetY = start.y + forward * std::cos(headingRad) - sideways * std::sin(headingRad);

    uint32_t settleStart = 0;
    constexpr uint32_t settleTime = 120;

    PID xPID(0, 0, 0, 0), yPID(0, 0, 0, 0), tPID(0, 0, 0, 0);
    Timer timeoutTimer(params.timeout);
    MotionCancelHelper helper(10);

    while (helper.wait() && !timeoutTimer.isDone()) {
        Pose curr = settings.poseGetter(false);

        float distErr = std::hypot(targetX - curr.x, targetY - curr.y);
        float angleError = getAngleError(start.theta, curr.theta);

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
        Eigen::Vector2f targetPos(targetX, targetY);
        Eigen::Vector2f activeTarget = targetPos;

        if (settings.obstacles.getAvoidanceMode() == ObstacleManager::AvoidanceMode::On) {
            activeTarget = settings.obstacles.getPotentialFieldTarget(robotPos, targetPos, settings.poseGetter(true).theta);
        }

        float ex = activeTarget.x() - curr.x;
        float ey = activeTarget.y() - curr.y;

        xPID.setGains(settings.xSched.getGains(ex));
        yPID.setGains(settings.ySched.getGains(ey));
        tPID.setGains(settings.thetaSched.getGains(angleError));

        float outX_g = static_cast<float>(xPID.update(ex));
        float outY_g = static_cast<float>(yPID.update(ey));
        float outT = holdHeading ? static_cast<float>(tPID.update(angleError)) : 0.0f;

        float mag = std::hypot(outX_g, outY_g);
        if (mag > 1e-3f && mag < params.minSpeed) {
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

void moveDistance(float distance, bool holdHeading, MoveParams params, MoveSettings settings) {
    moveRelative(distance, 0.0f, holdHeading, params, settings);
}

void strafeDistance(float distance, bool holdHeading, MoveParams params, MoveSettings settings) {
    moveRelative(0.0f, distance, holdHeading, params, settings);
}
} // namespace hololib
