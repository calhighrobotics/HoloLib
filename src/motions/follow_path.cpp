#include "hololib/motions/motion_cancel_helper.hpp"
#include "hololib/motions/motions.hpp"
#include "hololib/util/PID.hpp"
#include "hololib/util/Timer.hpp"
#include "hololib/util/util.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace hololib {
void followPath(const std::vector<PathPoint>& path, float lookaheadDistance, HeadingMode headingMode,
                float holdAngleDeg, bool reversed, MoveParams params, MoveSettings settings) {
    if (path.size() < 2) {
        return;
    }

    PID forwardPID(0, 0, 0, 0);
    PID strafePID(0, 0, 0, 0);
    PID thetaPID(0, 0, 0, 0);

    uint32_t settleStart = 0;
    constexpr uint32_t settleTime = 120;

    const int n = static_cast<int>(path.size());
    std::vector<float> arcLen(n, 0.0f);
    for (int i = 1; i < n; ++i) {
        float dx = path[i].x - path[i - 1].x;
        float dy = path[i].y - path[i - 1].y;
        arcLen[i] = arcLen[i - 1] + std::hypot(dx, dy);
    }
    float totalPathLen = arcLen[n - 1];

    float lockedHeading = (headingMode == HeadingMode::HoldAngle) ? holdAngleDeg : settings.poseGetter(false).theta;

    float pathProgress = 0.0f;
    Pose prevPose = settings.poseGetter(false);

    auto samplePath = [&](float s) -> PathPoint {
        s = std::clamp(s, 0.0f, totalPathLen);
        int lo = 0;
        int hi = n - 2;
        while (lo < hi) {
            int mid = (lo + hi + 1) / 2;
            if (arcLen[mid] <= s)
                lo = mid;
            else
                hi = mid - 1;
        }

        float segLen = arcLen[lo + 1] - arcLen[lo];
        float t = (segLen > 1e-6f) ? (s - arcLen[lo]) / segLen : 0.0f;

        PathPoint p;
        p.x = path[lo].x + t * (path[lo + 1].x - path[lo].x);
        p.y = path[lo].y + t * (path[lo + 1].y - path[lo].y);
        p.theta = path[lo].theta + t * getAngleError(path[lo + 1].theta, path[lo].theta);
        return p;
    };

    auto pathTangentDeg = [&](float s) -> float {
        s = std::clamp(s, 0.0f, totalPathLen);
        int lo = 0;
        int hi = n - 2;
        while (lo < hi) {
            int mid = (lo + hi + 1) / 2;
            if (arcLen[mid] <= s)
                lo = mid;
            else
                hi = mid - 1;
        }

        float dx = path[lo + 1].x - path[lo].x;
        float dy = path[lo + 1].y - path[lo].y;
        return std::atan2(dx, dy) * RAD2DEG;
    };

    Timer timeoutTimer(params.timeout);
    MotionCancelHelper helper(10);

    while (helper.wait() && !timeoutTimer.isDone()) {
        Pose curr = settings.poseGetter(false);

        {
            float dxOdom = curr.x - prevPose.x;
            float dyOdom = curr.y - prevPose.y;
            float dist = std::hypot(dxOdom, dyOdom);

            if (dist > 1e-4f) {
                float tangentDeg = pathTangentDeg(pathProgress);
                float tangentRad = tangentDeg * DEG2RAD;
                float tx = std::sin(tangentRad);
                float ty = std::cos(tangentRad);

                float along = dxOdom * tx + dyOdom * ty;
                along = std::max(along, -0.05f * dist);

                pathProgress += along;
                pathProgress = std::clamp(pathProgress, 0.0f, totalPathLen);
            }

            prevPose = curr;
        }

        {
            PathPoint onPath = samplePath(pathProgress);
            float exError = curr.x - onPath.x;
            float eyError = curr.y - onPath.y;

            float tangentDeg = pathTangentDeg(pathProgress);
            float tangentRad = tangentDeg * DEG2RAD;
            float tx = std::sin(tangentRad);
            float ty = std::cos(tangentRad);

            float sErr = exError * tx + eyError * ty;

            float correction = std::clamp(sErr * 0.05f, -lookaheadDistance * 0.25f, lookaheadDistance * 0.25f);
            pathProgress = std::clamp(pathProgress + correction, 0.0f, totalPathLen);
        }

        float lookaheadS = std::min(pathProgress + lookaheadDistance, totalPathLen);
        PathPoint lookahead = samplePath(lookaheadS);

        Eigen::Vector2f robotPos(curr.x, curr.y);
        Eigen::Vector2f lookaheadVec(lookahead.x, lookahead.y);
        Eigen::Vector2f activeTarget = lookaheadVec;

        if (settings.obstacles.getAvoidanceMode() == ObstacleManager::AvoidanceMode::On) {
            activeTarget = settings.obstacles.getPotentialFieldTarget(robotPos, lookaheadVec, settings.poseGetter(true).theta);
        }

        float headingRad = curr.theta * DEG2RAD;
        float forwardX = std::sin(headingRad);
        float forwardY = std::cos(headingRad);
        float strafeX = std::cos(headingRad);
        float strafeY = -std::sin(headingRad);

        float globalDX = activeTarget.x() - curr.x;
        float globalDY = activeTarget.y() - curr.y;

        float localForward = globalDX * forwardX + globalDY * forwardY;
        float localStrafe = globalDX * strafeX + globalDY * strafeY;

        if (reversed) {
            localForward = -localForward;
            localStrafe = -localStrafe;
        }

        float distToEnd = totalPathLen - pathProgress;

        float targetHeading;
        switch (headingMode) {
        case HeadingMode::FollowPath: {
            float tangentS = std::min(pathProgress + 2.0f, totalPathLen);
            targetHeading = pathTangentDeg(tangentS);
            if (reversed)
                targetHeading += 180.0f;
            break;
        }
        case HeadingMode::HoldAngle:
            targetHeading = lockedHeading;
            break;
        case HeadingMode::CustomAngles:
            targetHeading = lookahead.theta;
            if (reversed)
                targetHeading += 180.0f;
            break;
        default:
            targetHeading = curr.theta;
            break;
        }

        float angleError = getAngleError(targetHeading, curr.theta);

        if (params.earlyExitRange > 0.0f && distToEnd <= params.earlyExitRange)
            break;

        bool settledPos = distToEnd < params.exitRange;
        bool settledAngle = std::abs(angleError) < 2.0f;

        if (settledPos && settledAngle) {
            if (settleStart == 0)
                settleStart = pros::millis();
            if (pros::millis() - settleStart >= settleTime)
                break;
        } else {
            if (distToEnd > params.exitRange * 1.5f || std::abs(angleError) > 4.0f)
                settleStart = 0;
        }

        forwardPID.setGains(settings.ySched.getGains(localForward));
        strafePID.setGains(settings.xSched.getGains(localStrafe));
        thetaPID.setGains(settings.thetaSched.getGains(angleError));

        float forward = static_cast<float>(forwardPID.update(localForward));
        float strafe = static_cast<float>(strafePID.update(localStrafe));
        float turn = static_cast<float>(thetaPID.update(angleError));

        float translationalMag = std::hypot(forward, strafe);

        if (translationalMag > params.maxTranslationSpeed) {
            float scale = params.maxTranslationSpeed / translationalMag;
            forward *= scale;
            strafe *= scale;
        }

        if (translationalMag > 1e-3f && translationalMag < params.minSpeed && distToEnd > params.exitRange) {
            float scale = params.minSpeed / translationalMag;
            forward *= scale;
            strafe *= scale;
        }

        turn = std::clamp(turn, -params.maxRotationSpeed, params.maxRotationSpeed);

        float total = std::abs(forward) + std::abs(strafe) + std::abs(turn);
        if (total > params.maxTranslationSpeed) {
            float scale = params.maxTranslationSpeed / total;
            forward *= scale;
            strafe *= scale;
            turn *= scale;
        }

        float outX_global = forward * forwardX + strafe * strafeX;
        float outY_global = forward * forwardY + strafe * strafeY;

        settings.chassis.xdrive(outX_global, outY_global, turn, curr.theta);
        pros::delay(10);
    }

    settings.chassis.brake();
}
} // namespace hololib
