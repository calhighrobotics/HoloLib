#include "chassis.h"
#include <algorithm>
#include <cmath>
#include <iostream>

static constexpr float DEG2RAD = M_PI / 180.0f;
static constexpr float RAD2DEG = 180.0f / M_PI;

/**
 *@brief Follows a path with PID control.
 *@param path The path to follow.
 *@param lookaheadDistance The distance to lookahead.
 *@param params The movement parameters.
 *@param headingMode The heading mode.
 *@param holdAngleDeg The angle to hold.
 *@param reversed Whether to reverse the path.
 *@return void
 */
void Chassis::followPath(const std::vector<PathPoint> &path,
                         float lookaheadDistance, MoveParams params,
                         HeadingMode headingMode, float holdAngleDeg,
                         bool reversed) {

  if (path.size() < 2) {
    std::cout << "[followPathPID] Invalid path." << std::endl;
    return;
  }

  motion.enqueue(
      [=, this]() {
        PID forwardPID(0, 0, 0, 0);
        PID strafePID(0, 0, 0, 0);
        PID thetaPID(0, 0, 0, 0);

        uint32_t start = pros::millis();
        uint32_t settleStart = 0;
        constexpr uint32_t settleTime = 120;

        const int N = static_cast<int>(path.size());
        std::vector<float> arcLen(N, 0.0f);
        for (int i = 1; i < N; ++i) {
          float dx = path[i].x - path[i - 1].x;
          float dy = path[i].y - path[i - 1].y;
          arcLen[i] = arcLen[i - 1] + std::hypot(dx, dy);
        }
        float totalPathLen = arcLen[N - 1];

        float lockedHeading = (headingMode == HeadingMode::HoldAngle)
                                  ? holdAngleDeg
                                  : getPose(false).theta;

        float pathProgress = 0.0f;
        Pose prevPose = getPose(false);

        auto samplePath = [&](float s) -> PathPoint {
          s = std::clamp(s, 0.0f, totalPathLen);
          int lo = 0, hi = N - 2;
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
          p.theta = path[lo].theta +
                    t * getAngleError(path[lo + 1].theta, path[lo].theta);
          return p;
        };

        auto pathTangentDeg = [&](float s) -> float {
          s = std::clamp(s, 0.0f, totalPathLen);
          int lo = 0, hi = N - 2;
          while (lo < hi) {
            int mid = (lo + hi + 1) / 2;
            if (arcLen[mid] <= s)
              lo = mid;
            else
              hi = mid - 1;
          }
          float dx = path[lo + 1].x - path[lo].x;
          float dy = path[lo + 1].y - path[lo].y;
          return std::atan2(dx, dy) * (180.0f / static_cast<float>(M_PI));
        };

        while (pros::millis() - start < params.timeout) {

          Pose curr = getPose(false);

          {
            float dxOdom = curr.x - prevPose.x;
            float dyOdom = curr.y - prevPose.y;
            float dist = std::hypot(dxOdom, dyOdom);

            if (dist > 1e-4f) {
              float tangentDeg = pathTangentDeg(pathProgress);
              float tangentRad =
                  tangentDeg * (static_cast<float>(M_PI) / 180.0f);
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
            float tangentRad = tangentDeg * (static_cast<float>(M_PI) / 180.0f);
            float tx = std::sin(tangentRad);
            float ty = std::cos(tangentRad);

            float sErr = exError * tx + eyError * ty;

            float correction =
                std::clamp(sErr * 0.05f, -lookaheadDistance * 0.25f,
                           lookaheadDistance * 0.25f);
            pathProgress =
                std::clamp(pathProgress + correction, 0.0f, totalPathLen);
          }

          float lookaheadS =
              std::min(pathProgress + lookaheadDistance, totalPathLen);
          PathPoint lookahead = samplePath(lookaheadS);

          Eigen::Vector2f robotPos(curr.x, curr.y);
          Eigen::Vector2f lookaheadVec(lookahead.x, lookahead.y);
          Eigen::Vector2f activeTarget = lookaheadVec;

          if (avoidanceMode == AvoidanceMode::On) {
            activeTarget = obstacles.getPotentialFieldTarget(
                robotPos, lookaheadVec, pf_ka, pf_kr, pf_influence_radius,
                getPose(true).theta);
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

          if (params.earlyExitRange > 0.0f &&
              distToEnd <= params.earlyExitRange)
            break;

          bool settledPos = distToEnd < params.exitRange;
          bool settledAngle = std::abs(angleError) < 2.0f;

          if (settledPos && settledAngle) {
            if (settleStart == 0)
              settleStart = pros::millis();
            if (pros::millis() - settleStart >= settleTime)
              break;
          } else {
            if (distToEnd > params.exitRange * 1.5f ||
                std::abs(angleError) > 4.0f)
              settleStart = 0;
          }

          forwardPID.setGains(ySched.getGains(localForward));
          strafePID.setGains(xSched.getGains(localStrafe));
          thetaPID.setGains(thetaSched.getGains(angleError));

          float forward = static_cast<float>(forwardPID.update(localForward));
          float strafe = static_cast<float>(strafePID.update(localStrafe));
          float turn = static_cast<float>(thetaPID.update(angleError));

          float translationalMag = std::hypot(forward, strafe);

          if (translationalMag > params.maxTranslationSpeed) {
            float scale = params.maxTranslationSpeed / translationalMag;
            forward *= scale;
            strafe *= scale;
          }

          if (translationalMag > 1e-3f && translationalMag < params.minSpeed &&
              distToEnd > params.exitRange) {
            float scale = params.minSpeed / translationalMag;
            forward *= scale;
            strafe *= scale;
          }

          turn = std::clamp(turn, -params.maxRotationSpeed,
                            params.maxRotationSpeed);

          float total = std::abs(forward) + std::abs(strafe) + std::abs(turn);
          if (total > params.maxTranslationSpeed) {
            float scale = params.maxTranslationSpeed / total;
            forward *= scale;
            strafe *= scale;
            turn *= scale;
          }

          setMotorVoltages(calculateHolonomic(strafe, forward, turn));
          pros::delay(10);
        }

        brake();
      },
      params.async);
}

/**
 *@brief Turns to a specific heading using PID control.
 *@param targetDeg The target heading in degrees.
 *@param params The movement parameters.
 *@return void
 */
void Chassis::turnToHeading(float targetDeg, MoveParams params) {
  motion.enqueue(
      [=, this]() {
        uint32_t start = pros::millis();
        uint32_t settleStart = 0;
        constexpr uint32_t settleTime = 100;

        PID tPID(0, 0, 0, 0);
        float prevError = 0.0f;

        while (pros::millis() - start < params.timeout) {
          float error = getAngleError(targetDeg, getPose(false).theta);

          if (params.earlyExitRange > 0.0f &&
              std::abs(error) <= params.earlyExitRange)
            return;

          if (std::abs(error) < params.exitRange) {
            if (settleStart == 0)
              settleStart = pros::millis();
            float vel = (error - prevError) / 0.01f;
            if (pros::millis() - settleStart >= settleTime &&
                std::abs(vel) < 0.5f)
              break;
          } else {
            settleStart = 0;
          }

          tPID.setGains(thetaSched.getGains(error));
          float output = (float)tPID.update(error);

          if (std::abs(output) > 1e-3f && std::abs(output) < params.minSpeed)
            output = std::copysign(params.minSpeed, output);
          output = std::clamp(output, -params.maxRotationSpeed,
                              params.maxRotationSpeed);

          setMotorVoltages(calculateHolonomic(0, 0, output));
          prevError = error;
          pros::delay(10);
        }
        brake();
      },
      params.async);
}

/**
 *@brief Turns to a specific point using PID control.
 *@param tx The x-coordinate of the target point.
 *@param ty The y-coordinate of the target point.
 *@param params The movement parameters.
 *@return void
 */
void Chassis::turnToPoint(float tx, float ty, MoveParams params) {
  motion.enqueue(
      [=, this]() {
        uint32_t start = pros::millis();
        uint32_t settleStart = 0;
        constexpr uint32_t settleTime = 100;

        PID tPID(0, 0, 0, 0);
        float prevError = 0.0f;

        while (pros::millis() - start < params.timeout) {
          Pose p = getPose(false);
          float ex = tx - p.x;
          float ey = ty - p.y;
          float targetDeg = std::atan2(ex, ey) * RAD2DEG;
          float error = getAngleError(targetDeg, p.theta);

          if (params.earlyExitRange > 0.0f &&
              std::abs(error) <= params.earlyExitRange)
            return;

          if (std::abs(error) < params.exitRange) {
            if (settleStart == 0)
              settleStart = pros::millis();
            float vel = (error - prevError) / 0.01f;
            if (pros::millis() - settleStart >= settleTime &&
                std::abs(vel) < 0.5f)
              break;
          } else {
            settleStart = 0;
          }

          tPID.setGains(thetaSched.getGains(error));
          float output = (float)tPID.update(error);

          if (std::abs(output) > 1e-3f && std::abs(output) < params.minSpeed)
            output = std::copysign(params.minSpeed, output);
          output = std::clamp(output, -params.maxRotationSpeed,
                              params.maxRotationSpeed);

          setMotorVoltages(calculateHolonomic(0, 0, output));
          prevError = error;
          pros::delay(10);
        }
        brake();
      },
      params.async);
}

/**
 *@brief Moves the robot to a specific point using PID control.
 *@param tx The x-coordinate of the target point.
 *@param ty The y-coordinate of the target point.
 *@param params The movement parameters.
 *@param angleCorrection Whether to correct the angle.
 *@return void
 */
void Chassis::moveToPoint(float tx, float ty, MoveParams params,
                          bool angleCorrection) {
  motion.enqueue(
      [=, this]() {
        uint32_t start = pros::millis();
        uint32_t settleStart = 0;
        constexpr uint32_t settleTime = 100;

        PID xPID(0, 0, 0, 0), yPID(0, 0, 0, 0), tPID(0, 0, 0, 0);
        float holdHeading = getPose(false).theta;

        while (pros::millis() - start < params.timeout) {
          Pose curr = getPose(false);

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

          if (avoidanceMode == AvoidanceMode::On) {
            activeTarget = obstacles.getPotentialFieldTarget(
                robotPos, targetPos, pf_ka, pf_kr, pf_influence_radius,
                getPose(true).theta);
          }

          float ex_g = activeTarget.x() - curr.x;
          float ey_g = activeTarget.y() - curr.y;

          float targetHeading =
              angleCorrection ? std::atan2(ex_g, ey_g) * RAD2DEG : holdHeading;
          float angleError = getAngleError(targetHeading, curr.theta);

          float rad = curr.theta * DEG2RAD;
          float cosH = std::cos(rad), sinH = std::sin(rad);

          float ex_local = ex_g * cosH - ey_g * sinH;
          float ey_local = ex_g * sinH + ey_g * cosH;

          xPID.setGains(xSched.getGains(std::abs(ex_local)));
          yPID.setGains(ySched.getGains(std::abs(ey_local)));
          tPID.setGains(thetaSched.getGains(angleError));

          float outX_local = (float)xPID.update(ex_local);
          float outY_local = (float)yPID.update(ey_local);
          float outT = angleCorrection ? (float)tPID.update(angleError) : 0.0f;

          float mag = std::hypot(outX_local, outY_local);
          if (mag > 1e-3f && mag < params.minSpeed) {
            float s = params.minSpeed / mag;
            outX_local *= s;
            outY_local *= s;
          }
          if (mag > params.maxTranslationSpeed) {
            float s = params.maxTranslationSpeed / mag;
            outX_local *= s;
            outY_local *= s;
          }
          if (distErr < 2.0f)
            outT = 0.0f;
          outT = std::clamp(outT, -params.maxRotationSpeed,
                            params.maxRotationSpeed);

          setMotorVoltages(calculateHolonomic(outX_local, outY_local, outT));
          pros::delay(10);
        }
        brake();
      },
      params.async);
}

/**
 *@brief Moves the robot relative to its current position using PID control.
 *@param forward The forward movement distance.
 *@param sideways The sideways movement distance.
 *@param params The movement parameters.
 *@param holdHeading Whether to hold the current heading.
 *@return void
 */
void Chassis::moveRelative(float forward, float sideways, MoveParams params,
                           bool holdHeading) {
  motion.enqueue(
      [=, this]() {
        Pose start = getPose(false);
        float headingRad = start.theta * DEG2RAD;

        float targetX = start.x + forward * std::sin(headingRad) +
                        sideways * std::cos(headingRad);
        float targetY = start.y + forward * std::cos(headingRad) -
                        sideways * std::sin(headingRad);

        uint32_t startTime = pros::millis();
        uint32_t settleStart = 0;
        constexpr uint32_t settleTime = 120;

        PID xPID(0, 0, 0, 0), yPID(0, 0, 0, 0), tPID(0, 0, 0, 0);

        while (pros::millis() - startTime < params.timeout) {
          Pose curr = getPose(false);

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

          if (avoidanceMode == AvoidanceMode::On) {
            activeTarget = obstacles.getPotentialFieldTarget(
                robotPos, targetPos, pf_ka, pf_kr, pf_influence_radius,
                getPose(true).theta);
          }

          float ex = activeTarget.x() - curr.x;
          float ey = activeTarget.y() - curr.y;

          xPID.setGains(xSched.getGains(ex));
          yPID.setGains(ySched.getGains(ey));
          tPID.setGains(thetaSched.getGains(angleError));

          float outX_g = (float)xPID.update(ex);
          float outY_g = (float)yPID.update(ey);
          float outT = holdHeading ? (float)tPID.update(angleError) : 0.0f;

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
          outT = std::clamp(outT, -params.maxRotationSpeed,
                            params.maxRotationSpeed);

          float rad = curr.theta * DEG2RAD;
          float cosH = std::cos(rad), sinH = std::sin(rad);

          float outX_local = outX_g * cosH - outY_g * sinH;
          float outY_local = outX_g * sinH + outY_g * cosH;

          setMotorVoltages(calculateHolonomic(outX_local, outY_local, outT));
          pros::delay(10);
        }
        brake();
      },
      params.async);
}

/**
 *@brief Moves the robot a specific distance using PID control.
 *@param distance The distance to move.
 *@param params The movement parameters.
 *@param holdHeading Whether to hold the current heading.
 *@return void
 */
void Chassis::moveDistance(float distance, MoveParams params,
                           bool holdHeading) {
  moveRelative(distance, 0.0f, params, holdHeading);
}

/**
 *@brief Strafes the robot a specific distance using PID control.
 *@param distance The distance to strafe.
 *@param params The movement parameters.
 *@param holdHeading Whether to hold the current heading.
 *@return void
 */
void Chassis::strafeDistance(float distance, MoveParams params,
                             bool holdHeading) {
  moveRelative(0.0f, distance, params, holdHeading);
}

/**
 *@brief Moves the robot to a specific pose using PID control.
 *@param tx The x-coordinate of the target pose.
 *@param ty The y-coordinate of the target pose.
 *@param targetThetaDeg The heading of the target pose in degrees.
 *@param params The movement parameters.
 *@return void
 */
void Chassis::moveToPose(float tx, float ty, float targetThetaDeg,
                         MoveParams params) {
  motion.enqueue(
      [=, this]() {
        uint32_t start = pros::millis();
        uint32_t settleStart = 0;
        constexpr uint32_t settleTime = 120;
        constexpr float angleExitDeg = 2.0f;

        PID xPID(0, 0, 0, 0), yPID(0, 0, 0, 0), tPID(0, 0, 0, 0);

        while (pros::millis() - start < params.timeout) {
          Pose curr = getPose(false);
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

          xPID.setGains(xSched.getGains(ex));
          yPID.setGains(ySched.getGains(ey));
          tPID.setGains(thetaSched.getGains(angleError));

          float outX_g = (float)xPID.update(ex);
          float outY_g = (float)yPID.update(ey);
          float outT = (float)tPID.update(angleError);

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
          outT = std::clamp(outT, -params.maxRotationSpeed,
                            params.maxRotationSpeed);

          float rad = curr.theta * DEG2RAD;
          float cosH = std::cos(rad), sinH = std::sin(rad);

          float outX_local = outX_g * cosH - outY_g * sinH;
          float outY_local = outX_g * sinH + outY_g * cosH;

          setMotorVoltages(calculateHolonomic(outX_local, outY_local, outT));
          pros::delay(10);
        }
        brake();
      },
      params.async);
}

enum class SwingSide { Left, Right };

/**
 *@brief Strafes the robot a specific distance using PID control.
 *@param distance The distance to strafe.
 *@param params The movement parameters.
 *@param holdHeading Whether to hold the current heading.
 *@return void
 */
void Chassis::swingTurn(float targetThetaDeg, SwingSide lockedSide,
                        MoveParams params) {
  motion.enqueue(
      [=, this]() {
        uint32_t start = pros::millis();
        uint32_t settleStart = 0;
        constexpr uint32_t settleTime = 120;
        constexpr float angleExitDeg = 2.0f;
        PID tPID(0, 0, 0, 0);
        float maxRotation = std::min(params.maxRotationSpeed, 127.0f);

        while (pros::millis() - start < params.timeout) {
          Pose curr = getPose(false);
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

          tPID.setGains(thetaSched.getGains(angleError));
          float outT = (float)tPID.update(angleError);
          if (!angleSettled && std::abs(outT) < params.minSpeed) {
            outT = std::copysign(params.minSpeed, outT);
          }
          outT = std::clamp(outT, -maxRotation, maxRotation);

          float outX_local = 0.0f;
          float outY_local = 0.0f;
          if (lockedSide == SwingSide::Left) {
            outY_local = -outT;
          } else {
            outY_local = outT;
          }
          setMotorVoltages(calculateHolonomic(outX_local, outY_local, outT));
          pros::delay(10);
        }
        brake();
      },
      params.async);
}
