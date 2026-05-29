#include "chassis.h"
#include "pros/misc.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

static constexpr float METER_TO_INCH = 39.3700787f;
static constexpr float DEG2RAD = M_PI / 180.0f;
static constexpr float RAD2DEG = 180.0f / M_PI;

void Chassis::calibrate()
{
  backLeft.tare_position();
  backRight.tare_position();
  frontLeft.tare_position();
  frontRight.tare_position();
  motionDistTraveled = 0.0f;
  prev_fl = 0, prev_fr = 0, prev_bl = 0, prev_br = 0;
  prev_heading = 0;
  targetHeadingDriveControl = 0;
  imu.reset(true);
  pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}

std::vector<PathPoint> parsePathData(const std::string &input_source,
                                     bool convertFromMeters) {
  std::vector<PathPoint> path;
  const float scale = convertFromMeters ? METER_TO_INCH : 1.0f;

  bool is_file = (input_source.find('\n') == std::string::npos) &&
                 (input_source.find('/') != std::string::npos ||
                  input_source.find('.') != std::string::npos);

  auto processLine = [&](const std::string &raw) {
    std::string line = raw;
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n' ||
                             line.back() == ' ' || line.back() == '\t'))
      line.pop_back();

    size_t start = line.find_first_not_of(" \t");
    if (start == std::string::npos)
      return;
    line = line.substr(start);
    if (line.empty())
      return;

    std::stringstream ss(line);
    std::string cx, cy, ct;
    if (std::getline(ss, cx, ',') && std::getline(ss, cy, ',') &&
        std::getline(ss, ct, ',')) {
      try {
        PathPoint pt;
        pt.x = std::stof(cx) * scale;
        pt.y = std::stof(cy) * scale;
        pt.theta = std::stof(ct);
        path.push_back(pt);
      } catch (...) {
      }
    }
  };

  if (is_file) {
    std::ifstream file(input_source);
    if (!file.is_open()) {
      std::cout << "[PARSER ERROR] Cannot open: " << input_source << std::endl;
      return path;
    }
    std::string line;
    while (std::getline(file, line))
      processLine(line);
  } else {
    std::stringstream ss(input_source);
    std::string line;
    while (std::getline(ss, line))
      processLine(line);
  }

  std::cout << "[PARSER] Loaded " << path.size() << " points." << std::endl;
  return path;
}

void GainScheduler::addStep(float threshold, float kP, float kI, float kD, float slew) {
  schedules.push_back({threshold, {kP, kI, kD, 0.0f, slew}});
  std::sort(schedules.begin(), schedules.end(),
            [](const ScheduledGain &a, const ScheduledGain &b) {
              return a.threshold < b.threshold;
            });
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
    const auto &lo = schedules[i];
    const auto &hi = schedules[i + 1];
    if (absErr >= lo.threshold && absErr <= hi.threshold) {
      float t = (absErr - lo.threshold) / (hi.threshold - lo.threshold);
      return {lo.gains.kP + t * (hi.gains.kP - lo.gains.kP),
              lo.gains.kI + t * (hi.gains.kI - lo.gains.kI),
              lo.gains.kD + t * (hi.gains.kD - lo.gains.kD), 
              0.0f,
              lo.gains.slew + t * (hi.gains.slew - lo.gains.slew)};
    }
  }

  return schedules.back().gains;
}

void GainScheduler::clear() { schedules.clear(); }

EncoderKalmanFilter::EncoderKalmanFilter(float process_noise,
                                         float measurement_noise) {
  x.setZero();
  P.setIdentity();
  Q << process_noise, 0, 0, process_noise;
  R = measurement_noise;
  H << 1, 0;
}

float EncoderKalmanFilter::update(float measured_position, float dt) {
  Eigen::Matrix2f F;
  F << 1, dt, 0, 1;
  x = F * x;
  P = F * P * F.transpose() + Q;

  float y = measured_position - H * x;
  float S = (H * P * H.transpose())(0, 0) + R;
  Eigen::Vector2f K = P * H.transpose() / S;

  x = x + K * y;
  P = (Eigen::Matrix2f::Identity() - K * H) * P;
  return x(0);
}

void odomTaskTrampoline(void *param) {
  static_cast<Chassis *>(param)->odometryTask();
}

Chassis::Chassis(pros::Motor fl, pros::Motor fr, pros::Motor bl, pros::Motor br,
                 pros::Imu imu_sensor, ChassisConfig cfg)
    : frontLeft(fl), frontRight(fr), backLeft(bl), backRight(br),
      imu(imu_sensor), config(cfg) {
  if (config.drivetrainWidth <= 0.0f)
    config.drivetrainWidth = config.trackWidth;
  if (config.drivetrainLength <= 0.0f)
    config.drivetrainLength = config.drivetrainWidth;

  auto safePos = [](pros::Motor &m) {
    float v = m.get_position();
    return (std::isinf(v) || std::isnan(v)) ? 0.0f : v;
  };
  prev_fl = safePos(frontLeft);
  prev_fr = safePos(frontRight);
  prev_bl = safePos(backLeft);
  prev_br = safePos(backRight);

  float h = imu.get_rotation();
  prev_heading = (std::isinf(h) || std::isnan(h)) ? 0.0f : h * DEG2RAD;

  motionDistTraveled = 0.0f;
  motion.setOnMotionStart([this]() {
    poseMutex.take();
    motionDistTraveled = 0.0f;
    poseMutex.give();
  });

  pros::Task odom_task(odomTaskTrampoline, this, "Odometry Task");
}

void Chassis::odometryTask() {
  uint32_t now = pros::millis();
  const float d_per_deg = (M_PI * config.wheelDiameter * config.gearRatio) / 360.0f;
  constexpr float wheel_angle = 45.0f * DEG2RAD;
  const float x_component = std::sin(wheel_angle);
  const float y_component = std::cos(wheel_angle);
  const float x_scale = 1.0f / (4.0f * x_component);
  const float y_scale = 1.0f / (4.0f * y_component);
  Eigen::Matrix<float, 2, 4> kinematics;
  kinematics << x_scale, -x_scale, -x_scale, x_scale,
                y_scale,  y_scale,  y_scale, y_scale;

  auto safeEnc = [](pros::Motor &m, float prev) {
    float v = m.get_position();
    return (std::isinf(v) || std::isnan(v)) ? prev : v;
  };

  while (true) {
    float raw_fl = safeEnc(frontLeft, prev_fl);
    float raw_fr = safeEnc(frontRight, prev_fr);
    float raw_bl = safeEnc(backLeft, prev_bl);
    float raw_br = safeEnc(backRight, prev_br);

    float raw_h = imu.get_rotation();
    float current_heading_meas = (std::isinf(raw_h) || std::isnan(raw_h))
                                     ? prev_heading
                                     : raw_h * DEG2RAD;

    Eigen::Vector4f wheel_deltas(
        (raw_fl - prev_fl) * d_per_deg, 
        (raw_fr - prev_fr) * d_per_deg,
        (raw_bl - prev_bl) * d_per_deg, 
        (raw_br - prev_br) * d_per_deg
    );
    float d_theta_meas = std::remainder(current_heading_meas - prev_heading, 2.0f * M_PI);
    if (std::isnan(d_theta_meas)) {
        d_theta_meas = 0.0f;
    }
    Eigen::Vector2f local_delta = kinematics * wheel_deltas;
    poseMutex.take();
    ekf.predict(local_delta.x(), local_delta.y(), d_theta_meas);
    ekf.updateIMU(current_heading_meas);
    
    float step_dist = std::hypot(local_delta.x(), local_delta.y());
    motionDistTraveled += step_dist;

    currentPose.x = ekf.getX();
    currentPose.y = ekf.getY();
    currentPose.theta = ekf.getTheta();
    if(velocityCalculationsOn) {
      currentPose.velocity.vx = local_delta.x() / 0.01;
      currentPose.velocity.vy = local_delta.y() / 0.01;
      currentPose.velocity.w = d_theta_meas / 0.01;
    }
    poseMutex.give();
    prev_fl = raw_fl;
    prev_fr = raw_fr;
    prev_bl = raw_bl;
    prev_br = raw_br;
    prev_heading = current_heading_meas;

    pros::Task::delay_until(&now, 10);
  }
}

XDriveVoltages Chassis::calculateHolonomic(float vx, float vy, float vt) {

  constexpr float scale = 12000.0f / 127.0f;

  XDriveVoltages v;
  v.fl = (vy + vx + vt) * scale;
  v.fr = (vy - vx - vt) * scale;
  v.bl = (vy - vx + vt) * scale;
  v.br = (vy + vx - vt) * scale;

  float maxV = std::max({std::abs(v.fl), std::abs(v.fr), std::abs(v.bl),
                         std::abs(v.br), 12000.0f});
  if (maxV > 12000.0f) {
    float r = 12000.0f / maxV;
    v.fl *= r;
    v.fr *= r;
    v.bl *= r;
    v.br *= r;
  }
  return v;
}

void Chassis::setMotorVoltages(XDriveVoltages v) {
  frontLeft.move_voltage((int32_t)v.fl);
  frontRight.move_voltage((int32_t)v.fr);
  backLeft.move_voltage((int32_t)v.bl);
  backRight.move_voltage((int32_t)v.br);
}

void Chassis::brake() {
  frontLeft.brake();
  frontRight.brake();
  backLeft.brake();
  backRight.brake();
}

void Chassis::setPose(float x, float y, float theta) {
  poseMutex.take();
  float theta_rad = theta * DEG2RAD;
  imu.set_rotation(theta);

  currentPose = {x, y, theta_rad};
  prev_heading = theta_rad;

  ekf.setPose(x, y, theta_rad);

  poseMutex.give();
}

void Chassis::setPose(Pose pose) {
  setPose(pose.x, pose.y, pose.theta);
}

Pose Chassis::getPose(bool radians) {
  poseMutex.take();
  Pose p = currentPose;
  poseMutex.give();
  if (!radians)
    p.theta *= RAD2DEG;
  return p;
}

void Chassis::setXGains(std::vector<ScheduledGain> steps) {
  xSched.clear();
  for (auto &s : steps)
    xSched.addStep(s.threshold, s.gains.kP, s.gains.kI, s.gains.kD, s.gains.slew);
}

void Chassis::setYGains(std::vector<ScheduledGain> steps) {
  ySched.clear();
  for (auto &s : steps)
    ySched.addStep(s.threshold, s.gains.kP, s.gains.kI, s.gains.kD, s.gains.slew);
}

void Chassis::setThetaGains(std::vector<ScheduledGain> steps) {
  thetaSched.clear();
  for (auto &s : steps)
    thetaSched.addStep(s.threshold, s.gains.kP, s.gains.kI, s.gains.kD, s.gains.slew);
}

void Chassis::driveControl(float forward, float sideways, float rotation,
                           DriveCurves drivecurves,
                           bool fieldCentric,
                           float headingOffset,
                           DriveCorrection correction)
{
    static bool headingInitialized = false;
    static float targetHeading = 0.0f;
    static PID headingPID(correction.kP, 0.0f, 0.0f, 0.0f);
    static uint32_t lastRotationTime = 0;
    constexpr float MAX_DRIVE_INPUT = 127.0f;
    if (!headingInitialized) {
        targetHeading = getPose(false).theta;
        headingInitialized = true;
    }
    auto applyCurve = [&](float x, const DriveCurve& c) -> float {
        if (std::abs(x) < c.deadzone)
            return 0.0f;

        float sign = (x >= 0.0f) ? 1.0f : -1.0f;
        float normalized =
            (std::abs(x) - c.deadzone) /
            (MAX_DRIVE_INPUT - c.deadzone);

        normalized = std::clamp(normalized, 0.0f, 1.0f);
        normalized =
            std::pow(normalized, c.curve_multipler);

        float output = normalized * MAX_DRIVE_INPUT;
        if (output > 0.0f &&
            output < c.minimum_output)
        {
            output = c.minimum_output;
        }

        return output * sign;
    };
    forward  = applyCurve(forward,  drivecurves.movement);
    sideways = applyCurve(sideways, drivecurves.movement);

    float robotForward  = forward;
    float robotSideways = sideways;
    if (fieldCentric) {

        float adjustedTheta =
            getPose(false).theta - headingOffset;

        float thetaRad = adjustedTheta * DEG2RAD;

        robotSideways =
            sideways * std::cos(thetaRad) -
            forward  * std::sin(thetaRad);

        robotForward =
            sideways * std::sin(thetaRad) +
            forward  * std::cos(thetaRad);
        float inputMagnitude =
            std::sqrt(forward * forward +
                      sideways * sideways);

        float rotatedMagnitude =
            std::sqrt(robotForward * robotForward +
                      robotSideways * robotSideways);

        if (rotatedMagnitude > 0.001f) {

            float scale =
                inputMagnitude / rotatedMagnitude;

            robotForward  *= scale;
            robotSideways *= scale;
        }
    }
    if (std::abs(rotation) >= drivecurves.rotation.deadzone) {

        rotation =
            applyCurve(rotation,
                       drivecurves.rotation);

        targetHeading = getPose(false).theta;

        lastRotationTime = pros::millis();

    } else {
        if (pros::millis() - lastRotationTime < 500) {

            rotation = 0.0f;
            targetHeading = getPose(false).theta;

        } else {

            if (correction.correctionOn) {

                float currentHeading =
                    getPose(false).theta;

                float angleError =
                    getAngleError(targetHeading,
                                  currentHeading);

                headingPID.setGains(
                    thetaSched.getGains(angleError));

                rotation =
                    (float)headingPID.update(angleError);

                rotation =
                    std::clamp(rotation,
                               -30.0f,
                               30.0f);
            }
        }
    }
    setMotorVoltages(
        calculateHolonomic(
            robotSideways,
            robotForward,
            rotation
        )
    );
}

enum class HeadingMode { 
  FollowPath, 
  HoldAngle, 
  CustomAngles 
};

void Chassis::followPathPID(const std::vector<PathPoint> &path,
                            float lookaheadDistance,
                            MoveParams params,
                            HeadingMode headingMode,
                            float holdAngleDeg,
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

        int closestSegment = 0;
        float closestT = 0.0f;

        float lockedHeading =
            (headingMode == HeadingMode::HoldAngle)
                ? holdAngleDeg
                : getPose(false).theta;
        std::cout << "Locked Heading: " << lockedHeading << std::endl;

        while (pros::millis() - start < params.timeout) {

          Pose curr = getPose(false);

          float headingRad = curr.theta * DEG2RAD;

          float forwardX = std::sin(headingRad);
          float forwardY = std::cos(headingRad);

          float strafeX = std::cos(headingRad);
          float strafeY = -std::sin(headingRad);

          float bestDist = std::numeric_limits<float>::max();

          PathPoint closestPoint = path.front();

          for (int i = closestSegment;
               i < (int)path.size() - 1;
               ++i) {

            const PathPoint &a = path[i];
            const PathPoint &b = path[i + 1];

            float abx = b.x - a.x;
            float aby = b.y - a.y;

            float apx = curr.x - a.x;
            float apy = curr.y - a.y;

            float ab2 = abx * abx + aby * aby;

            if (ab2 < 1e-6f)
              continue;

            float t = (apx * abx + apy * aby) / ab2;
            t = std::clamp(t, 0.0f, 1.0f);

            float projX = a.x + abx * t;
            float projY = a.y + aby * t;

            float dist =
                std::hypot(projX - curr.x,
                           projY - curr.y);
            if (dist < bestDist) {
              bestDist = dist;
              closestPoint.x = projX;
              closestPoint.y = projY;
              float segmentAngleDiff = getAngleError(b.theta, a.theta);
              closestPoint.theta = a.theta + segmentAngleDiff * t;

              closestSegment = i;
              closestT = t;
            }
          }

          PathPoint lookahead = closestPoint;
          float remaining = lookaheadDistance;
          int seg = closestSegment;

          while (seg < (int)path.size() - 1) {

            const PathPoint &a = path[seg];
            const PathPoint &b = path[seg + 1];

            float startX;
            float startY;

            if (seg == closestSegment) {
              startX = closestPoint.x;
              startY = closestPoint.y;
            } else {
              startX = a.x;
              startY = a.y;
            }

            float dx = b.x - startX;
            float dy = b.y - startY;

            float segLen = std::hypot(dx, dy);

            if (segLen >= remaining) {

              float ratio = remaining / segLen;

              lookahead.x = startX + dx * ratio;
              lookahead.y = startY + dy * ratio;
              float angleDiff = getAngleError(b.theta, a.theta);
              lookahead.theta = a.theta + angleDiff * ratio;

              break;
            }

            remaining -= segLen;
            seg++;
          }

          if (seg >= (int)path.size() - 1) {
            lookahead = path.back();
          }

          float distToEnd =
              std::hypot(path.back().x - curr.x,
                         path.back().y - curr.y);

          float globalDX = lookahead.x - curr.x;
          float globalDY = lookahead.y - curr.y;

          float localForward =
              globalDX * forwardX +
              globalDY * forwardY;

          float localStrafe =
              globalDX * strafeX +
              globalDY * strafeY;

          if (reversed) {
            localForward *= -1.0f;
            localStrafe *= -1.0f;
          }

          float targetHeading;

          switch (headingMode) {

          case HeadingMode::FollowPath:

            if (distToEnd > 4.0f) {

              targetHeading =
                  std::atan2(globalDX, globalDY) *
                  (180.0f / M_PI);

              if (reversed)
                targetHeading += 180.0f;

            } else {

              float finalDX =
                  path.back().x -
                  path[path.size() - 2].x;

              float finalDY =
                  path.back().y -
                  path[path.size() - 2].y;

              targetHeading =
                  std::atan2(finalDX, finalDY) *
                  (180.0f / M_PI);

              if (reversed)
                targetHeading += 180.0f;
            }

            break;

          case HeadingMode::HoldAngle:
            targetHeading = lockedHeading;
            break;

          case HeadingMode::CustomAngles:
            targetHeading = lookahead.theta;
            if (reversed) {
              targetHeading += 180.0f;
            }
            break;

          default:
            targetHeading = curr.theta;
            break;
          }

          float angleError =
              getAngleError(targetHeading,
                            curr.theta);

          if (params.earlyExitRange > 0.0f &&
              distToEnd <= params.earlyExitRange) {
            break;
          }

          bool settledPos =
              distToEnd < params.exitRange;

          bool settledAngle =
              std::abs(angleError) < 2.0f;

          if (settledPos && settledAngle) {

            if (settleStart == 0)
              settleStart = pros::millis();

            if (pros::millis() - settleStart >= settleTime)
              break;

          } else {
            settleStart = 0;
          }

          forwardPID.setGains(
              ySched.getGains(localForward));

          strafePID.setGains(
              xSched.getGains(localStrafe));

          thetaPID.setGains(
              thetaSched.getGains(angleError));

          float forward =
              (float)forwardPID.update(localForward);

          float strafe =
              (float)strafePID.update(localStrafe);

          float turn =
              (float)thetaPID.update(angleError);

          float translationalMag =
              std::hypot(forward, strafe);

          if (translationalMag >
              params.maxTranslationSpeed) {

            float scale =
                params.maxTranslationSpeed /
                translationalMag;

            forward *= scale;
            strafe *= scale;
          }

          if (translationalMag > 1e-3f &&
              translationalMag < params.minSpeed &&
              distToEnd > params.exitRange) {

            float scale =
                params.minSpeed /
                translationalMag;

            forward *= scale;
            strafe *= scale;
          }

          turn = std::clamp(
              turn,
              -params.maxRotationSpeed,
              params.maxRotationSpeed);

          float total =
              std::abs(forward) +
              std::abs(strafe) +
              std::abs(turn);

          float maxTotal =
              params.maxTranslationSpeed;

          if (total > maxTotal) {

            float scale = maxTotal / total;

            forward *= scale;
            strafe *= scale;
            turn *= scale;
          }

          setMotorVoltages(
              calculateHolonomic(
                  strafe,
                  forward,
                  turn));

          pros::delay(10);
        }

        brake();
      },
      params.async);
}

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

void Chassis::turnToPoint(float tx, float ty, MoveParams params) {
  Pose p = getPose(false);
  float targetDeg = std::atan2(ty - p.y, tx - p.x) * RAD2DEG;
  turnToHeading(targetDeg, params);
}

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
          float ex = tx - curr.x;
          float ey = ty - curr.y;
          float distErr = std::hypot(ex, ey);

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

          float targetHeading =
              angleCorrection ? std::atan2(ex, ey) * RAD2DEG : holdHeading;
          float angleError = getAngleError(targetHeading, curr.theta);

          xPID.setGains(xSched.getGains(ex));
          yPID.setGains(ySched.getGains(ey));
          tPID.setGains(thetaSched.getGains(angleError));

          float outX_g = (float)xPID.update(ex);
          float outY_g = (float)yPID.update(ey);
          float outT = angleCorrection ? (float)tPID.update(angleError) : 0.0f;

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
          if (distErr < 2.0f)
            outT = 0.0f;
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
        constexpr float angleExitDeg = 2.0f;

        PID xPID(0, 0, 0, 0), yPID(0, 0, 0, 0), tPID(0, 0, 0, 0);

        while (pros::millis() - startTime < params.timeout) {
          Pose curr = getPose(false);
          float ex = targetX - curr.x;
          float ey = targetY - curr.y;
          float distErr = std::hypot(ex, ey);
          float angleError = getAngleError(start.theta, curr.theta);

          if (params.earlyExitRange > 0.0f &&
              distErr <= params.earlyExitRange)
            return;

          bool posSettled = distErr < params.exitRange;
          bool angleSettled =
              !holdHeading || std::abs(angleError) < angleExitDeg;
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
          float outT = holdHeading ? (float)tPID.update(angleError) : 0.0f;

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

void Chassis::moveDistance(float distance, MoveParams params,
                           bool holdHeading) {
  moveRelative(distance, 0.0f, params, holdHeading);
}

void Chassis::strafeDistance(float distance, MoveParams params,
                             bool holdHeading) {
  moveRelative(0.0f, distance, params, holdHeading);
}

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
void Chassis::curveCircle(float targetThetaDeg, float radius, MoveParams params,
                          CurveDirection direction) {
  if (std::abs(radius) < 1e-3f) {
    turnToHeading(targetThetaDeg, params);
    return;
  }

  motion.enqueue(
      [=, this]() {
        uint32_t start = pros::millis();
        uint32_t settleStart = 0;
        constexpr uint32_t settleTime = 120;
        constexpr float angleExitDeg = 2.0f;

        PID xPID(0, 0, 0, 0);
        PID yPID(0, 0, 0, 0);
        PID tPID(0, 0, 0, 0);

        Pose sp = getPose(false);
        
        auto directedAngleError = [](float target, float current,
                                     CurveDirection dir) {
          float shortest = getAngleError(target, current); 
          if (dir == CurveDirection::Auto)
            return shortest;

          if (dir == CurveDirection::CW) {
            if (shortest <= -90.0f) {
              return shortest + 360.0f;
            }
            return shortest;
          } else {
            if (shortest >= 90.0f) {
              return shortest - 360.0f;
            }
            return shortest;
          }
        };

        float initErr = directedAngleError(targetThetaDeg, sp.theta, direction);
        float dir = (initErr >= 0) ? 1.0f : -1.0f;
        float arcRadius = std::abs(radius);
        float maxCurveTranslation =
            std::min(params.maxTranslationSpeed, 60.0f);
        float maxCurveRotation = std::min(params.maxRotationSpeed, 70.0f);

        float startRad = sp.theta * DEG2RAD;
        float centerX = sp.x + dir * arcRadius * std::cos(startRad);
        float centerY = sp.y - dir * arcRadius * std::sin(startRad);
        float targetRad = targetThetaDeg * DEG2RAD;
        float finalX = centerX - dir * arcRadius * std::cos(targetRad);
        float finalY = centerY + dir * arcRadius * std::sin(targetRad);

        while (pros::millis() - start < params.timeout) {
          Pose curr = getPose(false);
          float angleError =
              directedAngleError(targetThetaDeg, curr.theta, direction);
          float finalDistErr = std::hypot(finalX - curr.x, finalY - curr.y);

          if (params.earlyExitRange > 0.0f &&
              finalDistErr <= params.earlyExitRange)
            return;

          bool posSettled = finalDistErr < params.exitRange;
          bool angleSettled = std::abs(angleError) < angleExitDeg;
          if (posSettled && angleSettled) {
            if (settleStart == 0)
              settleStart = pros::millis();
            if (pros::millis() - settleStart >= settleTime)
              break;
          } else {
            settleStart = 0;
          }

          float toCenterX = centerX - curr.x;
          float toCenterY = centerY - curr.y;
          float distToCenter = std::hypot(toCenterX, toCenterY);
          float radiusError = distToCenter - arcRadius;

          float rad = curr.theta * DEG2RAD;
          float cosH = std::cos(rad), sinH = std::sin(rad);
          float centerLocalX = toCenterX * cosH - toCenterY * sinH;
          float centerSide = centerLocalX >= 0.0f ? 1.0f : -1.0f;

          float arcRemaining = std::abs(angleError) * DEG2RAD * arcRadius;

          xPID.setGains(xSched.getGains(radiusError));
          yPID.setGains(ySched.getGains(arcRemaining));
          tPID.setGains(thetaSched.getGains(angleError));

          float outX_local = (float)xPID.update(radiusError * centerSide);
          float outY_local = (float)yPID.update(arcRemaining);
          float outT = (float)tPID.update(angleError);

          float mag = std::hypot(outX_local, outY_local);
          if (!posSettled && mag > 1e-3f && mag < params.minSpeed) {
            float s = params.minSpeed / mag;
            outX_local *= s;
            outY_local *= s;
          }
          if (mag > maxCurveTranslation) {
            float s = maxCurveTranslation / mag;
            outX_local *= s;
            outY_local *= s;
          }
          outT = std::clamp(outT, -maxCurveRotation, maxCurveRotation);

          setMotorVoltages(calculateHolonomic(outX_local, outY_local, outT));
          pros::delay(10);
        }
        brake();
      },
      params.async);
}

void Chassis::waitUntilDone() {
  motion.waitUntilDone();
}

void Chassis::waitUntil(float dist) {
  uint32_t targetId = motion.getLastEnqueuedId();
  while (true) {
    poseMutex.take();
    float currentDist = motionDistTraveled;
    poseMutex.give();

    uint32_t runningId = motion.getCurrentRunningId();
    bool empty = motion.isQueueEmpty();
    if (empty && runningId < targetId) {
      break;
    }

    if (runningId >= targetId) {
      if (runningId > targetId || currentDist >= dist) {
        break;
      }
    }

    pros::delay(10);
  }
}

void Chassis::cancelMotion() {
  motion.cancelMotion();
  brake();
}

void Chassis::cancelAllMotions() {
  motion.cancelAll();
  brake();
}

void Chassis::setEKFGains(float xProcessNoise, float yProcessNoise, float thetaProcessNoise, float measurementNoise) {
  ekf.setProcessNoise(xProcessNoise, yProcessNoise, thetaProcessNoise, measurementNoise);
}


void Chassis::setVelocityCalculations(bool state)
{
  velocityCalculationsOn = state;
}


bool Chassis::detectCollision() {
    const double TARGET_VEL_THRESHOLD = 30.0;   
    const uint32_t DEBOUNCE_TIME_MS = 250;      

    static uint32_t last_check_time = pros::millis();
    static uint32_t stall_accumulator_ms = 0;

    uint32_t now = pros::millis();
    uint32_t dt = now - last_check_time;
    last_check_time = now;
    auto is_wheel_slipping = [&](pros::Motor& motor) {
        double target = std::abs(motor.get_target_velocity());
        double actual = std::abs(motor.get_actual_velocity());
        int32_t current = motor.get_current_draw();
        double temp = motor.get_temperature();     
        if (target < TARGET_VEL_THRESHOLD) {
            return false;
        }
        int32_t dynamic_current_threshold = 1200; 
        if (temp > 50.0) {
            dynamic_current_threshold = 900;    
        }
        bool speed_deficit = actual < (target * 0.70);
        bool heavy_load = current > dynamic_current_threshold;

        return speed_deficit && heavy_load;
    };
    int slip_count = 0;
    if (is_wheel_slipping(frontLeft)) slip_count++;
    if (is_wheel_slipping(frontRight)) slip_count++;
    if (is_wheel_slipping(backLeft)) slip_count++;
    if (is_wheel_slipping(backRight)) slip_count++;
    bool physically_blocked = (slip_count >= 2);

    if (physically_blocked) {
        stall_accumulator_ms += dt;
    } else {
        stall_accumulator_ms = 0; 
    }

    return stall_accumulator_ms >= DEBOUNCE_TIME_MS;
}


void Chassis::openLoop(float forward, float sideways, float rotation) {
  setMotorVoltages(calculateHolonomic(forward, sideways, rotation));
}



