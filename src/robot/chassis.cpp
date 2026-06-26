#include "chassis.h"
#include "pros/misc.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

static constexpr float DEG2RAD = M_PI / 180.0f;
static constexpr float RAD2DEG = 180.0f / M_PI;
MoveParams Chassis::defaultParams = {};

/**
 *@brief Calibrates the chassis and calibrates all sensors.
 *@return void
 *@note This function should be called before using chassis motions (recommended
 * to be run in initialize)
 */
void Chassis::calibrate() {
  cancelAllMotions();
  backLeft.tare_position();
  backRight.tare_position();
  frontLeft.tare_position();
  frontRight.tare_position();
  motionDistTraveled = 0.0f;
  prev_fl = 0, prev_fr = 0, prev_bl = 0, prev_br = 0;
  prev_heading = 0;
  targetHeadingDriveControl = 0;
  for (int i = 0; i < motors.size(); i++) {
    try {
      if (motors[i].get_temperature() > 60) {
        std::cout << "Motor " + std::to_string(i) + " overheating" << std::endl;
      }
    } catch (...) {
      std::cout << "Error when evaluating Motor " + std::to_string(i)
                << std::endl;
    }
  }

  for (size_t i = 0; i < trackingWheelSensors.size(); ++i) {
    trackingWheelSensors[i].reset_position();
    prevTrackingPositions[i] = 0.0f;
  }
  imu.reset(true);
  while (imu.is_calibrating()) {
    pros::delay(10);
  }
  pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
  std::cout << "Chassis Calibrated" << std::endl;
}

/**
 *@brief Calculates the holonomic voltages for the motors.
 *@param vx The x-component of the velocity.
 *@param vy The y-component of the velocity.
 *@param vt The angular velocity.
 *@return XDriveVoltages The voltages for the motors.
 */
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

/**
 *@brief Sets the motor voltages.
 *@param v The voltages to set.
 *@return void
 */
void Chassis::setMotorVoltages(XDriveVoltages v) {
  frontLeft.move_voltage((int32_t)v.fl);
  frontRight.move_voltage((int32_t)v.fr);
  backLeft.move_voltage((int32_t)v.bl);
  backRight.move_voltage((int32_t)v.br);
}

/**
 *@brief Brakes all motors.
 *@return void
 */
void Chassis::brake() {
  frontLeft.brake();
  frontRight.brake();
  backLeft.brake();
  backRight.brake();
}

/**
 *@brief Sets the pose of the robot.
 *@param x The x-coordinate of the pose.
 *@param y The y-coordinate of the pose.
 *@param theta The heading of the pose.
 *@return void
 */
void Chassis::setPose(float x, float y, float theta) {
  poseMutex.take();
  float theta_rad = theta * DEG2RAD;
  imu.set_rotation(theta);

  currentPose = {x, y, theta_rad};
  prev_heading = theta_rad;

  ekf.setPose(x, y, theta_rad);

  poseMutex.give();
}

/**
 *@brief Sets the pose of the robot.
 *@param pose The pose to set.
 *@return void
 */
void Chassis::setPose(Pose pose) { setPose(pose.x, pose.y, pose.theta); }

/**
 *@brief Gets the pose of the robot.
 *@param radians Whether to return the heading in radians.
 *@return Pose The pose of the robot.
 */
Pose Chassis::getPose(bool radians) {
  poseMutex.take();
  Pose p = currentPose;
  poseMutex.give();
  if (!radians)
    p.theta *= RAD2DEG;
  return p;
}

/**
 *@brief Sets the PID gains for the x-axis.
 *@param steps The PID gains for the x-axis.
 *@return void
 */
void Chassis::setXGains(std::vector<ScheduledGain> steps) {
  xSched.clear();
  for (auto &s : steps)
    xSched.addStep(s.threshold, s.gains.kP, s.gains.kI, s.gains.kD,
                   s.gains.slew);
}

/**
 *@brief Sets the PID gains for the y-axis.
 *@param steps The PID gains for the y-axis.
 *@return void
 */
void Chassis::setYGains(std::vector<ScheduledGain> steps) {
  ySched.clear();
  for (auto &s : steps)
    ySched.addStep(s.threshold, s.gains.kP, s.gains.kI, s.gains.kD,
                   s.gains.slew);
}

/**
 *@brief Sets the PID gains for the theta-axis.
 *@param steps The PID gains for the theta-axis.
 *@return void
 */
void Chassis::setThetaGains(std::vector<ScheduledGain> steps) {
  thetaSched.clear();
  for (auto &s : steps)
    thetaSched.addStep(s.threshold, s.gains.kP, s.gains.kI, s.gains.kD,
                       s.gains.slew);
}

/**
 *@brief Controls the drive motors.
 *@param forward The forward velocity.
 *@param sideways The sideways velocity.
 *@param rotation The rotation velocity.
 *@param drivecurves The drive curves to use.
 *@param fieldCentric Whether to use field centric control.
 *@param headingOffset The heading offset.
 *@param correction The correction to apply.
 *@return void
 */
void Chassis::driveControl(float forward, float sideways, float rotation,
                           DriveCurves drivecurves, bool fieldCentric,
                           float headingOffset, DriveCorrection correction) {
  static bool headingInitialized = false;
  static float targetHeading = 0.0f;
  static PID headingPID(0, 0, 0, 0);
  static uint32_t lastRotationTime = 0;
  static bool wasRotating = false;
  constexpr float MAX_DRIVE_INPUT = 127.0f;
  constexpr float SETTLE_DELAY_MS = 150.0f;
  constexpr float MAX_CORRECTION = 40.0f;

  if (!headingInitialized) {
    targetHeading = getPose(false).theta;
    headingPID.setGains(
        {correction.kP, correction.kI, correction.kD, 0.0, 0.0});
    headingInitialized = true;
  }

  auto applyCurve = [&](float x, const DriveCurve &c) -> float {
    if (std::abs(x) < c.deadzone)
      return 0.0f;

    float sign = (x >= 0.0f) ? 1.0f : -1.0f;
    float normalized =
        (std::abs(x) - c.deadzone) / (MAX_DRIVE_INPUT - c.deadzone);

    normalized = std::clamp(normalized, 0.0f, 1.0f);
    normalized = std::pow(normalized, c.curve_multipler);

    float output = normalized * MAX_DRIVE_INPUT;
    if (output > 0.0f && output < c.minimum_output) {
      output = c.minimum_output;
    }

    return output * sign;
  };
  forward = applyCurve(forward, drivecurves.movement);
  sideways = applyCurve(sideways, drivecurves.movement);

  float robotForward = forward;
  float robotSideways = sideways;
  if (fieldCentric) {

    float adjustedTheta = getPose(false).theta - headingOffset;

    float thetaRad = adjustedTheta * DEG2RAD;

    robotSideways =
        sideways * std::cos(thetaRad) - forward * std::sin(thetaRad);

    robotForward = sideways * std::sin(thetaRad) + forward * std::cos(thetaRad);
    float inputMagnitude = std::sqrt(forward * forward + sideways * sideways);

    float rotatedMagnitude =
        std::sqrt(robotForward * robotForward + robotSideways * robotSideways);

    if (rotatedMagnitude > 0.001f) {

      float scale = inputMagnitude / rotatedMagnitude;

      robotForward *= scale;
      robotSideways *= scale;
    }
  }

  bool isRotating = std::abs(rotation) >= drivecurves.rotation.deadzone;

  if (isRotating) {
    rotation = applyCurve(rotation, drivecurves.rotation);
    targetHeading = getPose(false).theta;
    lastRotationTime = pros::millis();
    wasRotating = true;

  } else {
    uint32_t timeSinceRotation = pros::millis() - lastRotationTime;

    if (wasRotating) {
      targetHeading = getPose(false).theta;
      headingPID.reset();
      headingPID.setGains(
          {correction.kP, correction.kI, correction.kD, 0.0, 0.0});
      wasRotating = false;
    }

    if (timeSinceRotation < (uint32_t)SETTLE_DELAY_MS) {
      rotation = 0.0f;
      targetHeading = getPose(false).theta;
    } else if (correction.correctionOn) {
      float currentHeading = getPose(false).theta;
      float angleError = getAngleError(targetHeading, currentHeading);

      if (std::abs(angleError) < 0.5f) {
        rotation = 0.0f;
      } else {
        rotation = (float)headingPID.update(angleError);
        rotation = std::clamp(rotation, -MAX_CORRECTION, MAX_CORRECTION);
      }
    } else {
      rotation = 0.0f;
    }
  }
  setMotorVoltages(calculateHolonomic(robotSideways, robotForward, rotation));
}

/**
 *@brief Waits for all motion to complete.
 *@return void
 */
void Chassis::waitUntilDone() { motion.waitUntilDone(); }

/**
 *@brief Waits for the robot to travel a specific distance.
 *@param dist The distance to travel.
 *@return void
 */
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

/**
 *@brief Cancels the current motion.
 *@return void
 */
void Chassis::cancelMotion() {
  motion.cancelMotion();
  brake();
}

/**
 *@brief Cancels all motion.
 *@return void
 */
void Chassis::cancelAllMotions() {
  motion.cancelAll();
  brake();
}

/**
 *@brief Gets the distance the robot has traveled.
 *@param convertToMeters Whether to convert to meters.
 *@return float The distance traveled.
 */
float Chassis::getDistanceTraveled(bool convertToMeters) {
  poseMutex.take();
  float dist = motionDistTraveled;
  poseMutex.give();
  if (convertToMeters)
    return dist * 0.0254f;
  return dist;
}

/**
 *@brief Sets the EKF gains.
 *@param xProcessNoise The x-process noise.
 *@param yProcessNoise The y-process noise.
 *@param thetaProcessNoise The theta-process noise.
 *@param measurementNoise The measurement noise.
 *@return void
 */
void Chassis::setEKFGains(float xProcessNoise, float yProcessNoise,
                          float thetaProcessNoise, float measurementNoise) {
  ekf.setProcessNoise(xProcessNoise, yProcessNoise, thetaProcessNoise,
                      measurementNoise);
}

/**
 *@brief Enables or disables velocity calculations.
 *@param state Whether to enable velocity calculations.
 *@return void
 */
void Chassis::setVelocityCalculations(bool state) {
  velocityCalculationsOn = state;
}

/**
 *@brief Detects if the robot is colliding.
 *@return bool True if the robot is colliding.
 */
bool Chassis::detectCollision() {
  const int32_t TARGET_VOLTAGE_THRESHOLD = 3000;
  const uint32_t DEBOUNCE_TIME_MS = 250;

  if (last_collision_check_time == 0)
    last_collision_check_time = pros::millis();

  uint32_t now = pros::millis();
  uint32_t dt = now - last_collision_check_time;
  last_collision_check_time = now;
  auto is_wheel_slipping = [&](pros::Motor &motor) {
    int32_t commanded_voltage = std::abs(motor.get_voltage());
    if (commanded_voltage < TARGET_VOLTAGE_THRESHOLD) {
      return false;
    }

    double actual = std::abs(motor.get_actual_velocity());
    int32_t current = motor.get_current_draw();
    double temp = motor.get_temperature();

    int32_t dynamic_current_threshold = 1200;
    if (temp > 50.0) {
      dynamic_current_threshold = 900;
    }

    bool speed_deficit = actual < 30.0;
    bool heavy_load = current > dynamic_current_threshold;

    return speed_deficit && heavy_load;
  };
  int slip_count = 0;
  if (is_wheel_slipping(frontLeft))
    slip_count++;
  if (is_wheel_slipping(frontRight))
    slip_count++;
  if (is_wheel_slipping(backLeft))
    slip_count++;
  if (is_wheel_slipping(backRight))
    slip_count++;
  bool physically_blocked = (slip_count >= 2);

  if (physically_blocked) {
    stall_accumulator_ms += dt;
  } else {
    stall_accumulator_ms = 0;
  }

  return stall_accumulator_ms >= DEBOUNCE_TIME_MS;
}

/**
 *@brief Moves the robot using open-loop control.
 *@param forward The forward movement distance.
 *@param sideways The sideways movement distance.
 *@param rotation The rotation distance.
 *@return void
 */
void Chassis::openLoop(float forward, float sideways, float rotation) {
  setMotorVoltages(calculateHolonomic(sideways, forward, rotation));
}

/**
 *@brief Converts radians to degrees.
 *@param rad The angle in radians.
 *@return float The angle in degrees.
 */
float Chassis::radToDeg(float rad) { return rad * RAD2DEG; }

/**
 *@brief Converts degrees to radians.
 *@param deg The angle in degrees.
 *@return float The angle in radians.
 */
float Chassis::degToRad(float deg) { return deg * DEG2RAD; }

/**
 *@brief Enables or disables the EKF.
 *@param state Whether to enable the EKF.
 *@return void
 */
void Chassis::setEKFstate(bool state) { this->config.kfEnabled = state; }
