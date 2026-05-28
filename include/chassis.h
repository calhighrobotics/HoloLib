#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "PID.h"
#include "api.h"
#include "motion_handler.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <vector>

class PoseEKF {
private:
  Eigen::Vector3f x;
  Eigen::Matrix3f P;
  Eigen::Matrix3f Q;
  Eigen::Matrix<float, 1, 3> H;
  float R;
  float xProcessNoise = 0.01f, yProcessNoise = 0.01f, thetaProcessNoise = 0.001f, measurementNoise = 0.00045f;


public:
  void setProcessNoise(float xNoise, float yNoise, float thetaNoise, float measurementNoise) {
    xProcessNoise = xNoise;
    yProcessNoise = yNoise;
    thetaProcessNoise = thetaNoise;
    measurementNoise = measurementNoise;
    Q << xProcessNoise, 0, 0, 0, yProcessNoise, 0, 0, 0, thetaProcessNoise;
    R = measurementNoise;
  }


  PoseEKF(float initial_x, float initial_y, float initial_theta) {
    x << initial_x, initial_y, initial_theta;
    P.setZero();

    Q << xProcessNoise, 0, 0, 0, yProcessNoise, 0, 0, 0, thetaProcessNoise;

    H << 0, 0, 1;
    R = measurementNoise;
  }

  void predict(float dx_local, float dy_local, float dtheta) {
    float s, c;
    if (std::abs(dtheta) < 1e-4f) {
      s = 1.0f - (dtheta * dtheta) / 6.0f;
      c = dtheta / 2.0f;
    } else {
      s = std::sin(dtheta) / dtheta;
      c = (1.0f - std::cos(dtheta)) / dtheta;
    }

    float dx_arc = dx_local * s + dy_local * c;
    float dy_arc = -dx_local * c + dy_local * s;

    float prev_theta = x(2);
    float cos_t = std::cos(prev_theta);
    float sin_t = std::sin(prev_theta);

    float dx_global = dx_arc * cos_t + dy_arc * sin_t;
    float dy_global = -dx_arc * sin_t + dy_arc * cos_t;

    x(0) += dx_global;
    x(1) += dy_global;
    x(2) += dtheta;

    x(2) = std::remainder(x(2), 2.0f * M_PI);

    Eigen::Matrix3f F = Eigen::Matrix3f::Identity();
    F(0, 2) = dy_global;
    F(1, 2) = -dx_global;

    P = F * P * F.transpose() + Q;
  }

  void updateIMU(float measured_theta) {
    float y = measured_theta - x(2);
    y = std::remainder(y, 2.0f * M_PI);
    float S = (H * P * H.transpose())(0, 0) + R;
    Eigen::Vector3f K = P * H.transpose() / S;
    x = x + K * y;
    x(2) = std::remainder(x(2), 2.0f * M_PI);
    P = (Eigen::Matrix3f::Identity() - K * H) * P;
  }

  float getX() const { return x(0); }
  float getY() const { return x(1); }
  float getTheta() const { return x(2); }

  void setPose(float new_x, float new_y, float new_theta) {
    x << new_x, new_y, new_theta;
    P.setZero();
  }
};

struct Pose {
  float x = 0;
  float y = 0;
  float theta = 0;
};

struct PathPoint {
  float x = 0;
  float y = 0;
  float theta = 0;
};

struct XDriveVoltages {
  float fl, fr, bl, br;
};

struct ChassisConfig {
  float trackWidth;
  float drivetrainWidth = 0.0f;
  float drivetrainLength = 0.0f;
  float wheelDiameter;
  float gearRatio;
  bool kfEnabled = true;
};

struct DriveCurve {
  float curve_multipler = 1.0f;
  float deadzone = 0.0f;
  float minimum_output = 0.0f;
};

struct DriveCurves {
  DriveCurve movement;
  DriveCurve rotation;
};

struct DriveCorrection
{
  bool correctionOn = true;
  float kP = 1.0f;
  float kI = 0.01f;
  float kD = 0.1f;
};

struct MoveParams {
  float maxTranslationSpeed = 127.0f;
  float maxRotationSpeed = 127.0f;
  float minSpeed = 0.0f;
  float exitRange = 2.0f;
  float earlyExitRange = 0.0f;
  uint32_t timeout = 5000;
  bool async = true;
};

struct ScheduledGain {
  float threshold;
  PIDGains gains;
};

std::vector<PathPoint> parsePathData(const std::string &input_source,
                                     bool convertFromMeters = false);

class GainScheduler {
public:
  void addStep(float threshold, float kP, float kI, float kD);

  PIDGains getGains(float error) const;

  void clear();

private:
  std::vector<ScheduledGain> schedules;
};

class EncoderKalmanFilter {
public:
  EncoderKalmanFilter(float process_noise = 0.077f,
                      float measurement_noise = 7.2f);
  float update(float measured_position, float dt);

private:
  Eigen::Vector2f x;
  Eigen::Matrix2f P, Q;
  Eigen::RowVector2f H;
  float R;
};

class Chassis {
public:
  Chassis(pros::Motor fl, pros::Motor fr, pros::Motor bl, pros::Motor br,
          pros::Imu imu_sensor, ChassisConfig config);

  void calibrate();

  void setXGains(std::vector<ScheduledGain> steps);
  void setYGains(std::vector<ScheduledGain> steps);
  void setThetaGains(std::vector<ScheduledGain> steps);

  void setPose(float x, float y, float theta = 0.0f);

  Pose getPose(bool radians = false);

  XDriveVoltages calculateHolonomic(float vx, float vy, float vt);
  void setMotorVoltages(XDriveVoltages v);
  void brake();

  void driveControl(float forward, float sideways, float rotation,
                    DriveCurves drivecurves, bool fieldCentric, float headingOffset = 0.0f, DriveCorrection correction = {});

  enum class HeadingMode { FollowPath, HoldAngle, CustomAngles};
  enum class CurveDirection { Auto, CW, CCW };

  void followPathPID(const std::vector<PathPoint> &path, float lookahead_inches,
                     MoveParams params = {},
                     HeadingMode headingMode = HeadingMode::FollowPath,
                     float holdAngleDeg = 0.0f, bool reversed = false);

  void turnToHeading(float targetDeg, MoveParams params = {});

  void turnToPoint(float tx, float ty, MoveParams params = {});

  void moveToPoint(float tx, float ty, MoveParams params = {},
                   bool angleCorrection = true);

  void moveRelative(float forward, float sideways, MoveParams params = {},
                    bool holdHeading = true);

  void moveDistance(float distance, MoveParams params = {},
                    bool holdHeading = true);

  void strafeDistance(float distance, MoveParams params = {},
                      bool holdHeading = true);

  void moveToPose(float tx, float ty, float targetThetaDeg,
                  MoveParams params = {});

  void curveCircle(float targetThetaDeg, float radius, MoveParams params = {},
                   CurveDirection direction = CurveDirection::Auto);

  void waitUntilDone();

  void cancelAllMotions();

  MotionHandler motion;

  void odometryTask();

  void getDistanceTraveled(bool convertToMeters = false);

  void cancelMotion();

  void waitUntil(float distance);

  void setEKFGains(float xProcessNoise, float yProcessNoise, float thetaProcessNoise,
                   float measurementNoise);

private:
  pros::Motor frontLeft, frontRight, backLeft, backRight;
  pros::Imu imu;
  ChassisConfig config;

  GainScheduler xSched, ySched, thetaSched;

  Pose currentPose{0, 0, 0};
  pros::Mutex poseMutex;
  PoseEKF ekf{0, 0, 0};
  EncoderKalmanFilter kf_fl, kf_fr, kf_bl, kf_br;
  float prev_fl = 0, prev_fr = 0, prev_bl = 0, prev_br = 0;
  float prev_heading = 0;
  float targetHeadingDriveControl = 0;
  friend void odomTaskTrampoline(void *);
  float motionDistTraveled = 0.0f;
  float xProcessNoise = 0.01f, yProcessNoise = 0.01f, thetaProcessNoise = 0.001f, measurementNoise = 0.00045f;
};

inline float getAngleError(float target, float current) {
  return std::remainder(target - current, 360.0f);
}
