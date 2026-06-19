#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "PID.h"
#include "api.h"
#include "motion_handler.h"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <vector>

struct Obstacle {
  Eigen::Vector2f position;
  float radius;
};

class ObstacleManager {
public:
  ObstacleManager() = default;

  void setRobotDimensions(float width, float length);
  void addObstacle(float x, float y, float radius);
  void removeObstacle(size_t index);
  void clearObstacles();
  const std::vector<Obstacle> &getObstacles() const;

  bool checkIntersection(const Eigen::Vector2f &start,
                         const Eigen::Vector2f &end, float safety_margin,
                         Obstacle &out_obstacle,
                         Eigen::Vector2f &out_closest) const;

  Eigen::Vector2f getAvoidanceTarget(const Eigen::Vector2f &robot_pos,
                                     const Eigen::Vector2f &target_pos,
                                     float safety_margin, float clearance,
                                     float robot_heading_rad,
                                     int recursion_depth = 0) const;

  Eigen::Vector2f getPotentialFieldTarget(const Eigen::Vector2f &robot_pos,
                                          const Eigen::Vector2f &target_pos,
                                          float ka, float kr,
                                          float influence_radius,
                                          float robot_heading_rad) const;

private:
  std::vector<Obstacle> obstacles;
  float robot_width = 18.0f;
  float robot_length = 18.0f;
};

enum class TrackingWheelOrientation { HORIZONTAL, VERTICAL };

struct TrackingWheelConfig {
  int8_t port;
  TrackingWheelOrientation orientation;
  float xOffset;
  float yOffset;
  float wheelDiameter;
  float gearRatio;
};

class PoseEKF {
private:
  Eigen::Vector3f x;
  Eigen::Matrix3f P;
  Eigen::Matrix<float, 1, 3> H;
  float measurementNoise;
  float xProcessNoise = 0.01f, yProcessNoise = 0.01f,
        thetaProcessNoise = 0.001f;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void setProcessNoise(float xNoise, float yNoise, float thetaNoise,
                       float measNoise) {
    xProcessNoise = xNoise;
    yProcessNoise = yNoise;
    thetaProcessNoise = thetaNoise;
    measurementNoise = measNoise;
  }

  PoseEKF(float initial_x, float initial_y, float initial_theta) {
    x << initial_x, initial_y, initial_theta;
    P.setZero();
    H << 0, 0, 1;
    measurementNoise = 0.00005f;
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

    Eigen::Vector2f local_d(dx_local, dy_local);
    Eigen::Matrix2f R_arc;
    R_arc << s, c, -c, s;
    Eigen::Vector2f arc_d = R_arc * local_d;

    float prev_theta = x(2);
    float cos_t = std::cos(prev_theta);
    float sin_t = std::sin(prev_theta);

    Eigen::Matrix2f R_global;
    R_global << cos_t, sin_t, -sin_t, cos_t;
    Eigen::Vector2f global_d = R_global * arc_d;

    x.head<2>() += global_d;
    x(2) += dtheta;
    x(2) = std::remainder(x(2), 2.0f * M_PI);

    Eigen::Matrix3f F = Eigen::Matrix3f::Identity();
    F(0, 2) = global_d.y();
    F(1, 2) = -global_d.x();

    Eigen::Matrix3f Q_step = Eigen::Matrix3f::Zero();
    float var_x = xProcessNoise * std::abs(dx_local) + 1e-6f;
    float var_y = yProcessNoise * std::abs(dy_local) + 1e-6f;
    Eigen::Matrix2f Q_local;
    Q_local << var_x, 0, 0, var_y;

    Q_step.block<2, 2>(0, 0) = R_global * Q_local * R_global.transpose();
    Q_step(2, 2) = thetaProcessNoise * std::abs(dtheta) + 1e-7f;

    P = F * P * F.transpose() + Q_step;
  }

  void updateIMU(float measured_theta, float dynamic_R) {
    float y = measured_theta - x(2);
    y = std::remainder(y, 2.0f * M_PI);
    float S = (H * P * H.transpose())(0, 0) + dynamic_R;
    Eigen::Vector3f K = P * H.transpose() / S;
    x = x + K * y;
    x(2) = std::remainder(x(2), 2.0f * M_PI);
    P = (Eigen::Matrix3f::Identity() - K * H) * P;
  } 
  void updateTrackingWheels(const std::vector<TrackingWheelConfig> &configs,
                            const Eigen::VectorXf &measured_deltas,
                            float dx_local, float dy_local, float dtheta,
                            float wheel_noise) {
    const int n = static_cast<int>(configs.size());
    if (n == 0 || measured_deltas.size() != n)
      return;
    float theta = x(2);
    float cos_t = std::cos(theta);
    float sin_t = std::sin(theta);

    Eigen::MatrixXf H_tw(n, 3);
    Eigen::VectorXf z_pred(n);

    for (int i = 0; i < n; ++i) {
      float ox = configs[i].xOffset;
      float oy = configs[i].yOffset;

      if (configs[i].orientation == TrackingWheelOrientation::VERTICAL) {
        z_pred(i) = dy_local - ox * dtheta;
        H_tw(i, 0) = -sin_t;
        H_tw(i, 1) = cos_t;
        H_tw(i, 2) = -ox;
      } else {


        z_pred(i) = dx_local + oy * dtheta;
        H_tw(i, 0) = cos_t;
        H_tw(i, 1) = sin_t;
        H_tw(i, 2) = oy;
      }
    }

    Eigen::VectorXf y_innov = measured_deltas - z_pred;
    Eigen::MatrixXf R_tw = Eigen::MatrixXf::Identity(n, n) * wheel_noise;
    Eigen::MatrixXf S = H_tw * P * H_tw.transpose() + R_tw;
    Eigen::MatrixXf K = P * H_tw.transpose() * S.inverse();
    x += K * y_innov;
    x(2) = std::remainder(x(2), 2.0f * static_cast<float>(M_PI));


    Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f IKH = I3 - K * H_tw;
    P = IKH * P * IKH.transpose() + K * R_tw * K.transpose();
  }

  void setTrackingWheelNoise(float xNoise, float yNoise, float thetaNoise) {
    xProcessNoise = xNoise;
    yProcessNoise = yNoise;
    thetaProcessNoise = thetaNoise;
  }

  float getX() const { return x(0); }
  float getY() const { return x(1); }
  float getTheta() const { return x(2); }

  void setPose(float new_x, float new_y, float new_theta) {
    x << new_x, new_y, new_theta;
    P.setZero();
  }
};

struct VelocityComponents {
  float vx, vy, w;
};

struct Pose {
  float x = 0;
  float y = 0;
  float theta = 0;
  VelocityComponents velocity{0, 0, 0};
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
  float trackWidth = 0;
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

struct DriveCorrection {
  bool correctionOn = true;
  float kP = 1.0f;
  float kI = 0.01f;
  float kD = 0.1f;
};

struct MoveParams {
  float maxTranslationSpeed = 127.0f;
  float maxRotationSpeed = 127.0f;
  float minSpeed = 0.0f;
  float exitRange = 0.5f;
  float earlyExitRange = 0.0f;
  uint32_t timeout = 5000;
  bool async = true;
};

struct ScheduledGain {
  float threshold;
  PIDGains gains;
};

struct ReplayData
{
  float forwards, sideways, rotation;
  Pose pose;
};

struct ControllerButton {
  pros::controller_digital_e_t button;
  std::function<void()> callback;
};

inline std::vector<ControllerButton> controllerButtons
{
  {pros::E_CONTROLLER_DIGITAL_UP, [](){}},  
  {pros::E_CONTROLLER_DIGITAL_DOWN, [](){}},
  {pros::E_CONTROLLER_DIGITAL_LEFT, [](){}},
  {pros::E_CONTROLLER_DIGITAL_RIGHT, [](){}},
  {pros::E_CONTROLLER_DIGITAL_A, [](){}},
  {pros::E_CONTROLLER_DIGITAL_B, [](){}},
  {pros::E_CONTROLLER_DIGITAL_X, [](){}},
  {pros::E_CONTROLLER_DIGITAL_Y, [](){}},
  {pros::E_CONTROLLER_DIGITAL_L1, [](){}},
  {pros::E_CONTROLLER_DIGITAL_L2, [](){}},
  {pros::E_CONTROLLER_DIGITAL_R1, [](){}},
  {pros::E_CONTROLLER_DIGITAL_R2, [](){}},
  
};


std::vector<PathPoint> parsePathData(const std::string &input_source,
                                     bool convertFromMeters = false);
                        

class GainScheduler {
public:
  void addStep(float threshold, float kP, float kI, float kD,
               float slew = 0.0f);

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

enum class HeadingMode { FollowPath, HoldAngle, CustomAngles };

class Chassis {
public:
  Chassis(pros::Motor fl, pros::Motor fr, pros::Motor bl, pros::Motor br,
          pros::Imu imu_sensor, ChassisConfig config);

  void calibrate();

  void setXGains(std::vector<ScheduledGain> steps);
  void setYGains(std::vector<ScheduledGain> steps);
  void setThetaGains(std::vector<ScheduledGain> steps);

  void setPose(float x, float y, float theta = 0.0f);

  void setPose(Pose pose);

  Pose getPose(bool radians = false);

  XDriveVoltages calculateHolonomic(float vx, float vy, float vt);
  void setMotorVoltages(XDriveVoltages v);
  void brake();

  void driveControl(float forward, float sideways, float rotation,
                    DriveCurves drivecurves, bool fieldCentric,
                    float headingOffset = 0.0f,
                    DriveCorrection correction = {});

  enum class CurveDirection { Auto, CW, CCW };
  enum class AvoidanceMode { Off, On };

  void followPath(const std::vector<PathPoint> &path, float lookahead_inches,
                     MoveParams params = defaultParams,
                     HeadingMode headingMode = HeadingMode::FollowPath,
                     float holdAngleDeg = 0.0f, bool reversed = false);

  void turnToHeading(float targetDeg, MoveParams params = defaultParams);

  void turnToPoint(float tx, float ty, MoveParams params = defaultParams);

  void moveToPoint(float tx, float ty, MoveParams params = defaultParams,
                   bool angleCorrection = true);

  void moveRelative(float forward, float sideways, MoveParams params = defaultParams,
                    bool holdHeading = true);

  void moveDistance(float distance, MoveParams params = {},
                    bool holdHeading = true);

  void strafeDistance(float distance, MoveParams params = defaultParams,
                      bool holdHeading = true);

  void moveToPose(float tx, float ty, float targetThetaDeg,
                  MoveParams params = defaultParams);

  void curveCircle(float targetThetaDeg, float radius, MoveParams params = defaultParams,
                   CurveDirection direction = CurveDirection::Auto);

  void waitUntilDone();

  void cancelAllMotions();

  MotionHandler motion;

  void odometryTask();

  float getDistanceTraveled(bool convertToMeters = false);

  void cancelMotion();

  void waitUntil(float distance);

  void setEKFGains(float xProcessNoise, float yProcessNoise,
                   float thetaProcessNoise, float measurementNoise);

  void setVelocityCalculations(bool state);

  bool detectCollision();

  void openLoop(float forward, float sideways, float rotation);

  void addObstacle(float x, float y, float radius);
  void removeObstacle(size_t index);
  void clearObstacles();
  void setAvoidanceMode(AvoidanceMode mode);
  void setAvoidanceParams(float safetyMargin, float clearance);
  void setPotentialFieldParams(float ka, float kr, float influenceRadius);

  void setRobotDimensionsAvoidance(float width, float height);
  float radToDeg(float rad);
  float degToRad(float deg);

  void setEKFstate(bool state);

  void addTrackingWheel(TrackingWheelConfig config);
  void clearTrackingWheels();

  template <typename F>
  void move(F updateFunction, MoveParams params = defaultParams, bool fieldCentric = true,
            float headingOffset = 0.0f, DriveCorrection correction = {}) {}

  void setMoveParams(MoveParams params);

  enum class SwingSide {Right, Left};
  void swingTurn(float targetThetaDeg, SwingSide lockedSide, MoveParams params = defaultParams);
  
  void getControllerInput(pros::Controller master);
  void logReplayData(pros::Controller master, int timeout_ms = 50);
  void disableReplayDataLogging();

  void runDriverReplay(std::vector<PathPoint> data, float lookahead);

private:
  pros::Motor frontLeft, frontRight, backLeft, backRight;
  std::vector<pros::Motor> motors{frontLeft, frontRight, backLeft, backRight};
  pros::Imu imu;
  ChassisConfig config;
  static MoveParams defaultParams;
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
  float xProcessNoise = 0.001f, yProcessNoise = 0.001f,
        thetaProcessNoise = 0.003f, measurementNoise = 0.0001f;
  bool velocityCalculationsOn = false;

  ObstacleManager obstacles;
  AvoidanceMode avoidanceMode = AvoidanceMode::Off;
  float avoidanceSafetyMargin = 4.0f;
  float avoidanceClearance = 8.0f;
  float pf_ka = 5.0f;
  float pf_kr = 50.0f;
  float pf_influence_radius = 15.0f;

  GainScheduler savedXSched, savedYSched, savedThetaSched;

  uint32_t last_collision_check_time = 0;
  uint32_t stall_accumulator_ms = 0;


  std::vector<TrackingWheelConfig> trackingWheelConfigs;
  std::vector<pros::Rotation> trackingWheelSensors;
  std::vector<float> prevTrackingPositions;
  bool useTrackingWheels = false;
  float trackingWheelMeasNoise = 0.0005f;
};

inline float getAngleError(float target, float current) {
  return std::remainder(target - current, 360.0f);
}
