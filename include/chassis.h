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

/**
 * @brief Represents a physical obstacle on the field.
 */
struct Obstacle {
  Eigen::Vector2f position; /**< 2D coordinate position of the obstacle */
  float radius;             /**< Radius of the obstacle used for avoidance */
};

/**
 * @brief Manages a collection of obstacles and provides avoidance computations.
 *
 * Provides methods for checking path intersections and generating vectors to
 * steer away from potential collisions.
 */
class ObstacleManager {
public:
  /**
   * @brief Default constructor for ObstacleManager.
   */
  ObstacleManager() = default;

  /**
   * @brief Set the dimensions of the robot to define its bounding box for
   * collisions.
   *
   * @param width Width of the robot.
   * @param length Length of the robot.
   */
  void setRobotDimensions(float width, float length);

  /**
   * @brief Add a circular obstacle to the field representation.
   *
   * @param x X coordinate of the obstacle.
   * @param y Y coordinate of the obstacle.
   * @param radius Radius of the obstacle.
   */
  void addObstacle(float x, float y, float radius);

  /**
   * @brief Remove an obstacle by its index.
   *
   * @param index The index of the obstacle in the internal vector.
   */
  void removeObstacle(size_t index);

  /**
   * @brief Clear all tracked obstacles.
   */
  void clearObstacles();

  /**
   * @brief Get the list of currently tracked obstacles.
   *
   * @return const std::vector<Obstacle>& Reference to the obstacle vector.
   */
  const std::vector<Obstacle> &getObstacles() const;

  /**
   * @brief Check if a path segment intersects with any obstacle.
   *
   * @param start Start point of the segment.
   * @param end End point of the segment.
   * @param safety_margin Extra margin to add to the robot/obstacle bounds.
   * @param out_obstacle Output parameter for the intersected obstacle.
   * @param out_closest Output parameter for the closest point of intersection.
   * @return true if an intersection occurs, false otherwise.
   */
  bool checkIntersection(const Eigen::Vector2f &start,
                         const Eigen::Vector2f &end, float safety_margin,
                         Obstacle &out_obstacle,
                         Eigen::Vector2f &out_closest) const;

  /**
   * @brief Calculate a target vector that avoids collisions while navigating
   * towards a goal.
   *
   * @param robot_pos Current robot position.
   * @param target_pos Desired goal position.
   * @param safety_margin Extra margin around obstacles.
   * @param clearance Desired clearance distance from obstacles.
   * @param robot_heading_rad Current robot heading in radians.
   * @param recursion_depth Internal use for recursive path-finding limitations.
   * @return Eigen::Vector2f The corrected target point to aim for.
   */
  Eigen::Vector2f getAvoidanceTarget(const Eigen::Vector2f &robot_pos,
                                     const Eigen::Vector2f &target_pos,
                                     float safety_margin, float clearance,
                                     float robot_heading_rad,
                                     int recursion_depth = 0) const;

  /**
   * @brief Get an artificial potential field target vector.
   *
   * @param robot_pos Current robot position.
   * @param target_pos Desired goal position.
   * @param ka Attractive force constant.
   * @param kr Repulsive force constant.
   * @param influence_radius Distance at which obstacles begin exerting
   * repulsive force.
   * @param robot_heading_rad Current robot heading in radians.
   * @return Eigen::Vector2f The resultant vector from potential fields.
   */
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

/**
 * @brief Defines the orientation of an unpowered tracking wheel.
 */
enum class TrackingWheelOrientation { HORIZONTAL, VERTICAL };

/**
 * @brief Configuration parameters for a tracking wheel.
 */
struct TrackingWheelConfig {
  int8_t port; /**< Sensor port number */
  TrackingWheelOrientation
      orientation;     /**< Orientation (horizontal or vertical) */
  float xOffset;       /**< Offset from tracking center on the X axis */
  float yOffset;       /**< Offset from tracking center on the Y axis */
  float wheelDiameter; /**< Diameter of the tracking wheel */
  float gearRatio;     /**< Gear ratio between the wheel and the sensor */
};

/**
 * @brief Extended Kalman Filter implementation for maintaining the robot's
 * pose.
 *
 * Fuses IMU heading and tracking wheel (or motor encoder) deltas to estimate
 * X, Y, and Theta coordinates on the field.
 */
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

  /**
   * @brief Set the process noise values for the Kalman filter matrices.
   *
   * @param xNoise Process noise in the X direction.
   * @param yNoise Process noise in the Y direction.
   * @param thetaNoise Process noise for the heading.
   * @param measNoise Measurement noise parameter.
   */
  void setProcessNoise(float xNoise, float yNoise, float thetaNoise,
                       float measNoise) {
    xProcessNoise = xNoise;
    yProcessNoise = yNoise;
    thetaProcessNoise = thetaNoise;
    measurementNoise = measNoise;
  }

  /**
   * @brief Construct a new Pose EKF object.
   *
   * @param initial_x Starting X position.
   * @param initial_y Starting Y position.
   * @param initial_theta Starting heading.
   */
  PoseEKF(float initial_x, float initial_y, float initial_theta) {
    x << initial_x, initial_y, initial_theta;
    P.setZero();
    H << 0, 0, 1;
    measurementNoise = 0.00005f;
  }

  /**
   * @brief The prediction step of the EKF based on local relative movements.
   *
   * @param dx_local Change in local X position.
   * @param dy_local Change in local Y position.
   * @param dtheta Change in heading.
   */
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

  /**
   * @brief Update the state using absolute heading measured by an IMU.
   *
   * @param measured_theta Heading read from the IMU.
   * @param dynamic_R Dynamic measurement noise.
   */
  void updateIMU(float measured_theta, float dynamic_R) {
    float y = measured_theta - x(2);
    y = std::remainder(y, 2.0f * M_PI);
    float S = (H * P * H.transpose())(0, 0) + dynamic_R;
    Eigen::Vector3f K = P * H.transpose() / S;
    x = x + K * y;
    x(2) = std::remainder(x(2), 2.0f * M_PI);
    P = (Eigen::Matrix3f::Identity() - K * H) * P;
  }

  /**
   * @brief Update the state using unpowered tracking wheels data.
   *
   * @param configs Vector of tracking wheel configurations.
   * @param measured_deltas Measurement differences for each wheel since the
   * last step.
   * @param dx_local Estimated local X change.
   * @param dy_local Estimated local Y change.
   * @param dtheta Estimated heading change.
   * @param wheel_noise Sensor noise for the tracking wheels.
   */
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

  /**
   * @brief Set specific process noises when using tracking wheels.
   *
   * @param xNoise Process noise in X direction.
   * @param yNoise Process noise in Y direction.
   * @param thetaNoise Process noise for the heading.
   */
  void setTrackingWheelNoise(float xNoise, float yNoise, float thetaNoise) {
    xProcessNoise = xNoise;
    yProcessNoise = yNoise;
    thetaProcessNoise = thetaNoise;
  }

  /**
   * @brief Get current X position.
   * @return float The X position.
   */
  float getX() const { return x(0); }

  /**
   * @brief Get current Y position.
   * @return float The Y position.
   */
  float getY() const { return x(1); }

  /**
   * @brief Get current heading angle.
   * @return float The heading in radians.
   */
  float getTheta() const { return x(2); }

  /**
   * @brief Reset the state to a specific pose.
   *
   * @param new_x New X position.
   * @param new_y New Y position.
   * @param new_theta New heading angle.
   */
  void setPose(float new_x, float new_y, float new_theta) {
    x << new_x, new_y, new_theta;
    P.setZero();
  }
};

/**
 * @brief Contains internal robot velocities.
 */
struct VelocityComponents {
  float vx, vy, w; /**< Velocities in X, Y, and Rotational domains */
};

/**
 * @brief Contains global coordinates and velocity.
 */
struct Pose {
  float x = 0;                          /**< Global X coordinate */
  float y = 0;                          /**< Global Y coordinate */
  float theta = 0;                      /**< Global heading angle */
  VelocityComponents velocity{0, 0, 0}; /**< Current velocity components */
};

/**
 * @brief Represents a single point along an autonomous path.
 */
struct PathPoint {
  float x = 0;     /**< Target X coordinate */
  float y = 0;     /**< Target Y coordinate */
  float theta = 0; /**< Target heading at this point */
};

/**
 * @brief Holonomic target voltages for a 4-motor X-drive or mecanum base.
 */
struct XDriveVoltages {
  float fl, fr, bl,
      br; /**< Front-left, Front-right, Back-left, Back-right voltages */
};

/**
 * @brief Core constants to configure the chassis dimensions and hardware.
 */
struct ChassisConfig {
  float trackWidth = 0;          /**< Distance between left and right wheels */
  float drivetrainWidth = 0.0f;  /**< Physical width of the drivetrain */
  float drivetrainLength = 0.0f; /**< Physical length of the drivetrain */
  float wheelDiameter;           /**< Diameter of the drive wheels */
  float gearRatio;       /**< Gear ratio (motor rotations / wheel rotations) */
  bool kfEnabled = true; /**< Use Kalman filtering on encoders if true */
};

/**
 * @brief Represents a simple deadzone-minimum output exponential or linear
 * drive curve.
 */
struct DriveCurve {
  float curve_multipler = 1.0f; /**< Curvature scale */
  float deadzone = 0.0f;        /**< Deadzone before response begins */
  float minimum_output =
      0.0f; /**< Minimum voltage output once deadzone is surpassed */
};

/**
 * @brief Holds the drive curves for movement and rotation inputs.
 */
struct DriveCurves {
  DriveCurve movement; /**< Curve mapping for forward/sideways translation */
  DriveCurve rotation; /**< Curve mapping for rotational input */
};

/**
 * @brief Configuration for active heading correction during driver control.
 */
struct DriveCorrection {
  bool correctionOn = true; /**< True to actively maintain heading when rotation
                               joystick is released */
  float kP = 1.0f;          /**< Proportional gain for heading hold */
  float kI = 0.01f;         /**< Integral gain for heading hold */
  float kD = 0.1f;          /**< Derivative gain for heading hold */
};

/**
 * @brief Parameters defining bounds and conditions for autonomous movements.
 */
struct MoveParams {
  float maxTranslationSpeed = 127.0f; /**< Maximum speed for translation */
  float maxRotationSpeed = 127.0f;    /**< Maximum speed for rotation */
  float minSpeed = 0.0f;              /**< Minimum speed (avoid stalling) */
  float exitRange =
      0.5f; /**< Acceptable position error to consider motion complete */
  float earlyExitRange =
      0.0f; /**< Distance to exit early before the movement is fully settled */
  uint32_t timeout = 5000; /**< Maximum time allowed for movement (ms) */
  bool async = true;       /**< Should the motion run asynchronously */
};

/**
 * @brief A mapping of error threshold to specific PID gains.
 */
struct ScheduledGain {
  float threshold; /**< The error bound for these gains */
  PIDGains
      gains; /**< The PID gains applied when error is below this threshold */
};

/**
 * @brief A snapshot of joystick and pose data for macro recording/playback.
 */
struct ReplayData {
  float forwards, sideways, rotation; /**< Raw joystick outputs */
  Pose pose;                          /**< Captured pose at the time */
};

/**
 * @brief Links a controller button to a lambda callback function.
 */
struct ControllerButton {
  pros::controller_digital_e_t button; /**< The specific controller button */
  std::function<void()> callback; /**< The callback to execute when pressed */
};

/**
 * @brief List of predefined controller button callbacks.
 */
inline std::vector<ControllerButton> controllerButtons{
    {pros::E_CONTROLLER_DIGITAL_UP, []() {}},
    {pros::E_CONTROLLER_DIGITAL_DOWN, []() {}},
    {pros::E_CONTROLLER_DIGITAL_LEFT, []() {}},
    {pros::E_CONTROLLER_DIGITAL_RIGHT, []() {}},
    {pros::E_CONTROLLER_DIGITAL_A, []() {}},
    {pros::E_CONTROLLER_DIGITAL_B, []() {}},
    {pros::E_CONTROLLER_DIGITAL_X, []() {}},
    {pros::E_CONTROLLER_DIGITAL_Y, []() {}},
    {pros::E_CONTROLLER_DIGITAL_L1, []() {}},
    {pros::E_CONTROLLER_DIGITAL_L2, []() {}},
    {pros::E_CONTROLLER_DIGITAL_R1, []() {}},
    {pros::E_CONTROLLER_DIGITAL_R2, []() {}},

};

/**
 * @brief Parses path point data from a file or string.
 *
 * @param input_source The source identifier (e.g. filename).
 * @param convertFromMeters True to scale the values from meters to inches.
 * @return std::vector<PathPoint> The parsed vector of path points.
 */
std::vector<PathPoint> parsePathData(const std::string &input_source,
                                     bool convertFromMeters = false);

/**
 * @brief A scheduler that interpolates and selects PID gains based on the
 * current error.
 */
class GainScheduler {
public:
  /**
   * @brief Add a scheduling step.
   *
   * @param threshold Max error for this step.
   * @param kP Proportional gain.
   * @param kI Integral gain.
   * @param kD Derivative gain.
   * @param slew Slew rate.
   */
  void addStep(float threshold, float kP, float kI, float kD,
               float slew = 0.0f);

  /**
   * @brief Compute the correct gains based on the current error distance.
   *
   * @param error The current positional or rotational error.
   * @return PIDGains Interpolated gains for this specific error level.
   */
  PIDGains getGains(float error) const;

  /**
   * @brief Clears the current schedules.
   */
  void clear();

private:
  std::vector<ScheduledGain> schedules;
};

/**
 * @brief Standard 1D Kalman filter for smoothing raw motor encoder values.
 */
class EncoderKalmanFilter {
public:
  /**
   * @brief Construct an Encoder Kalman Filter.
   *
   * @param process_noise Estimated variability of the process.
   * @param measurement_noise Noise characteristics of the encoder.
   */
  EncoderKalmanFilter(float process_noise = 0.077f,
                      float measurement_noise = 7.2f);

  /**
   * @brief Updates the filter with a new measurement.
   *
   * @param measured_position The raw encoder position.
   * @param dt Time elapsed since last update.
   * @return float The filtered/smoothed position.
   */
  float update(float measured_position, float dt);

private:
  Eigen::Vector2f x;
  Eigen::Matrix2f P, Q;
  Eigen::RowVector2f H;
  float R;
};

/**
 * @brief Specifies the heading behavior during path following.
 */
enum class HeadingMode { FollowPath, HoldAngle, CustomAngles };

/**
 * @brief The main Chassis controller class.
 *
 * Integrates odometry, PID tuning via scheduling, autonomous movement APIs,
 * driver control functions, obstacle avoidance, and path following algorithms.
 */
class Chassis {
public:
  /**
   * @brief Construct a new Chassis object.
   *
   * @param fl Front left motor.
   * @param fr Front right motor.
   * @param bl Back left motor.
   * @param br Back right motor.
   * @param imu_sensor IMU sensor for heading.
   * @param config The hardware configuration.
   */
  Chassis(pros::Motor fl, pros::Motor fr, pros::Motor bl, pros::Motor br,
          pros::Imu imu_sensor, ChassisConfig config);

  /**
   * @brief Calibrates sensors (e.g. resets IMU, zeros encoders).
   */
  void calibrate();

  /**
   * @brief Provide scheduled gains for the X-axis translation controller.
   *
   * @param steps A vector of ScheduledGain objects.
   */
  void setXGains(std::vector<ScheduledGain> steps);

  /**
   * @brief Provide scheduled gains for the Y-axis translation controller.
   *
   * @param steps A vector of ScheduledGain objects.
   */
  void setYGains(std::vector<ScheduledGain> steps);

  /**
   * @brief Provide scheduled gains for the rotational (Theta) controller.
   *
   * @param steps A vector of ScheduledGain objects.
   */
  void setThetaGains(std::vector<ScheduledGain> steps);

  /**
   * @brief Manually reset the robot's odometry position.
   *
   * @param x X coordinate.
   * @param y Y coordinate.
   * @param theta Heading angle in degrees.
   */
  void setPose(float x, float y, float theta = 0.0f);

  /**
   * @brief Reset the robot's odometry position to a Pose object.
   *
   * @param pose The Pose object representing the new position and heading.
   */
  void setPose(Pose pose);

  /**
   * @brief Get the current odometry pose of the robot.
   *
   * @param radians True to return heading in radians, false for degrees.
   * @return Pose The current pose.
   */
  Pose getPose(bool radians = false);

  /**
   * @brief Calculate the individual motor voltages for a holonomic movement
   * command.
   *
   * @param vx Desired X velocity.
   * @param vy Desired Y velocity.
   * @param vt Desired rotational velocity.
   * @return XDriveVoltages Structure containing the voltages for the 4 motors.
   */
  XDriveVoltages calculateHolonomic(float vx, float vy, float vt);

  /**
   * @brief Command the base motors with a set of voltages.
   *
   * @param v Struct containing voltages.
   */
  void setMotorVoltages(XDriveVoltages v);

  /**
   * @brief Instructs all motors to brake using their configured brake mode.
   */
  void brake();

  /**
   * @brief Main driver control mapping logic.
   *
   * Maps forward, sideways, and rotational input to motor commands. Allows for
   * field-centric or robot-centric control modes with customizable curves.
   *
   * @param forward Forward translation input (typically from a joystick).
   * @param sideways Sideways translation input.
   * @param rotation Rotational input.
   * @param drivecurves Optional curves to smooth inputs.
   * @param fieldCentric True if driver inputs map to absolute field axes
   * instead of relative to the robot heading.
   * @param headingOffset An offset to adjust the definition of "forward" for
   * field-centric mode.
   * @param correction Configuration for maintaining heading.
   */
  void driveControl(float forward, float sideways, float rotation,
                    DriveCurves drivecurves, bool fieldCentric,
                    float headingOffset = 0.0f,
                    DriveCorrection correction = {});

  /**
   * @brief Automatic, clockwise, or counter-clockwise rotation preference.
   */
  enum class CurveDirection { Auto, CW, CCW };

  /**
   * @brief Status of the Obstacle Avoidance subsystem.
   */
  enum class AvoidanceMode { Off, On };

  /**
   * @brief Autonomously follow a path defined by points.
   *
   * @param path A vector of path points to follow.
   * @param lookahead_inches Pure-pursuit lookahead distance.
   * @param params Movement parameters and constraints.
   * @param headingMode Defines how the heading behaves while traversing the
   * path.
   * @param holdAngleDeg A specific angle to hold if `HeadingMode::HoldAngle` is
   * selected.
   * @param reversed If true, the robot traverses the path in reverse.
   */
  void followPath(const std::vector<PathPoint> &path, float lookahead_inches,
                  MoveParams params = defaultParams,
                  HeadingMode headingMode = HeadingMode::FollowPath,
                  float holdAngleDeg = 0.0f, bool reversed = false);

  /**
   * @brief Autonomously turn the robot to face a specific heading.
   *
   * @param targetDeg The target heading in degrees.
   * @param params Movement parameters.
   */
  void turnToHeading(float targetDeg, MoveParams params = defaultParams);

  /**
   * @brief Autonomously turn the robot to face a specific coordinate.
   *
   * @param tx Target X coordinate.
   * @param ty Target Y coordinate.
   * @param params Movement parameters.
   */
  void turnToPoint(float tx, float ty, MoveParams params = defaultParams);

  /**
   * @brief Autonomously drive directly to a coordinate on the field.
   *
   * @param tx Target X coordinate.
   * @param ty Target Y coordinate.
   * @param params Movement parameters.
   * @param angleCorrection If true, actively adjusts heading to face the point
   * while moving.
   */
  void moveToPoint(float tx, float ty, MoveParams params = defaultParams,
                   bool angleCorrection = true);

  /**
   * @brief Move a relative distance using X and Y offsets.
   *
   * @param forward Distance to move forward.
   * @param sideways Distance to move sideways.
   * @param params Movement parameters.
   * @param holdHeading Actively maintain heading while translating.
   */
  void moveRelative(float forward, float sideways,
                    MoveParams params = defaultParams, bool holdHeading = true);

  /**
   * @brief Drive straight for a specified distance.
   *
   * @param distance Distance in inches.
   * @param params Movement parameters.
   * @param holdHeading Actively maintain heading while translating.
   */
  void moveDistance(float distance, MoveParams params = {},
                    bool holdHeading = true);

  /**
   * @brief Strafe for a specified distance.
   *
   * @param distance Distance in inches.
   * @param params Movement parameters.
   * @param holdHeading Actively maintain heading while translating.
   */
  void strafeDistance(float distance, MoveParams params = defaultParams,
                      bool holdHeading = true);

  /**
   * @brief Move the robot to an exact X, Y, and Theta simultaneously.
   *
   * @param tx Target X coordinate.
   * @param ty Target Y coordinate.
   * @param targetThetaDeg Target heading.
   * @param params Movement parameters.
   */
  void moveToPose(float tx, float ty, float targetThetaDeg,
                  MoveParams params = defaultParams);

  /**
   * @brief Drive in a circular arc until reaching a specified heading.
   *
   * @param targetThetaDeg The final heading of the arc.
   * @param radius The radius of the curved path.
   * @param params Movement parameters.
   * @param direction Whether to turn clockwise or counter-clockwise.
   */
  void curveCircle(float targetThetaDeg, float radius,
                   MoveParams params = defaultParams,
                   CurveDirection direction = CurveDirection::Auto);

  /**
   * @brief Blocks the current thread until the current motion is finished.
   */
  void waitUntilDone();

  /**
   * @brief Halts all running background motions in the motion handler queue.
   */
  void cancelAllMotions();

  MotionHandler motion; /**< Background task handler instance */

  /**
   * @brief Infinite loop function typically passed to a background PROS task to
   * update position continuously.
   */
  void odometryTask();

  /**
   * @brief Retrieves the total distance traveled during a motion segment.
   *
   * @param convertToMeters If true, returns value in meters rather than inches.
   * @return float Distance traveled.
   */
  float getDistanceTraveled(bool convertToMeters = false);

  /**
   * @brief Cancels the currently executing autonomous motion.
   */
  void cancelMotion();

  /**
   * @brief Wait until a certain linear distance has been covered during a
   * motion command.
   *
   * Useful for triggering events mid-way through a path or `moveToPoint` call.
   *
   * @param distance The distance threshold.
   */
  void waitUntil(float distance);

  /**
   * @brief Set specific internal Extended Kalman Filter variables manually.
   *
   * @param xProcessNoise Noise in the X dimension.
   * @param yProcessNoise Noise in the Y dimension.
   * @param thetaProcessNoise Noise in heading dimension.
   * @param measurementNoise IMU noise characteristic.
   */
  void setEKFGains(float xProcessNoise, float yProcessNoise,
                   float thetaProcessNoise, float measurementNoise);

  /**
   * @brief Toggles whether to calculate velocities via odometry
   * (computationally heavier).
   *
   * @param state True to enable.
   */
  void setVelocityCalculations(bool state);

  /**
   * @brief Detect sudden impact indicating a potential collision using
   * acceleration delta.
   *
   * @return true if collision detected.
   */
  bool detectCollision();

  /**
   * @brief Apply direct open-loop power to the drivetrain without PID.
   *
   * @param forward Linear power.
   * @param sideways Strafe power.
   * @param rotation Rotational power.
   */
  void openLoop(float forward, float sideways, float rotation);

  /**
   * @brief Add an obstacle to the avoidance system.
   *
   * @param x X coordinate.
   * @param y Y coordinate.
   * @param radius Avoidance radius.
   */
  void addObstacle(float x, float y, float radius);

  /**
   * @brief Remove an obstacle by index.
   *
   * @param index The index.
   */
  void removeObstacle(size_t index);

  /**
   * @brief Clears all current obstacles from memory.
   */
  void clearObstacles();

  /**
   * @brief Enables or disables the obstacle avoidance system globally.
   *
   * @param mode `AvoidanceMode::On` or `AvoidanceMode::Off`.
   */
  void setAvoidanceMode(AvoidanceMode mode);

  /**
   * @brief Tunes the obstacle avoidance distance thresholds.
   *
   * @param safetyMargin Padding to treat the obstacle as larger than its
   * radius.
   * @param clearance Required distance from the padded radius.
   */
  void setAvoidanceParams(float safetyMargin, float clearance);

  /**
   * @brief Configures Artificial Potential Field parameters.
   *
   * @param ka Attraction to goal.
   * @param kr Repulsion from obstacles.
   * @param influenceRadius Distance at which an obstacle begins pushing the
   * robot away.
   */
  void setPotentialFieldParams(float ka, float kr, float influenceRadius);

  /**
   * @brief Set robot dimensions used specifically for avoidance calculations.
   *
   * @param width Robot width.
   * @param height Robot height.
   */
  void setRobotDimensionsAvoidance(float width, float height);

  /**
   * @brief Utility: Convert Radians to Degrees.
   *
   * @param rad Radians.
   * @return float Degrees.
   */
  float radToDeg(float rad);

  /**
   * @brief Utility: Convert Degrees to Radians.
   *
   * @param deg Degrees.
   * @return float Radians.
   */
  float degToRad(float deg);

  /**
   * @brief Toggle the internal EKF feature entirely.
   *
   * @param state True to run EKF, false for raw basic odometry.
   */
  void setEKFstate(bool state);

  /**
   * @brief Register a tracking wheel with the odometry module.
   *
   * @param config The hardware configuration for the tracking wheel.
   */
  void addTrackingWheel(TrackingWheelConfig config);

  /**
   * @brief Unregisters all external tracking wheels.
   */
  void clearTrackingWheels();

  /**
   * @brief Generic templated loop to accept external calculation functions.
   *
   * @tparam F Lambda type.
   * @param updateFunction The function doing calculations and returning motor
   * voltages.
   * @param params Movement boundaries.
   * @param fieldCentric Treat update inputs as field centric.
   * @param headingOffset Global heading offset.
   * @param correction Correction params.
   */
  template <typename F>
  void move(F updateFunction, MoveParams params = defaultParams,
            bool fieldCentric = true, float headingOffset = 0.0f,
            DriveCorrection correction = {}) {}

  /**
   * @brief Sets global default move parameters.
   *
   * @param params Parameters to use as default.
   */
  void setMoveParams(MoveParams params);

  /**
   * @brief Determines which side to lock during a swing turn.
   */
  enum class SwingSide { Right, Left };

  /**
   * @brief Execute a swing turn where one side of the robot is held stationary.
   *
   * @param targetThetaDeg The target heading.
   * @param lockedSide Which side of the drive train to hold zero power to.
   * @param params Movement parameters.
   */
  void swingTurn(float targetThetaDeg, SwingSide lockedSide,
                 MoveParams params = defaultParams);

  /**
   * @brief Reads inputs from the standard PROS controller into the drive
   * control method.
   *
   * @param master The PROS Controller object.
   */
  void getControllerInput(pros::Controller master);

  /**
   * @brief Logs the driver's movements in real time to enable playback
   * features.
   *
   * @param master The controller to read from.
   * @param timeout_ms Interval to poll and log.
   */
  void logReplayData(pros::Controller master, int timeout_ms = 50);

  /**
   * @brief Stops driver motion logging.
   */
  void disableReplayDataLogging();

  /**
   * @brief Playback previously recorded driver inputs autonomously.
   *
   * @param data Recorded sequence of PathPoints or equivalent macro.
   * @param lookahead Follow lookahead parameter for smoothing playback
   * tracking.
   */
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

/**
 * @brief Helper utility to calculate the shortest path between two angles.
 *
 * @param target The desired angle in degrees.
 * @param current The current angle in degrees.
 * @return float The minimal error difference [-180, 180].
 */
inline float getAngleError(float target, float current) {
  return std::remainder(target - current, 360.0f);
}
