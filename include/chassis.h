#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "PID.h"
#include "api.h"
#include "hololib/config.h"
#include "hololib/encoder_filter.h"
#include "hololib/gain_scheduler.h"
#include "hololib/obstacle.h"
#include "hololib/path.h"
#include "hololib/pose.h"
#include "hololib/pose_ekf.h"
#include "hololib/replay.h"
#include "hololib/tracking_wheel.h"
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
