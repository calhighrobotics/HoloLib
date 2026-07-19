#pragma once

#include "Eigen/Core" // IWYU pragma: export
#include "hololib/util/PoseEKF.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include <cmath>
#include <optional>
#include <vector>

namespace hololib {

class PoseEKF;

/**
 * @brief Defines the orientation of an unpowered tracking wheel.
 */
enum class TrackingWheelOrientation { HORIZONTAL, VERTICAL };

/**
 * @brief Configuration parameters for a tracking wheel.
 */
struct TrackingWheelConfig {
    int8_t port;                          /**< Sensor port number */
    TrackingWheelOrientation orientation; /**< Orientation (horizontal or vertical) */
    float xOffset;                        /**< Offset from tracking center on the X axis */
    float yOffset;                        /**< Offset from tracking center on the Y axis */
    float wheelDiameter;                  /**< Diameter of the tracking wheel */
    float gearRatio;                      /**< Gear ratio between the wheel and the sensor */
};
/**
 * @brief Core constants to configure the chassis dimensions and hardware.
 */
struct ChassisConfig {
    float trackWidth = 0;          /**< Distance between left and right wheels */
    float drivetrainWidth = 0.0f;  /**< Physical width of the drivetrain */
    float drivetrainLength = 0.0f; /**< Physical length of the drivetrain */
    float wheelDiameter;           /**< Diameter of the drive wheels */
    float gearRatio;               /**< Gear ratio (motor rotations / wheel rotations) */
};

struct VelocityComponents {
    float vx, vy, w;
};

struct Pose {
    float x = 0, y = 0, theta = 0;
    VelocityComponents velocity{0, 0, 0};
};

class EncoderEKFOdometry {

public:
    /**
     * @brief Constructor initializing the EKF Odometry system.
     */
    EncoderEKFOdometry(
        pros::Motor& fl, pros::Motor& fr, pros::Motor& bl, pros::Motor& br, pros::IMU& imu_sensor, ChassisConfig cfg);

    /**
     * @brief Main loop running the processing pipeline.
     * @param period Task execution period in milliseconds.
     */
    void update(uint32_t period_ms);

    /**
     * @brief Spawns the processing task thread if not already running.
     */
    void startTask(uint32_t period_ms = 10);

    /**
     * @brief Adds a tracking wheel configuration to the pipeline.
     */
    void addTrackingWheel(TrackingWheelConfig config);

    /**
     * @brief Clears tracking wheels and falls back to motor layout.
     */
    void clearTrackingWheels();

    /**
     * @brief Sets the pose of the robot.
     */
    void setPose(float x, float y, float theta);

    /**
     * @brief Sets the pose of the robot.
     */
    void setPose(Pose pose);

    /**
     * @brief Gets the pose of the robot.
     */
    Pose getPose(bool radians);

    void setVelocityCalculations(bool state);
    /**
     * @brief Enable or disable the internal Extended Kalman Filter.
     */
    void setKalmanFilterEnabled(bool enabled);

private:
    // Core hardware connections
    pros::Motor& frontLeft;
    pros::Motor& frontRight;
    pros::Motor& backLeft;
    pros::Motor& backRight;
    pros::IMU& imu;
    ChassisConfig config;

    // Previous iteration tracking metrics
    float prev_fl;
    float prev_fr;
    float prev_bl;
    float prev_br;
    float prev_heading;

    // Internal pipeline flags and state trackers
    bool useTrackingWheels = false;
    bool kfEnabled = true;
    float trackingWheelMeasNoise = 0.0005f;
    float measurementNoise = 0.001f; // Assumed baseline variant
    bool velocityCalculationsOn = true;
    float motionDistTraveled = 0.0f;

    // Tracking sensor vectors
    std::vector<TrackingWheelConfig> trackingWheelConfigs;
    std::vector<pros::Rotation> trackingWheelSensors; // Matches explicit construction type (.port)
    std::vector<float> prevTrackingPositions;

    // Mutex, underlying filter system, and runtime task handles
    pros::Mutex poseMutex;
    std::optional<pros::Task> m_task = std::nullopt;

    PoseEKF ekf;
    Pose currentPose;
};
} // namespace hololib