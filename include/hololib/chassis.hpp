#pragma once
#include "hololib/localization/odometry.hpp"
#include "hololib/util/PID.hpp"
#include "hololib/util/Timer.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"

#define chassisAsync(f) hololib::motion_handler::move([&] { f; });

namespace hololib {

class Chassis {
public:
    Chassis(pros::Motor& frontLeft,
            pros::Motor& frontRight,
            pros::Motor& backLeft,
            pros::Motor& backRight,
            pros::IMU& imu,
            EncoderEKFOdometry& odom)
        : frontLeft(frontLeft), frontRight(frontRight), backLeft(backLeft), backRight(backRight),
          imu(imu), odom(odom), headingPID(PID{0, 0, 0, 0}), settled(Timer{0}), targetHeading(0.0f),
          headingInitialized(false), receivedRotateInput(false), MAX_DRIVE_INPUT(127.0f),
          SETTLE_DELAY_MS(150), MAX_CORRECTION(40.0f) {};

    /**
     * @brief Represents a simple deadzone-minimum output exponential or linear
     * drive curve.
     */
    struct DriveCurve {
        float curve_multipler = 1.0f; /**< Curvature scale */
        float deadzone = 0.0f;        /**< Deadzone before response begins */
        float minimum_output = 0.0f;  /**< Minimum voltage output once deadzone is surpassed */
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
        bool correctionOn = true; /**< True to actively maintain heading when
                                     rotation joystick is released */
        float kP = 1.0f;          /**< Proportional gain for heading hold */
        float kI = 0.01f;         /**< Integral gain for heading hold */
        float kD = 0.1f;          /**< Derivative gain for heading hold */
    };
    void calibrate();

    void xdrive(float vx, float vy, float omega, float theta);

    void driveControl(float forward,
                      float sideways,
                      float rotation,
                      DriveCurves drivecurves,
                      bool fieldCentric,
                      float headingOffset,
                      DriveCorrection correction);

    void brake();

    bool detectCollision();

private:
    pros::Motor& frontLeft;
    pros::Motor& frontRight;
    pros::Motor& backLeft;
    pros::Motor& backRight;
    pros::IMU& imu;

    EncoderEKFOdometry& odom;

    // Driver control global variables
    const float MAX_DRIVE_INPUT;
    const uint32_t SETTLE_DELAY_MS;
    const float MAX_CORRECTION;
    PID headingPID;
    Timer settled;
    float targetHeading;
    bool headingInitialized;
    bool receivedRotateInput;

    // Collision detection variables
    uint32_t last_collision_check_time = 0;
    uint32_t stall_accumulator_ms = 0;
};
} // namespace hololib