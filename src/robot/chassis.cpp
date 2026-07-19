#include "hololib/chassis.hpp"
#include "Eigen/Core"
#include "hololib/localization/odometry.hpp"
#include "hololib/util/PID.hpp"
#include "hololib/util/Timer.hpp"
#include "hololib/util/util.hpp"
#include <cmath>
#include <print>

namespace hololib {

/**
 *@brief Calibrates the chassis and calibrates all sensors.
 *@return void
 *@note This function should be called before using chassis motions (recommended
 * to be run in initialize)
 */
void Chassis::calibrate() {
    imu.reset(true);
    while (imu.is_calibrating()) {
        pros::delay(10);
    }
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
    std::println("Chassis Calibrated!");
}

void Chassis::xdrive(float vx, float vy, float omega, float theta) {
    Eigen::Vector3f state(vx, vy, omega);
    Eigen::Matrix3f rotationMatrix{
        {std::cos(theta),  std::sin(theta), 0}, //
        {-std::sin(theta), std::cos(theta), 0}, //
        {0,                0,               1}  //
    };
    Eigen::Matrix<float, 4, 3> inverseKinematicsMatrix{
        {1,  1, 1 }, //
        {-1, 1, -1}, //
        {1,  1, -1}, //
        {-1, 1, 1 }  //
    };
    Eigen::Vector4f motorVoltageVector = inverseKinematicsMatrix * rotationMatrix * state;
    int max = motorVoltageVector.cwiseAbs().maxCoeff();
    if (max > 12000) {
        motorVoltageVector *= 12000.0f / max;
    }
    frontLeft.move_voltage(motorVoltageVector(0));
    frontRight.move_voltage(motorVoltageVector(1));
    backRight.move_voltage(motorVoltageVector(2));
    backLeft.move_voltage(motorVoltageVector(3));
}

void Chassis::brake() {
    frontLeft.brake();
    frontRight.brake();
    backLeft.brake();
    backRight.brake();
}

void Chassis::driveControl(float forward,
                           float sideways,
                           float rotation,
                           DriveCurves drivecurves,
                           bool fieldCentric,
                           float headingOffset,
                           DriveCorrection correction) {
    if (!headingInitialized) {
        targetHeading = odom.getPose(false).theta;
        headingPID.setGains({correction.kP, correction.kI, correction.kD, 0.0, 0.0});
        headingInitialized = true;
    }

    auto applyCurve = [&](float x, const DriveCurve& c) -> float {
        if (std::abs(x) < c.deadzone)
            return 0.0f;

        float sign = (x >= 0.0f) ? 1.0f : -1.0f;
        float normalized = (std::abs(x) - c.deadzone) / (MAX_DRIVE_INPUT - c.deadzone);

        normalized = std::clamp(normalized, 0.0f, 1.0f);
        normalized = std::pow(normalized, c.curve_multipler);

        float output = normalized * MAX_DRIVE_INPUT;
        if (output > 0.0f && output < c.minimum_output) {
            output = c.minimum_output;
        }

        return output * sign;
    };

    // Apply the drive curves to joystick inputs
    forward = applyCurve(forward, drivecurves.movement);
    sideways = applyCurve(sideways, drivecurves.movement);
    float currentHeading = odom.getPose(false).theta;

    // Heading correction logic
    if (std::abs(rotation) >= drivecurves.rotation.deadzone) {
        rotation = applyCurve(rotation, drivecurves.rotation);
        receivedRotateInput = true;

    } else if (receivedRotateInput) {
        targetHeading = odom.getPose(false).theta;
        headingPID.reset();
        headingPID.setGains({correction.kP, correction.kI, correction.kD, 0.0, 0.0});
        receivedRotateInput = false;
        if (settled.isDone()) {
            settled.set(SETTLE_DELAY_MS);
        }
    } else if (settled.isDone()) {
        float angleError = getAngleError(targetHeading, currentHeading);

        if (std::abs(angleError) < 0.5f) {
            rotation = 0.0f;
        } else {
            rotation = (float)headingPID.update(angleError);
            rotation = std::clamp(rotation, -MAX_CORRECTION, MAX_CORRECTION);
        }
    }
    if (fieldCentric) {
        xdrive(forward, sideways, rotation, currentHeading);
    } else {
        xdrive(forward, sideways, rotation, 0);
    }
}

bool Chassis::detectCollision() {
    const int32_t TARGET_VOLTAGE_THRESHOLD = 3000;
    const uint32_t DEBOUNCE_TIME_MS = 250;

    if (last_collision_check_time == 0)
        last_collision_check_time = pros::millis();

    uint32_t now = pros::millis();
    uint32_t dt = now - last_collision_check_time;
    last_collision_check_time = now;
    auto is_wheel_slipping = [&](pros::Motor& motor) {
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

} // namespace hololib