#pragma once

#include <cstdint>

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
