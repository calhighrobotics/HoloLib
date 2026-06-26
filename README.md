# HoloLib

**HoloLib** is an advanced, high-performance C++ control library for holonomic (X-Drive) VEX V5 robots, built on top of the **PROS operating system** and powered by the **Eigen linear algebra library**. 

Designed for competitive excellence, HoloLib combines state-of-the-art sensor fusion, reactive obstacle avoidance, and precise motion profiling to give teams unmatched reliability in both autonomous and driver-control periods.

---

### 1. Extended Kalman Filter (EKF) Odometry
Traditional odometry suffers from wheel slippage, sensor noise, and cumulative drift. HoloLib solves this by implementing a custom **Pose EKF** (`PoseEKF`) that dynamically fuses:
- High-frequency IMU gyroscope rotation data.
- Dedicated tracking wheels (supporting customizable vertical and horizontal layouts).
- Chassis motor encoders.
The EKF predicts state updates through holonomic kinematics and corrects them using sensor measurement models in real-time, keeping your coordinate tracking accurate.
**It is not recommended to use the Pose EKF if you do not understand how to tune or manage it.**

### 2. Intelligent Obstacle Avoidance
Navigate around preplanned obstacles.  HoloLib features two advanced path-planning and obstacle avoidance modes:
- **Recursive Waypoint Generation:** Automatically detects path intersections with predefined circular obstacles and recursively calculates safe bypass waypoints.
- **Artificial Potential Fields (APF):** Models the target coordinate as an attractive force and obstacles as repulsive forces, generating real time bypass vectors. It features tangential bypass force scaling to elegantly "glide" around obstacles without stopping.
**This feature is mainly used as a system for avoiding unplanned crashes, may require more tuning for dynamic autonomous routines**

### 3. Dynamic Gain Scheduling
No single PID tuning configuration is perfect for both long-distance travels and micro adjustments. HoloLib's `GainScheduler` allows you to define multiple steps of PID gains. The chassis automatically scales and transitions its C++ PID controllers (`kP`, `kI`, `kD`, and `slew` rate) depending on the error magnitude, ensuring high acceleration at distance and zero overshoot settling at the target.

This allows you to have more accurate control over a larger set of movements, as well as during movements.

### 4. Holonomic Kinematics & Advanced Motion
Control your X-Drive robot easily with native coordinate transformations:
- **Field-Centric Drive:** Drive relative to the field coordinates, ignoring the robot's current orientation.
- **Complex Path Following:** Follow custom coordinate trajectories with selectable heading modes (`FollowPath`, `HoldAngle`, `CustomAngles`).
- **Precision Motions:** Built-in support for relative movements, turn-to-point, turn-to-heading, circle curves, and single-side swing turns.

### 5. Driver Replay System
Log driver actions during practice runs and replay them during the autonomous period. The logger EKF positions, while the replay engine reconstructs driver movements, giving you a repeatable autonomous macro system.

**Currently the replay system only logs positions and velocities, joystick mapping is not enabled as to avoid buffer overflow**
---

## Project Structure

- `include/chassis.h` & `src/robot/chassis.cpp`: The core chassis controller, odometry task, and holonomic kinematics.
- `include/PID.h` & `src/robot/controllers/PID.cpp`: Implementation of the custom PID controller.
- `include/motion_handler.h` & `src/robot/motion_handler.cpp`: Enqueues and manages async robot movements.
- `tools/sim_auton.py`: A Python simulation tool to visualize and debug autonomous routes in your web browser.

---

## Quick Start

### Basic Setup
Initialize the `Chassis` class in your `main.cpp` by defining the motors, IMU sensor, and physical configurations:

```cpp
#include "main.h"

// Define Motors (Ports: Front-Left, Front-Right, Back-Left, Back-Right)
pros::Motor front_left(1, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor front_right(2, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_left(3, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_right(4, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);

// Define IMU
pros::Imu imu(5);

// Configure Chassis Dimensions
ChassisConfig config {
  .trackWidth = 12.5,       // Width between left/right wheels (inches)
  .drivetrainWidth = 14.0,
  .drivetrainLength = 14.0,
  .wheelDiameter = 3.25,    // Wheel diameter (inches)
  .gearRatio = 1.0,         // Direct drive
  .kfEnabled = true         // Use internal Kalman Filters for encoders
};

Chassis chassis(front_left, front_right, back_left, back_right, imu, config);

void initialize() {
  // Calibrate IMU and encoders
  chassis.calibrate();

  // Configure PID Gains
  chassis.setXGains({
      {12.0, {8.5, 0.1, 0.5, 0.0, 10.0}}, // Error threshold, {kP, kI, kD, kF, slew}
      {0.0, {15.0, 0.2, 1.2, 0.0, 5.0}}
  });
  chassis.setYGains({
      {12.0, {8.5, 0.1, 0.5, 0.0, 10.0}},
      {0.0, {15.0, 0.2, 1.2, 0.0, 5.0}}
  });
  chassis.setThetaGains({
      {90.0, {2.8, 0.01, 0.04}},
      {0.0, {3.0, 0.0, 0.04}}
  });
}
```

### Navigating in Autonomous
Instruct the robot to perform complex, field-centric operations during the autonomous period:

```cpp
void autonomous() {
  // Set starting pose on the field: (x, y, theta_degrees)
  chassis.setPose(0.0, 0.0, 0.0);

  // Move to a target coordinate while automatically orienting to face 90 degrees
  chassis.moveToPose(24.0, 24.0, 90.0);

  // Smoothly turn to face a specific point
  chassis.turnToPoint(0.0, 0.0);

  // Perform a circular arc turn
  chassis.curveCircle(180.0, 10.0);
}
```

### Driver Control with Field-Centric Driving
In your `opcontrol` loop, pass controller inputs to the chassis with field-centric scaling and automated heading correction:

```cpp
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  DriveCurves curves {
    .movement = { .curve_multipler = 1.2f, .deadzone = 5.0f },
    .rotation = { .curve_multipler = 1.5f, .deadzone = 5.0f }
  };

  while (true) {
    chassis.driveControl(
      master.get_analog(ANALOG_LEFT_Y),   // Forward/backward
      master.get_analog(ANALOG_LEFT_X),   // Strafe left/right
      master.get_analog(ANALOG_RIGHT_X),  // Yaw rotation
      curves,
      true                                // Enable Field-Centric drive
    );
    pros::delay(10);
  }
}
```

---

## Visual Simulation
HoloLib comes with a built-in Python autonomous path simulation tool. To design and verify your paths before running them on physical hardware:
1. Run the simulator script:
   ```bash
   python tools/sim_auton.py
   ```
2. Open the generated interactive file in your web browser of choice to inspect the visual path traversal and check for overshoot.

If you are going to use HoloLib on your robot, please make sure to abide by the rules of the Apache 2.0 license.
Also this is a one man project, so help and pull requests are highly encouraged.

