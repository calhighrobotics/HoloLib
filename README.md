# HoloLib

![Stars badge](https://img.shields.io/github/stars/calhighrobotics/HoloLib?style=flat) ![License badge](https://img.shields.io/github/license/calhighrobotics/HoloLib)

> A feature-rich PROS template, with first-class holonomic drivetrain support.

HoloLib is a highly extensible, feature-rich [PROS](https://pros.cs.purdue.edu/) template, using the [Eigen](https://eigen.tuxfamily.org/) linear-algebra library for vectorization.
It handles the hard parts of programming an X-Drive well: knowing where the robot
is, getting it where you want it, and keeping it from running into things.

## Quick start

### Basic setup

Create the chassis, odometry, and tuning objects in your `main.cpp` by giving
them the motors, the IMU, and the physical dimensions of your robot:

```cpp
#include "main.h"
#include "hololib/chassis.hpp"
#include "hololib/config.hpp"
#include "hololib/localization/odometry.hpp"
#include "hololib/util/GainScheduler.hpp"

// Motors (ports: front-left, front-right, back-left, back-right)
int frontLPort = -3;
int frontRPort = 2;
int backLPort = -4;
int backRPort = 1;

// IMU
int imuPort = 10;

// Instantiate your motors and sensors into the global config
pros::Motor frontLeft = pros::Motor(frontLPort, pros::MotorGear::blue);
pros::Motor frontRight = pros::Motor(frontRPort, pros::MotorGear::blue);
pros::Motor backLeft = pros::Motor(backLPort, pros::MotorGear::blue);
pros::Motor backRight = pros::Motor(backRPort, pros::MotorGear::blue);
pros::Imu imu = pros::Imu(imuPort);

// Define chassis constants
hololib::ChassisConfig chassis_config = {
  .drivetrainWidth = 9.1,
  .drivetrainLength = 10.25,
  .wheelDiameter = 3.25,
  .gearRatio = 0.5
};

// Instantiate odometry, chassis, and gain scheduler objects
hololib::EncoderEKFOdometry odom = hololib::EncoderEKFOdometry(frontLeft, frontRight, backLeft, backRight, imu, chassis_config);
hololib::Chassis chassis = hololib::Chassis(frontLeft, frontRight, backLeft, backRight, imu, odom);
hololib::GainScheduler xSched = hololib::GainScheduler();
hololib::GainScheduler ySched = hololib::GainScheduler();
hololib::GainScheduler thetaSched = hololib::GainScheduler();

void initialize() {
  // Calibrate the chassis and start odometry
  chassis.calibrate();
  odom.startTask();

  // Gain schedule: {error threshold, {kP, kI, kD, kF, slew}}
  xSched.setGains({
      {36.0, {15, 0, 2.4}},
      {0.0,  {25, 0, 0.5}}
  });
  ySched.setGains({
      {36.0, {15, 0, 1.6}},
      {0.0,  {20, 0, 1.5}}
  });
  thetaSched.setGains({
      {90.0, {2.76411f, 0.0116046f, 0.0384008f}},
      {0.0,  {3.0f, 0.0f, 0.04f}}
  });

  odom.setVelocityCalculations(true);
}
```

### Driving in autonomous

Once the chassis is set up, autonomous moves read like instructions:

```cpp
void autonomous() {
  // Starting pose on the field: (x, y, theta in degrees)
  chassis.setPose(0.0, 0.0, 0.0);

  // Drive a sequence of motions asynchronously
  chassisAsync(hololib::moveToPoint(24, 24));
  chassisAsync(hololib::turnToPoint(12, 12));

  // Motion chaining
  chassisAsync(hololib::moveToPoint(12, 12, {.minSpeed = 35, .earlyExitRange = 7, .timeout = 2000}));
  chassisAsync(hololib::moveToPoint(-12, 36));

  // Drive to a coordinate while turning to face 180 degrees
  chassisAsync(hololib::moveToPose(0, 0, 180));

  // Relative motions
  chassisAsync(hololib::moveRelative(20, 10));
  chassisAsync(hololib::strafeDistance(10));
}
```

### Field-centric driver control

In `opcontrol`, hand the controller inputs to the chassis. This example matches
the driver-control in `main.cpp`:

```cpp
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  // Customize drive curves
  hololib::Chassis::DriveCurve movement_curve{.curve_multipler = 1.01, .deadzone = 5, .minimum_output = 5};
  hololib::Chassis::DriveCurve rotation_curve{.curve_multipler = 1.028, .deadzone = 5, .minimum_output = 5};

  while (true) {
    int forward = master.get_analog(ANALOG_LEFT_Y); // forward / back
    int sideways = master.get_analog(ANALOG_LEFT_X); // strafe left / right
    int rotation = master.get_analog(ANALOG_RIGHT_X); // rotate independenly

    chassis.driveControl(
      forward,
      sideways,
      rotation,
      {.movement = movement_curve, .rotation = rotation_curve},
      true, // turn field-centric driving on
      90, // define inital heading offset
      {.correctionOn = true, .kP = 0.15f, .kI = 0.01f, .kD = 0.01f} // active heading correction
    );

    pros::delay(20);
  }
}
```

---

## What's inside

HoloLib is five systems that work together. Each one solves a specific problem
you hit when you try to make an X-Drive do something precise:

| System | The problem it solves |
| --- | --- |
| [Encoder EKF odometry](#1-encoder-ekf-odometry) | Knowing where the robot is, even when wheels slip |
| [Obstacle avoidance](#2-obstacle-avoidance) | Getting around things in the way |
| [Gain scheduling](#3-gain-scheduling) | One PID tune can't do everything well |
| [Holonomic motion](#4-holonomic-motion) | Driving and following paths in any direction |
| [Driver replay](#5-driver-replay) | Turning a practice run into an autonomous routine |

The rest of this page walks through each one. For the hands-on side, function by
function, with code and tuning advice, see the **[Usage Guide](docs/usage_guide.md)**.

---

## 1. Encoder EKF odometry

To do anything in autonomous, the robot has to know where it is on the field.
The usual approach is to count wheel rotations and add them up. That works
until a wheel slips, the sensor reads a little noisy, or small errors pile up
over a long run. After a few seconds the robot's idea of "where I am" has
drifted away from reality.

HoloLib's odometry layer, `EncoderEKFOdometry`, fixes this by not trusting any
single sensor. It blends drive motor encoders, optional tracking wheels, and
the IMU at once, while `PoseEKF` handles the actual Kalman filter math:

- the IMU gyro, for fast and accurate rotation,
- dedicated tracking wheels (vertical and horizontal layouts are configurable),
- the chassis motor encoders.

An Extended Kalman Filter does this in two repeating steps. First it
**predicts** where the robot should be now, using the holonomic motion model.
Then it **corrects** that guess by comparing it against what the sensors
actually read, and nudges the estimate toward the more trustworthy ones.

![How the encoder EKF fuses sensors each cycle](docs/diagrams/ekf-loop.svg)

Think of it like figuring out where you are on a walk by combining a pedometer
that's slightly off with a compass that's slightly off. Neither is right on its
own, but together they beat either one alone. That's the whole idea, run a few
hundred times a second.

> **Heads up:** the EKF stack is the most involved part of the library to
> tune. If you're not comfortable adjusting filter noise values, leave it off
> until you are. A badly tuned filter is worse than no filter.

## 2. Obstacle avoidance

Sometimes the straight line to where you want to go runs through something you
can't drive over. HoloLib has two ways to deal with that, depending on how much
control you want.

**Recursive waypoint generation** is the planned approach. Before the robot
moves, `ObstacleManager` checks whether the straight path to the target crosses
any obstacle you've defined (each one a circle on the field). If it does, the
library inserts a waypoint that clears the obstacle, then checks the new path
again, and keeps going until the whole route is clear. It's like rerouting
around a building on a map before you start driving.

**Artificial Potential Fields (APF)** is the reactive approach. The target pulls
the robot toward it like a magnet, and every obstacle pushes the robot away.
Add those forces together each tick and you get a direction to drive. HoloLib
also adds a sideways "tangential" push so the robot glides around an obstacle
instead of stalling against it head-on.

![Attractive and repulsive forces steering the robot around an obstacle](docs/diagrams/apf-forces.svg)

> **Heads up:** this is meant to keep you from crashing into things you didn't
> plan for. For tight, repeatable autonomous routines you'll usually want the
> planned waypoints, and APF may need extra tuning before it behaves the way you
> want.

## 3. Gain scheduling

A PID controller has gains (`kP`, `kI`, `kD`) that decide how hard it pushes to
close the gap between where the robot is and where it should be. The trouble is
that one set of gains can't be good at everything. Tune it to sprint across the
field and it overshoots on small moves. Tune it to settle gently on small moves
and it crawls across long ones.

HoloLib's `GainScheduler` lets you define several sets of gains, each tied to a
range of error (how far you still are from the target). As the robot closes in,
the controller transitions between those sets: aggressive gains when there's a
lot of distance to cover, gentle gains as it settles in on the target. The slew
rate (how fast the output is allowed to change) scales the same way.

![Different PID gains take over as the robot gets closer to the target](docs/diagrams/gain-schedule.svg)

The result is a single movement that accelerates hard at the start and settles
without overshooting at the end, which is hard to get from a fixed tune.

## 4. Holonomic motion

An X-Drive can move in any direction and rotate at the same time, which is what
makes it worth the trouble. HoloLib gives you that control directly through the
`hololib::motions` API instead of making you think about individual wheels.

- **Field-centric drive:** push the stick toward the far wall and the robot goes
  toward the far wall, no matter which way it's currently facing. The library
  rotates your input by the robot's heading so "forward" always means the same
  direction on the field.
- **Path following:** follow a list of coordinates with a heading mode that
  fits the move, `FollowPath`, `HoldAngle`, or `CustomAngles`.
- **Precision moves:** relative moves, turn-to-point, turn-to-heading, circular
  arcs, and single-side swing turns are all built in.

![Field-centric driving keeps "forward" pointed the same way regardless of robot heading](docs/diagrams/field-centric.svg)

## 5. Driver replay

A good driver run is itself a kind of autonomous routine, you just have to
capture it. `DriverReplay` logs the robot's position and velocity while you
drive a practice run, then reconstructs that movement during the autonomous
period. It's a macro system for the whole robot.

> **Heads up:** right now the replay only logs positions and velocities. Joystick
> inputs aren't recorded, on purpose, to keep the log from overflowing the
> buffer.

## Project Structure

**Chassis & motion**

| Path | What it does |
| --- | --- |
| `include/hololib/chassis.hpp` | Public interface for the chassis controller |
| `src/robot/chassis.cpp` | Core chassis controller and holonomic kinematics |
| `include/hololib/localization/odometry.hpp`, `src/localization/odometry.cpp` | Encoder EKF odometry and pose tracking |
| `include/hololib/motions/motions.hpp`, `src/motions/*.cpp` | Path-following and point-to-point movement routines |
| `include/hololib/motions/motion_handler.hpp`, `src/motions/MotionHandler.cpp` | Queues and manages async movements |
| `include/hololib/motions/motion_cancel_helper.hpp`, `src/motions/MotionCancelHelper.cpp` | Shared cancellation helpers for async motion |

**Control & estimation**

| Path | What it does |
| --- | --- |
| `include/hololib/util/PID.hpp`, `src/motions/controllers/PID.cpp` | The custom PID controller |
| `include/hololib/util/GainScheduler.hpp`, `src/util/GainScheduler.cpp` | Error-threshold-based PID gain scheduling |
| `include/hololib/util/PoseEKF.hpp`, `src/util/PoseEKF.cpp` | Extended Kalman Filter for robot pose estimation |
| `include/hololib/localization/odometry.hpp` | Tracking-wheel configuration, chassis config, and pose access |
| `include/hololib/util/Timer.hpp`, `src/util/Timer.cpp` | Small timing helper used by motion control |

**Localization & paths**

| Path | What it does |
| --- | --- |
| `include/hololib/localization/odometry.hpp` | Pose, velocity, chassis config, and tracking wheel types |
| `include/hololib/localization/distanceReset.hpp`, `src/localization/distanceReset.cpp` | Distance-sensor-based odometry reset |
| `include/hololib/util/obstacle_manager.hpp`, `src/util/obstacle_manager.cpp` | Field obstacle representation and avoidance |
| `include/hololib/util/replay.hpp`, `src/util/replay.cpp` | Driver replay capture and playback |

**Subsystems & utilities**

| Path | What it does |
| --- | --- |
| `include/hololib/config.hpp` | Chassis dimension and hardware constants |
| `include/hololib/util/util.hpp`, `src/util/util.cpp` | Small shared utility helpers |
| `include/hololib/util/modular_lift.hpp`, `src/motions/controllers/modular_lift.cpp` | Configurable lift subsystem with background control task |
| `tools/sim_auton.py` | Python tool to visualize and debug autonomous routes in the browser |

---

## Visual simulation

HoloLib ships with a Python tool for designing and checking autonomous paths
before you run them on a real robot:

```bash
python tools/sim_auton.py
```

Open the file it generates in a browser to step through the path and watch for
overshoot.

---

## Contributing

This is a one-person project, so contributions, issses, and pull requests are welcome!

## License, attributions, & third-party licenses

HoloLib is released under the Apache 2.0 license. If your robot uses this template,
please abide by the terms of the [license](https://github.com/calhighrobotics/HoloLib/blob/main/LICENSE).

This project includes software developed by the [LemLib](https://github.com/LemLib/LemLib) project, licensed under the [MIT License](https://github.com/LemLib/LemLib/blob/master/LICENSE). The copyright of those helper components belong to the LemLib project. Their work inspired HoloLib, so please check them out!

View the terms of the [Eigen License](https://github.com/bolderflight/eigen/blob/main/LICENSE.md).

The PROS operating system is licensed under the [Mozilla Public License Version 2.0](https://github.com/purduesigbots/pros/blob/develop-pros-4/LICENSE).
