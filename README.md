# HoloLib

**A control library for holonomic (X-Drive) VEX V5 robots.**

Website with documentation and useful information can be found here: https://calhighrobotics.github.io/HoloLib/

HoloLib runs on top of [PROS](https://pros.cs.purdue.edu/) and uses the
[Eigen](https://eigen.tuxfamily.org/) linear-algebra library for the math.
It handles the hard parts of driving an X-Drive well: knowing where the robot
is, getting it where you want it, and keeping it from running into things.

It is built for competition, but the goal of this page is simpler than that:
explain how each piece actually works so you can use it (and tune it) without
guessing.

---

## What's inside

HoloLib is five systems that work together. Each one solves a specific problem
you hit when you try to make an X-Drive do something precise:

| System | The problem it solves |
| --- | --- |
| [Pose EKF odometry](#1-pose-ekf-odometry) | Knowing where the robot is, even when wheels slip |
| [Obstacle avoidance](#2-obstacle-avoidance) | Getting around things in the way |
| [Gain scheduling](#3-gain-scheduling) | One PID tune can't do everything well |
| [Holonomic motion](#4-holonomic-motion) | Driving and following paths in any direction |
| [Driver replay](#5-driver-replay) | Turning a practice run into an autonomous routine |

The rest of this page walks through each one.

---

## 1. Pose EKF Odometry

To do anything in autonomous, the robot has to know where it is on the field.
The usual approach is to count wheel rotations and add them up. That works
until a wheel slips, the sensor reads a little noisy, or small errors pile up
over a long run. After a few seconds the robot's idea of "where I am" has
drifted away from reality.

HoloLib's `PoseEKF` fixes this by not trusting any single sensor. It blends
three sources at once and weights each one by how reliable it is:

- the IMU gyro, for fast and accurate rotation,
- dedicated tracking wheels (vertical and horizontal layouts are configurable),
- the chassis motor encoders.

An Extended Kalman Filter does this in two repeating steps. First it
**predicts** where the robot should be now, using the holonomic motion model.
Then it **corrects** that guess by comparing it against what the sensors
actually read, and nudges the estimate toward the more trustworthy ones.

![How the Pose EKF fuses sensors each cycle](docs/diagrams/ekf-loop.svg)

Think of it like figuring out where you are on a walk by combining a pedometer
that's slightly off with a compass that's slightly off. Neither is right on its
own, but together they beat either one alone. That's the whole idea, run a few
hundred times a second.

> **Heads up:** the Pose EKF is the most involved part of the library to tune.
> If you're not comfortable adjusting filter noise values, leave it off until
> you are. A badly tuned filter is worse than no filter.

## 2. Obstacle Avoidance

Sometimes the straight line to where you want to go runs through something you
can't drive over. HoloLib has two ways to deal with that, depending on how much
control you want.

**Recursive waypoint generation** is the planned approach. Before the robot
moves, it checks whether the straight path to the target crosses any obstacle
you've defined (each one a circle on the field). If it does, the library inserts
a waypoint that clears the obstacle, then checks the new path again, and keeps
going until the whole route is clear. It's like rerouting around a building on a
map before you start driving.

**Artificial Potential Fields (APF)** is the reactive approach. The target pulls
the robot toward it like a magnet, and every obstacle pushes the robot away. Add
those forces together each tick and you get a direction to drive. HoloLib also
adds a sideways "tangential" push so the robot glides around an obstacle instead
of stalling against it head-on.

![Attractive and repulsive forces steering the robot around an obstacle](docs/diagrams/apf-forces.svg)

> **Heads up:** this is meant to keep you from crashing into things you didn't
> plan for. For tight, repeatable autonomous routines you'll usually want the
> planned waypoints, and APF may need extra tuning before it behaves the way you
> want.

## 3. Gain Scheduling

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

## 4. Holonomic Motion

An X-Drive can move in any direction and rotate at the same time, which is what
makes it worth the trouble. HoloLib gives you that control directly in field
coordinates instead of making you think about individual wheels.

- **Field-centric drive:** push the stick toward the far wall and the robot goes
  toward the far wall, no matter which way it's currently facing. The library
  rotates your input by the robot's heading so "forward" always means the same
  direction on the field.
- **Path following:** follow a list of coordinates with a heading mode that fits
  the move, `FollowPath`, `HoldAngle`, or `CustomAngles`.
- **Precision moves:** relative moves, turn-to-point, turn-to-heading, circular
  arcs, and single-side swing turns are all built in.

![Field-centric driving keeps "forward" pointed the same way regardless of robot heading](docs/diagrams/field-centric.svg)

## 5. Driver Replay

A good driver run is itself a kind of autonomous routine, you just have to
capture it. HoloLib logs the robot's position and velocity while you drive a
practice run, then reconstructs that movement during the autonomous period.
It's a macro system for the whole robot.

> **Heads up:** right now the replay only logs positions and velocities. Joystick
> inputs aren't recorded, on purpose, to keep the log from overflowing the
> buffer.

## Project Structure

| Path | What it does |
| --- | --- |
| `include/chassis.h`, `src/robot/chassis.cpp` | Core chassis controller, odometry task, holonomic kinematics |
| `include/PID.h`, `src/robot/controllers/PID.cpp` | The custom PID controller |
| `include/motion_handler.h`, `src/robot/motion_handler.cpp` | Queues and manages async movements |
| `tools/sim_auton.py` | Python tool to visualize and debug autonomous routes in the browser |

---

## Quick Start

### Basic setup

Create a `Chassis` in your `main.cpp` by giving it the motors, the IMU, and the
physical dimensions of your robot:

```cpp
#include "main.h"

// Motors (ports: front-left, front-right, back-left, back-right)
pros::Motor front_left(1, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor front_right(2, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_left(3, pros::E_MOTOR_GEAR_GREEN, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor back_right(4, pros::E_MOTOR_GEAR_GREEN, true, pros::E_MOTOR_ENCODER_DEGREES);

// IMU
pros::Imu imu(5);

// Chassis dimensions
ChassisConfig config {
  .trackWidth = 12.5,       // distance between left/right wheels (inches)
  .drivetrainWidth = 14.0,
  .drivetrainLength = 14.0,
  .wheelDiameter = 3.25,    // wheel diameter (inches)
  .gearRatio = 1.0,         // direct drive
  .kfEnabled = true         // per-encoder Kalman filtering
};

Chassis chassis(front_left, front_right, back_left, back_right, imu, config);

void initialize() {
  // Calibrate the IMU and encoders
  chassis.calibrate();

  // Gain schedule: {error threshold, {kP, kI, kD, kF, slew}}
  chassis.setXGains({
      {12.0, {8.5, 0.1, 0.5, 0.0, 10.0}},
      {0.0,  {15.0, 0.2, 1.2, 0.0, 5.0}}
  });
  chassis.setYGains({
      {12.0, {8.5, 0.1, 0.5, 0.0, 10.0}},
      {0.0,  {15.0, 0.2, 1.2, 0.0, 5.0}}
  });
  chassis.setThetaGains({
      {90.0, {2.8, 0.01, 0.04}},
      {0.0,  {3.0, 0.0,  0.04}}
  });
}
```

### Driving in autonomous

Once the chassis is set up, autonomous moves read like instructions:

```cpp
void autonomous() {
  // Starting pose on the field: (x, y, theta in degrees)
  chassis.setPose(0.0, 0.0, 0.0);

  // Drive to a coordinate while turning to face 90 degrees
  chassis.moveToPose(24.0, 24.0, 90.0);

  // Turn to face a specific point
  chassis.turnToPoint(0.0, 0.0);

  // Arc turn
  chassis.curveCircle(180.0, 10.0);
}
```

### Field-centric driver control

In `opcontrol`, hand the controller inputs to the chassis. The last argument
turns on field-centric driving:

```cpp
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  DriveCurves curves {
    .movement = { .curve_multipler = 1.2f, .deadzone = 5.0f },
    .rotation = { .curve_multipler = 1.5f, .deadzone = 5.0f }
  };

  while (true) {
    chassis.driveControl(
      master.get_analog(ANALOG_LEFT_Y),   // forward / back
      master.get_analog(ANALOG_LEFT_X),   // strafe
      master.get_analog(ANALOG_RIGHT_X),  // rotate
      curves,
      true                                // field-centric on
    );
    pros::delay(10);
  }
}
```

---

## Visual Simulation

HoloLib ships with a Python tool for designing and checking autonomous paths
before you run them on a real robot:

```bash
python tools/sim_auton.py
```

Open the file it generates in a browser to step through the path and watch for
overshoot.

---

## License & Contributing

HoloLib is released under the Apache 2.0 license, so if you put it on your robot,
please follow the terms of that license.

This is a one-person project, so help and pull requests are very welcome.
