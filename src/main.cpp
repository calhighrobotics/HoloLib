#include "main.h"
#include "Subsystems/modular_lift.h"
#include "chassis.h"
#include "distanceReset.h"
#include "pros/imu.hpp"

pros::MotorGroup lift_motors({18, 19}, pros::MotorGear::blue);
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor frontl(2, pros::MotorGear::blue);
pros::Motor frontr(-3, pros::MotorGear::blue);
pros::Motor backl(1, pros::MotorGear::blue);
pros::Motor backr(-4, pros::MotorGear::blue);
pros::Imu imu(20);

Chassis chassis(frontl, frontr, backl, backr, imu,
                {.trackWidth = 13,
                 .drivetrainWidth = 12, //side to side
                 .drivetrainLength = 13.25, //front to back
                 .wheelDiameter = 4.05,
                 .gearRatio = 1,
                 .kfEnabled = true});

LiftConfig my_lift_config = {
                             .gear_ratio = 12.0f / 84.0f,
                             .kG = 1750.0f,
                             .tolerance = 5.0f,
                             .K =
                                 Eigen::Matrix<float, 1, 2>{{2.9331f, 1.4557f}},
                             };

ModularLift my_lift(&lift_motors, LiftMechanism::CASCADE, my_lift_config);

void initialize() {

  pros::lcd::initialize();
  chassis.calibrate();
  chassis.setPose(0, 0, 0);
  chassis.setXGains({
      {36.0, {15, 0, 2.4}},
      {0.0, {20, 0, 1}},
  });
  chassis.setYGains({
      {36.0, {15, 0, 1.2}},
      {0.0, {20, 0, 1}},
  });
  chassis.setThetaGains({
      {90.0, {2.15, 0.015, 0.155}},
      {0.0, {3.0, 0.02, 0.2}},
  });

  pros::Task screen_task([&]() {
    while (true) {
      Pose pose = chassis.getPose(false);
      pros::lcd::print(0, "X: %.3f", pose.x);
      pros::lcd::print(1, "Y: %.3f", pose.y);
      pros::lcd::print(2, "Theta: %.3f", pose.theta);
      pros::delay(50);
    }
  });
}

void disabled() {
  chassis.setPose(0, 0, 0);
  my_lift.cancel();
}

void competition_initialize() {}

/*
To run the simulation place your autonomous code inside of this function.

Then run these commands in the terminal:
      python3 tools/sim_auton.py
      open bin/auton_viewer.html
*/
void simulation()
{
  chassis.setPose(0, 0, 0);
  std::string path = R"(
  0, 0, 74.567
  0.137, 1.994, 71.083
  0.567, 3.943, 67.422
  1.333, 5.785, 64.275
  2.512, 7.393, 65.643
  3.996, 8.726, 69.516
  5.729, 9.718, 78.882
  7.583, 10.465, 83.046
  9.512, 10.988, 89.404
  11.477, 11.353, 91.713
  13.461, 11.605, 93.582
  15.454, 11.769, 95.115
  17.452, 11.867, 97.502
  19.451, 11.917, 98.484
  21.451, 11.938, 99.395
  23.451, 11.943, 99.723
  25.451, 11.945, 97.75
  27.451, 11.955, 95.116
  29.45, 11.987, 92.407
  31.449, 12.053, 89.617
  33.446, 12.17, 86.736
  35.436, 12.363, 83.757
  37.414, 12.659, 80.668
  39.367, 13.083, 77.457
  41.277, 13.674, 74.107
  43.085, 14.523, 70.601
  44.746, 15.631, 66.913
  46.111, 17.086, 63.723
  47.149, 18.789, 70.311
  47.764, 20.688, 75.499
  48.063, 22.662, 80.569
  48.106, 24.659, 87.651
  47.983, 26.654, 84.705
  47.701, 28.633, 81.653
  47.203, 30.568, 78.484
  46.476, 32.429, 76.205
  45.478, 34.158, 73.412
  44.222, 35.71, 73.987
  42.713, 37.019, 75.452
  41.018, 38.076, 79.712
  39.199, 38.903, 84.348
  37.296, 39.514, 86.467
  35.346, 39.955, 90.064
  33.369, 40.255, 91.538
  31.379, 40.446, 89.554
  29.381, 40.542, 86.671
  27.382, 40.57, 83.689
  25.382, 40.542, 80.597
  23.383, 40.466, 77.382
  21.386, 40.358, 74.027
  19.391, 40.225, 70.512
  17.396, 40.074, 66.813
  15.403, 39.911, 62.896
  13.41, 39.745, 58.719
  11.417, 39.581, 54.221
  9.423, 39.424, 49.315
  7.428, 39.282, 43.863
  5.432, 39.162, 37.629
  3.434, 39.071, 30.133
  1.435, 39.017, 20
  0, 39, 0
	)";

  chassis.curveCircle(180, 12, {}, Chassis::CurveDirection::CW);
}


void autonomous() {
  chassis.setPose(0, 0, 0);
  std::string path = R"(
0, 0, 74.567
0.137, 1.994, 71.083
0.567, 3.943, 67.422
1.333, 5.785, 64.275
2.512, 7.393, 65.643
3.996, 8.726, 69.516
5.729, 9.718, 78.882
7.583, 10.465, 83.046
9.512, 10.988, 89.404
11.477, 11.353, 91.713
13.461, 11.605, 93.582
15.454, 11.769, 95.115
17.452, 11.867, 97.502
19.451, 11.917, 98.484
21.451, 11.938, 99.395
23.451, 11.943, 99.723
25.451, 11.945, 97.75
27.451, 11.955, 95.116
29.45, 11.987, 92.407
31.449, 12.053, 89.617
33.446, 12.17, 86.736
35.436, 12.363, 83.757
37.414, 12.659, 80.668
39.367, 13.083, 77.457
41.277, 13.674, 74.107
43.085, 14.523, 70.601
44.746, 15.631, 66.913
46.111, 17.086, 63.723
47.149, 18.789, 70.311
47.764, 20.688, 75.499
48.063, 22.662, 80.569
48.106, 24.659, 87.651
47.983, 26.654, 84.705
47.701, 28.633, 81.653
47.203, 30.568, 78.484
46.476, 32.429, 76.205
45.478, 34.158, 73.412
44.222, 35.71, 73.987
42.713, 37.019, 75.452
41.018, 38.076, 79.712
39.199, 38.903, 84.348
37.296, 39.514, 86.467
35.346, 39.955, 90.064
33.369, 40.255, 91.538
31.379, 40.446, 89.554
29.381, 40.542, 86.671
27.382, 40.57, 83.689
25.382, 40.542, 80.597
23.383, 40.466, 77.382
21.386, 40.358, 74.027
19.391, 40.225, 70.512
17.396, 40.074, 66.813
15.403, 39.911, 62.896
13.41, 39.745, 58.719
11.417, 39.581, 54.221
9.423, 39.424, 49.315
7.428, 39.282, 43.863
5.432, 39.162, 37.629
3.434, 39.071, 30.133
1.435, 39.017, 20
0, 39, 0
0, 39, 0
	)";

  chassis.curveCircle(180, 12, {}, Chassis::CurveDirection::CW);
}

void opcontrol() {
  chassis.setPose(0, 0, 90);
  DriveCurve movement_curve{
      .curve_multipler = 1.01, .deadzone = 5, .minimum_output = 5};
  DriveCurve rotation_curve{
      .curve_multipler = 1.028, .deadzone = 5, .minimum_output = 5};
  int prev_forward = 0;
  int prev_sideways = 0;
  int prev_rotation = 0;
  while (true) {

    int forward = master.get_analog(ANALOG_LEFT_Y);
    int sideways = master.get_analog(ANALOG_LEFT_X);
    int rotation = master.get_analog(ANALOG_RIGHT_X);
    if (prev_forward != forward || prev_sideways != sideways ||
        prev_rotation != rotation) {
      prev_forward = forward;
      prev_sideways = sideways;
      prev_rotation = rotation;
    }

    chassis.driveControl(
        forward, sideways, rotation,
        {.movement = movement_curve, .rotation = rotation_curve}, true, 90, {.correctionOn = true, .kP = 0.07f, .kI = 0.0f, .kD = 0.0f});
    pros::delay(20);
  }
}
