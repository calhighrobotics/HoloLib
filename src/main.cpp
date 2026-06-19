#include "main.h"
#include "Subsystems/modular_lift.h"
#include "chassis.h"
#include "distanceReset.h"
#include "pros/imu.hpp"

std::vector<LiftMotorConfig> lift_motor_configs = {{18, pros::MotorGear::blue},
                                                   {19, pros::MotorGear::blue}};

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor frontl(-3, pros::MotorGear::blue);
pros::Motor frontr(2, pros::MotorGear::blue);
pros::Motor backl(-4, pros::MotorGear::blue);
pros::Motor backr(1, pros::MotorGear::blue);
pros::Imu imu(10);

Chassis chassis(frontl, frontr, backl, backr, imu,
                {
                 .drivetrainWidth = 9.1,
                 .drivetrainLength = 10.25,
                 .wheelDiameter = 2.85,
                 .gearRatio = 0.5,
                 .kfEnabled = false});

LiftConfig my_lift_config = {.gear_ratio = 12.0f / 84.0f,
                             .arm_length = 15.0f,
                             .arm_mass_kg = 2.0f,
                             .payload_mass_kg = 0.0f,
                             .kG_base = 1750.0f / (2.0f * 9.81f),
                             .tolerance = 5.0f,
                             .K =
                                 Eigen::Matrix<float, 1, 2>{{2.9331f, 1.4557f}},
                             .spool_radius = 1.5f};

ModularLift my_lift(lift_motor_configs, LiftMechanism::CASCADE, my_lift_config);

void initialize() {
  pros::lcd::initialize();
  chassis.calibrate();
  chassis.setPose(0, 0, 0);
  chassis.setXGains({
      {36.0, {15, 0, 2.4}},
      {0.0, {25, 0, 0.5}},
  });
  chassis.setYGains({
      {36.0, {15, 0, 1.6}},
      {0.0, {20, 0, 1.5}},
  });
  chassis.setThetaGains({
      {90.0, {2.76411f, 0.0116046f, 0.0384008f}},
      {0, {3, 0, 0.04}}
  });
  chassis.setVelocityCalculations(true);
  pros::Task screen_task([&]() {
    while (true) {
      Pose pose = chassis.getPose(false);
      pros::lcd::print(0, "X: %.3f", pose.x);
      pros::lcd::print(1, "Y: %.3f", pose.y);
      pros::lcd::print(2, "Theta: %.3f", pose.theta);
      pros::lcd::print(3, "X Velocity: %.3f", pose.velocity.vx);
      pros::lcd::print(4, "Y Velocity: %.3f", pose.velocity.vy);
      pros::lcd::print(5, "Theta Velocity: %.3f", pose.velocity.w);
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
Run:
python tools/sim_auton.py
then open the file with the browser of your choice.
*/
void simulation() {
}

void autonomous() {
  
}

void testFunction()
{
  std::cout << "Function called" << std::endl;
}

void opcontrol() {
  chassis.setPose(0, 0, 90);
  chassis.setEKFstate(true);
  DriveCurve movement_curve{
      .curve_multipler = 1.01, .deadzone = 5, .minimum_output = 5};
  DriveCurve rotation_curve{
      .curve_multipler = 1.028, .deadzone = 5, .minimum_output = 5};
  int prev_forward = 0;
  int prev_sideways = 0;
  int prev_rotation = 0;
  chassis.logReplayData(master, 100);
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

    if (chassis.detectCollision()) {
      std::cout << "Collision Detected!" << std::endl;
    }
    chassis.driveControl(
        forward, sideways, rotation,
        {.movement = movement_curve, .rotation = rotation_curve}, true, 90,
        {.correctionOn = true, .kP = 0.15f, .kI = 0.01f, .kD = 0.01f});
    pros::delay(20);
  }
}
