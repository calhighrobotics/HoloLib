#include "main.h"
#include "Subsystems/modular_lift.h"
#include "chassis.h"
#include "distanceReset.h"
#include "pros/imu.hpp"

// Initialize motor ports (negative for reversing motor)
int frontLPort = -3;
int frontRPort = 2;
int backLPort = -4;
int backRPort = 1;

// Initialize IMU port
int imuPort = 10;

std::vector<LiftMotorConfig> lift_motor_configs = {{18, pros::MotorGear::blue},
                                                   {19, pros::MotorGear::blue}};

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor frontl(frontLPort, pros::MotorGear::blue);
pros::Motor frontr(frontRPort, pros::MotorGear::blue);
pros::Motor backl(backLPort, pros::MotorGear::blue);
pros::Motor backr(backRPort, pros::MotorGear::blue);
pros::Imu imu(imuPort);

// Initialize Chassis
Chassis
    chassis(frontl, frontr, backl, backr, imu,
            {.drivetrainWidth = 9.1,    // width from wheel to wheel
             .drivetrainLength = 10.25, // length from wheel to wheel
             .wheelDiameter = 3.25, // wheel diameter in inches (should be tuned
                                    // to EFFECTIVE wheel diameter)
             .gearRatio = 0.5,     // gear ratio of the drivetrain
             .kfEnabled = false}); // Enables ekf, only use if you know how to
                                   // tune the process and measurement noise.

// Initialize lift motor configs
LiftConfig my_lift_config = {
    .gear_ratio = 12.0f / 84.0f,
    .arm_length = 15.0f,
    .arm_mass_kg = 2.0f,
    .payload_mass_kg = 0.0f,
    .kG_base = 1750.0f / (2.0f * 9.81f),
    .tolerance = 5.0f,
    .K =
        Eigen::Matrix<float, 1, 2>{
            {2.9331f, 1.4557f}}, // Initialize k gain matrix for lqr
    .spool_radius = 1.5f};

ModularLift my_lift(lift_motor_configs, LiftMechanism::CASCADE,
                    my_lift_config); // initialize lift type and config (in
                                     // beta, uses lqr control)

void initialize() {
  pros::lcd::initialize();

  // Calibrate the chassis
  chassis.calibrate();
  chassis.setPose(0, 0, 0);

  // Set PID gains for chassis
  chassis.setXGains({
      {36.0, {15, 0, 2.4}},
      {0.0, {25, 0, 0.5}},
  });
  chassis.setYGains({
      {36.0, {15, 0, 1.6}},
      {0.0, {20, 0, 1.5}},
  });
  chassis.setThetaGains(
      {{90.0, {2.76411f, 0.0116046f, 0.0384008f}}, {0, {3, 0, 0.04}}});

  // Basically allows you to see the velocity of the chassis (in/s) (helpful for
  // making custom motions)
  chassis.setVelocityCalculations(true);

  // LCD screen task to display chassis data
  pros::Task screen_task([&]() {
    while (true) {
      Pose pose =
          chassis.getPose(false); // false means degrees, true means radians
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
void simulation() {}

void autonomous() {
  // Example autonomous routine
  chassis.setPose(0, 0, 0);
  chassis.moveToPoint(24, 24, {});
  chassis.turnToPoint(12, 12, {});

  // Motion Chaining
  chassis.moveToPoint(12, 12,
                      {.minSpeed = 35, .earlyExitRange = 7, .timeout = 2000});
  chassis.moveToPose(0, 0, 180, {});

  // Relative Motions
  chassis.moveRelative(20, 10);
  chassis.strafeDistance(10);

  // Driver replay, motion cancelling and other features can be learned from the
  // docs

  // If you want to use the object avoidance (ex.)
  chassis.addObstacle(0, 0,
                      5); // initializes an obstacle at 0,0 with a radius of 5in
  chassis.setAvoidanceParams(2.0f, 4.0f); // set the avoidance parameters
  chassis.setPotentialFieldParams(1.0f, 100.0f,
                                  24.0f); // set the potential field parameters
  chassis.setRobotDimensionsAvoidance(9.1f, 10.25f); // set the robot dimensions
  chassis.setAvoidanceMode(Chassis::AvoidanceMode::On); // set the avoidance
                                                        // mode
}

void testFunction() { std::cout << "Function called" << std::endl; }

void opcontrol() {
  chassis.setPose(0, 0, 90);

  chassis.setEKFstate(false); // turn off EKF for driver control

  // Example drive curves
  DriveCurve movement_curve{
      .curve_multipler = 1.01, .deadzone = 5, .minimum_output = 5};
  DriveCurve rotation_curve{
      .curve_multipler = 1.028, .deadzone = 5, .minimum_output = 5};

  chassis.logReplayData(master, 100); // allows logging for driver replay

  while (true) {

    // Recieve inputs from controller
    int forward = master.get_analog(ANALOG_LEFT_Y);
    int sideways = master.get_analog(ANALOG_LEFT_X);
    int rotation = master.get_analog(ANALOG_RIGHT_X);

    if (chassis.detectCollision()) { // Detects collisions utilizing multiple
                                     // information from the motors (velocity,
                                     // current, motor load and effeciency)
      std::cout << "Collision Detected!" << std::endl;
    }
    chassis.driveControl(
        forward, sideways, rotation,
        {.movement = movement_curve, .rotation = rotation_curve}, true, 90,
        {.correctionOn = true,
         .kP = 0.15f,
         .kI = 0.01f,
         .kD = 0.01f}); // minor heading correction to assist drivers (only on
                        // when no joystick output)

    pros::delay(20); // delay for buffer
  }
}