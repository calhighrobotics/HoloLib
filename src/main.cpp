#include "main.h"
#include "Eigen/Core" // IWYU pragma: export
#include "hololib/chassis.hpp"
#include "hololib/config.hpp"
#include "hololib/localization/ApriltagLocalization.hpp"
#include "hololib/localization/odometry.hpp"
#include "hololib/motions/motion_handler.hpp"
#include "hololib/motions/motions.hpp"
#include "hololib/util/GainScheduler.hpp"
#include "hololib/util/modular_lift.hpp"
#include "hololib/util/replay.hpp"
// Keep liftlib's types under liftlib:: explicitly. Without this, PID and
// Subsystem would also be reachable unqualified, and hololib::PID already
// exists -- easy to collide if anyone later adds `using namespace hololib`.
#define LIFTLIB_NO_GLOBAL_NAMES
#include "liftlib/liftlib.hpp"
#include "pros/ai_vision.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include <cstdio>
#include <print>



// Motor ports (negative for reversing motor)
int frontLPort = 11;
int frontRPort = 12;
int backLPort = 20;
int backRPort = 19;

// IMU port
int imuPort = 18;
std::vector<LiftMotorConfig> lift_motor_configs = {
    {6, pros::MotorGear::blue},
    {-7, pros::MotorGear::blue},
};

Eigen::Matrix3f cameraMatrix = (Eigen::Matrix3f() << 383.57019326565296f * 0.5f,
                                0.0f,
                                322.50382974986405f * 0.5f,
                                0.0f,
                                385.93627187500545f * 0.5f,
                                230.00229561364546f * 0.5f,
                                0.0f,
                                0.0f,
                                1.0f)
                                   .finished();



Eigen::Vector<float, 5> distCoeffs = (Eigen::Vector<float, 5>() << -0.047271311979687029f,
                                      0.19589273312693006f,
                                      -0.0052232338824990303f,
                                      0.00058677141664816013f,
                                      -0.14778998923627218f)
                                         .finished();

pros::Controller controller(pros::E_CONTROLLER_MASTER);



// Initalize motors, IMU, odometry, and chassis
pros::Motor frontLeft = pros::Motor(frontLPort, pros::MotorGear::blue);
pros::Motor frontRight = pros::Motor(frontRPort, pros::MotorGear::blue);
pros::Motor backLeft = pros::Motor(backLPort, pros::MotorGear::blue);
pros::Motor backRight = pros::Motor(backRPort, pros::MotorGear::blue);

pros::Imu imu = pros::Imu(imuPort);

pros::MotorGroup liftMotors = pros::MotorGroup({6, -7});
pros::Motor clawGripper = pros::Motor(10, pros::MotorGear::green); // open/close

pros::Motor intake = pros::Motor(8, pros::MotorGear::blue);

// --- Claw PID subsystems (liftlib) ------------------------------------------
// Each claw mechanism gets its own liftlib::Subsystem: a motor group under a
// tunable PID, with the gear ratio applied for you so targets are in degrees
// at the claw, not at the motor. These are not wired to any button yet -- get
// each one settling cleanly on a manual moveTo() first, then macros can just
// call moveTo() by name.
//
// PID(kP, kI, kD, threshold): threshold is the error (in output degrees) below
// which isSettled() reports true. Gains below are untuned starting points, not
// measured values -- raise kP until the claw gets close with a little
// overshoot, then add just enough kD to damp the overshoot out. Leave kI at 0
// unless it settles short of target under load.
//
// clawRotation (port 9): vertical angular movement -- pivots the whole claw
// between facing the ceiling and facing the ground, 90 degrees between
// perpendicular-to-ground and parallel-to-ground. gear_ratio is a real 12:60
// (5:1 reduction) gearbox spec, not the earlier empirical guess (1/12) -- it
// is output-degrees-per-motor-degree, since liftlib::Subsystem multiplies the
// raw encoder reading by it, so 12:60 becomes 12.0f/60.0f here.
//
// This ratio alone will NOT fix an overshoot that gets much worse at bigger
// targets (e.g. clean at 45 degrees, ~180 at a 90 degree target) -- that
// pattern is what kD=0 looks like: a constant ratio error would be wrong by
// the same factor at every target, not perfect at half and double at full.
// It's also a bigger number than the old guess (0.2 vs ~0.083), so the same
// kP will now behave faster/twitchier in output-degree terms -- expect to
// need a lower kP, or kD sooner, not the same numbers as before.
//
// Tuning phase 1, oscillation hunt: kD and kI at 0 on purpose -- any damping
// would mask exactly the point you're looking for. Run autonomous(), and if it
// creeps up without ever overshooting, double kP and run again. Once it
// visibly overshoots and oscillates back and forth around 90 before settling
// (or doesn't settle at all), back off to roughly 50-60% of that value and only
// then start adding kD to damp the overshoot out.
liftlib::PID clawRotationPID(/*kP=*/15.0f, /*kI=*/0.0001f, /*kD=*/16.0f, /*threshold=*/2.0f);
liftlib::Subsystem clawRotationLift(
    {liftlib::MotorConfig{.port = -9, // negative reverses direction (jog, moveTo, and position readings together)
                          .gear_ratio = 12.0f / 60.0f,
                          .brakeType = pros::E_MOTOR_BRAKE_HOLD,
                          .gearset = pros::MotorGears::green}},
    clawRotationPID);

// clawGripper (port 10) is a flex-wheel roller pair, not a positional
// mechanism -- it just spins one way or the other to pull/push game pieces,
// with no target angle to hold. No PID applies here; it's driven by raw
// move_voltage on the B/Y buttons below, which is already correct as-is.

// --- Elevator lift PID subsystem (liftlib) ----------------------------------
// liftMotors (ports 6/-7) is a CASCADE mechanism -- the same LiftMechanism
// hololib::ModularLift (my_lift, below) was built for -- but my_lift is never
// actually commanded anywhere (only my_lift.cancel() is called, in
// disabled()); R1/R2 drive these motors open loop instead. This gives the
// elevator an actual tunable PID, same pattern as clawRotationLift: build
// first, tune with a manual moveTo(), wire buttons later.
//
// Units: INCHES of vertical travel, not degrees -- this is a cascade (a
// winch: the motor spins a spool, the spool pays out string/chain, the carriage
// rises a linear distance). Degrees would only make sense for a mechanism that
// pivots, like the claw. Correction from an earlier version of this comment:
// it claimed ModularLift::getLiftRadians() confirmed gear_ratio carries over
// as "output degrees", but that's wrong for a CASCADE specifically --
// ModularLift::moveTo() for a CASCADE takes raw, un-scaled motor degrees as
// its target (see controlLoopImpl() in modular_lift.cpp: `x(pos, vel)` is
// built straight from motor.get_position(), gear_ratio is never applied to
// it). gear_ratio/spool_radius in LiftConfig are only ever read by
// getLiftRadians(), which only runs for FOUR_BAR/SIX_BAR/VIRTUAL feedforward
// -- for a CASCADE they're dead config fields today. So there's no existing
// convention to reuse here; this builds the motor-degrees -> inches
// conversion itself:
//
//   inches = motor_degrees * gear_ratio(12/84, output-rotations-per-motor-
//            rotation) * (pi/180, degrees->radians) * spool_radius(inches)
//
// since arc length = radius * angle_in_radians. spool_radius = 1.5f is
// LiftConfig::spool_radius's value below, reused as a literal for the same
// reason gear_ratio is (LiftConfig isn't declared yet at this point in the
// file). Measure spool_radius directly (half the spool's diameter) if 1.5"
// isn't actually what's on this robot -- it's a physical constant, not
// something to tune.
//
// Feedforward::constant, not cosine: a cascade's load doesn't change with
// height the way a pivoting arm's does. kG below is a unit conversion, not a
// fresh measurement -- ModularLift's own feedforward (kG_base * mass * 9.81,
// see LiftConfig below) computes in millivolts, but liftlib's default Voltage
// OutputMode wants -127..127, so it's divided by the same 12000/127 scale
// factor used for the joystick elsewhere in this file: 1750 / (12000/127) ~=
// 18.5. Verify by watching whether liftLift holds height without sagging or
// climbing, same as clawRotation's kG.
//
// Gain schedule, not a single PID: a cascade lift is floppier extended than
// retracted, so one set of gains ends up either too soft at the top or too
// aggressive at the bottom -- same reason xSched/ySched/thetaSched pick gains
// by error magnitude above. liftlib::Subsystem's schedule instead picks by
// *position* (now inches of height), and interpolates linearly between the
// two surrounding points rather than snapping at a boundary.
//
// Real measured values (replacing the earlier LiftConfig-borrowed guesses):
// no external gearbox between motor and spool (direct drive, ratio 1:1), and
// spool radius is 0.4". liftLift.initialize() tares to 0 wherever the lift
// sits at boot (fully retracted, the claw at 6.5" off the ground) -- so
// moveTo() targets and the schedule below are in that from-boot inches space,
// not the raw real-world height off the floor.
//
// Only two positions are ever actually commanded -- 9.5" and 19" (the claw's
// two working heights) -- the lift is never moveTo()'d back to 0 (retracted)
// under PID, so there's nothing to tune a gain point there for. Two points is
// enough for the schedule either way: liftlib interpolates between whichever
// two points surround the live position and holds flat outside them, so a
// target below 9.5" just runs the 9.5" gains rather than needing a third
// point to cover it.
//
// threshold is 1.0" for now (down from the old degrees-era 2.0f) -- still a
// guess, tune it once real gains are in.
//
// Tune each point independently: oscillation-hunt kP/kD with moveTo(9.5) by
// itself first, then moveTo(19) by itself -- expect 19" (more extended) to
// want less kP than 9.5", since the same motor torque swings a
// fully-extended cascade faster than a partially-retracted one.
constexpr float LIFT_SPOOL_RADIUS_IN = 0.4f; // measured
constexpr float LIFT_GEAR_RATIO = 1.0f; // direct drive, no external reduction
constexpr float LIFT_INCHES_PER_MOTOR_DEGREE =
    LIFT_GEAR_RATIO * (M_PI / 180.0f) * LIFT_SPOOL_RADIUS_IN;
liftlib::Subsystem liftLift(
    {liftlib::MotorConfig{.port = 6,
                          .gear_ratio = LIFT_INCHES_PER_MOTOR_DEGREE,
                          .brakeType = pros::E_MOTOR_BRAKE_HOLD,
                          .gearset = pros::MotorGears::blue},
     liftlib::MotorConfig{.port = -7,
                          .gear_ratio = LIFT_INCHES_PER_MOTOR_DEGREE,
                          .brakeType = pros::E_MOTOR_BRAKE_HOLD,
                          .gearset = pros::MotorGears::blue}},
    std::vector<liftlib::Subsystem::GainPoint>{
        {liftlib::PID(/*kP=*/10.0f, /*kI=*/0.0f, /*kD=*/0.0f, /*threshold=*/1.0f), /*position_in=*/10.0f},
        {liftlib::PID(/*kP=*/1.0f, /*kI=*/0.0f, /*kD=*/0.0f, /*threshold=*/1.0f), /*position_in=*/19.0f},
    });






// Initialize odometry configuration. This has to be constructed before chassis:
// EncoderEKFOdometry reads the motors and IMU in its constructor, and chassis
// binds a reference to it, so defining chassis first would capture an object
// that has not run its constructor yet.

hololib::ChassisConfig chassis_config = {
    .drivetrainWidth = 9.1, .drivetrainLength = 10.25, .wheelDiameter = 3.25, .gearRatio = 0.5};
hololib::EncoderEKFOdometry odom =
hololib::EncoderEKFOdometry(frontLeft, frontRight, backLeft, backRight, imu, chassis_config);
const std::function<hololib::Pose(bool)> poseGetter = [](bool radians) { return odom.getPose(radians); };

hololib::Chassis chassis = hololib::Chassis(frontLeft, frontRight, backLeft, backRight, imu, odom);
hololib::GainScheduler xSched = hololib::GainScheduler();
hololib::GainScheduler ySched = hololib::GainScheduler();
hololib::GainScheduler thetaSched = hololib::GainScheduler();

// Initialize obstacle manager
hololib::ObstacleManager obstacles = hololib::ObstacleManager();

// Initialize lift motor configs
LiftConfig my_lift_config = {
    .gear_ratio = 12.0f / 84.0f,
    .arm_length = 15.0f,
    .arm_mass_kg = 2.0f,
    .payload_mass_kg = 0.0f,
    .kG_base = 1750.0f / (2.0f * 9.81f),
    .tolerance = 5.0f,
    .K = Eigen::Matrix<float, 1, 2>{2.9331f, 1.4557f}, // Initialize k gain matrix for lqr
    .spool_radius = 1.5f
};



// Initialize lift using lqr control

ModularLift my_lift(lift_motor_configs, LiftMechanism::CASCADE, my_lift_config);

void initialize() {
    pros::lcd::initialize();
    // Calibrate the chassis
    chassis.calibrate();
    odom.startTask();

    // Tares the pivot motor and seeds its position reading. Whatever the claw
    // is resting at when the program starts becomes 0 for this subsystem.
    clawRotationLift.initialize();

    // clawRotation pivots (it can point at the ceiling or the ground), so the
    // torque needed to hold it depends on angle -- Cosine, not a constant push.
    // Both numbers are untuned:
    //   kG: raise from 0 until holdActively() (used in opcontrol below) holds
    //       the claw level under its own weight, no sag, no climb.
    //   horizontal: the clawRotationLift.getPosition() reading (LCD line 6)
    //       where the claw is actually level. 0 assumes it starts level when
    //       the program boots -- fix this first if that's not true, since a
    //       wrong horizontal will bias the hold in one direction no matter
    //       what kG is.
    clawRotationLift.setFeedforward(liftlib::Feedforward::cosine(/*kG=*/0.0f, /*horizontal=*/0.0f, /*degreesPerUnit=*/1.0f));

    // Tares the elevator and seeds its position reading, same as clawRotationLift.
    liftLift.initialize();

    // Constant, not cosine -- a cascade's load doesn't change with height. kG is
    // a unit conversion from ModularLift's own kG_base (see comment where liftLift
    // is declared), not a fresh measurement -- verify by watching whether it
    // holds height without sagging or climbing.
    liftLift.setFeedforward(liftlib::Feedforward::constant(/*kG=*/18.5f));


    // Set PID gains for chassis
    xSched.setGains({
        {36.0, {15, 0, 2.4}},
        {0.0,  {25, 0, 0.5}},
    });

    ySched.setGains({
        {36.0, {15, 0, 1.6}},
        {0.0,  {20, 0, 1.5}},
    });

    thetaSched.setGains({
        {90.0, {2.76411f, 0.0116046f, 0.0384008f}},
        {0,    {3, 0, 0.04}                      }
    });



    // Basically allows you to see the velocity of the chassis (in/s) (helpful
    // for making custom motions)
    odom.setVelocityCalculations(true);

    // LCD screen task to display chassis data

    pros::Task screen_task([&]() {
        while (true) {
            hololib::Pose pose = odom.getPose(false); // false means degrees, true means radians
            pros::lcd::print(0, "X: %.3f", pose.x);
            pros::lcd::print(1, "Y: %.3f", pose.y);
            pros::lcd::print(2, "Theta: %.3f", pose.theta);
            pros::lcd::print(3, "X Velocity: %.3f", pose.velocity.vx);
            pros::lcd::print(4, "Y Velocity: %.3f", pose.velocity.vy);
            pros::lcd::print(5, "Theta Velocity: %.3f", pose.velocity.w);
            pros::lcd::print(6, "Claw rot: %.2f", clawRotationLift.getPosition());
            pros::lcd::print(7, "Lift (in): %.2f", liftLift.getPosition());
            pros::delay(50);
        }
    });
}



void disabled() {
    odom.setPose(0, 0, 0);
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

    // chassisAsync(hololib::turnToHeading(90));
    // hololib::motion_handler::waitUntilDone();

    // --- clawRotation: lock in place while the lift is tested ---------------
    // Re-tare, then hold actively at wherever the claw is resting so it
    // doesn't sag or drift under gravity while liftLift's test below runs.
    // This replaces the earlier 90-degree tuning moveTo() for now -- that
    // block is preserved below, commented out, to go back to once the lift
    // test isn't the priority.
    clawRotationLift.initialize();
    clawRotationLift.holdActively();

    // --- clawRotation PID tuning (paused -- see above) -----------------------
    // Blocking moveTo: turns 90 degrees and waits (up to 3s) for it to settle
    // before autonomous() returns, so the LCD's "Claw rot" reading (line 6)
    // shows where it actually stopped. Sign is a guess -- if it swings toward
    // the ceiling instead of the ground, change 90.0f to -90.0f.
    //
    // Tune kP/kD on clawRotationPID above first (raise kP until it gets close
    // with a little overshoot, add just enough kD to kill the overshoot), then
    // come back and tune kG on the Feedforward in initialize() by watching
    // whether it holds the 90-degree position or sags after settling.
    //
    // clawRotationLift.initialize();
    // clawRotationLift.moveTo(90.0f, /*async=*/false, /*timeout=*/3000);

    // --- liftLift PID tuning -------------------------------------------------
    // Blocking moveTo to the lower of the two real working heights (9.5",
    // see the gain-schedule comment above liftLift's declaration). Waits (up
    // to 3s) for it to settle before autonomous() returns, so the LCD's
    // "Lift (in)" reading (line 7) shows where it actually stopped.
    //
    // Direction is a guess, same as the claw's was: ports are {6, -7}, the
    // same pair R1/R2 already drive raw in opcontrol where R1 ("Lift up")
    // sends +12000 to both -- so a positive moveTo() target SHOULD raise the
    // lift the same way, but confirm by watching it, not just reading the
    // LCD number. If it drives down/into itself instead of up, negate both
    // ports' signs in liftLift's MotorConfig (6 -> -6, -7 -> 7) rather than
    // negating the target here, so moveTo() and the R1/R2 raw jog stay
    // pointed the same way as each other.
    //
    // Tune kP/kD on the 9.5" GainPoint above first (raise kP until it gets
    // close with a little overshoot, add just enough kD to kill the
    // overshoot), then come back and tune kG on liftLift's Feedforward in
    // initialize() by watching whether it holds 9.5" or sags after settling.
    // Once 9.5" is clean, change the 9.5f below to 19.0f and repeat for the
    // top GainPoint.
    liftLift.initialize();
    liftLift.moveTo(10.0f, /*async=*/false, /*timeout=*/5000);
}



void opcontrol() {
  pros::lcd::print(6,"test");
  odom.setKalmanFilterEnabled(false);
  odom.setPose(0, 0, 0);
  hololib::Chassis::DriveCurve movement_curve{.curve_multipler = 1.01, .deadzone = 5, .minimum_output = 5};
  hololib::Chassis::DriveCurve rotation_curve{.curve_multipler = 1.028, .deadzone = 5, .minimum_output = 5};
  int prev_forward = 0;
  int prev_sideways = 0;
  int prev_rotation = 0;

  constexpr float CLAW_ROT_JOG_POWER = 60.0f; // -127..127, untuned -- raise if it jogs too slowly
  bool clawRotJogging = false;

  // Actively hold from the start, not just after the first press/release --
  // otherwise the claw only starts fighting gravity once the driver has
  // touched L1/L2 once.
  clawRotationLift.holdActively();

  while (true) {
    int forward = controller.get_analog(ANALOG_LEFT_Y);
    int sideways = controller.get_analog(ANALOG_LEFT_X);
    int rotation = controller.get_analog(ANALOG_RIGHT_X);
    if (prev_forward != forward || prev_sideways != sideways ||
        prev_rotation != rotation) {
      prev_forward = forward;
      prev_sideways = sideways;
      prev_rotation = rotation;
    }

    liftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clawGripper.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      std::cout << "Lift up" << std::endl;
      liftMotors.move_voltage(12000);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      std::cout << "Lift down" << std::endl;
      liftMotors.move_voltage(-12000);
    } else {
      liftMotors.move_voltage(0);

    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      std::cout << "Intake in" << std::endl;
      intake.move_voltage(12000);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      std::cout << "Intake out" << std::endl;
      intake.move_voltage(-12000);
    } else {
      intake.move_voltage(0);
    }


    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      clawGripper.move_voltage(12000);
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
      clawGripper.move_voltage(-12000);
    } else {
      clawGripper.move_voltage(0);
    }

    // setOutput() bypasses the PID and stops any active hold task itself, so
    // jogging here can't fight the hold task below -- no manual bookkeeping
    // needed for that part. What IS tracked here is the release edge: calling
    // holdActively() every idle tick would tear down and restart its
    // background task 50 times a second for nothing, so it only fires once,
    // right when the driver lets go.
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      clawRotationLift.setOutput(CLAW_ROT_JOG_POWER);
      clawRotJogging = true;
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      clawRotationLift.setOutput(-CLAW_ROT_JOG_POWER);
      clawRotJogging = true;
    } else if (clawRotJogging) {
      clawRotationLift.holdActively();
      clawRotJogging = false;
    }


    chassis.driveControl(
        forward, sideways, rotation,
        {.movement = movement_curve, .rotation = rotation_curve}, false, 90,
        {.correctionOn = false, .kP = 0.15f, .kI = 0.01f, .kD = 0.01f});
    pros::delay(20);
  }

}
