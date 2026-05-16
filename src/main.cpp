#include "main.h"
#include "Subsystems/modular_lift.h"
#include "chassis.h"
#include "pros/imu.hpp"
#include "distanceReset.h"


pros::MotorGroup lift_motors({1, -2}, pros::MotorGear::blue); 
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::Motor frontl(3);
pros::Motor frontr(4);
pros::Motor backl(5);
pros::Motor backr(6);
pros::Imu imu(7);

pros::Distance front(8);
pros::Distance right(9);
pros::Distance left(10);
pros::Distance back(11);

DistanceSensor front_sensor(&front, 0, 0, 0);
DistanceSensor right_sensor(&right, 0, 0, 0);
DistanceSensor left_sensor(&left, 0, 0, 0);
DistanceSensor back_sensor(&back, 0, 0, 0);

Chassis chassis(frontl, frontr, backl, backr, imu, {.trackWidth = 10, .wheelDiameter = 3.25, .gearRatio = 4/3});
DistanceReset distanceReset(&chassis, {front_sensor, right_sensor, left_sensor, back_sensor}, 20, 3);

//Automatic K matrices Calculator in python
/*
import numpy as np

dt = 0.02 

A = np.array([
    [1.0, dt ],
    [0.0, 1.0]
])

B = np.array([
    [0.0],
    [0.1]
])


Q = np.array([
    [10.0, 0.0],  # Positional weight (Higher = fight position errors harder)
    [0.0,  1.0]   # Velocity weight (Higher = resist moving too fast / dampen oscillations)
])

R = np.array([[1.0]]) 

def dlqr_numpy(A, B, Q, R):
    P = Q.copy()
    for _ in range(100):
        term1 = A.T @ P @ A
        term2 = A.T @ P @ B
        term3 = np.linalg.inv(R + B.T @ P @ B)
        term4 = B.T @ P @ A
        P_next = term1 - (term2 @ term3 @ term4) + Q
        
        if np.max(np.abs(P_next - P)) < 1e-10:
            P = P_next
            break
        P = P_next
        
    K = np.linalg.inv(R + B.T @ P @ B) @ (B.T @ P @ A)
    return K

K = dlqr_numpy(A, B, Q, R)

]
print(f".K = Eigen::Matrix<float, 1, 2>{{{{{K[0][0]:.4f}f, {K[0][1]:.4f}f}}}},")
*/

LiftConfig my_lift_config = {
	.K = Eigen::Matrix<float, 1, 2>{{2.9331f, 1.4557f}},
    .kG = 1750.0f,                                  // Millivolts required to hold arm horizontal
    .gear_ratio = 12.0f / 84.0f,                    // E.g., 12T gear driving an 84T gear
    .tolerance = 5.0f                               // 5 degrees error is "close enough"
};

ModularLift my_lift(&lift_motors, LiftMechanism::CASCADE, my_lift_config);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	lift_motors.tare_position();
	pros::lcd::initialize();
	chassis.setPose(0,0,0);
	chassis.setXGains({
        {12.0, {1.8, 0.0, 0.1}}, // Use for errors > 12 inches
        {3.0,  {1.2, 0.01, 0.3}}, // Use for errors > 3 inches
        {0.0,  {0.8, 0.05, 0.5}}  // Use for small adjustments/settling
    });

	chassis.setYGains(
		{
			    {12.0, {1.8, 0.0, 0.1}}, // Use for errors > 12 inches
    	    	{3.0,  {1.2, 0.01, 0.3}}, // Use for errors > 3 inches
       		 	{0.0,  {0.8, 0.05, 0.5}}  // Use for small adjustments/settling
		}
	);
    // 2. Define Theta logic (Degrees)
    chassis.setThetaGains({
        {45.0, {3.0, 0.0, 0.2}}, // Aggressive turn for large angles
        {0.0,  {1.5, 0.02, 0.4}} // Precise turn for small angles
    });
	while(imu.is_calibrating())
	{
		pros::delay(10);
	}

	pros::Task screen_task([&]() {
        while (true) {
            Pose pose = chassis.getPose(false);
			pros::lcd::print(0, "X: %.2f", pose.x);
			pros::lcd::print(1, "Y: %.2f", pose.y);
			pros::lcd::print(2, "Theta: %.2f", pose.theta);
			pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	my_lift.cancel();
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	chassis.setPose(0,0,0);
	chassis.turnToHeading(90, {.timeout = 2000, .async = true});
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
	DriveCurve movement_curve{.curve_multipler = 1, .deadzone = 0, .minimum_output = 0};
	DriveCurve rotation_curve{.curve_multipler = 1, .deadzone = 0, .minimum_output = 0};

	while (true) {

		// Arcade control scheme
		int forward = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int sideways = master.get_analog(ANALOG_LEFT_X);  // Gets the turn left/right from right joystick
		int rotation = master.get_analog(ANALOG_RIGHT_X);
		chassis.driveControl(forward, sideways, rotation, {.movement = movement_curve, .rotation = rotation_curve});                    // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}