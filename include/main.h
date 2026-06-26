#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#define PROS_USE_SIMPLE_NAMES

#define PROS_USE_LITERALS

#include "api.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Entry point for the simulation environment.
 * 
 * Executed via Python tools/sim_auton.py. Use this to simulate paths 
 * visually prior to testing on the actual robot.
 */
void simulation(void);

/**
 * @brief Autonomous phase entry point.
 * 
 * Runs the robot's autonomous code. This is executed once during 
 * the autonomous period of a competition.
 */
void autonomous(void);

/**
 * @brief Initialization phase entry point.
 * 
 * Runs immediately upon startup. Used for sensor calibration, 
 * device initialization, etc.
 */
void initialize(void);

/**
 * @brief Disabled phase entry point.
 * 
 * Runs when the robot is disabled (either by competition switch or 
 * field control). Useful to ensure motors are stopped safely.
 */
void disabled(void);

/**
 * @brief Competition initialization phase entry point.
 * 
 * Runs after `initialize()`, before autonomous or opcontrol phases, 
 * but only if connected to a competition switch.
 */
void competition_initialize(void);

/**
 * @brief Operator control phase entry point.
 * 
 * Runs the robot's driver control code. Contains the main loop that 
 * reads joystick values and updates the drivetrain and subsystems.
 */
void opcontrol(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#endif

#endif
