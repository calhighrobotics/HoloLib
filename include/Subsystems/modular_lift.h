#pragma once

#include "Eigen/Core"
#include "api.h"
#include <atomic>

// Forward declaration of ModularLift for TaskParams
class ModularLift;

/**
 * @brief Helper structure used to pass the class instance into the PROS task trampoline.
 */
struct TaskParams {
  ModularLift* instance;
};

/**
 * @brief Types of mechanical lifts supported by gravity feedforward calculations.
 */
enum class LiftMechanism {
  CASCADE,
  FOUR_BAR,
  SIX_BAR,
  VIRTUAL
};

/**
 * @brief Configuration parameters for ModularLift dynamics and control loops.
 */
struct LiftConfig {
  float gear_ratio;                  // Gear reduction from motor to physical arm
  float kG;                          // Gravity feedforward voltage constant
  float tolerance;                   // Settling target tolerance in motor degrees
  Eigen::Matrix<float, 1, 2> K;      // LQR State-feedback gains matrix [Kp_pos, Kd_vel]
};

class ModularLift {
public:
  /**
   * @brief Construct a new Modular Lift object.
   * 
   * @param m_group Pointer to the PROS MotorGroup driving the lift.
   * @param t The mechanical architecture type.
   * @param c Configuration parameters (gains, tolerances, gear ratios).
   */
  ModularLift(pros::MotorGroup *m_group, LiftMechanism t, LiftConfig c);

  /**
   * @brief Safely shuts down the control thread and cleans up memory.
   */
  ~ModularLift();

  /**
   * @brief Converts motor encoder degrees to physical lift arm rotation in radians.
   */
  float getLiftRadians(float motor_degrees);

  /**
   * @brief Computes gravity compensation voltage based on lift type and current position.
   */
  float calculateFeedforward(float current_motor_degrees);

  /**
   * @brief Starts the asynchronous control loop to move the lift to the target position.
   */
  void moveTo(float target_motor_degrees);

  /**
   * @brief Stops the control loop, terminates active movement, and brakes the motors.
   */
  void cancel();

  /**
   * @brief Blocks the calling thread until the lift has settled at the target position.
   */
  void waitUntilDone();

  /**
   * @brief Returns true if the asynchronous control task is currently running.
   */
  bool isRunning();

private:
  pros::MotorGroup *motors;          // Pointer to motor hardware interface
  LiftMechanism type;                // Mechanical link configuration
  LiftConfig config;                 // System gains and physical limits

  // Thread-safe state variables
  std::atomic<bool> is_running;      // True if the control thread is executing
  std::atomic<bool> cancel_request;  // True if cancel() has been requested
  std::atomic<bool> is_settled;      // True if lift is within settling parameters

  pros::Task *task;                  // Pointer to the underlying PROS RTOS task wrapper
  pros::Mutex target_mutex;          // Mutex protecting target_position read/writes
  float target_position = 0.0f;      // Current setpoint in motor degrees

  /**
   * @brief Static entry point wrapper required by PROS RTOS task creator.
   */
  static void task_trampoline(void *params);

  /**
   * @brief Internal loop implementation containing the active closed-loop LQR control.
   */
  void controlLoopImpl();
};