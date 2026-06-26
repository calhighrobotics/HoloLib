#pragma once

#include "Eigen/Core"
#include "api.h"
#include <atomic>


class ModularLift;

/**
 * @brief Task parameters for the lift's background control task.
 */
struct TaskParams {
  ModularLift* instance; /**< Pointer to the lift instance */
};

/**
 * @brief Configuration for a single lift motor.
 */
struct LiftMotorConfig {
  int port;                 /**< Motor port number */
  pros::MotorGear gearset;  /**< Motor internal gearset */
};

/**
 * @brief Enum defining the physical mechanism of the lift.
 */
enum class LiftMechanism {
  CASCADE,
  FOUR_BAR,
  SIX_BAR,
  VIRTUAL
};

/**
 * @brief Configuration constants and physical properties of the lift.
 */
struct LiftConfig {
  float gear_ratio;           /**< External gear ratio applied to the motors */
  float arm_length;           /**< Length of the lift arm */
  float arm_mass_kg;          /**< Mass of the arm mechanism in kilograms */
  float payload_mass_kg;      /**< Mass of the payload being lifted in kilograms */
  float kG_base;              /**< Base gravity feed-forward gain */
  float tolerance;            /**< Acceptable error tolerance for considering a move finished */
  Eigen::Matrix<float, 1, 2> K; /**< State-feedback gain matrix [kP, kD] */
  float spool_radius = 1.0f;  /**< Radius of the spool (for cascade/elevator mechanisms) */
};

/**
 * @brief A modular controller for various lift mechanisms.
 * 
 * Handles background task processing, feed-forward calculations based on physics, 
 * and LQR/state-feedback control for accurate positioning.
 */
class ModularLift {
public:
  /**
   * @brief Construct a new ModularLift object.
   * 
   * @param m_configs Vector of motor configurations for the lift.
   * @param t The type of lift mechanism (e.g. FOUR_BAR, CASCADE).
   * @param c The configuration containing physical constants and control gains.
   */
  ModularLift(const std::vector<LiftMotorConfig>& m_configs, LiftMechanism t, LiftConfig c);

  /**
   * @brief Destroy the ModularLift object.
   * 
   * Cleans up the background task and associated resources.
   */
  ~ModularLift();

  /**
   * @brief Convert raw motor degrees into lift arm radians (or meters for cascade).
   * 
   * @param motor_degrees The raw sensor value in degrees.
   * @return float The calculated position.
   */
  float getLiftRadians(float motor_degrees);

  /**
   * @brief Set the mass of the current payload to update feed-forward calculations dynamically.
   * 
   * @param mass_kg Mass of the payload in kilograms.
   */
  void setPayload(float mass_kg);

  /**
   * @brief Calculate the required feed-forward voltage to hold the lift at its current position.
   * 
   * @param current_motor_degrees The current position of the lift.
   * @return float The calculated feed-forward voltage (in millivolts or specific units).
   */
  float calculateFeedforward(float current_motor_degrees);

  /**
   * @brief Command the lift to move to a target position.
   * 
   * @param target_motor_degrees The target position in motor degrees.
   */
  void moveTo(float target_motor_degrees);

  /**
   * @brief Cancel the current movement command.
   */
  void cancel();

  /**
   * @brief Block the current thread until the lift reaches its target position.
   */
  void waitUntilDone();

  /**
   * @brief Check if the lift is currently trying to reach a target position.
   * 
   * @return true if running, false otherwise.
   */
  bool isRunning();

private:
  std::vector<pros::Motor> motors;
  LiftMechanism type;
  LiftConfig config;

  std::atomic<bool> is_running;
  std::atomic<bool> cancel_request;
  std::atomic<bool> is_settled;

  pros::Task *task;
  pros::Mutex target_mutex;
  float target_position = 0.0f;

  static void task_trampoline(void *params);
  void controlLoopImpl();
};