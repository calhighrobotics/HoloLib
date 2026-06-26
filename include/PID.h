#pragma once
#include "pros/rtos.hpp"
#include <cmath>

/**
 * @brief Structure holding PID controller gains and parameters.
 */
struct PIDGains {
  double kP;       /**< Proportional gain */
  double kI;       /**< Integral gain */
  double kD;       /**< Derivative gain */
  double kF;       /**< Feed-forward gain */
  double slew = 0; /**< Maximum allowed change in output per update (slew rate) */
};

/**
 * @brief A standard PID controller with advanced features.
 * 
 * Includes support for windup range limits, sign-flip resets, slew rate limits, 
 * filtered derivatives, and integral limits.
 */
class PID {
private:
  PIDGains m_gains;
  double m_windupRange;
  bool m_signFlipReset;

  double m_previousError = 0;
  double m_previousMeasurement = 0;
  double m_integral = 0;
  double m_filteredDerivative = 0;
  uint32_t m_previousTime = 0;
  bool m_initialized = false;
  double m_previousOutput = 0; 

  double m_alpha = 0.2;      
  double m_integralLimit = 0.0;    
  double m_staticThreshold = 3.0;   

  template <typename T> inline int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

public:
  /**
   * @brief Construct a new PID controller.
   * 
   * @param kP Proportional gain.
   * @param kI Integral gain.
   * @param kD Derivative gain.
   * @param kF Feed-forward gain.
   * @param windupRange Range within which the integral term accumulates.
   * @param signFlipReset If true, resets the integral when the error sign flips.
   * @param slew Maximum change in output per update.
   */
  PID(double kP, double kI, double kD, double kF = 0, double windupRange = 0,
      bool signFlipReset = false, double slew = 0);

  /**
   * @brief Construct a new PID controller from a PIDGains struct.
   * 
   * @param gains A struct containing the Kp, Ki, Kd, Kf, and slew rate.
   * @param windupRange Range within which the integral term accumulates.
   * @param signFlipReset If true, resets the integral when the error sign flips.
   */
  PID(const PIDGains &gains, double windupRange = 0,
      bool signFlipReset = false);

  /**
   * @brief Get the current gains of the PID controller.
   * 
   * @return PIDGains The current gains.
   */
  PIDGains getGains();

  /**
   * @brief Set new gains for the PID controller.
   * 
   * @param gains The new gains to apply.
   */
  void setGains(PIDGains gains);

  /**
   * @brief Update the PID controller based on the current error.
   * 
   * @param error The current error (target - measurement).
   * @return double The computed output from the PID controller.
   */
  double update(double error);

  /**
   * @brief Update the PID controller based on error and actual measurement (to prevent derivative kick).
   * 
   * @param error The current error.
   * @param measurement The current measurement/sensor value.
   * @return double The computed output from the PID controller.
   */
  double update(double error, double measurement);

  /**
   * @brief Reset the PID controller's internal state (integral, previous error, etc.).
   */
  void reset();

  /**
   * @brief Enable or disable resetting the integral term when the error sign flips.
   * 
   * @param signFlipReset True to enable, false to disable.
   */
  void setSignFlipReset(bool signFlipReset);

  /**
   * @brief Check if sign-flip integral reset is enabled.
   * 
   * @return true if enabled, false otherwise.
   */
  bool getSignFlipReset();

  /**
   * @brief Set the windup range within which the integral term is allowed to accumulate.
   * 
   * @param windupRange The windup range.
   */
  void setWindupRange(double windupRange);

  /**
   * @brief Get the current windup range.
   * 
   * @return double The windup range.
   */
  double getWindupRange();

  /**
   * @brief Set the alpha value for the derivative filter.
   * 
   * @param alpha The filter constant (0.0 to 1.0).
   */
  void setAlpha(double alpha) { m_alpha = alpha; }

  /**
   * @brief Get the alpha value for the derivative filter.
   * 
   * @return double The current alpha value.
   */
  double getAlpha() { return m_alpha; }

  /**
   * @brief Set the maximum absolute value for the integral term.
   * 
   * @param limit The maximum integral value.
   */
  void setIntegralLimit(double limit) { m_integralLimit = limit; }

  /**
   * @brief Get the integral limit.
   * 
   * @return double The integral limit.
   */
  double getIntegralLimit() { return m_integralLimit; }

  /**
   * @brief Set the static threshold error value.
   * 
   * @param threshold The threshold value.
   */
  void setStaticThreshold(double threshold) { m_staticThreshold = threshold; }

  /**
   * @brief Get the static threshold error value.
   * 
   * @return double The static threshold value.
   */
  double getStaticThreshold() { return m_staticThreshold; }
};