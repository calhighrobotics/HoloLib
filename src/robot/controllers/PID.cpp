#include "PID.h"
#include <cmath>
#include <algorithm>

PID::PID(double kP, double kI, double kD, double kF, double windupRange,
         bool signFlipReset, double slew)
    : m_gains({kP, kI, kD, kF, slew}), m_windupRange(windupRange),
      m_signFlipReset(signFlipReset), m_integralLimit(windupRange),
      m_alpha(0.2), m_staticThreshold(3.0) {}

PID::PID(const PIDGains &gains, double windupRange, bool signFlipReset)
    : m_gains(gains), m_windupRange(windupRange),
      m_signFlipReset(signFlipReset), m_integralLimit(windupRange),
      m_alpha(0.2), m_staticThreshold(3.0) {}

PIDGains PID::getGains() { return m_gains; }

void PID::setGains(PIDGains gains) { m_gains = gains; }

double PID::update(double error) {
  return update(error, -error);
}

double PID::update(double error, double measurement) {
  uint32_t now = pros::millis();
  double dt = 0;

  if (m_initialized) {
    dt = (now - m_previousTime) / 1000.0;
    dt = std::clamp(dt, 0.0, 0.1);
  } else {
    m_initialized = true;
  }

  m_previousTime = now;

  double derivative = 0;
  if (dt > 0) {
    derivative = -(measurement - m_previousMeasurement) / dt;
  }

  m_filteredDerivative = m_alpha * derivative + (1.0 - m_alpha) * m_filteredDerivative;

  m_integral += error * dt;

  if (sgn(error) != sgn(m_previousError) && m_signFlipReset) {
    m_integral = 0;
  }

  if (m_integralLimit != 0) {
    m_integral = std::clamp(m_integral, -m_integralLimit, m_integralLimit);
  }

  m_previousError = error;
  m_previousMeasurement = measurement;

  double output =
      error * m_gains.kP +
      m_integral * m_gains.kI +
      m_filteredDerivative * m_gains.kD;

  if (std::abs(output) > m_staticThreshold) {
    output += sgn(output) * m_gains.kF;
  }
  if (m_gains.slew != 0 && dt > 0) {
    double maxChange = m_gains.slew * dt;
    double change = output - m_previousOutput;

    if (std::abs(change) > maxChange) {
      output =
          m_previousOutput +
          std::copysign(maxChange, change);
    }
  }

  m_previousOutput = output;

  return output;
}

void PID::reset() {
  m_previousError = 0;
  m_previousMeasurement = 0;
  m_integral = 0;
  m_filteredDerivative = 0;
  m_initialized = false;
  m_previousOutput = 0; 
}

void PID::setSignFlipReset(bool signFlipReset) {
  m_signFlipReset = signFlipReset;
}

bool PID::getSignFlipReset() { return m_signFlipReset; }

void PID::setWindupRange(double windupRange) { 
  m_windupRange = windupRange; 
  m_integralLimit = windupRange;
}

double PID::getWindupRange() { return m_windupRange; }