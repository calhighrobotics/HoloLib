#pragma once
#include "pros/rtos.hpp"
#include <cmath>

struct PIDGains {
  double kP;
  double kI;
  double kD;
  double kF;
  double slew = 0; 
};

class PID {
private:
  PIDGains m_gains;
  double m_windupRange;
  bool m_signFlipReset;

  double m_previousError = 0;
  double m_integral = 0;
  uint32_t m_previousTime = 0;
  bool m_initialized = false;
  double m_previousOutput = 0; 

  template <typename T> inline int sgn(T val) {
    return (T(0) < val) - (val < T(0));
  }

public:
  PID(double kP, double kI, double kD, double kF = 0, double windupRange = 0,
      bool signFlipReset = false, double slew = 0);
  PID(const PIDGains &gains, double windupRange = 0,
      bool signFlipReset = false);

  PIDGains getGains();
  void setGains(PIDGains gains);

  double update(double error);
  void reset();

  void setSignFlipReset(bool signFlipReset);
  bool getSignFlipReset();

  void setWindupRange(double windupRange);
  double getWindupRange();
};