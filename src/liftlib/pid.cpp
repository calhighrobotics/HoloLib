#include "liftlib/pid.hpp"

#include <algorithm>
#include <cmath>

namespace liftlib {

PID::PID(float kp, float ki, float kd, float threshold, float slew)
    : KP(kp),
      KI(ki),
      KD(kd),
      threshold(std::abs(threshold)),
      slew(std::abs(slew)),
      previousOutput(0),
      hasPreviousError(false),
      maxIntegral(0),
      integralZone(0),
      maxOutput(0) {}

float PID::calculate(float setpoint, float actual) {
	return calculate(setpoint - actual);
}

float PID::calculate(float error) {
	float derivative = hasPreviousError ? error - previousError : 0;
	previousError = error;
	hasPreviousError = true;

	if (integralZone <= 0 || std::abs(error) <= integralZone) {
		integral += error;
	} else {
		integral = 0;
	}

	if (maxIntegral > 0) {
		integral = std::clamp(integral, -maxIntegral, maxIntegral);
	}

	float output = KP * error + KI * integral + KD * derivative;

	if (maxOutput > 0 && std::abs(output) > maxOutput) {
		integral -= error;
		if (maxIntegral > 0) {
			integral = std::clamp(integral, -maxIntegral, maxIntegral);
		}
		output = KP * error + KI * integral + KD * derivative;
		output = std::clamp(output, -maxOutput, maxOutput);
	}

	if (slew > 0) {
		output = std::clamp(output, previousOutput - slew, previousOutput + slew);
	}

	previousOutput = output;
	return output;
}

void PID::reset() {
	integral = 0;
	previousError = 0;
	previousOutput = 0;
	hasPreviousError = false;
}

bool PID::isSettled(float error) const {
	return std::abs(error) < threshold;
}

void PID::setGains(float kp, float ki, float kd) {
	KP = kp;
	KI = ki;
	KD = kd;
	reset();
}

float PID::getKP() const {
	return KP;
}

float PID::getKI() const {
	return KI;
}

float PID::getKD() const {
	return KD;
}

float PID::getIntegral() const {
	return integral;
}

float PID::getPreviousError() const {
	return previousError;
}

float PID::getPreviousOutput() const {
	return previousOutput;
}

bool PID::hasPrevious() const {
	return hasPreviousError;
}

void PID::restoreState(float integral, float previousError, float previousOutput,
                       bool hasPreviousError) {
	this->integral = integral;
	this->previousError = previousError;
	this->previousOutput = previousOutput;
	this->hasPreviousError = hasPreviousError;
}

void PID::setThreshold(float threshold) {
	this->threshold = std::abs(threshold);
}

void PID::setSlew(float slew) {
	this->slew = std::abs(slew);
}

float PID::getSlew() const {
	return slew;
}

void PID::setMaxIntegral(float maxIntegral) {
	this->maxIntegral = std::abs(maxIntegral);
}

float PID::getMaxIntegral() const {
	return maxIntegral;
}

void PID::setIntegralZone(float integralZone) {
	this->integralZone = std::abs(integralZone);
}

float PID::getIntegralZone() const {
	return integralZone;
}

void PID::setMaxOutput(float maxOutput) {
	this->maxOutput = std::abs(maxOutput);
}

float PID::getMaxOutput() const {
	return maxOutput;
}

float PID::getThreshold() const {
	return threshold;
}

}  // namespace liftlib
