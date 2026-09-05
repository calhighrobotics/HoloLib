#pragma once

namespace liftlib {

class PID {
   private:
	float KP = 0, KI = 0, KD = 0;
	float previousError = 0;
	float integral = 0;
	float threshold = 0;

	float slew = 0;
	float previousOutput = 0;
	bool hasPreviousError = false;

	float maxIntegral = 0;
	float integralZone = 0;
	float maxOutput = 0;

   public:
	PID(float kp, float ki, float kd, float threshold, float slew = 0);

	float calculate(float setpoint, float actual);
	float calculate(float error);

	void reset();

	bool isSettled(float error) const;

	void setGains(float kp, float ki, float kd);
	float getKP() const;
	float getKI() const;
	float getKD() const;

	/**
	 * Accumulated state, exposed so gain scheduling can carry it across a gain
	 * change.
	 *
	 * setGains() resets this deliberately: new gains applied to an integral
	 * wound up under the old ones can kick hard. A gain schedule changes gains
	 * by a sliver every tick, where resetting would zero the derivative term
	 * continuously and leave kD doing nothing, so it saves and restores instead.
	 */
	float getIntegral() const;
	float getPreviousError() const;
	float getPreviousOutput() const;
	bool hasPrevious() const;

	/** Puts back state saved around a setGains(). See getIntegral(). */
	void restoreState(float integral, float previousError, float previousOutput,
	                  bool hasPreviousError);

	void setThreshold(float threshold);

	void setSlew(float slew);
	float getSlew() const;

	void setMaxIntegral(float maxIntegral);
	float getMaxIntegral() const;

	void setIntegralZone(float integralZone);
	float getIntegralZone() const;

	void setMaxOutput(float maxOutput);
	float getMaxOutput() const;

	float getThreshold() const;
};

}  // namespace liftlib
