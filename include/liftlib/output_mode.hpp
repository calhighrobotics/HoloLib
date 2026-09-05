#pragma once

namespace liftlib {

/** How a subsystem's output value is sent to its motors. */
enum class OutputMode {
	/**
	 * Output is a -127..127 command sent straight to the motor as voltage.
	 *
	 * This is the default. The motor applies it open loop, so the controller in
	 * this library is the only loop in the path and a feedforward term means a
	 * physical push rather than a velocity request.
	 */
	Voltage,

	/**
	 * Output is a velocity in RPM, handed to the motor's own velocity
	 * controller. The usable range depends on the cartridge: 100 for red, 200
	 * for green, 600 for blue.
	 *
	 * The V5 firmware runs its own loop in this mode, so it fights a tightly
	 * tuned outer loop. Prefer Voltage unless you specifically want the motor
	 * holding a velocity for you.
	 */
	Velocity,
};

/** The -127..127 command range used by OutputMode::Voltage. */
inline constexpr float VOLTAGE_OUTPUT_LIMIT = 127.0f;

/** Millivolts corresponding to a full-scale voltage command. */
inline constexpr float MAX_MILLIVOLTS = 12000.0f;

}  // namespace liftlib
