#pragma once

#include <cstdint>

#include "api.h"

namespace liftlib {

/**
 * Which V5 smart motor is on the port.
 *
 * The two differ in torque, not in speed: a 5.5W with a 200 RPM cartridge free
 * spins as fast as an 11W with a green one, but stalls at roughly half the
 * torque and heats up sooner under the same load.
 *
 * PROS keeps type and cartridge independent, so a cartridge alone does not say
 * which motor it is in. State it here when the port holds a 5.5W.
 */
enum class MotorType {
	/** The 11W V5 smart motor, the default. */
	W11,
	/** The 5.5W V5 smart motor, sometimes called EXP. */
	W5_5,
};

/** Stall torque of a 5.5W relative to an 11W, used to share load between them. */
inline constexpr float W5_5_TORQUE_FRACTION = 0.5f;

struct MotorConfig {
	std::int8_t port;
	float gear_ratio = 1;
	pros::motor_brake_mode_e brakeType = pros::E_MOTOR_BRAKE_COAST;
	pros::MotorGears gearset = pros::MotorGears::green;

	/**
	 * The motor on this port. Defaults to the 11W.
	 *
	 * This is declared rather than detected, so it is never read from an
	 * unplugged port, and it is on you to keep it matching the hardware.
	 */
	MotorType type = MotorType::W11;
};

}  // namespace liftlib
