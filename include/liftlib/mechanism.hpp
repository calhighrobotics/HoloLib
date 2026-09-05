#pragma once

#include <cstdint>

namespace liftlib {

/**
 * Anything a Lift can own and a conditional action can be scoped to.
 *
 * Lift coordinates mechanisms by identity, not by motor: it needs to stop them,
 * settle them, and know whether they are busy. A Subsystem satisfies this with a
 * motor group under PID, a Piston with a solenoid, and both can be named in the
 * stage list of a conditional action.
 */
class Mechanism {
   public:
	static constexpr std::uint32_t LOOP_DELAY_MS = 20;
	static constexpr std::uint32_t NO_TIMEOUT = 0;

	Mechanism() = default;

	Mechanism(const Mechanism&) = delete;
	Mechanism& operator=(const Mechanism&) = delete;

	virtual ~Mechanism() = default;

	/** Prepares the hardware. Safe to call more than once. */
	virtual void initialize() = 0;

	/** True once the mechanism has reached whatever it was asked to do. */
	virtual bool isSettled() const = 0;

	/** Returns the mechanism to a neutral, unpowered-or-braked state. */
	virtual void brake() = 0;

	/** Settles in place: holds position where that means something, else brakes. */
	virtual void hold() = 0;

	/** Clears controller state so the next command starts fresh. */
	virtual void reset() = 0;

	/** Cancels in-progress motion and brakes. */
	virtual void stop() = 0;

	/** True while an async command is still running. */
	virtual bool isMoving() const = 0;

	/**
	 * The mechanism as a positional one, or nullptr when it has no position.
	 *
	 * Lift uses this to tell which stages can take a numeric target. It is a
	 * virtual rather than a dynamic_cast so the library does not depend on RTTI
	 * being enabled, which also keeps the type information out of the binary.
	 */
	virtual class Subsystem* asSubsystem() {
		return nullptr;
	}

	const class Subsystem* asSubsystem() const {
		return const_cast<Mechanism*>(this)->asSubsystem();
	}
};

}  // namespace liftlib
