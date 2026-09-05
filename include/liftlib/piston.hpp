#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <vector>

#include "api.h"
#include "liftlib/mechanism.hpp"

namespace liftlib {

/** One solenoid: an ADI port, and which electrical state counts as extended. */
struct PistonConfig {
	/**
	 * ADI port, 'A' through 'H', or an index 1 through 8.
	 *
	 * There is no valid default, so this must be set. PROS reports a bad port
	 * through errno rather than failing loudly, which would leave a piston that
	 * silently never fires; solenoidCount() returning less than you configured
	 * is the sign that one was rejected.
	 */
	std::uint8_t port;

	/**
	 * Set when the cylinder retracts on a high signal, which is what you get
	 * from a double-acting cylinder plumbed the other way round.
	 */
	bool reversed = false;

	/** Electrical state the port is driven to by initialize(). */
	bool startExtended = false;
};

/**
 * A pneumatic actuator, driven as a Lift stage.
 *
 * A piston has two states rather than a position, so most of the Mechanism
 * contract collapses: it settles once the air has had time to move, and holding
 * means leaving the solenoid where it is. The point of implementing that
 * contract is scoping, so a clamp or a set of wings can be named in a
 * conditional action's stage list alongside motorised stages.
 */
class Piston final : public Mechanism {
   public:
	/**
	 * Time a move waits before it reports settled.
	 *
	 * A solenoid switches instantly but the cylinder takes time to travel, and
	 * there is no sensor to tell us when it arrived. Everything downstream keys
	 * off this delay, so it is the one number worth tuning per mechanism.
	 */
	static constexpr std::uint32_t DEFAULT_ACTUATION_MS = 250;

	explicit Piston(PistonConfig config, std::uint32_t actuationTime = DEFAULT_ACTUATION_MS);

	Piston(std::vector<PistonConfig> configs,
	       std::uint32_t actuationTime = DEFAULT_ACTUATION_MS);

	~Piston() override;

	void initialize() override;

	bool isSettled() const override;

	/** Leaves the solenoid where it is; air holds the cylinder without power. */
	void brake() override;

	/** Identical to brake(): a piston holds its state with no output. */
	void hold() override;

	void reset() override;

	void stop() override;

	bool isMoving() const override;

	/**
	 * Drives the piston to a state.
	 *
	 * When async is true (the default) this returns as soon as the solenoid is
	 * switched, and the mechanism reports unsettled until the actuation time has
	 * elapsed. When async is false it blocks for the actuation time so a routine
	 * can rely on the cylinder having travelled.
	 *
	 * Setting the state it is already in still restarts the timer, since the
	 * cylinder may not have finished its last trip.
	 */
	void set(bool extended, bool async = true);

	void extend(bool async = true);

	void retract(bool async = true);

	/** Flips to the opposite state. */
	void toggle(bool async = true);

	/** The state last commanded, which leads the cylinder by the actuation time. */
	bool isExtended() const;

	/** Blocks until the actuation time has elapsed. False if it timed out first. */
	bool waitUntilSettled(std::uint32_t timeout = NO_TIMEOUT);

	void setActuationTime(std::uint32_t actuationTime);

	std::uint32_t getActuationTime() const;

	std::size_t solenoidCount() const;

   private:
	struct Solenoid {
		pros::adi::DigitalOut out;
		bool reversed;

		Solenoid(std::uint8_t port, bool reversed, bool initial)
		    : out(port, reversed ? !initial : initial), reversed(reversed) {}
	};

	std::vector<PistonConfig> configs;
	std::vector<Solenoid> solenoids;

	/**
	 * Atomic because isSettled() runs on the Lift poll task while
	 * setActuationTime() may be called from opcontrol.
	 */
	std::atomic<std::uint32_t> actuationTime;

	std::atomic<bool> extended;
	std::atomic<std::uint32_t> lastChange;

	/**
	 * Forces isSettled() true regardless of the timer.
	 *
	 * stop() cannot shorten the travel, so it declares the wait over rather
	 * than backdating lastChange, which would underflow when the program has
	 * been up for less than one actuation time.
	 */
	std::atomic<bool> settledEarly;

	void write(bool state);
};

}  // namespace liftlib
