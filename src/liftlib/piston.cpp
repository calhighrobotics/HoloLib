#include "liftlib/piston.hpp"

#include <utility>
#include <vector>

namespace liftlib {

namespace {

/** Mirrors what the ADI accepts: 1 through 8, 'a' to 'h', or 'A' to 'H'. */
bool isValidPort(std::uint8_t port) {
	return (port >= 1 && port <= 8) || (port >= 'a' && port <= 'h') ||
	       (port >= 'A' && port <= 'H');
}

}  // namespace

Piston::Piston(PistonConfig config, std::uint32_t actuationTime)
    : Piston(std::vector<PistonConfig>{config}, actuationTime) {}

Piston::Piston(std::vector<PistonConfig> configs, std::uint32_t actuationTime)
    : configs(std::move(configs)),
      actuationTime(actuationTime),
      extended(false),
      lastChange(0),
      settledEarly(true) {
	// Every solenoid shares one commanded state, so the group can only start in
	// a single state. The first config decides it; the rest supply ports and
	// their own reversal.
	const bool initial = this->configs.empty() ? false : this->configs.front().startExtended;
	extended.store(initial);

	// Drop ports the ADI would reject. Constructing one anyway would leave a
	// solenoid that quietly never fires, so a short solenoidCount() is a much
	// easier symptom to spot than a clamp that does nothing.
	std::vector<PistonConfig> valid;
	valid.reserve(this->configs.size());
	for (const PistonConfig& config : this->configs) {
		if (isValidPort(config.port)) {
			valid.push_back(config);
		}
	}
	this->configs = std::move(valid);

	solenoids.reserve(this->configs.size());
	for (const PistonConfig& config : this->configs) {
		solenoids.emplace_back(config.port, config.reversed, initial);
	}

	lastChange.store(pros::millis());
}

Piston::~Piston() = default;

void Piston::initialize() {
	// Re-assert the commanded state, in case something else drove the port
	// between construction and the start of a match.
	write(extended.load());
	lastChange.store(pros::millis());

	// The cylinder is already in this state, so there is nothing to wait for.
	settledEarly.store(true);
}

void Piston::write(bool state) {
	for (Solenoid& solenoid : solenoids) {
		solenoid.out.set_value(solenoid.reversed ? !state : state);
	}
}

void Piston::set(bool state, bool async) {
	extended.store(state);
	write(state);
	lastChange.store(pros::millis());
	settledEarly.store(false);

	if (!async) {
		pros::delay(actuationTime.load());
	}
}

void Piston::extend(bool async) {
	set(true, async);
}

void Piston::retract(bool async) {
	set(false, async);
}

void Piston::toggle(bool async) {
	set(!extended.load(), async);
}

bool Piston::isExtended() const {
	return extended.load();
}

bool Piston::isSettled() const {
	if (settledEarly.load()) {
		return true;
	}
	return pros::millis() - lastChange.load() >= actuationTime.load();
}

bool Piston::isMoving() const {
	return !isSettled();
}

void Piston::brake() {
	// Nothing to do: a solenoid holds its state without being driven, and
	// dropping the signal would retract the cylinder rather than stop it.
}

void Piston::hold() {
	brake();
}

void Piston::reset() {
	// No controller state to clear. The cylinder keeps its commanded state.
}

void Piston::stop() {
	// A piston cannot be stopped part way, and cutting the signal would move it
	// rather than hold it. Declare the travel over so a stopped Lift does not
	// report this stage as still moving.
	settledEarly.store(true);
}

bool Piston::waitUntilSettled(std::uint32_t timeout) {
	const std::uint32_t start = pros::millis();
	std::uint32_t now = start;

	while (!isSettled()) {
		if (timeout != NO_TIMEOUT && pros::millis() - start >= timeout) {
			return false;
		}
		pros::Task::delay_until(&now, LOOP_DELAY_MS);
	}
	return true;
}

void Piston::setActuationTime(std::uint32_t actuationTime) {
	this->actuationTime.store(actuationTime);
}

std::uint32_t Piston::getActuationTime() const {
	return actuationTime.load();
}

std::size_t Piston::solenoidCount() const {
	return solenoids.size();
}

}  // namespace liftlib
