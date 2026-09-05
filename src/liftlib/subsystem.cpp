#include "liftlib/subsystem.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace liftlib {

namespace {

// Limits the velocity based on what the gearset is
float velocityLimit(pros::MotorGears gearset) {
	switch (gearset) {
		case pros::MotorGears::red:
			return 100;
		case pros::MotorGears::blue:
			return 600;
		default:
			return 200;
	}
}

constexpr float DEFAULT_FEEDFORWARD_HEADROOM = 0.8f;

}

Subsystem::Subsystem(std::vector<MotorConfig> motorConfigs, PID pid)
    : motorConfigs(motorConfigs),
      pid(pid),
      minPosition(0),
      maxPosition(0),
      limited(false),
      lastError(std::numeric_limits<float>::max()),
      settled(false),
      settleTicks(0),
      holdTarget(0),
      holding(false),
      outputMode(OutputMode::Voltage),
      derivedOutputLimit(0),
      torqueSharing(false),
      lastPosition(0),
      feedforwardHeadroom(DEFAULT_FEEDFORWARD_HEADROOM),
      cancelRequested(false),
      holdStop(false),
      activeHoldTarget(0) {
	buildMotors();
}

Subsystem::Subsystem(std::vector<MotorConfig> motorConfigs, PID pid, float minPosition,
                     float maxPosition)
    : motorConfigs(motorConfigs),
      pid(pid),
      minPosition(std::min(minPosition, maxPosition)),
      maxPosition(std::max(minPosition, maxPosition)),
      limited(true),
      lastError(std::numeric_limits<float>::max()),
      settled(false),
      settleTicks(0),
      holdTarget(0),
      holding(false),
      outputMode(OutputMode::Voltage),
      derivedOutputLimit(0),
      torqueSharing(false),
      lastPosition(0),
      feedforwardHeadroom(DEFAULT_FEEDFORWARD_HEADROOM),
      cancelRequested(false),
      holdStop(false),
      activeHoldTarget(0) {
	buildMotors();
}

std::vector<Subsystem::GainPoint> Subsystem::normalizeSchedule(std::vector<GainPoint> schedule) {
	// NaN first: every comparison against it is false, which violates the strict
	// weak ordering std::sort requires and is undefined behaviour rather than
	// merely a strange order.
	schedule.erase(std::remove_if(schedule.begin(), schedule.end(),
	                              [](const GainPoint& point) { return std::isnan(point.second); }),
	               schedule.end());

	// Sorting lets selectGains walk the points in travel order, so a user can
	// list them in whatever order the tuning session happened to produce.
	std::sort(schedule.begin(), schedule.end(),
	          [](const GainPoint& a, const GainPoint& b) { return a.second < b.second; });

	// Two points at one position disagree about what the gains there are, and
	// interpolating between them would divide by a zero span. Keep the first.
	schedule.erase(std::unique(schedule.begin(), schedule.end(),
	                           [](const GainPoint& a, const GainPoint& b) {
		                           return a.second == b.second;
	                           }),
	               schedule.end());

	return schedule;
}

PID Subsystem::baseOf(const std::vector<GainPoint>& schedule) {
	// Everything that is not kP, kI, or kD comes from the lowest point, so the
	// threshold and slew the user tuned there hold across the whole travel.
	if (!schedule.empty()) {
		return schedule.front().first;
	}

	// No points at all is not a controller. Zero gains hold the mechanism where
	// it is instead of driving it somewhere arbitrary.
	return PID(0, 0, 0, 0);
}

void Subsystem::applyGains(float kp, float ki, float kd) {
	// setGains() resets the integral, previous error, and slew memory. This runs
	// every tick, so going through it would zero the derivative continuously and
	// leave kD doing nothing. Only the three gain fields are touched here.
	if (pid.getKP() == kp && pid.getKI() == ki && pid.getKD() == kd) {
		return;
	}

	const float integral = pid.getIntegral();
	const float previousError = pid.getPreviousError();
	const float previousOutput = pid.getPreviousOutput();
	const bool hasPrevious = pid.hasPrevious();

	pid.setGains(kp, ki, kd);
	pid.restoreState(integral, previousError, previousOutput, hasPrevious);
}

bool Subsystem::lookUpGains(float position, float& kp, float& ki, float& kd) const {
	if (gainSchedule.size() < 2) {
		return false;
	}

	// A position source that faults reads as NaN upstream. Every comparison
	// below would be false, so the walk would fall off the end and leave the
	// outputs untouched; refusing here says so plainly.
	if (std::isnan(position)) {
		return false;
	}

	// Outside the outermost points, hold the nearest one's gains flat rather
	// than extrapolating, which would invent gains no one tuned.
	const PID* flat = nullptr;
	if (position <= gainSchedule.front().second) {
		flat = &gainSchedule.front().first;
	} else if (position >= gainSchedule.back().second) {
		flat = &gainSchedule.back().first;
	}

	if (flat != nullptr) {
		kp = flat->getKP();
		ki = flat->getKI();
		kd = flat->getKD();
		return true;
	}

	for (std::size_t i = 1; i < gainSchedule.size(); i++) {
		const GainPoint& upper = gainSchedule[i];
		if (position > upper.second) {
			continue;
		}

		const GainPoint& lower = gainSchedule[i - 1];

		// Duplicates are removed in normalizeSchedule, so the span is nonzero.
		const float span = upper.second - lower.second;
		const float t = (position - lower.second) / span;

		kp = lower.first.getKP() + t * (upper.first.getKP() - lower.first.getKP());
		ki = lower.first.getKI() + t * (upper.first.getKI() - lower.first.getKI());
		kd = lower.first.getKD() + t * (upper.first.getKD() - lower.first.getKD());
		return true;
	}

	return false;
}

void Subsystem::selectGains(float position) {
	float kp = 0;
	float ki = 0;
	float kd = 0;
	if (lookUpGains(position, kp, ki, kd)) {
		applyGains(kp, ki, kd);
	}
}

PID Subsystem::gainsAt(float position) const {
	// A copy of the live controller, so the probe carries the same threshold,
	// slew, and limits the scheduled gains would actually run under.
	PID probe = pid;

	float kp = 0;
	float ki = 0;
	float kd = 0;
	if (lookUpGains(position, kp, ki, kd)) {
		probe.setGains(kp, ki, kd);
	}
	return probe;
}

void Subsystem::setGainSchedule(std::vector<GainPoint> schedule) {
	// An async move selects gains every tick from its own task, so rebuilding
	// the schedule under it would have it interpolating across a vector being
	// reallocated.
	cancelTask();

	gainSchedule = normalizeSchedule(std::move(schedule));

	if (gainSchedule.empty()) {
		// Nothing to schedule from. Leave the controller exactly as it was, so
		// clearing a schedule this way is not also a silent retune to zero.
		return;
	}

	// Only the gains come from the schedule. The live controller keeps its
	// threshold, slew, integral limits, and output limit, because those are
	// settings on this subsystem rather than on a tune: replacing the whole PID
	// here would silently discard a setIntegralZone or setSlew the user made
	// after construction, which the constructor path never does to them.
	if (gainSchedule.size() == 1) {
		// One point is not something to interpolate across, so selectGains would
		// decline it and the point would sit there doing nothing. Take it as the
		// gains to run everywhere, which is what one point can only mean.
		const PID& only = gainSchedule.front().first;
		applyGains(only.getKP(), only.getKI(), only.getKD());
		return;
	}

	selectGains(getPosition());
}

void Subsystem::clearGainSchedule() {
	cancelTask();
	gainSchedule.clear();
}

bool Subsystem::hasGainSchedule() const {
	return gainSchedule.size() > 1;
}

const std::vector<Subsystem::GainPoint>& Subsystem::getGainSchedule() const {
	return gainSchedule;
}

Subsystem::Subsystem(std::vector<MotorConfig> motorConfigs, std::vector<GainPoint> schedule)
    : motorConfigs(motorConfigs),
      gainSchedule(normalizeSchedule(std::move(schedule))),
      pid(baseOf(gainSchedule)),
      minPosition(0),
      maxPosition(0),
      limited(false),
      lastError(std::numeric_limits<float>::max()),
      settled(false),
      settleTicks(0),
      holdTarget(0),
      holding(false),
      outputMode(OutputMode::Voltage),
      derivedOutputLimit(0),
      torqueSharing(false),
      lastPosition(0),
      feedforwardHeadroom(DEFAULT_FEEDFORWARD_HEADROOM),
      cancelRequested(false),
      holdStop(false),
      activeHoldTarget(0) {
	buildMotors();
}

Subsystem::Subsystem(std::vector<MotorConfig> motorConfigs, std::vector<GainPoint> schedule,
                     float minPosition, float maxPosition)
    : motorConfigs(motorConfigs),
      gainSchedule(normalizeSchedule(std::move(schedule))),
      pid(baseOf(gainSchedule)),
      minPosition(std::min(minPosition, maxPosition)),
      maxPosition(std::max(minPosition, maxPosition)),
      limited(true),
      lastError(std::numeric_limits<float>::max()),
      settled(false),
      settleTicks(0),
      holdTarget(0),
      holding(false),
      outputMode(OutputMode::Voltage),
      derivedOutputLimit(0),
      torqueSharing(false),
      lastPosition(0),
      feedforwardHeadroom(DEFAULT_FEEDFORWARD_HEADROOM),
      cancelRequested(false),
      holdStop(false),
      activeHoldTarget(0) {
	buildMotors();
}

float Subsystem::velocityCeiling() const {
	// The slowest cartridge in the group sets the ceiling, since commanding
	// past it would desynchronise a group of mixed gearings.
	float limit = 0;
	for (const MotorConfig& config : motorConfigs) {
		const float motorLimit = velocityLimit(config.gearset);
		limit = limit == 0 ? motorLimit : std::min(limit, motorLimit);
	}
	return limit;
}

float Subsystem::outputLimit() const {
	return outputMode == OutputMode::Voltage ? VOLTAGE_OUTPUT_LIMIT : velocityCeiling();
}

void Subsystem::buildMotors() {
	motors.reserve(motorConfigs.size());
	for (const MotorConfig& config : motorConfigs) {
		motors.emplace_back(config.port, config.gearset);
	}

	buildOutputScales();

	const float limit = outputLimit();
	if (limit > 0 && pid.getMaxOutput() == 0) {
		pid.setMaxOutput(limit);
		derivedOutputLimit = limit;
	}
}

void Subsystem::buildOutputScales() {
	outputScales.assign(motorConfigs.size(), 1.0f);

	// Sharing only means something when the group mixes types. In a group of
	// one type every motor would get the same scale, which is what it already
	// has.
	if (!torqueSharing || !isMixedGroup()) {
		return;
	}

	for (std::size_t i = 0; i < motorConfigs.size(); i++) {
		if (motorConfigs[i].type == MotorType::W5_5) {
			outputScales[i] = W5_5_TORQUE_FRACTION;
		}
	}
}

void Subsystem::setTorqueSharing(bool enabled) {
	torqueSharing = enabled;
	buildOutputScales();
}

bool Subsystem::getTorqueSharing() const {
	return torqueSharing;
}

bool Subsystem::isMixedGroup() const {
	if (motorConfigs.empty()) {
		return false;
	}

	const MotorType first = motorConfigs.front().type;
	for (const MotorConfig& config : motorConfigs) {
		if (config.type != first) {
			return true;
		}
	}
	return false;
}

std::size_t Subsystem::motorCountOfType(MotorType type) const {
	std::size_t count = 0;
	for (const MotorConfig& config : motorConfigs) {
		if (config.type == type) {
			count++;
		}
	}
	return count;
}

void Subsystem::setOutputMode(OutputMode mode) {
	if (mode == outputMode) {
		return;
	}

	// Whether the limit still belongs to us has to be judged against the old
	// mode, before anything changes.
	const bool limitIsOurs = pid.getMaxOutput() == derivedOutputLimit;

	outputMode = mode;

	const float limit = outputLimit();
	if (limit <= 0) {
		// Velocity mode with no motors to read a cartridge from. Clear the
		// derived limit rather than leaving one carried over from the units we
		// just left, which would cap the controller at a meaningless number.
		if (limitIsOurs) {
			pid.setMaxOutput(0);
			derivedOutputLimit = 0;
		}
		return;
	}

	// Only rescale a limit this subsystem chose. A value the user set means
	// something to them that does not survive a change of units, so leave it
	// and let the retune they already have to do cover it.
	if (limitIsOurs) {
		pid.setMaxOutput(limit);
		derivedOutputLimit = limit;
	}
}

OutputMode Subsystem::getOutputMode() const {
	return outputMode;
}

void Subsystem::setPositionSource(PositionSource source) {
	// An async move calls the source every tick from its own task. Replacing a
	// std::function destroys the old target, so swapping it under a running
	// move would be a use-after-free rather than a merely stale read.
	cancelTask();
	positionSource = std::move(source);
}

void Subsystem::clearPositionSource() {
	cancelTask();
	positionSource = nullptr;
}

bool Subsystem::hasPositionSource() const {
	return positionSource != nullptr;
}

void Subsystem::initialize() {
	for (std::size_t i = 0; i < motors.size(); i++) {
		motors[i].motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
		motors[i].motor.set_brake_mode(motorConfigs[i].brakeType);

		// Taring is meaningless against an external sensor: it would move the
		// encoder zero without moving the zero the subsystem actually reads,
		// silently shifting the soft limits.
		if (positionSource == nullptr) {
			motors[i].motor.tare_position();
		}
	}

	// Seed the fallback from a real reading. Until something reads cleanly it
	// stays at zero, and a sensor that faults from the very first tick would
	// otherwise fall back on a zero that means nothing.
	getPosition();
}

float Subsystem::getPosition() const {
	if (positionSource != nullptr) {
		const float position = positionSource();

		// A faulted sensor must not read as a plausible position. Reporting 0
		// would make the error the whole target and drive the mechanism into a
		// hard stop, and PROS_ERR passed through arithmetic reads as a huge
		// value that does the same in the other direction. Freeze at the last
		// good sample instead, which holds station until the sensor returns.
		if (!isUsablePosition(position)) {
			return lastPosition;
		}

		lastPosition = position;
		return position;
	}

	float total = 0;
	int counted = 0;
	for (std::size_t i = 0; i < motors.size(); i++) {
		double position = motors[i].motor.get_position();
		if (position == PROS_ERR_F || std::isnan(position)) {
			continue;
		}
		total += static_cast<float>(position) * motorConfigs[i].gear_ratio;
		counted++;
	}

	// Every motor failed to report. Hold the last good average rather than
	// claiming the mechanism is at zero.
	if (counted == 0) {
		return lastPosition;
	}

	lastPosition = total / counted;
	return lastPosition;
}

bool Subsystem::isUsablePosition(float position) {
	if (std::isnan(position) || std::isinf(position)) {
		return false;
	}

	// PROS reports a failed read as PROS_ERR (INT32_MAX). Scaled by a gear
	// ratio or divided down by a user's conversion it lands somewhere absurd
	// rather than exactly on the sentinel, so reject the magnitude.
	return std::abs(position) < POSITION_SANITY_LIMIT;
}

float Subsystem::clampTarget(float target) const {
	return limited ? std::clamp(target, minPosition, maxPosition) : target;
}

void Subsystem::update(float target) {
	// Read the position once: two reads a tick cost twice the port traffic and
	// can disagree, which would feed the PID and the feedforward slightly
	// different pictures of where the mechanism is.
	updateAt(target, getPosition());
}

void Subsystem::updateAt(float target, float position) {
	// Gains follow where the mechanism is, not where it is going, so a move into
	// a stiffer region stiffens as it arrives rather than the moment it is
	// commanded. Every motion path reaches the controller through here, so this
	// covers moveTo, hold, and a hand-driven update alike.
	selectGains(position);

	lastError = clampTarget(target) - position;

	if (pid.isSettled(lastError)) {
		if (settleTicks < SETTLE_COUNT) {
			settleTicks++;
		}
	} else {
		settleTicks = 0;
	}
	settled = settleTicks >= SETTLE_COUNT;

	const float limit = pid.getMaxOutput();

	// Cap the gravity term below the output limit so the PID always keeps some
	// authority. Without this a heavy mechanism whose kG approaches the limit
	// leaves the controller unable to push any further in that direction.
	float gravity = feedforward.calculate(position);
	if (limit > 0) {
		const float ceiling = limit * feedforwardHeadroom;
		gravity = std::clamp(gravity, -ceiling, ceiling);
	}

	float output = pid.calculate(lastError) + gravity;

	if (limit > 0) {
		output = std::clamp(output, -limit, limit);
	}

	writeOutput(output);
}

void Subsystem::writeOutput(float output) const {
	const float limit = outputLimit();
	if (limit > 0) {
		output = std::clamp(output, -limit, limit);
	}

	// outputScales is all ones unless torque sharing is on in a mixed group, so
	// the usual path is a multiply by 1.0f rather than a branch per motor.
	if (outputMode == OutputMode::Voltage) {
		// -127..127 maps onto the motor's full millivolt range.
		const float millivolts = output * (MAX_MILLIVOLTS / VOLTAGE_OUTPUT_LIMIT);
		for (std::size_t i = 0; i < motors.size(); i++) {
			motors[i].motor.move_voltage(
			    static_cast<std::int32_t>(millivolts * outputScales[i]));
		}
		return;
	}

	for (std::size_t i = 0; i < motors.size(); i++) {
		motors[i].motor.move_velocity(static_cast<std::int32_t>(output * outputScales[i]));
	}
}

void Subsystem::setFeedforwardHeadroom(float fraction) {
	feedforwardHeadroom = std::clamp(fraction, 0.0f, 1.0f);
}

float Subsystem::getFeedforwardHeadroom() const {
	return feedforwardHeadroom;
}

bool Subsystem::isSettled() const {
	return settled;
}

bool Subsystem::isSettledAt(float target) const {
	return pid.isSettled(clampTarget(target) - getPosition());
}

void Subsystem::reset() {
	pid.reset();
	lastError = std::numeric_limits<float>::max();
	settled = false;
	settleTicks = 0;
	holding = false;
}

void Subsystem::brake() {
	for (std::size_t i = 0; i < motors.size(); i++) {
		motors[i].motor.set_brake_mode(motorConfigs[i].brakeType);
		motors[i].motor.brake();
	}
}

void Subsystem::hold() {
	if (!feedforward.isEnabled()) {
		brake();
		return;
	}

	// One read serves both the latch and the loop, so a position source is
	// called once per tick and the latched target cannot be offset from the
	// position the controller then measures against it.
	const float position = getPosition();

	// Latch the position on the first call so repeated calls close the loop
	// around one target rather than chasing wherever the mechanism has drifted.
	if (!holding) {
		holdTarget = position;
		holding = true;
	}

	updateAt(holdTarget, position);
}

void Subsystem::holdAt(float target) {
	if (!feedforward.isEnabled()) {
		brake();
		return;
	}

	// Latch the commanded target rather than the measured position. A move that
	// settles a little low under load would otherwise bake that droop into the
	// hold target, and every later move would latch from the previous sag.
	if (!holding) {
		holdTarget = clampTarget(target);
		holding = true;
	}

	updateAt(holdTarget, getPosition());
}

void Subsystem::holdActively(float target) {
	// No gravity term means nothing to hold with, and the brake already does
	// the job. Spinning a task to write zero every tick would only keep the
	// motors energised for nothing.
	if (!feedforward.isEnabled()) {
		brake();
		return;
	}

	// Re-latch on the new target rather than stacking a second task on the
	// motors.
	stopHoldTask();

	activeHoldTarget.store(clampTarget(target));
	releaseHold();
	holdStop.store(false);

	auto started = std::make_unique<pros::Task>([this]() {
		std::uint32_t now = pros::millis();
		while (!holdStop.load()) {
			holdAt(activeHoldTarget.load());
			pros::Task::delay_until(&now, LOOP_DELAY_MS);
		}
	});

	// An async move starts its hold from inside the move task while the next
	// command stops it from the caller's, so publishing the pointer is the one
	// step that needs guarding. The task is already running by now; the lock
	// covers the handoff, not the work.
	holdMutex.take(TIMEOUT_MAX);
	holdTask = std::move(started);
	holdMutex.give();
}

void Subsystem::holdActively() {
	holdActively(getPosition());
}

bool Subsystem::isHoldingActively() const {
	holdMutex.take(TIMEOUT_MAX);
	const bool active =
	    holdTask != nullptr && holdTask->get_state() != pros::E_TASK_STATE_DELETED;
	holdMutex.give();
	return active;
}

void Subsystem::stopHoldTask() {
	// Take the pointer out from under the lock and join it outside, so the lock
	// is never held across a join. Holding it there would block a move that is
	// publishing its own hold, and the two would wait on each other.
	holdMutex.take(TIMEOUT_MAX);
	std::unique_ptr<pros::Task> stopping = std::move(holdTask);
	holdMutex.give();

	if (stopping == nullptr) {
		return;
	}

	holdStop.store(true);
	stopping->join();
	stopping.reset();
	holdStop.store(false);
}

void Subsystem::releaseHold() {
	holding = false;
}

bool Subsystem::isHolding() const {
	return holding;
}

void Subsystem::setFeedforward(const Feedforward& feedforward) {
	this->feedforward = feedforward;
}

Feedforward& Subsystem::getFeedforward() {
	return feedforward;
}

float Subsystem::holdOutput() const {
	float gravity = feedforward.calculate(getPosition());

	// Report the value update() would actually apply, so a caller that biases
	// around the holding output sees the same number the controller uses.
	const float limit = pid.getMaxOutput();
	if (limit > 0) {
		const float ceiling = limit * feedforwardHeadroom;
		gravity = std::clamp(gravity, -ceiling, ceiling);
	}
	return gravity;
}

void Subsystem::setOutput(float output) {
	// Driving by hand takes the motors, so a hold task still writing them every
	// tick would fight this and the mechanism would judder between the two.
	stopHoldTask();

	float limit = pid.getMaxOutput();
	if (limit > 0) {
		output = std::clamp(output, -limit, limit);
	}

	writeOutput(output);
}

bool Subsystem::hasLimits() const {
	return limited;
}

float Subsystem::getMinPosition() const {
	return minPosition;
}

float Subsystem::getMaxPosition() const {
	return maxPosition;
}

void Subsystem::moveToBlocking(float target, std::uint32_t timeout) {
	reset();

	std::uint32_t start = pros::millis();
	std::uint32_t now = start;

	while (!isSettled() && !cancelRequested.load()) {
		if (timeout != NO_TIMEOUT && pros::millis() - start >= timeout) {
			break;
		}
		update(target);
		pros::Task::delay_until(&now, LOOP_DELAY_MS);
	}

	if (cancelRequested.load()) {
		brake();
		return;
	}

	// Hold against the target that was commanded, not against wherever the
	// mechanism came to rest. Settling a little low under load would otherwise
	// be latched as the new target and compounded by every later move.
	//
	// The hold runs in its own task so the loop stays closed after this returns;
	// one output write then silence is a sag on a loaded lift. It is not the
	// move task, so the move still reports finished and waitUntilSettled()
	// returns normally. Without a feedforward this brakes instead.
	holdActively(target);
}

void Subsystem::moveTo(float target, bool async, std::uint32_t timeout) {
	cancelTask();

	if (!async) {
		cancelRequested.store(false);
		moveToBlocking(target, timeout);
		// A blocking move never runs through cancelTask(), so a stop() from
		// another task would otherwise leave this latched and make the next
		// blocking move abort on its first tick without moving anything.
		cancelRequested.store(false);
		return;
	}

	cancelRequested.store(false);
	task = std::make_unique<pros::Task>(
	    [this, target, timeout]() { moveToBlocking(target, timeout); });
}

bool Subsystem::addPosition(const std::string& name, float position) {
	if (name.empty()) {
		return false;
	}

	positionMutex.take(TIMEOUT_MAX);

	bool replaced = false;
	for (NamedPosition& entry : positions) {
		if (entry.name == name) {
			entry.position = position;
			replaced = true;
			break;
		}
	}

	if (!replaced) {
		positions.push_back({name, position});
	}

	// Keep the table ordered so stepping is a scan. Ties keep a stable order by
	// falling back to the name, so two names at the same height still step
	// through predictably rather than depending on insertion order.
	std::sort(positions.begin(), positions.end(),
	          [](const NamedPosition& a, const NamedPosition& b) {
		          return a.position != b.position ? a.position < b.position : a.name < b.name;
	          });

	positionMutex.give();
	return true;
}

bool Subsystem::removePosition(const std::string& name) {
	positionMutex.take(TIMEOUT_MAX);

	bool removed = false;
	for (std::size_t i = 0; i < positions.size(); i++) {
		if (positions[i].name == name) {
			positions.erase(positions.begin() + i);
			removed = true;
			break;
		}
	}

	positionMutex.give();
	return removed;
}

void Subsystem::clearPositions() {
	positionMutex.take(TIMEOUT_MAX);
	positions.clear();
	positionMutex.give();
}

bool Subsystem::lookUp(const std::string& name, float& out) const {
	positionMutex.take(TIMEOUT_MAX);

	bool found = false;
	for (const NamedPosition& entry : positions) {
		if (entry.name == name) {
			out = entry.position;
			found = true;
			break;
		}
	}

	positionMutex.give();
	return found;
}

bool Subsystem::hasPosition(const std::string& name) const {
	float ignored = 0;
	return lookUp(name, ignored);
}

bool Subsystem::tryPositionOf(const std::string& name, float& out) const {
	return lookUp(name, out);
}

float Subsystem::positionOf(const std::string& name, float fallback) const {
	float value = 0;
	return lookUp(name, value) ? value : fallback;
}

std::vector<std::string> Subsystem::positionNames() const {
	positionMutex.take(TIMEOUT_MAX);

	std::vector<std::string> names;
	names.reserve(positions.size());
	for (const NamedPosition& entry : positions) {
		names.push_back(entry.name);
	}

	positionMutex.give();
	return names;
}

std::size_t Subsystem::positionCount() const {
	positionMutex.take(TIMEOUT_MAX);
	const std::size_t count = positions.size();
	positionMutex.give();
	return count;
}

bool Subsystem::moveTo(const std::string& name, bool async, std::uint32_t timeout) {
	float target = 0;
	// Resolve first and release the lock, so the move never runs under it.
	if (!lookUp(name, target)) {
		return false;
	}

	moveTo(target, async, timeout);
	return true;
}

std::string Subsystem::nearestPosition(float tolerance) const {
	const float current = getPosition();

	positionMutex.take(TIMEOUT_MAX);

	std::string best;
	float bestDistance = 0;
	for (const NamedPosition& entry : positions) {
		const float distance = std::abs(entry.position - current);
		if (best.empty() || distance < bestDistance) {
			best = entry.name;
			bestDistance = distance;
		}
	}

	positionMutex.give();

	if (!best.empty() && tolerance >= 0 && bestDistance > tolerance) {
		return {};
	}
	return best;
}

bool Subsystem::isAtPosition(const std::string& name, float tolerance) const {
	float target = 0;
	if (!lookUp(name, target)) {
		return false;
	}
	return std::abs(clampTarget(target) - getPosition()) <= std::abs(tolerance);
}

bool Subsystem::isAtPosition(const std::string& name) const {
	float target = 0;
	if (!lookUp(name, target)) {
		return false;
	}
	return pid.isSettled(clampTarget(target) - getPosition());
}

bool Subsystem::step(bool forward, bool async, std::uint32_t timeout) {
	const float current = getPosition();

	// A step has to clear the threshold, or a mechanism sitting a hair below a
	// name would "step up" to the name it is already at and never advance.
	const float epsilon = std::max(pid.getThreshold(), 1e-3f);

	positionMutex.take(TIMEOUT_MAX);

	bool found = false;
	float target = 0;

	if (forward) {
		// Sorted ascending, so the first name above the current position wins.
		for (const NamedPosition& entry : positions) {
			if (entry.position > current + epsilon) {
				target = entry.position;
				found = true;
				break;
			}
		}
	} else {
		for (std::size_t i = positions.size(); i > 0; i--) {
			const NamedPosition& entry = positions[i - 1];
			if (entry.position < current - epsilon) {
				target = entry.position;
				found = true;
				break;
			}
		}
	}

	positionMutex.give();

	if (!found) {
		return false;
	}

	moveTo(target, async, timeout);
	return true;
}

bool Subsystem::next(bool async, std::uint32_t timeout) {
	return step(true, async, timeout);
}

bool Subsystem::previous(bool async, std::uint32_t timeout) {
	return step(false, async, timeout);
}

void Subsystem::cancelTask() {
	// A hold owns the motors just as a move does, so it has to end before
	// anything else drives them. Two tasks writing the same motors every tick
	// would fight and the mechanism would judder.
	stopHoldTask();

	if (task == nullptr) {
		return;
	}

	cancelRequested.store(true);
	task->join();
	task.reset();
	cancelRequested.store(false);
}

bool Subsystem::waitUntilSettled(std::uint32_t timeout) {
	if (task == nullptr) {
		return true;
	}

	if (timeout == NO_TIMEOUT) {
		task->join();
		task.reset();
		return true;
	}
	std::uint32_t start = pros::millis();
	std::uint32_t now = start;
	while (isMoving()) {
		if (pros::millis() - start >= timeout) {
			stop();
			return false;
		}
		pros::Task::delay_until(&now, LOOP_DELAY_MS);
	}

	task->join();
	task.reset();
	return true;
}

void Subsystem::stop() {
	cancelTask();
	brake();
}

bool Subsystem::isMoving() const {
	return task != nullptr && task->get_state() != pros::E_TASK_STATE_DELETED;
}

PID& Subsystem::getPID() {
	return pid;
}

Subsystem::~Subsystem() {
	// cancelTask() stops the hold task too, so both are joined before any
	// member the tasks touch is destroyed.
	cancelTask();
	brake();
}

}  // namespace liftlib
