#include "liftlib/lift.hpp"

#include <algorithm>

namespace liftlib {

void Lift::adopt(Mechanism* stage) {
	if (stage == nullptr || hasStage(stage)) {
		return;
	}

	stages.push_back(stage);

	// Remember which stages carry a position so target lists can index them
	// without tripping over a piston.
	if (Subsystem* subsystem = stage->asSubsystem()) {
		positional.push_back(subsystem);
	}
}

Lift::Lift(std::initializer_list<Mechanism*> stages)
    : cancelRequested(false), watchStop(false), nextActionId(0), busyBlocking(false) {
	for (Mechanism* stage : stages) {
		adopt(stage);
	}
}

Lift::Lift(std::vector<Mechanism*> stages)
    : cancelRequested(false), watchStop(false), nextActionId(0), busyBlocking(false) {
	for (Mechanism* stage : stages) {
		adopt(stage);
	}
}

void Lift::initialize() {
	for (Mechanism* stage : stages) {
		stage->initialize();
	}
}

std::vector<std::pair<Subsystem*, float>> Lift::resolve(
    const std::vector<std::pair<Subsystem*, float>>& moves) const {
	std::vector<std::pair<Subsystem*, float>> resolved;
	for (const std::pair<Subsystem*, float>& move : moves) {
		if (move.first == nullptr || !hasStage(move.first)) {
			continue;
		}

		auto existing = std::find_if(
		    resolved.begin(), resolved.end(),
		    [&move](const std::pair<Subsystem*, float>& kept) { return kept.first == move.first; });

		if (existing != resolved.end()) {
			existing->second = move.second;
		} else {
			resolved.push_back(move);
		}
	}
	return resolved;
}

void Lift::update(const std::vector<float>& targets) {
	std::size_t count = std::min(positional.size(), targets.size());
	for (std::size_t i = 0; i < count; i++) {
		positional[i]->update(targets[i]);
	}
}

void Lift::update(const std::vector<std::pair<Subsystem*, float>>& moves) {
	for (const std::pair<Subsystem*, float>& move : moves) {
		move.first->update(move.second);
	}
}

bool Lift::isSettled() const {
	for (const Mechanism* stage : stages) {
		if (!stage->isSettled()) {
			return false;
		}
	}
	return true;
}

bool Lift::isSettled(const std::vector<std::pair<Subsystem*, float>>& moves) const {
	for (const std::pair<Subsystem*, float>& move : moves) {
		if (move.first == nullptr || !move.first->isSettled()) {
			return false;
		}
	}
	return true;
}

void Lift::reset() {
	for (Mechanism* stage : stages) {
		stage->reset();
	}
}

void Lift::brake() {
	for (Mechanism* stage : stages) {
		stage->brake();
	}
}

void Lift::hold() {
	for (Mechanism* stage : stages) {
		stage->hold();
	}
}

void Lift::stopStages() {
	// Only the positional stages. A piston has no motion to halt, and stopping
	// one declares its travel over, which would make a cylinder that is still
	// moving report settled just because something else was stopped.
	for (Subsystem* stage : positional) {
		stage->stop();
	}
}

void Lift::claimStages(const std::vector<Mechanism*>& claimed, bool blocking) {
	busyMutex.take(TIMEOUT_MAX);
	busyStages = claimed;
	busyBlocking = blocking;
	busyMutex.give();
}

void Lift::claimStages(const std::vector<Subsystem*>& claimed, bool blocking) {
	std::vector<Mechanism*> widened;
	widened.reserve(claimed.size());
	for (Subsystem* stage : claimed) {
		widened.push_back(stage);
	}
	claimStages(widened, blocking);
}

void Lift::releaseStages() {
	busyMutex.take(TIMEOUT_MAX);
	busyStages.clear();
	busyBlocking = false;
	busyMutex.give();
}

bool Lift::canRunNow(Precedence precedence, const std::vector<Mechanism*>& wanted) const {
	if (precedence == Precedence::Absolute) {
		return true;
	}

	busyMutex.take(TIMEOUT_MAX);

	bool overlaps = false;
	if (!busyStages.empty()) {
		if (wanted.empty()) {
			// An action that names no stage speaks for all of them.
			overlaps = true;
		} else {
			for (Mechanism* stage : wanted) {
				if (std::find(busyStages.begin(), busyStages.end(), stage) != busyStages.end()) {
					overlaps = true;
					break;
				}
			}
		}
	}

	const bool blocking = busyBlocking;
	busyMutex.give();

	if (!overlaps) {
		return true;
	}

	if (blocking) {
		return false;
	}

	// Only High may cut in front of an async move on the same stages. Low and
	// Medium wait, and differ in what happens while they wait.
	return precedence == Precedence::High;
}

int Lift::addConditionalAction(std::function<bool()> condition, std::function<void()> action,
                               Precedence precedence, std::vector<Mechanism*> stages) {
	if (condition == nullptr || action == nullptr) {
		return 0;
	}

	// Drop anything that is not part of this lift so precedence cannot be judged
	// against a stage the lift does not drive.
	std::vector<Mechanism*> owned;
	for (Mechanism* stage : stages) {
		if (stage != nullptr && hasStage(stage) &&
		    std::find(owned.begin(), owned.end(), stage) == owned.end()) {
			owned.push_back(stage);
		}
	}

	if (owned.empty() && !stages.empty()) {
		return 0;
	}

	actionMutex.take(TIMEOUT_MAX);

	ConditionalAction entry;
	entry.condition = std::move(condition);
	entry.action = std::move(action);
	entry.precedence = precedence;
	entry.stages = std::move(owned);
	entry.id = ++nextActionId;

	conditionalActions.push_back(std::move(entry));
	conditionWasTrue.push_back(false);
	actionPending.push_back(false);

	const int id = nextActionId;
	actionMutex.give();
	return id;
}

int Lift::addConditionalAction(std::function<bool()> condition, std::function<void()> action,
                               Precedence precedence) {
	return addConditionalAction(std::move(condition), std::move(action), precedence, {});
}

int Lift::addConditionalAction(const ConditionalAction& action) {
	return addConditionalAction(action.condition, action.action, action.precedence, action.stages);
}

bool Lift::removeConditionalAction(int id) {
	actionMutex.take(TIMEOUT_MAX);

	bool removed = false;
	for (std::size_t i = 0; i < conditionalActions.size(); i++) {
		if (conditionalActions[i].id == id) {
			conditionalActions.erase(conditionalActions.begin() + i);
			conditionWasTrue.erase(conditionWasTrue.begin() + i);
			actionPending.erase(actionPending.begin() + i);
			removed = true;
			break;
		}
	}

	actionMutex.give();
	return removed;
}

void Lift::clearConditionalActions() {
	actionMutex.take(TIMEOUT_MAX);
	conditionalActions.clear();
	conditionWasTrue.clear();
	actionPending.clear();
	actionMutex.give();
}

std::size_t Lift::conditionalActionCount() const {
	actionMutex.take(TIMEOUT_MAX);
	const std::size_t count = conditionalActions.size();
	actionMutex.give();
	return count;
}

void Lift::pollActions() {
	// Copy out what is due so the callbacks run without the lock held, which
	// keeps an action free to call moveTo or add another action.
	std::vector<std::function<void()>> due;

	actionMutex.take(TIMEOUT_MAX);

	for (std::size_t i = 0; i < conditionalActions.size(); i++) {
		const ConditionalAction& entry = conditionalActions[i];

		bool nowTrue = false;
		if (entry.condition != nullptr) {
			nowTrue = entry.condition();
		}

		if (nowTrue && !conditionWasTrue[i]) {
			// Low fires only if the lift is free at that moment. The others queue
			// up and wait for their turn.
			if (entry.precedence == Precedence::Low) {
				if (canRunNow(entry.precedence, entry.stages)) {
					due.push_back(entry.action);
				}
			} else {
				actionPending[i] = true;
			}
		}
		conditionWasTrue[i] = nowTrue;

		if (actionPending[i] && canRunNow(entry.precedence, entry.stages)) {
			due.push_back(entry.action);
			actionPending[i] = false;
		}
	}

	actionMutex.give();

	for (const std::function<void()>& action : due) {
		if (action != nullptr) {
			action();
		}
	}
}

void Lift::checkConditions() {
	pollActions();
}

void Lift::watchConditions() {
	if (watchTask != nullptr) {
		return;
	}

	watchStop.store(false);
	watchTask = std::make_unique<pros::Task>([this]() {
		std::uint32_t now = pros::millis();
		while (!watchStop.load()) {
			pollActions();
			pros::Task::delay_until(&now, Mechanism::LOOP_DELAY_MS);
		}
	});
}

void Lift::stopWatching() {
	if (watchTask == nullptr) {
		return;
	}

	watchStop.store(true);
	watchTask->join();
	watchTask.reset();
	watchStop.store(false);
}

bool Lift::isWatching() const {
	return watchTask != nullptr;
}

std::vector<std::pair<Subsystem*, float>> Lift::pairWithPositional(
    const std::vector<float>& targets) const {
	// Pair each target with the stage it drives, so the blocking loop waits on
	// exactly the stages it commands. Waiting on a stage this move never drove
	// would hang until the timeout: reset() clears its settled flag and nothing
	// here would ever set it again.
	const std::size_t count = std::min(positional.size(), targets.size());

	std::vector<std::pair<Subsystem*, float>> moves;
	moves.reserve(count);
	for (std::size_t i = 0; i < count; i++) {
		moves.push_back({positional[i], targets[i]});
	}
	return moves;
}

bool Lift::moveTo(const std::vector<float>& targets, bool async, std::uint32_t timeout) {
	if (positional.empty() || targets.size() < positional.size()) {
		return false;
	}

	cancelTask();

	// Only stop what this move drives. Stopping a piston here would be wrong
	// anyway, but it also has no business being cancelled by an arm move.
	for (Subsystem* stage : positional) {
		stage->stop();
	}

	const std::vector<std::pair<Subsystem*, float>> owned = pairWithPositional(targets);

	if (!async) {
		cancelRequested.store(false);
		claimStages(positional, true);
		moveToBlocking(owned, timeout);
		releaseStages();
		// A blocking move never runs through cancelTask(), so a stop() from
		// another task would otherwise leave this latched and abort the next
		// move on its first tick.
		cancelRequested.store(false);
		return true;
	}

	cancelRequested.store(false);
	claimStages(positional, false);
	task = std::make_unique<pros::Task>([this, owned, timeout]() {
		moveToBlocking(owned, timeout);
		releaseStages();
	});
	return true;
}

void Lift::moveToBlocking(const std::vector<std::pair<Subsystem*, float>>& moves,
                          std::uint32_t timeout) {
	for (const std::pair<Subsystem*, float>& move : moves) {
		move.first->reset();
	}

	std::uint32_t start = pros::millis();
	std::uint32_t now = start;

	while (!isSettled(moves) && !cancelRequested.load()) {
		if (timeout != NO_TIMEOUT && pros::millis() - start >= timeout) {
			break;
		}
		update(moves);
		pros::Task::delay_until(&now, Mechanism::LOOP_DELAY_MS);
	}

	const bool cancelled = cancelRequested.load();
	for (const std::pair<Subsystem*, float>& move : moves) {
		if (cancelled) {
			move.first->brake();
		} else {
			// Hold at the commanded target, not at wherever the stage came to
			// rest, so droop under load is corrected instead of latched.
			move.first->holdAt(move.second);
		}
	}
}

bool Lift::moveTo(std::initializer_list<std::pair<Subsystem*, float>> moves, bool async,
                  std::uint32_t timeout) {
	std::vector<std::pair<Subsystem*, float>> resolved = resolve(moves);
	if (resolved.empty()) {
		return false;
	}

	cancelTask();
	for (const std::pair<Subsystem*, float>& move : resolved) {
		move.first->stop();
	}

	std::vector<Subsystem*> claimed;
	claimed.reserve(resolved.size());
	for (const std::pair<Subsystem*, float>& move : resolved) {
		claimed.push_back(move.first);
	}

	if (!async) {
		cancelRequested.store(false);
		claimStages(claimed, true);
		moveToBlocking(resolved, timeout);
		releaseStages();
		// See the target-list overload: a blocking move has to clear this
		// itself, or a stop() mid-move poisons the next one.
		cancelRequested.store(false);
		return true;
	}

	cancelRequested.store(false);
	claimStages(claimed, false);
	task = std::make_unique<pros::Task>([this, resolved, timeout]() {
		moveToBlocking(resolved, timeout);
		releaseStages();
	});
	return true;
}

void Lift::cancelTask() {
	if (task == nullptr) {
		return;
	}

	cancelRequested.store(true);
	task->join();
	task.reset();
	cancelRequested.store(false);
	releaseStages();
}

bool Lift::waitUntilSettled(std::uint32_t timeout) {
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
		pros::Task::delay_until(&now, Mechanism::LOOP_DELAY_MS);
	}

	task->join();
	task.reset();
	return true;
}

void Lift::stop() {
	cancelTask();
	releaseStages();
	stopStages();
	brake();
}

bool Lift::isMoving() const {
	return task != nullptr && task->get_state() != pros::E_TASK_STATE_DELETED;
}

Lift::~Lift() {
	stopWatching();
	cancelTask();
}

bool Lift::hasStage(Mechanism* stage) const {
	return std::find(stages.begin(), stages.end(), stage) != stages.end();
}

bool Lift::moveStageTo(std::size_t index, float target, bool async, std::uint32_t timeout) {
	if (index >= positional.size()) {
		return false;
	}

	cancelTask();
	positional[index]->moveTo(target, async, timeout);
	return true;
}

Mechanism* Lift::getStage(std::size_t index) const {
	return index < stages.size() ? stages[index] : nullptr;
}

std::size_t Lift::stageCount() const {
	return stages.size();
}

Subsystem* Lift::getPositionalStage(std::size_t index) const {
	return index < positional.size() ? positional[index] : nullptr;
}

std::size_t Lift::positionalStageCount() const {
	return positional.size();
}

}  
