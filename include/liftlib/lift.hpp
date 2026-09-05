#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <memory>
#include <utility>
#include <vector>

#include "api.h"
#include "liftlib/mechanism.hpp"
#include "liftlib/piston.hpp"
#include "liftlib/subsystem.hpp"

namespace liftlib {

/** How hard a conditional action pushes against motion that is already running. */
enum class Precedence {
	/** Never interrupts. Runs only while the lift is idle. */
	Low = 0,
	/** Waits its turn, then runs once the current move finishes. */
	Medium = 1,
	/** Interrupts an async move, but waits for a blocking one. */
	High = 2,
	/** Interrupts anything, including a blocking move. */
	Absolute = 3,
};

/** Something to do when a condition becomes true. */
struct ConditionalAction {
	/** Checked every tick. The action fires on a false to true change. */
	std::function<bool()> condition;

	/** Run when the condition fires. Keep it short, it runs on the poll task. */
	std::function<void()> action;

	Precedence precedence = Precedence::Low;

	/**
	 * Stages the action drives. Precedence is judged against these only, so a
	 * claw action is unaffected by a move that is busy with the lift.
	 *
	 * Any mechanism the lift owns can be named, pistons included, so an action
	 * that fires a clamp is not held up by a move on the arm.
	 *
	 * Leave it empty to mean every stage, which is the safe reading for an
	 * action that stops the whole lift.
	 */
	std::vector<Mechanism*> stages;

	/** Identifies the action so it can be removed later. */
	int id = 0;
};

class Lift {
   private:
	/** Everything the lift owns, in the order it was given. */
	std::vector<Mechanism*> stages;

	/** The subset that has a position, in stage order. Targets index into this. */
	std::vector<Subsystem*> positional;

	std::vector<std::pair<Subsystem*, float>> resolve(
	    const std::vector<std::pair<Subsystem*, float>>& moves) const;

	void adopt(Mechanism* stage);

	std::unique_ptr<pros::Task> task;
	std::atomic<bool> cancelRequested;

	void cancelTask();

	void stopStages();

	/** Pairs targets with the positional stages they drive, in stage order. */
	std::vector<std::pair<Subsystem*, float>> pairWithPositional(
	    const std::vector<float>& targets) const;

	void moveToBlocking(const std::vector<std::pair<Subsystem*, float>>& moves,
	                    std::uint32_t timeout);

	std::vector<ConditionalAction> conditionalActions;
	std::vector<bool> conditionWasTrue;
	std::vector<bool> actionPending;

	std::unique_ptr<pros::Task> watchTask;
	std::atomic<bool> watchStop;
	int nextActionId;

	/** Stages the current move owns, and whether that move is blocking. */
	std::vector<Mechanism*> busyStages;
	bool busyBlocking;
	mutable pros::Mutex busyMutex;

	mutable pros::Mutex actionMutex;

	void pollActions();

	void claimStages(const std::vector<Mechanism*>& claimed, bool blocking);

	void claimStages(const std::vector<Subsystem*>& claimed, bool blocking);

	void releaseStages();

	bool canRunNow(Precedence precedence, const std::vector<Mechanism*>& wanted) const;

   public:

	static constexpr std::uint32_t NO_TIMEOUT = Mechanism::NO_TIMEOUT;

	Lift(std::initializer_list<Mechanism*> stages);

	explicit Lift(std::vector<Mechanism*> stages);

	Lift(const Lift&) = delete;
	Lift& operator=(const Lift&) = delete;

	~Lift();

	void initialize();

	void update(const std::vector<float>& targets);

	void update(const std::vector<std::pair<Subsystem*, float>>& moves);

	bool isSettled() const;

	bool isSettled(const std::vector<std::pair<Subsystem*, float>>& moves) const;

	void reset();

	void brake();

	/** Settles every stage, holding against gravity where it is configured. */
	void hold();

	/**
	 * Runs an action whenever a condition becomes true.
	 *
	 * The condition is polled every 20 ms once watchConditions() is running, and
	 * the action fires on the change from false to true rather than for as long
	 * as the condition holds.
	 *
	 * Precedence is judged only against the stages the action names, so an action
	 * on the claw is unaffected by a move that is busy with the lift. Naming no
	 * stages means every stage, which is what a full stop wants.
	 *
	 * Low only runs while its stages are free, Medium waits for them, High
	 * interrupts an async move on them, and Absolute interrupts a blocking one.
	 *
	 * Returns an id for removeConditionalAction(), or 0 if the action was
	 * rejected.
	 */
	int addConditionalAction(std::function<bool()> condition, std::function<void()> action,
	                         Precedence precedence, std::vector<Mechanism*> stages);

	int addConditionalAction(std::function<bool()> condition, std::function<void()> action,
	                         Precedence precedence = Precedence::Low);

	int addConditionalAction(const ConditionalAction& action);

	/** Returns false when no action carries that id. */
	bool removeConditionalAction(int id);

	void clearConditionalActions();

	std::size_t conditionalActionCount() const;

	/** Starts polling conditions in their own task. Safe to call twice. */
	void watchConditions();

	/** Stops the polling task. Registered actions are kept. */
	void stopWatching();

	bool isWatching() const;

	/**
	 * Polls every condition once and runs whatever is due.
	 *
	 * Call this yourself from an existing loop instead of watchConditions() when
	 * you would rather not spend a task on it.
	 */
	void checkConditions();

	/**
	 * Moves every positional stage to its corresponding target.
	 *
	 * Targets line up with the stages that have a position, in the order they
	 * were given to the constructor. Pistons are skipped rather than counted,
	 * so a lift of {arm, clamp, claw} takes two targets, for the arm and claw.
	 *
	 * When async is true (the default) the motion runs in its own task and this
	 * call returns immediately. When async is false the call blocks until every
	 * stage settles or the timeout elapses.
	 *
	 * Returns false when the target count does not cover every positional stage.
	 *
	 * Starting a new move cancels any async move already in progress.
	 */
	bool moveTo(const std::vector<float>& targets, bool async = true,
	            std::uint32_t timeout = NO_TIMEOUT);

	/**
	 * Moves the named stages to their targets, leaving other stages untouched.
	 *
	 * When async is true (the default) the motion runs in its own task and this
	 * call returns immediately. When async is false the call blocks until the
	 * named stages settle or the timeout elapses.
	 *
	 * Returns false when no named stage belongs to this lift.
	 *
	 * Starting a new move cancels any async move already in progress.
	 */
	bool moveTo(std::initializer_list<std::pair<Subsystem*, float>> moves, bool async = true,
	            std::uint32_t timeout = NO_TIMEOUT);

	/** Indexes the positional stages, skipping any piston. */
	bool moveStageTo(std::size_t index, float target, bool async = true,
	                 std::uint32_t timeout = NO_TIMEOUT);

	/**
	 * Blocks until an in-progress async move finishes.
	 *
	 * Returns false when the timeout elapsed first, in which case the move is
	 * cancelled and every stage is braked. A timeout of NO_TIMEOUT waits
	 * indefinitely.
	 */
	bool waitUntilSettled(std::uint32_t timeout = NO_TIMEOUT);

	/** Stops an in-progress async move and brakes every stage. */
	void stop();

	/** True while an async move is still running. */
	bool isMoving() const;

	bool hasStage(Mechanism* stage) const;

	/** Indexes every stage, pistons included, in constructor order. */
	Mechanism* getStage(std::size_t index) const;

	std::size_t stageCount() const;

	/** Indexes only the stages that have a position, in stage order. */
	Subsystem* getPositionalStage(std::size_t index) const;

	std::size_t positionalStageCount() const;
};

}  // namespace liftlib
