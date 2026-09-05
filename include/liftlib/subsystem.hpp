#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "api.h"
#include "liftlib/feedforward.hpp"
#include "liftlib/mechanism.hpp"
#include "liftlib/motor_config.hpp"
#include "liftlib/output_mode.hpp"
#include "liftlib/pid.hpp"

namespace liftlib {

struct MotorSlot final {
	pros::Motor motor;
	MotorSlot(std::int8_t port, pros::MotorGears gearset) : motor(port, gearset) {}
};

class Subsystem : public Mechanism {
   public:
	/**
	 * One set of gains and the position they were tuned at.
	 *
	 * Written as {pid, position} so a schedule reads as a list of pairs:
	 *
	 *   {{PID(0.5, 0, 2, 1), 0}, {PID(1.4, 0, 5, 1), 90}}
	 */
	using GainPoint = std::pair<PID, float>;

   protected:
	std::vector<MotorConfig> motorConfigs;
	std::vector<MotorSlot> motors;

	/**
	 * Gain points sorted by position, empty when no schedule is active.
	 *
	 * Declared before pid because the scheduling constructors derive pid from
	 * it, and members initialize in declaration order.
	 *
	 * Only kP, kI, and kD are read from these; every other PID setting lives on
	 * the live controller, so tuning a threshold or a slew after construction is
	 * not undone by the next gain selection.
	 */
	std::vector<GainPoint> gainSchedule;

	PID pid;
	Feedforward feedforward;

	float minPosition;
	float maxPosition;

	bool limited;

   public:
	using Mechanism::LOOP_DELAY_MS;
	using Mechanism::NO_TIMEOUT;

	static constexpr int SETTLE_COUNT = 5;

	/**
	 * Largest magnitude a position read is believed at.
	 *
	 * PROS reports a failed read as PROS_ERR (INT32_MAX), which arrives here
	 * scaled by a gear ratio or a user's unit conversion rather than as the
	 * exact sentinel. No real mechanism travels this far in any unit, so a
	 * reading past it is a fault rather than a position.
	 */
	static constexpr float POSITION_SANITY_LIMIT = 1e6f;

	/**
	 * Reads the mechanism's position, in whatever units the subsystem commands.
	 *
	 * Return the position directly; the gear ratios in MotorConfig are not
	 * applied to it, since an external sensor does not sit behind the gearing
	 * the motors do.
	 */
	using PositionSource = std::function<float()>;

	Subsystem(std::vector<MotorConfig> motorConfigs, PID pid);

	Subsystem(std::vector<MotorConfig> motorConfigs, PID pid, float minPosition, float maxPosition);

	/**
	 * Schedules gains by position, for a mechanism that needs different tuning
	 * at different points in its travel.
	 *
	 * A lift is usually floppier extended than retracted, so one set of gains is
	 * either too soft at the top or too aggressive at the bottom. Tune a PID at
	 * each of a few heights and pass them with the position each was tuned at:
	 *
	 *   Subsystem arm({motors}, {
	 *       {PID(0.5f, 0, 2.0f, 1), 0},
	 *       {PID(0.9f, 0, 3.0f, 1), 45},
	 *       {PID(1.4f, 0, 5.0f, 1), 90},
	 *   });
	 *
	 * kP, kI, and kD are interpolated linearly between the two surrounding
	 * points, so gains change smoothly rather than stepping at a boundary.
	 * Outside the outermost points the nearest one's gains are held flat.
	 *
	 * Points may be given in any order; they are sorted by position. Two points
	 * at the same position are a contradiction about what the gains there are,
	 * so the later one is dropped.
	 *
	 * Everything other than kP, kI, and kD is taken from the point nearest the
	 * bottom of the travel and applies throughout: threshold, slew, integral
	 * limits, and output limit are properties of the mechanism rather than of a
	 * position, and a settle threshold that changed with position would make
	 * isSettled() mean different things at different heights.
	 *
	 * A schedule of one point behaves exactly like the single-PID constructor.
	 * An empty schedule is not a controller at all, so it yields a zero-gain PID
	 * that holds the mechanism still rather than driving it somewhere arbitrary.
	 */
	Subsystem(std::vector<MotorConfig> motorConfigs, std::vector<GainPoint> schedule);

	Subsystem(std::vector<MotorConfig> motorConfigs, std::vector<GainPoint> schedule,
	          float minPosition, float maxPosition);

	~Subsystem() override;

	Subsystem* asSubsystem() override {
		return this;
	}

	void initialize() override;

	virtual float getPosition() const;

	/**
	 * Reads position from a sensor instead of the motor encoders.
	 *
	 * A rotation sensor or potentiometer on the joint sees the mechanism
	 * directly, so it is free of the backlash and slip between the motor and
	 * the load that motor encoders accumulate:
	 *
	 *   arm.setPositionSource([&] { return rotation.get_angle() / 100.0f; });
	 *
	 * The callback runs once per control tick, from whichever task is driving
	 * the subsystem, so it must be cheap and safe to call from a task. Anything
	 * it captures by reference has to outlive the subsystem.
	 *
	 * Units must match the ones used for targets and soft limits. Gear ratios
	 * are not applied, since the sensor is already reading the output side.
	 *
	 * initialize() no longer tares the motors once a source is set, because the
	 * source defines zero and taring would not move it.
	 *
	 * Cancels an async move in progress, since that move would otherwise be
	 * calling the old source from its own task while it is being replaced.
	 * Set the source during setup, not mid-routine.
	 */
	void setPositionSource(PositionSource source);

	/** Reverts to averaging the motor encoders. Cancels an async move. */
	void clearPositionSource();

	bool hasPositionSource() const;

	float clampTarget(float target) const;

	virtual void update(float target);

	bool isSettled() const override;

	bool isSettledAt(float target) const;

	void reset() override;

	void brake() override;

	/**
	 * Settles the subsystem at its current position: holds against gravity when
	 * feedforward is configured, otherwise brakes.
	 *
	 * The first call latches the position, so calling this every tick keeps the
	 * loop closed around one target. A single call latches the holding output
	 * instead, which is enough to stop a lift sagging after a blocking move.
	 */
	void hold() override;

	/**
	 * Holds at a commanded target rather than at the measured position.
	 *
	 * This is what a finished move wants: a mechanism that settles a little low
	 * under load should keep being pulled towards where it was sent, not have
	 * that droop latched in as the new target and compounded by the next move.
	 *
	 * Latches on the first call exactly as hold() does, so calling it every tick
	 * closes the loop around one target.
	 */
	void holdAt(float target);

	/**
	 * Keeps the loop closed around a target in a task of its own.
	 *
	 * A single hold() writes one output and stops there, which a loaded lift
	 * answers by sagging. This drives the controller every tick until something
	 * commands otherwise, so the mechanism actually stays put.
	 *
	 * The hold task is separate from the move task on purpose: isMoving() means
	 * "a move is running", and a hold is not a move. A held subsystem reports
	 * settled and not moving, so waitUntilSettled() returns as it always did.
	 *
	 * Any new move, stop(), brake(), or releaseHold() ends the hold. Starting a
	 * hold while one is already running re-latches it on the new target.
	 *
	 * Does nothing without a feedforward configured: with no gravity term there
	 * is nothing for the hold to push with, and the brake already does the job.
	 */
	void holdActively(float target);

	/** Holds actively wherever the mechanism is now. */
	void holdActively();

	/** True while the hold task is driving the mechanism. */
	bool isHoldingActively() const;

	/** Forgets the latched hold position so the next hold() latches afresh. */
	void releaseHold();

	bool isHolding() const;

	/** Drives the motors directly, bypassing the PID. */
	virtual void setOutput(float output);

	/**
	 * Chooses how output values reach the motors.
	 *
	 * Voltage (the default) treats output as a -127..127 command applied open
	 * loop. Velocity treats it as RPM and hands it to the motor's own
	 * controller.
	 *
	 * Changing the mode rescales what every gain means, so retune kP, kD, and
	 * kG after switching. The PID's output limit is rescaled for you when it
	 * still holds the value this subsystem derived; an explicit setMaxOutput()
	 * is left alone, since only you know what it meant.
	 */
	void setOutputMode(OutputMode mode);

	OutputMode getOutputMode() const;

	/** The largest output the current mode accepts: 127, or the gearset's RPM. */
	float outputLimit() const;

	/**
	 * Splits output between motors by their stall torque.
	 *
	 * Off by default, and only meaningful in a group that mixes an 11W with a
	 * 5.5W. With it off both motors get the same command, so each contributes
	 * whatever it physically can; that is usually what you want, since a 5.5W
	 * commanded to full is already giving everything it has.
	 *
	 * Turn it on when the 5.5W is the one overheating. It scales the smaller
	 * motor's command down so the 11W carries proportionally more of the load,
	 * at the cost of some total output.
	 *
	 * Has no effect on a group of one motor type.
	 */
	void setTorqueSharing(bool enabled);

	bool getTorqueSharing() const;

	/** True when the group holds more than one motor type. */
	bool isMixedGroup() const;

	/** How many motors of a type the group holds. */
	std::size_t motorCountOfType(MotorType type) const;

	bool hasLimits() const;

	float getMinPosition() const;

	float getMaxPosition() const;

	/**
	 * Moves the subsystem to a target position.
	 *
	 * When async is true (the default) the motion runs in its own task and this
	 * call returns immediately. When async is false the call blocks until the
	 * subsystem settles or the timeout elapses.
	 *
	 * A timeout of NO_TIMEOUT waits indefinitely.
	 *
	 * Starting a new move cancels any async move already in progress.
	 */
	virtual void moveTo(float target, bool async = true, std::uint32_t timeout = NO_TIMEOUT);

	/**
	 * Names a position so routines can ask for it by meaning rather than by
	 * number.
	 *
	 * Adding a name that already exists moves it, which is what you want when
	 * tuning a height without restarting.
	 *
	 * The position is stored as given. Soft limits are applied when a move
	 * starts, not here, so a name added before the limits are known still ends
	 * up clamped correctly.
	 *
	 * Names are compared exactly, including case. Returns false only for an
	 * empty name.
	 */
	bool addPosition(const std::string& name, float position);

	/** Returns false when no position carries that name. */
	bool removePosition(const std::string& name);

	void clearPositions();

	bool hasPosition(const std::string& name) const;

	/**
	 * The position stored under a name, or fallback when there is none.
	 *
	 * Deliberately not an overload of getPosition(): that one is virtual, and a
	 * subclass overriding it would hide this one, which is a confusing error to
	 * land on.
	 *
	 * Prefer tryPositionOf() where telling "missing" from "stored zero"
	 * matters.
	 */
	float positionOf(const std::string& name, float fallback = 0) const;

	/** Writes the stored position into out and returns whether it existed. */
	bool tryPositionOf(const std::string& name, float& out) const;

	/** Every name, sorted by position, lowest first. */
	std::vector<std::string> positionNames() const;

	std::size_t positionCount() const;

	/**
	 * Moves to a named position.
	 *
	 * Returns false and does nothing when the name is unknown, so a typo
	 * cannot silently drive the mechanism to zero.
	 *
	 * Behaves exactly like the numeric moveTo once the name resolves: the same
	 * cancellation, timeout, and settling rules apply.
	 */
	bool moveTo(const std::string& name, bool async = true,
	            std::uint32_t timeout = NO_TIMEOUT);

	/**
	 * The name whose position is closest to where the mechanism is now, or an
	 * empty string when no names are stored.
	 *
	 * Only counts a name within tolerance; pass a negative tolerance to accept
	 * the nearest one no matter how far away it is.
	 */
	std::string nearestPosition(float tolerance = -1) const;

	/** True when the mechanism is within tolerance of that named position. */
	bool isAtPosition(const std::string& name, float tolerance) const;

	/**
	 * True when the mechanism is within the PID's settle threshold of that
	 * named position.
	 */
	bool isAtPosition(const std::string& name) const;

	/**
	 * Moves to the next named position above where the mechanism is now.
	 *
	 * This is the button-cycling case: bind next() to one button and
	 * previous() to another and a lift steps through its heights in order.
	 *
	 * Stepping is measured from the current position rather than from the last
	 * name commanded, so a mechanism the driver has dragged out of place still
	 * steps to the right neighbour.
	 *
	 * Returns false when already at or above the highest name, so a held
	 * button does not wrap around to the bottom.
	 */
	bool next(bool async = true, std::uint32_t timeout = NO_TIMEOUT);

	/** The mirror of next(). Returns false at or below the lowest name. */
	bool previous(bool async = true, std::uint32_t timeout = NO_TIMEOUT);

	/**
	 * Fraction of the output range the feedforward may claim, 0 to 1.
	 *
	 * The gravity term and the PID term share one output range, so on a heavy
	 * mechanism a large kG can consume most of it and leave the PID unable to
	 * push further in that direction. Capping the feedforward reserves the rest
	 * for the controller.
	 *
	 * Defaults to 0.8, so the PID always keeps at least a fifth of the range.
	 * Raise it towards 1 if the mechanism genuinely needs almost all of the
	 * output just to hold station, though that is usually a sign of being
	 * geared too fast for the load.
	 */
	void setFeedforwardHeadroom(float fraction);

	float getFeedforwardHeadroom() const;

	/**
	 * Blocks until an in-progress async move finishes.
	 *
	 * Returns false when the timeout elapsed first, in which case the move is
	 * cancelled and the motors are braked. A timeout of NO_TIMEOUT waits
	 * indefinitely.
	 */
	bool waitUntilSettled(std::uint32_t timeout = NO_TIMEOUT);

	/** Stops an in-progress async move and brakes the motors. */
	void stop() override;

	/** True while an async move is still running. */
	bool isMoving() const override;

	/**
	 * The live controller, carrying whichever gains the schedule last selected.
	 *
	 * Setting gains on it directly is overwritten on the next tick when a
	 * schedule is active; change the schedule instead.
	 */
	PID& getPID();

	/**
	 * Replaces the gain schedule. Sorting, deduplication, and interpolation work
	 * exactly as in the constructor.
	 *
	 * Unlike the constructor, only kP, kI, and kD are taken from the points. The
	 * live controller keeps its threshold, slew, integral limits, and output
	 * limit, since by now those may have been tuned on this subsystem and are
	 * not the schedule's to overwrite. Set them through getPID() as usual.
	 *
	 * An empty schedule leaves the gains alone rather than zeroing them; use
	 * clearGainSchedule() to stop scheduling.
	 *
	 * Cancels an async move, since that move is reading gains from its own task
	 * every tick and would otherwise be selecting from a schedule being rebuilt
	 * underneath it.
	 */
	void setGainSchedule(std::vector<GainPoint> schedule);

	/**
	 * Stops scheduling. The gains last selected stay in force everywhere, so the
	 * mechanism keeps running under the tune it had at the moment of the call
	 * rather than jumping to a different one.
	 */
	void clearGainSchedule();

	/** True when more than one gain point is scheduled. */
	bool hasGainSchedule() const;

	/** The scheduled points, sorted by position. */
	const std::vector<GainPoint>& getGainSchedule() const;

	/**
	 * The gains the schedule would select at a position, for checking a tune
	 * without having to drive the mechanism there.
	 */
	PID gainsAt(float position) const;

	/**
	 * Gravity compensation, added to the PID output so the controller does not
	 * have to build up error before it holds position.
	 *
	 * Use Feedforward::constant for a DR4B or cascade, Feedforward::cosine for a
	 * pivoting arm.
	 */
	void setFeedforward(const Feedforward& feedforward);

	Feedforward& getFeedforward();

	/** The holding output at the current position, before any PID term. */
	float holdOutput() const;

   private:
	float lastError;
	bool settled;
	int settleTicks;

	float holdTarget;
	bool holding;

	OutputMode outputMode;

	/** Output limit this subsystem derived, so setOutputMode knows to rescale. */
	float derivedOutputLimit;

	bool torqueSharing;

	/**
	 * Per-motor output multiplier, parallel to motorConfigs.
	 *
	 * All ones unless torque sharing is on in a mixed group, so the common case
	 * costs one multiply by 1.0f rather than a branch per motor.
	 */
	std::vector<float> outputScales;

	void buildOutputScales();

	struct NamedPosition {
		std::string name;
		float position;
	};

	/**
	 * Named positions, kept sorted by position so next() and previous() are a
	 * scan rather than a sort on every button press.
	 */
	std::vector<NamedPosition> positions;

	/**
	 * Guards the name table only.
	 *
	 * Every public entry point copies what it needs out from under this lock
	 * and releases it before doing anything else, so it is never held across a
	 * move, a motor write, or a user callback. That keeps it impossible to
	 * deadlock against the move task or Lift's action poll.
	 */
	mutable pros::Mutex positionMutex;

	/** Resolves a name to a position. Caller must NOT hold positionMutex. */
	bool lookUp(const std::string& name, float& out) const;

	/** Shared body of next() and previous(). */
	bool step(bool forward, bool async, std::uint32_t timeout);

	PositionSource positionSource;

	/**
	 * The last position that read as usable.
	 *
	 * A faulted sensor freezes the subsystem here rather than reporting zero,
	 * which would make the error the whole target and drive the mechanism into
	 * a hard stop. Mutable because getPosition() is const and is the only place
	 * a fresh sample arrives.
	 */
	mutable float lastPosition;

	/** False for NaN, infinity, and magnitudes only a failed read produces. */
	static bool isUsablePosition(float position);

	/**
	 * Points the schedule at a position, writing the interpolated gains onto the
	 * live PID.
	 *
	 * Assigns the gain fields directly rather than through setGains(), which
	 * resets the integral, the previous error, and the slew memory. Selecting
	 * gains happens every tick, so resetting there would zero the derivative
	 * term continuously and make kD do nothing.
	 */
	void selectGains(float position);

	/**
	 * The interpolated gains at a position, shared by selectGains and gainsAt.
	 *
	 * Returns false when no schedule is active or the position is unusable, in
	 * which case the outputs are left alone.
	 */
	bool lookUpGains(float position, float& kp, float& ki, float& kd) const;

	/** Writes kP, kI, and kD onto the live PID without disturbing its state. */
	void applyGains(float kp, float ki, float kd);

	/** Sorts by position and drops duplicates. Shared by the ctors and setter. */
	static std::vector<GainPoint> normalizeSchedule(std::vector<GainPoint> schedule);

	/** The controller a schedule starts from: its lowest point, or zero gains. */
	static PID baseOf(const std::vector<GainPoint>& schedule);

	float feedforwardHeadroom;

	std::unique_ptr<pros::Task> task;
	std::atomic<bool> cancelRequested;

	/**
	 * Drives the hold loop, separate from the move task.
	 *
	 * Kept apart so isMoving() can keep meaning "a move is running": a hold is
	 * not a move, and folding it into the move task would make every finished
	 * move report as still moving and hang waitUntilSettled().
	 */
	std::unique_ptr<pros::Task> holdTask;
	std::atomic<bool> holdStop;

	/**
	 * Guards holdTask itself.
	 *
	 * An async move starts its hold from inside the move task, while the next
	 * command stops it from the caller's task, so the pointer is written and
	 * read from two tasks. Only the pointer is guarded: the lock is never held
	 * across a join, or stopping a hold would block behind the move that is
	 * starting one.
	 */
	mutable pros::Mutex holdMutex;

	/** Target the hold task drives towards. Written before the task starts. */
	std::atomic<float> activeHoldTarget;

	/** Stops the hold task and joins it. Safe when no hold is running. */
	void stopHoldTask();

	void buildMotors();

	void cancelTask();

	void moveToBlocking(float target, std::uint32_t timeout);

	/**
	 * The body of update(), given a position already read.
	 *
	 * Keeping the read in the caller lets hold() latch and drive from one
	 * sample, so a position source is invoked once per tick rather than twice
	 * with a chance of disagreeing in between.
	 */
	void updateAt(float target, float position);

	/** Sends an output value to the motors in whichever mode is configured. */
	void writeOutput(float output) const;

	/** The gearset velocity ceiling, used as the limit in Velocity mode. */
	float velocityCeiling() const;
};

}  // namespace liftlib
