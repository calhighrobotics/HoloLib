#include "pros/rtos.hpp"
#include <mutex>
#include <print>

// The following code uses the motion handler from LemLib, licensed under MIT.
// Check out their repository here: https://github.com/LemLib/LemLib/

namespace hololib::motion_handler {

constexpr uint32_t NOTIFICATION_TIMEOUT = std::numeric_limits<std::uint32_t>::max();

static std::optional<std::function<void(void)>> _motion;
static pros::Mutex _mutex;
static uint32_t _priority = TASK_PRIORITY_DEFAULT;

// motion task
static pros::Task _motionTask([] {
    while (pros::Task::notify_take(true, NOTIFICATION_TIMEOUT)) {
        std::lock_guard lock(_mutex); // get mutex
        pros::Task::current().set_priority(_priority); // set priority back to regular value
        // run motion. _motion may legitimately be nullopt here if cancel() notified after the
        // previous motion completed naturally (benign race), so just skip in that case.
        if (_motion.has_value()) {
            try {
                _motion.value()();
            } catch (const std::exception& e) {
                std::println("motion threw an exception: {}", e.what());
            } catch (...) {
                std::println("motion threw an unknown exception");
            }
        }
        // set motion to nullopt
        _motion = std::nullopt;
    }
});

void move(std::function<void(void)> motion, std::optional<uint32_t> priority) {
    std::lock_guard lock(_mutex); // wait for any running motion to finish
    // run the motion
    _motion = motion;
    // set the priority of the task
    _priority = priority.value_or(pros::Task::current().get_priority());
    _motionTask.set_priority(TASK_PRIORITY_MAX); // temporarily set the motion task priority to max
    // notify the motion task. Since it's at MAX priority, it will preempt and start running
    // as soon as this function returns and the lock_guard releases the mutex.
    _motionTask.notify();
}

bool isMoving() {
    // if the mutex is free, no motion is running
    if (_mutex.take(0)) {
        _mutex.give();
        return false;
    }
    return true;
}

void cancel() {
    // if the task is currently running a motion, notify it so the motion can break out of its loop
    if (isMoving()) _motionTask.notify();
}

void waitUntilDone() {
    while (isMoving()) {
        pros::delay(5);
    }
}

} // namespace lemlib::motion_handler