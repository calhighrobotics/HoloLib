#pragma once

#include <cstdint>
namespace hololib {
/**
 * @class MotionCancelHelper
 *
 * @brief This class exists to simplify the cancellation logic in a motion function.
 */
class MotionCancelHelper {
    public:
        /**
         * @brief Construct a new Motion Cancel Helper object
         *
         * @param period how often to update (in millis)
         *
         * @b Example:
         * @code {.cpp}
         * void myMotion() {
         *   // construct the cancellation helper
         *   hololib::MotionCancelHelper helper(10);
         * }
         * @endcode
         */
        MotionCancelHelper(uint32_t period);
        /**
         * @brief wait a certain amount of time
         *
         * This function will return true normally. However, if the task has been notified
         * (the motion handler requests the motion to end), or if the competition state changes,
         * the task will return false, indicating that the motion should end.
         *
         * This function is meant to be used within the while loop of a motion function.
         * While waiting, other tasks can execute.
         * The amount of time it waits is dependent on how long each iteration of the
         * while loop its in takes. See example below.
         *
         * @returns true if the motion should continue, false otherwise
         *
         * @b Example:
         * @code {.cpp}
         * void myMotion() {
         *   // create an instance of the motion cancellation helper
         *   hololib::MotionCancelHelper helper(10_msec);
         *
         *   // if the loop starts at a global time of e.g 2015 msec, and each iteration
         *   // of the while loop takes 3 msec, the loop will still iterate at 2025,
         *   // 2035, 2045, etc.
         *   while(helper.wait()) {
         *     // motion stuff here
         *   }
         *
         *  // put stuff that should happen when the motion ends here
         *  // e.g deallocating memory, log a message saying its done,
         *  // etc
         * }
         * @endcode
         */
        bool wait();
        /**
         * @brief Get the amount of time between the current iteration and the last iteration
         *
         * @return Time the time between the current iteration and the last iteration
         *
         * @b Example:
         * @code {.cpp}
         * void myMotion() {
         *   // create an instance of the motion cancellation helper
         *   hololib::MotionCancelHelper helper(10);
         *
         *   while(helper.wait()) {
         *     helper.getDelta(); // this will return 10 milliseconds unless there isn't enough CPU time
         *   }
         * }
         * @endcode
         */
        uint32_t getDelta();
    private:
        bool m_firstIteration = true;
        uint32_t m_prevTime;
        uint32_t m_prevPrevTime = 0;
        const int m_originalCompStatus;
        const uint32_t m_period;
};
} // namespace hololib