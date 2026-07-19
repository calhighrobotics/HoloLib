#include "hololib/motions/motion_cancel_helper.hpp"
#include "pros/rtos.hpp"
#include "pros/misc.h"

namespace hololib {
MotionCancelHelper::MotionCancelHelper(uint32_t period)
    : m_originalCompStatus(pros::c::competition_get_status()),
      m_prevTime(pros::millis() - period),
      m_prevPrevTime(pros::millis() - period),
      m_period(period) {}

bool MotionCancelHelper::wait() {
    m_prevPrevTime = m_prevTime;
    const std::uint32_t processedTimeout = m_period;
    // if current time - previous time > timeout
    // then set previous time to current time
    // this is to prevent the motion iterating multiple times
    // with no delay in between
    const int64_t now = int64_t(pros::millis());
    if (now - int64_t(m_prevTime) > int64_t(processedTimeout)) m_prevTime = now - processedTimeout;
    // only delay if this is not the first iteration
    if (!m_firstIteration) pros::Task::delay_until(&m_prevTime, processedTimeout);
    else m_firstIteration = false;

    // if the competition state is not the same as when the motion started, then stop the motion
    if (pros::c::competition_get_status() != m_originalCompStatus) return 0;

    // check if there was a notification
    return pros::Task::notify_take(true, 0) == 0;
}

uint32_t MotionCancelHelper::getDelta() { return pros::millis() - m_prevPrevTime; }
} // namespace hololib