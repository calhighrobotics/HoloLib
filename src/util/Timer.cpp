#include "hololib/util/Timer.hpp"
#include "pros/rtos.hpp"

namespace hololib {

Timer::Timer(uint32_t time) : m_period(time) { m_lastTime = pros::millis(); }

// The clean, centralized update helper
void Timer::update() {
    const uint32_t time = pros::millis();
    if (!m_paused) {
        m_timeWaited += time - m_lastTime;
    }
    m_lastTime = time;
}

uint32_t Timer::getTimeSet() {
    this->update();
    return m_period;
}

uint32_t Timer::getTimeLeft() {
    this->update();
    const int delta = m_period - m_timeWaited;
    return (delta > 0) ? delta : 0;
}

uint32_t Timer::getTimePassed() {
    this->update();
    return m_timeWaited;
}

bool Timer::isDone() {
    this->update();
    const int delta = m_period - m_timeWaited;
    return delta <= 0;
}

bool Timer::isPaused() {
    // Replicates the specific behavior from the first implementation's isPaused()
    const uint32_t time = pros::millis();
    if (!m_paused) {
        m_timeWaited += time - m_lastTime;
    }
    return m_paused;
}

void Timer::set(uint32_t time) {
    m_period = time;
    reset();
}

void Timer::reset() {
    m_timeWaited = 0;
    m_lastTime = pros::millis();
}

void Timer::pause() {
    if (!m_paused)
        m_lastTime = pros::millis();
    m_paused = true;
}

void Timer::resume() {
    if (m_paused)
        m_lastTime = pros::millis();
    m_paused = false;
}

void Timer::waitUntilDone() {
    do {
        pros::delay(5);
    } while (!this->isDone());
}
} // namespace hololib