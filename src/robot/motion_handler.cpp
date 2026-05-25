#include "motion_handler.h"

void motionHandlerTask(void *param) {
  static_cast<MotionHandler *>(param)->loop();
}

MotionHandler::MotionHandler() {
  task = new pros::Task(motionHandlerTask, this, "Motion Handler Task");
}

void MotionHandler::enqueue(std::function<void()> motion, bool async) {
  auto done = std::make_shared<bool>(false);

  mutex.take();
  queue.push([motion, done]() {
    motion();
    *done = true;
  });
  mutex.give();

  if (!async) {

    while (!*done) {
      pros::delay(10);
    }
  }
}

void MotionHandler::cancelAll() {
  mutex.take();
  while (!queue.empty()) {
    queue.pop();
  }
  mutex.give();
}

void MotionHandler::waitUntilDone() {
  while (true) {
    mutex.take();
    bool empty = queue.empty();
    mutex.give();

    if (empty)
      break;
    pros::delay(10);
  }
}

void MotionHandler::loop() {
  while (true) {
    std::function<void()> currentMotion = nullptr;

    mutex.take();
    if (!queue.empty()) {
      currentMotion = queue.front();
    }
    mutex.give();

    if (currentMotion) {
      currentMotion();

      mutex.take();
      if (!queue.empty()) {
        queue.pop();
      }
      mutex.give();
    } else {
      pros::delay(10);
    }
  }
}
