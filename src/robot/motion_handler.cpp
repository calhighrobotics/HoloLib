#include "motion_handler.h"

void motionHandlerTask(void *param) {
  static_cast<MotionHandler *>(param)->loop();
}

MotionHandler::MotionHandler()
    : lastEnqueuedId(0), currentRunningId(0), onMotionStartCallback(nullptr) {
  task = new pros::Task(motionHandlerTask, this, "Motion Handler Task");
}

void MotionHandler::enqueue(std::function<void()> motion, bool async) {
  auto done = std::make_shared<bool>(false);

  mutex.take();
  lastEnqueuedId++;
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

void MotionHandler::cancelMotion() {
  mutex.take();
  if (task != nullptr) {
    task->remove();
    delete task;
    task = nullptr;
  }

  if (!queue.empty()) {
    queue.pop();
  }

  currentRunningId++;

  task = new pros::Task(motionHandlerTask, this, "Motion Handler Task");
  mutex.give();
}

void MotionHandler::cancelAll() {
  mutex.take();
  if (task != nullptr) {
    task->remove();
    delete task;
    task = nullptr;
  }

  while (!queue.empty()) {
    queue.pop();
  }

  currentRunningId = lastEnqueuedId;

  task = new pros::Task(motionHandlerTask, this, "Motion Handler Task");
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
      mutex.take();
      currentRunningId++;
      if (onMotionStartCallback) {
        onMotionStartCallback();
      }
      mutex.give();

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