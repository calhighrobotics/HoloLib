#pragma once

#include "api.h"
#include <functional>
#include <memory>
#include <queue>

class MotionHandler {
public:
  MotionHandler();

  void enqueue(std::function<void()> motion, bool async);

  void cancelAll();

  void waitUntilDone();

  void cancelMotion();

  bool isInMotion();

  uint32_t getLastEnqueuedId() const { return lastEnqueuedId; }
  uint32_t getCurrentRunningId() const { return currentRunningId; }
  
  bool isQueueEmpty() {
    mutex.take();
    bool empty = queue.empty();
    mutex.give();
    return empty;
  }

  void setOnMotionStart(std::function<void()> callback) {
    onMotionStartCallback = callback;
  }

private:
  std::queue<std::function<void()>> queue;
  pros::Mutex mutex;
  pros::Task *task = nullptr;
  uint32_t lastEnqueuedId = 0;
  uint32_t currentRunningId = 0;
  std::function<void()> onMotionStartCallback = nullptr;
  bool running = false;

  void loop();
  friend void motionHandlerTask(void *param);
};