#pragma once

#include "api.h"
#include <functional>
#include <memory>
#include <queue>

/**
 * @brief Manages the execution of motion commands asynchronously.
 * 
 * The MotionHandler class allows queueing of movement commands, which are executed 
 * in a separate PROS task. This permits asynchronous operation without blocking the 
 * main control loop.
 */
class MotionHandler {
public:
  /**
   * @brief Construct a new MotionHandler object.
   */
  MotionHandler();

  /**
   * @brief Enqueue a new motion function.
   * 
   * @param motion The function to execute.
   * @param async If false, waits until the motion is done before returning.
   */
  void enqueue(std::function<void()> motion, bool async);

  /**
   * @brief Cancel all pending motions in the queue.
   */
  void cancelAll();

  /**
   * @brief Wait until all queued motions have finished executing.
   */
  void waitUntilDone();

  /**
   * @brief Cancel the currently executing motion.
   */
  void cancelMotion();

  /**
   * @brief Check if the handler is currently executing a motion.
   * 
   * @return true if a motion is currently running, false otherwise.
   */
  bool isInMotion();

  /**
   * @brief Get the ID of the last enqueued motion.
   * 
   * @return uint32_t The ID of the last motion.
   */
  uint32_t getLastEnqueuedId() const { return lastEnqueuedId; }

  /**
   * @brief Get the ID of the currently running motion.
   * 
   * @return uint32_t The ID of the currently running motion.
   */
  uint32_t getCurrentRunningId() const { return currentRunningId; }
  
  /**
   * @brief Check if the motion queue is empty.
   * 
   * @return true if the queue is empty, false otherwise.
   */
  bool isQueueEmpty() {
    mutex.take();
    bool empty = queue.empty();
    mutex.give();
    return empty;
  }

  /**
   * @brief Set a callback that is executed whenever a new motion starts.
   * 
   * @param callback The callback function.
   */
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