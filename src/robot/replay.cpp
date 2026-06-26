#include "chassis.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>

/**
 *@brief Structure to store button records.
 */
struct ButtonRecord {
  pros::controller_digital_e_t button;
  uint32_t duration_ms;
  uint32_t timestamp_ms;
};

/**
 *@brief Gets controller input.
 *@param master The controller to get input from.
 *@return void
 */
void Chassis::getControllerInput(pros::Controller master) {
  static std::map<pros::controller_digital_e_t, uint32_t> pressStartTimes;
  static std::map<pros::controller_digital_e_t, bool> prevStates;
  static std::vector<ButtonRecord> buttonHistory;

  uint32_t currentTime = pros::millis();

  for (auto &btn : controllerButtons) {
    pros::controller_digital_e_t btn_enum = btn.button;
    bool isPressed = master.get_digital(btn_enum);
    bool wasPressed = prevStates[btn_enum];

    if (isPressed && !wasPressed) {
      pressStartTimes[btn_enum] = currentTime;
      if (btn.callback) {
        btn.callback();
      }
    } else if (!isPressed && wasPressed) {
      uint32_t duration = currentTime - pressStartTimes[btn_enum];
      buttonHistory.push_back({btn_enum, duration, pressStartTimes[btn_enum]});
      std::cout << "Button " << btn_enum << " held for " << duration << "ms\n";
    }
    prevStates[btn_enum] = isPressed;
  }
}

/**
 *@brief Logs replay data to a file.
 *@param master The controller to get input from.
 *@param timeout_ms The time to log data for.
 *@return void
 */
void Chassis::logReplayData(pros::Controller master, int timeout_ms) {
  pros::Task log_task([=, this]() {
    Pose lastPose = getPose();
    int safe_timeout = (timeout_ms < 20) ? 20 : timeout_ms;

    while (true) {
      Pose pose = getPose();

      double deltaX = std::abs(pose.x - lastPose.x);
      double deltaY = std::abs(pose.y - lastPose.y);
      double deltaTheta = std::abs(pose.theta - lastPose.theta);

      if (deltaX > 0.5 || deltaY > 0.5 || deltaTheta > 1.0) {
        printf("%.2f,%.2f,%.2f\n", pose.x, pose.y, pose.theta);

        lastPose = pose;
      }

      // getControllerInput(master);

      pros::delay(safe_timeout);
    }
  });
}

/**
 *@brief Runs the driver replay.
 *@param data The path data to use for the replay.
 *@param lookahead The lookahead distance for the path.
 *@return void
 */
void Chassis::runDriverReplay(std::vector<PathPoint> data, float lookahead) {
  if (data.empty()) {
    std::cout << "[runDriverReplay] Empty data provided." << std::endl;
    return;
  }

  std::vector<std::vector<PathPoint>> segments;
  std::vector<PathPoint> current_segment;

  current_segment.push_back(data[0]);

  float prev_dx = 0, prev_dy = 0;
  float prev_dist = 0;
  bool has_prev_vector = false;

  for (size_t i = 1; i < data.size(); ++i) {
    float dx = data[i].x - current_segment.back().x;
    float dy = data[i].y - current_segment.back().y;
    float dist = std::hypot(dx, dy);

    if (dist > 0.5f) {
      if (has_prev_vector) {
        float dot = (dx * prev_dx) + (dy * prev_dy);
        if (dot < (-0.5f * dist * prev_dist)) {
          segments.push_back(current_segment);
          PathPoint bridge_point = current_segment.back();
          current_segment.clear();
          current_segment.push_back(bridge_point);
        }
      }
      current_segment.push_back(data[i]);
      prev_dx = dx;
      prev_dy = dy;
      prev_dist = dist;
      has_prev_vector = true;
    } else {
      current_segment.back().theta = data[i].theta;
    }
  }

  if (current_segment.size() >= 2) {
    segments.push_back(current_segment);
  }
  bool is_reversed = false;

  for (const auto &seg : segments) {
    if (seg.size() >= 2) {
      followPath(seg, lookahead, defaultParams, HeadingMode::CustomAngles, 0.0f,
                 is_reversed);
      waitUntilDone();
      is_reversed = !is_reversed;
    }
  }
}
