#pragma once

#include "api.h"
#include "hololib/pose.h"
#include <functional>
#include <vector>

/**
 * @brief A snapshot of joystick and pose data for macro recording/playback.
 */
struct ReplayData {
  float forwards, sideways, rotation; /**< Raw joystick outputs */
  Pose pose;                          /**< Captured pose at the time */
};

/**
 * @brief Links a controller button to a lambda callback function.
 */
struct ControllerButton {
  pros::controller_digital_e_t button; /**< The specific controller button */
  std::function<void()> callback; /**< The callback to execute when pressed */
};

/**
 * @brief List of predefined controller button callbacks.
 */
inline std::vector<ControllerButton> controllerButtons{
    {pros::E_CONTROLLER_DIGITAL_UP, []() {}},
    {pros::E_CONTROLLER_DIGITAL_DOWN, []() {}},
    {pros::E_CONTROLLER_DIGITAL_LEFT, []() {}},
    {pros::E_CONTROLLER_DIGITAL_RIGHT, []() {}},
    {pros::E_CONTROLLER_DIGITAL_A, []() {}},
    {pros::E_CONTROLLER_DIGITAL_B, []() {}},
    {pros::E_CONTROLLER_DIGITAL_X, []() {}},
    {pros::E_CONTROLLER_DIGITAL_Y, []() {}},
    {pros::E_CONTROLLER_DIGITAL_L1, []() {}},
    {pros::E_CONTROLLER_DIGITAL_L2, []() {}},
    {pros::E_CONTROLLER_DIGITAL_R1, []() {}},
    {pros::E_CONTROLLER_DIGITAL_R2, []() {}},

};
