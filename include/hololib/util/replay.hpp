#pragma once

#include "hololib/localization/odometry.hpp"
#include "hololib/motions/motions.hpp"
#include <functional>
#include <vector>

namespace hololib {

class DriverReplay {
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
        std::function<void()> callback;      /**< The callback to execute when pressed */
    };
    // List of controller buttons and their callbacks
    static const std::vector<ControllerButton> controllerButtons;

    /**
     *@brief Structure to store button records.
     */
    struct ButtonRecord {
        pros::controller_digital_e_t button;
        uint32_t duration_ms;
        uint32_t timestamp_ms;
    };

public:
    /**
     *@brief Gets controller input.
     *@param master The controller to get input from.
     *@return void
     */
    static void getControllerInput(pros::Controller master);

    /**
     *@brief Logs replay data to a file.
     *@param master The controller to get input from.
     *@param timeout_ms The time to log data for.
     *@return void
     */
    static void logReplayData(pros::Controller master, int timeout_ms, std::function<Pose(bool)> poseGetter);

    /**
     *@brief Runs the driver replay.
     *@param data The path data to use for the replay.
     *@param lookahead The lookahead distance for the path.
     *@return void
     */
    static void runDriverReplay(std::vector<PathPoint> data, float lookahead);
};
} // namespace hololib