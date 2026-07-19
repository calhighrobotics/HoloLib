#include "hololib/util/replay.hpp"
#include "hololib/motions/motion_handler.hpp"
#include <cmath>
#include <cstdio>
#include <map>
namespace hololib {

const std::vector<DriverReplay::ControllerButton> DriverReplay::controllerButtons{
    {pros::E_CONTROLLER_DIGITAL_UP,    []() {}},
    {pros::E_CONTROLLER_DIGITAL_DOWN,  []() {}},
    {pros::E_CONTROLLER_DIGITAL_LEFT,  []() {}},
    {pros::E_CONTROLLER_DIGITAL_RIGHT, []() {}},
    {pros::E_CONTROLLER_DIGITAL_A,     []() {}},
    {pros::E_CONTROLLER_DIGITAL_B,     []() {}},
    {pros::E_CONTROLLER_DIGITAL_X,     []() {}},
    {pros::E_CONTROLLER_DIGITAL_Y,     []() {}},
    {pros::E_CONTROLLER_DIGITAL_L1,    []() {}},
    {pros::E_CONTROLLER_DIGITAL_L2,    []() {}},
    {pros::E_CONTROLLER_DIGITAL_R1,    []() {}},
    {pros::E_CONTROLLER_DIGITAL_R2,    []() {}},
};

void DriverReplay::getControllerInput(pros::Controller master) {
    static std::map<pros::controller_digital_e_t, uint32_t> pressStartTimes;
    static std::map<pros::controller_digital_e_t, bool> prevStates;
    static std::vector<ButtonRecord> buttonHistory;

    uint32_t currentTime = pros::millis();

    for (auto& btn : controllerButtons) {
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

void DriverReplay::logReplayData(pros::Controller master, int timeout_ms, std::function<Pose(bool)> poseGetter) {
    pros::Task log_task([&]() {
        Pose lastPose = poseGetter(false);
        int safe_timeout = (timeout_ms < 20) ? 20 : timeout_ms;

        while (true) {
            Pose pose = poseGetter(false);

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

void DriverReplay::runDriverReplay(std::vector<PathPoint> data, float lookahead) {
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

    for (const auto& seg : segments) {
        if (seg.size() >= 2) {
            followPath(seg, lookahead, HeadingMode::CustomAngles, 0.0f, is_reversed);
            hololib::motion_handler::waitUntilDone();
            
            is_reversed = !is_reversed;
        }
    }
}
}