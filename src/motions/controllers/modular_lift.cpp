#include "hololib/util/modular_lift.hpp"
#include <cmath>
#include <iostream>

ModularLift::ModularLift(const std::vector<LiftMotorConfig>& m_configs, LiftMechanism t, LiftConfig c)
    : type(t), config(c), is_running(false), cancel_request(false), is_settled(false) {

    for (const auto& m_config : m_configs) {
        motors.emplace_back(m_config.port, m_config.gearset);
        motors.back().set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    }
}

ModularLift::~ModularLift() {
    cancel();

    uint32_t start_wait = pros::millis();
    while (is_running && (pros::millis() - start_wait < 500)) {
        pros::delay(5);
    }

    if (task.has_value()) {
        task->remove();
        task.reset();
    }
}

float ModularLift::getLiftRadians(float motor_degrees) {
    return (motor_degrees / 360.0f) * config.gear_ratio * (2.0f * M_PI);
}

void ModularLift::setPayload(float mass_kg) { config.payload_mass_kg = mass_kg; }

float ModularLift::calculateFeedforward(float current_motor_degrees) {
    float total_mass = config.arm_mass_kg + config.payload_mass_kg;

    switch (type) {
    case LiftMechanism::CASCADE:

        return config.kG_base * total_mass * 9.81f;

    case LiftMechanism::FOUR_BAR:
    case LiftMechanism::SIX_BAR:
    case LiftMechanism::VIRTUAL: {
        float current_angle_rad = getLiftRadians(current_motor_degrees);

        return config.kG_base * total_mass * 9.81f * config.arm_length * std::cos(current_angle_rad);
    }
    default:
        return 0.0f;
    }
}

void ModularLift::moveTo(float target_motor_degrees) {
    target_mutex.take(TIMEOUT_MAX);
    target_position = target_motor_degrees;
    target_mutex.give();

    cancel_request = false;
    is_settled = false;

    startTask();
}

void ModularLift::startTask() {
    if (!task.has_value()) {
        is_running = true;
        task.emplace([this] { this->controlLoopImpl(); }, "ModularLiftTask");
    } else {
        std::cout << "[ModularLift] WARNING: Tried to start task, but it has already been started!" << std::endl;
    }
}

void ModularLift::controlLoopImpl() {
    uint32_t current_time = pros::millis();
    constexpr uint32_t LOOP_DELAY_MS = 20;

    while (!cancel_request) {
        target_mutex.take(TIMEOUT_MAX);
        float current_target = target_position;
        target_mutex.give();

        float total_pos = 0.0f;
        float total_vel = 0.0f;
        for (auto& motor : motors) {
            total_pos += motor.get_position();

            total_vel += motor.get_actual_velocity() * 6.0f;
        }
        float pos = motors.empty() ? 0.0f : total_pos / motors.size();
        float vel_deg_per_sec = motors.empty() ? 0.0f : total_vel / motors.size();

        Eigen::Vector2f x(pos, vel_deg_per_sec);
        Eigen::Vector2f x_ref(current_target, 0.0f);

        Eigen::Matrix<float, 1, 1> u_feedback = -config.K * (x - x_ref);
        float u_ff = calculateFeedforward(pos);

        float total_voltage = u_feedback(0, 0) + u_ff;

        if (total_voltage > 12000.0f)
            total_voltage = 12000.0f;
        if (total_voltage < -12000.0f)
            total_voltage = -12000.0f;

        for (auto& motor : motors) {
            motor.move_voltage(total_voltage);
        }

        if (std::abs(pos - current_target) < config.tolerance && std::abs(vel_deg_per_sec) < 10.0f) {
            is_settled = true;
        } else {
            is_settled = false;
        }

        pros::Task::delay_until(&current_time, LOOP_DELAY_MS);
    }

    is_running = false;
    is_settled = false;
}

void ModularLift::cancel() {
    cancel_request = true;
    for (auto& motor : motors) {
        motor.brake();
    }
}

void ModularLift::waitUntilDone() {
    if (!is_running)
        return;

    while (is_running && !is_settled) {
        pros::delay(10);
    }
}

bool ModularLift::isRunning() { return is_running; }