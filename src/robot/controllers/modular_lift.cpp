#include "Subsystems/modular_lift.h"
#include <cmath>
#include <iostream>


ModularLift::ModularLift(pros::MotorGroup* m_group, LiftMechanism t, LiftConfig c)
    : motors(m_group), type(t), config(c) {
    motors->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}

ModularLift::~ModularLift() {
    cancel();
    if (task != nullptr) {
        delete task;
    }
}

float ModularLift::getLiftRadians(float motor_degrees) {
    return (motor_degrees / 360.0f) * config.gear_ratio * (2.0f * M_PI);
}

float ModularLift::calculateFeedforward(float current_motor_degrees) {
    switch (type) {
        case LiftMechanism::CASCADE:
            return config.kG; 

        case LiftMechanism::FOUR_BAR:
        case LiftMechanism::SIX_BAR:
        case LiftMechanism::VIRTUAL: {
            float current_angle_rad = getLiftRadians(current_motor_degrees);
            return config.kG * std::cos(current_angle_rad);
        }
        default:
            return 0.0f;
    }
}

void ModularLift::moveTo(float target_motor_degrees) {
    target_mutex.take(TIMEOUT_MAX);
    target_position = target_motor_degrees;
    target_mutex.give();
    if (is_running) {
        return; 
    }

    is_running = true;
    cancel_request = false;

    TaskParams* params = new TaskParams{this};
    task = new pros::Task(task_trampoline, params, "ModularLiftTask");
    
    if (task == nullptr) {
        delete params;
        is_running = false;
        std::cout << "[Lift] Failed to start task!" << std::endl;
    }
}

void ModularLift::task_trampoline(void* params) {
    TaskParams* p = static_cast<TaskParams*>(params);
    if (p && p->instance) {
        p->instance->controlLoopImpl();
    }
    delete p;
}

void ModularLift::controlLoopImpl() {
    uint32_t current_time = pros::millis();
    constexpr uint32_t LOOP_DELAY_MS = 20;

    while (!cancel_request) {
        target_mutex.take(TIMEOUT_MAX);
        float current_target = target_position;
        target_mutex.give();

        float pos = motors->get_position();
        float vel = motors->get_actual_velocity();
        
        float vel_deg_per_sec = vel * 6.0f; 

        Eigen::Vector2f x(pos, vel_deg_per_sec);
        Eigen::Vector2f x_ref(current_target, 0.0f); 
    
        Eigen::Matrix<float, 1, 1> u_feedback = -config.K * (x - x_ref);
        
        float u_ff = calculateFeedforward(pos);
        
        float total_voltage = u_feedback(0,0) + u_ff;
        
        if (total_voltage > 12000.0f) total_voltage = 12000.0f;
        if (total_voltage < -12000.0f) total_voltage = -12000.0f;

        motors->move_voltage(total_voltage);

        if (std::abs(pos - current_target) < config.tolerance && std::abs(vel_deg_per_sec) < 10.0f) {
            motors->move_voltage(u_ff); 
            break; 
        }

        pros::Task::delay_until(&current_time, LOOP_DELAY_MS);
    }

    is_running = false;
}

void ModularLift::cancel() {
    cancel_request = true;
    motors->brake(); 
}

void ModularLift::waitUntilDone() {
    while (is_running) {
        pros::delay(10);
    }
}

bool ModularLift::isRunning() {
    return is_running;
}