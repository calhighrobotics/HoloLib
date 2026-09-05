#pragma once

#include "liftlib/feedforward.hpp"
#include "liftlib/lift.hpp"
#include "liftlib/mechanism.hpp"
#include "liftlib/motor_config.hpp"
#include "liftlib/output_mode.hpp"
#include "liftlib/pid.hpp"
#include "liftlib/piston.hpp"
#include "liftlib/subsystem.hpp"

#define LIFTLIB_VERSION_MAJOR 1
#define LIFTLIB_VERSION_MINOR 0
#define LIFTLIB_VERSION_PATCH 0
#define LIFTLIB_VERSION "1.0.0"

/**
 * Define LIFTLIB_NO_GLOBAL_NAMES before including this header to keep every
 * name inside the liftlib namespace. Without it the public types are also
 * reachable unqualified, which keeps existing code working.
 *
 * MotorType is deliberately left out: pros has a type of the same name, and
 * exporting ours would make the name ambiguous for anyone who uncomments the
 * `using namespace pros` in main.h. Write liftlib::MotorType.
 */
#ifndef LIFTLIB_NO_GLOBAL_NAMES
using liftlib::ConditionalAction;
using liftlib::Feedforward;
using liftlib::GravityModel;
using liftlib::Lift;
using liftlib::Mechanism;
using liftlib::Precedence;
using liftlib::MotorConfig;
using liftlib::OutputMode;
using liftlib::PID;
using liftlib::Piston;
using liftlib::PistonConfig;
using liftlib::Subsystem;
#endif
