#pragma once

#include "hololib/chassis.hpp"
#include "hololib/localization/odometry.hpp"
#include "hololib/util/GainScheduler.hpp"
#include "hololib/util/obstacle_manager.hpp"
#include "pros/imu.hpp"
#include "pros/motors.hpp"
#include <functional>

extern pros::Motor frontLeft;
extern pros::Motor frontRight;
extern pros::Motor backLeft;
extern pros::Motor backRight;
extern pros::Imu imu;
extern hololib::EncoderEKFOdometry odom;
extern hololib::GainScheduler xSched, ySched, thetaSched;
extern hololib::ObstacleManager obstacles;
extern hololib::Chassis chassis;
extern const std::function<hololib::Pose(bool)> poseGetter;
