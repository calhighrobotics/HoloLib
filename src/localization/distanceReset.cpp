#include "distanceReset.h"
#include <numeric>
#include <cmath>
#include <iostream>
#include "chassis.h"

distancePose DistanceReset::calculateGlobalPosition(
    const std::vector<DistanceSensor>& active_sensors,
    double heading_deg,
    double current_x,
    double current_y) 
{
    double est_x = current_x;
    double est_y = current_y;
    
    bool use_pos_x_wall = (est_x >= 0);
    bool use_pos_y_wall = (est_y >= 0);

    std::vector<double> x_cands;
    std::vector<double> y_cands;

    double heading_rad = heading_deg * (M_PI / 180.0);
    double cos_h = std::cos(heading_rad);
    double sin_h = std::sin(heading_rad);

    for (const auto& sensor : active_sensors) {
        if (!sensor.device) continue;

        double dist_in = sensor.device->get_distance() * MM_TO_IN;
        if (dist_in >= MAX_SENSOR_RANGE || dist_in <= MIN_SENSOR_RANGE) continue;

        double global_angle_deg = std::fmod(heading_deg + sensor.mounting_angle, 360.0);
        if (global_angle_deg < 0) global_angle_deg += 360.0;

        double global_offset_x = (sensor.forward_offset * sin_h) + (sensor.strafe_offset * cos_h);
        double global_offset_y = (sensor.forward_offset * cos_h) - (sensor.strafe_offset * sin_h);

        double diff_0   = std::min(std::fabs(global_angle_deg - 0.0), std::fabs(global_angle_deg - 360.0));
        double diff_90  = std::fabs(global_angle_deg - 90.0);
        double diff_180 = std::fabs(global_angle_deg - 180.0);
        double diff_270 = std::fabs(global_angle_deg - 270.0);

        if (diff_0 <= default_heading_tolerance && use_pos_y_wall) {
            double angle_err_rad = (global_angle_deg > 180 ? global_angle_deg - 360.0 : global_angle_deg) * (M_PI / 180.0);
            y_cands.push_back(HALF_HEIGHT - (dist_in * std::cos(angle_err_rad)) - global_offset_y);
        } 
        else if (diff_90 <= default_heading_tolerance && use_pos_x_wall) {
            double angle_err_rad = (global_angle_deg - 90.0) * (M_PI / 180.0);
            x_cands.push_back(HALF_WIDTH - (dist_in * std::cos(angle_err_rad)) - global_offset_x);
        } 
        else if (diff_180 <= default_heading_tolerance && !use_pos_y_wall) {
            double angle_err_rad = (global_angle_deg - 180.0) * (M_PI / 180.0);
            y_cands.push_back(-HALF_HEIGHT + (dist_in * std::cos(angle_err_rad)) - global_offset_y);
        } 
        else if (diff_270 <= default_heading_tolerance && !use_pos_x_wall) {
            double angle_err_rad = (global_angle_deg - 270.0) * (M_PI / 180.0);
            x_cands.push_back(-HALF_WIDTH + (dist_in * std::cos(angle_err_rad)) - global_offset_x);
        }
    }

    bool using_odom_x = true;
    bool using_odom_y = true;

    if (!x_cands.empty()) {
        est_x = std::accumulate(x_cands.begin(), x_cands.end(), 0.0) / x_cands.size();
        using_odom_x = false;
    }
    if (!y_cands.empty()) {
        est_y = std::accumulate(y_cands.begin(), y_cands.end(), 0.0) / y_cands.size();
        using_odom_y = false;
    }

    return {est_x, est_y, using_odom_x, using_odom_y};
}

DistanceReset::DistanceReset(Chassis* robot_chassis,
                             const std::vector<DistanceSensor>& robot_sensors, 
                             float heading_tolerance, 
                             float filter_range)
    : chassis(robot_chassis),
      sensors(robot_sensors), 
      default_heading_tolerance(heading_tolerance), 
      default_filter_range(filter_range) {}

distancePose DistanceReset::update(bool setPose, bool filter) {
    return updateSpecific(sensors, setPose, filter);
}

distancePose DistanceReset::update(const std::vector<bool>& use_flags, bool setPose, bool filter) {
    std::vector<DistanceSensor> active_sensors;
    for (size_t i = 0; i < sensors.size() && i < use_flags.size(); ++i) {
        if (use_flags[i]) active_sensors.push_back(sensors[i]);
    }
    return updateSpecific(active_sensors, setPose, filter);
}

distancePose DistanceReset::updateSpecific(const std::vector<DistanceSensor>& active_sensors, bool setPose, bool filter) {
    Pose current = chassis->getPose(false);
    distancePose pose = calculateGlobalPosition(active_sensors, current.theta, current.x, current.y);
    
    if (filter) {
        if (std::abs(pose.x - current.x) > default_filter_range) {
            pose.x = current.x;
            pose.using_odom_x = true;
            std::cout << "Filtered X" << std::endl;
        }
        if (std::abs(pose.y - current.y) > default_filter_range) {
            pose.y = current.y;
            pose.using_odom_y = true;
            std::cout << "Filtered Y" << std::endl;
        }
    }
    
    if (setPose) {
        chassis->setPose(pose.x, pose.y, current.theta);
        pros::delay(2);
    }

    std::cout << "Distance Reset Pose: " << pose.x << ", " << pose.y 
              << ", using_odom_x: " << pose.using_odom_x 
              << ", using_odom_y: " << pose.using_odom_y << std::endl;
              
    return pose;
}