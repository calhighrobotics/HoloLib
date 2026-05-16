#pragma once

#include "pros/distance.hpp"
#include "chassis.h"
#include <vector>

struct DistanceSensor {
    pros::Distance* device;
    double forward_offset;
    double strafe_offset;
    double mounting_angle;
};

struct distancePose {
    double x;
    double y;
    bool using_odom_x;
    bool using_odom_y;
};

class DistanceReset {
private:
    static constexpr double MM_TO_IN = 0.0393701;
    static constexpr double FIELD_WIDTH = 3566.668 * MM_TO_IN;
    static constexpr double FIELD_HEIGHT = 3566.668 * MM_TO_IN;
    static constexpr double HALF_WIDTH = FIELD_WIDTH / 2.0;
    static constexpr double HALF_HEIGHT = FIELD_HEIGHT / 2.0;
    static constexpr double MAX_SENSOR_RANGE = 2000 * MM_TO_IN;
    static constexpr double MIN_SENSOR_RANGE = 0 * MM_TO_IN;

    Chassis* chassis;
    std::vector<DistanceSensor> sensors;
    float default_heading_tolerance;
    float default_filter_range;

    distancePose calculateGlobalPosition(
        const std::vector<DistanceSensor>& active_sensors,
        double heading_deg,
        double current_x,
        double current_y);

public:
    DistanceReset(Chassis* robot_chassis,
                  const std::vector<DistanceSensor>& robot_sensors, 
                  float heading_tolerance = 40.0, 
                  float filter_range = 3.5);

    distancePose update(bool setPose = false, bool filter = true);

    distancePose update(const std::vector<bool>& use_flags, bool setPose = false, bool filter = true);

    distancePose updateSpecific(const std::vector<DistanceSensor>& active_sensors, bool setPose, bool filter);
};