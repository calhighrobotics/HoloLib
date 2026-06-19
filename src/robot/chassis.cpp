#include "chassis.h"
#include "pros/misc.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

static constexpr float METER_TO_INCH = 39.3700787f;
static constexpr float DEG2RAD = M_PI / 180.0f;
static constexpr float RAD2DEG = 180.0f / M_PI;
MoveParams Chassis::defaultParams = {};


void ObstacleManager::setRobotDimensions(float width, float length) {
  robot_width = width;
  robot_length = length;
}

void ObstacleManager::addObstacle(float x, float y, float radius) {
  obstacles.push_back({Eigen::Vector2f(x, y), radius});
}

void ObstacleManager::removeObstacle(size_t index) {
  if (index < obstacles.size()) {
    obstacles.erase(obstacles.begin() + index);
  }
}

void ObstacleManager::clearObstacles() {
  obstacles.clear();
}

const std::vector<Obstacle>& ObstacleManager::getObstacles() const {
  return obstacles;
}

bool ObstacleManager::checkIntersection(const Eigen::Vector2f& start, const Eigen::Vector2f& end, 
                                       float safety_margin, Obstacle& out_obstacle, 
                                       Eigen::Vector2f& out_closest) const {
  Eigen::Vector2f ab = end - start;
  float ab_len_sq = ab.squaredNorm();
  if (ab_len_sq < 1e-6f) {
    for (const auto& obs : obstacles) {
      float dist = (start - obs.position).norm();
      if (dist < obs.radius + safety_margin) {
        out_obstacle = obs;
        out_closest = start;
        return true;
      }
    }
    return false;
  }

  for (const auto& obs : obstacles) {
    Eigen::Vector2f ac = obs.position - start;
    float t = ac.dot(ab) / ab_len_sq;
    t = std::clamp(t, 0.0f, 1.0f);

    Eigen::Vector2f closest = start + t * ab;
    float dist = (closest - obs.position).norm();

    if (dist < obs.radius + safety_margin) {
      out_obstacle = obs;
      out_closest = closest;
      return true;
    }
  }
  return false;
}

Eigen::Vector2f ObstacleManager::getAvoidanceTarget(const Eigen::Vector2f& robot_pos, 
                                                   const Eigen::Vector2f& target_pos, 
                                                   float safety_margin, 
                                                   float clearance, 
                                                   float robot_heading_rad,
                                                   int recursion_depth) const {
  if (recursion_depth > 3) {
    return target_pos;
  }

  Obstacle obs;
  Eigen::Vector2f closest;
  if (checkIntersection(robot_pos, target_pos, safety_margin, obs, closest)) {
    Eigen::Vector2f to_obs = obs.position - robot_pos;
    Eigen::Vector2f perp(-to_obs.y(), to_obs.x());
    if (perp.squaredNorm() < 1e-6f) {
      return target_pos;
    }
    perp.normalize();
    float abs_obs_angle = std::atan2(to_obs.x(), to_obs.y());
    float alpha = abs_obs_angle - robot_heading_rad;
    float r_width = (robot_width / 2.0f) / (std::abs(std::sin(alpha)) + 1e-6f);
    float r_length = (robot_length / 2.0f) / (std::abs(std::cos(alpha)) + 1e-6f);
    float dynamic_clearance = std::min(r_width, r_length) + clearance;

    Eigen::Vector2f w1 = obs.position + perp * (obs.radius + dynamic_clearance);
    Eigen::Vector2f w2 = obs.position - perp * (obs.radius + dynamic_clearance);

    float d1 = (w1 - robot_pos).norm() + (target_pos - w1).norm();
    float d2 = (w2 - robot_pos).norm() + (target_pos - w2).norm();
    Eigen::Vector2f best_waypoint = (d1 < d2) ? w1 : w2;

    return getAvoidanceTarget(robot_pos, best_waypoint, safety_margin, clearance, robot_heading_rad, recursion_depth + 1);
  }

  return target_pos;
}

Eigen::Vector2f ObstacleManager::getPotentialFieldTarget(const Eigen::Vector2f& robot_pos, 
                                                        const Eigen::Vector2f& target_pos, 
                                                        float ka, 
                                                        float kr, 
                                                        float influence_radius,
                                                        float robot_heading_rad) const {
  Eigen::Vector2f to_target = target_pos - robot_pos;
  float dist_to_target = to_target.norm();
  if (dist_to_target < 1e-4f) {
    return target_pos;
  }

  Eigen::Vector2f F_attractive = (to_target / dist_to_target) * ka;
  Eigen::Vector2f F_repulsive = Eigen::Vector2f::Zero();

  for (const auto& obs : obstacles) {
    Eigen::Vector2f to_obs = robot_pos - obs.position; 
    float dist_to_center = to_obs.norm();
    
    float abs_obs_angle = std::atan2(to_obs.x(), to_obs.y());
    float alpha = abs_obs_angle - robot_heading_rad;
    
    float sin_alpha = std::abs(std::sin(alpha));
    float cos_alpha = std::abs(std::cos(alpha));
    
    float r_width = (robot_width / 2.0f) / (sin_alpha + 1e-6f);
    float r_length = (robot_length / 2.0f) / (cos_alpha + 1e-6f);
    float dynamic_clearance = std::min(r_width, r_length);

    float effective_radius = obs.radius + dynamic_clearance + 1.0f;
    float dist_to_boundary = dist_to_center - effective_radius;

    if (dist_to_boundary <= 0.0f) {
      Eigen::Vector2f dir = (dist_to_center > 1e-4f) ? to_obs / dist_to_center : Eigen::Vector2f(1.0f, 0.0f);
      F_repulsive += dir * kr * 5.0f; 
    } 
    else if (dist_to_boundary <= influence_radius) {
      Eigen::Vector2f dir = to_obs / dist_to_center; 
      float factor = 1.0f - (dist_to_boundary / influence_radius);
      
      float force_mag = kr * factor / (dist_to_boundary + 0.1f);
      
      Eigen::Vector2f direct_repulse = dir * force_mag;
      Eigen::Vector2f tangent(-dir.y(), dir.x());
      if (tangent.dot(to_target) < 0) {
          tangent = Eigen::Vector2f(dir.y(), -dir.x()); 
      }

      Eigen::Vector2f tangential_bypass = tangent * (force_mag * 0.85f);
      F_repulsive += direct_repulse + tangential_bypass;
    }
  }

  Eigen::Vector2f F_total = F_attractive + F_repulsive;
  if (F_total.norm() < 1e-3f) {
    F_total = F_attractive + Eigen::Vector2f(-F_attractive.y(), F_attractive.x()) * 0.5f;
  }
  float step_size = std::min(10.0f, dist_to_target);
  return robot_pos + F_total.normalized() * step_size;
}

void Chassis::addObstacle(float x, float y, float radius) {
  obstacles.addObstacle(x, y, radius);
}

void Chassis::removeObstacle(size_t index) {
  obstacles.removeObstacle(index);
}

void Chassis::clearObstacles() {
  obstacles.clearObstacles();
}

void Chassis::setAvoidanceMode(AvoidanceMode mode) {
  avoidanceMode = mode;
}

void Chassis::setAvoidanceParams(float safetyMargin, float clearance) {
  avoidanceSafetyMargin = safetyMargin;
  avoidanceClearance = clearance;
}

void Chassis::setPotentialFieldParams(float ka, float kr, float influenceRadius) {
  pf_ka = ka;
  pf_kr = kr;
  pf_influence_radius = influenceRadius;
}

void Chassis::calibrate()
{
  cancelAllMotions();
  backLeft.tare_position();
  backRight.tare_position();
  frontLeft.tare_position();
  frontRight.tare_position();
  motionDistTraveled = 0.0f;
  prev_fl = 0, prev_fr = 0, prev_bl = 0, prev_br = 0;
  prev_heading = 0;
  targetHeadingDriveControl = 0;
  for(int i = 0; i < motors.size(); i++)
  {
    try
    {
      if (motors[i].get_temperature() > 60) {
        std::cout << "Motor " + std::to_string(i) + " overheating" << std::endl;
      }
    }
    catch(...)
    { 
      std::cout << "Error when evaluating Motor " + std::to_string(i) << std::endl;
    }
  }

  for (size_t i = 0; i < trackingWheelSensors.size(); ++i) {
    trackingWheelSensors[i].reset_position();
    prevTrackingPositions[i] = 0.0f;
  }
  imu.reset(true);
  while(imu.is_calibrating())
  {
    pros::delay(10);
  }
  pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
  std::cout << "Chassis Calibrated" << std::endl;
}

std::vector<PathPoint> parsePathData(const std::string &input_source,
                                     bool convertFromMeters) {
  std::vector<PathPoint> path;
  const float scale = convertFromMeters ? METER_TO_INCH : 1.0f;

  bool is_file = (input_source.find('\n') == std::string::npos) &&
                 (input_source.find('/') != std::string::npos ||
                  input_source.find('.') != std::string::npos);

  auto processLine = [&](const std::string &raw) {
    std::string line = raw;
    while (!line.empty() && (line.back() == '\r' || line.back() == '\n' ||
                             line.back() == ' ' || line.back() == '\t'))
      line.pop_back();

    size_t start = line.find_first_not_of(" \t");
    if (start == std::string::npos)
      return;
    line = line.substr(start);
    if (line.empty())
      return;

    std::stringstream ss(line);
    std::string cx, cy, ct;
    if (std::getline(ss, cx, ',') && std::getline(ss, cy, ',') &&
        std::getline(ss, ct, ',')) {
      try {
        PathPoint pt;
        pt.x = std::stof(cx) * scale;
        pt.y = std::stof(cy) * scale;
        pt.theta = std::stof(ct);
        path.push_back(pt);
      } catch (...) {
      }
    }
  };

  if (is_file) {
    std::ifstream file(input_source);
    if (!file.is_open()) {
      std::cout << "[PARSER ERROR] Cannot open: " << input_source << std::endl;
      return path;
    }
    std::string line;
    while (std::getline(file, line))
      processLine(line);
  } else {
    std::stringstream ss(input_source);
    std::string line;
    while (std::getline(ss, line))
      processLine(line);
  }

  std::cout << "[PARSER] Loaded " << path.size() << " points." << std::endl;
  return path;
}

void GainScheduler::addStep(float threshold, float kP, float kI, float kD, float slew) {
  schedules.push_back({threshold, {kP, kI, kD, 0.0f, slew}});
  std::sort(schedules.begin(), schedules.end(),
            [](const ScheduledGain &a, const ScheduledGain &b) {
              return a.threshold < b.threshold;
            });
}

PIDGains GainScheduler::getGains(float error) const {
  if (schedules.empty())
    return {0, 0, 0, 0, 0};

  float absErr = std::abs(error);

  if (absErr <= schedules.front().threshold)
    return schedules.front().gains;

  if (absErr >= schedules.back().threshold)
    return schedules.back().gains;

  for (size_t i = 0; i < schedules.size() - 1; ++i) {
    const auto &lo = schedules[i];
    const auto &hi = schedules[i + 1];
    if (absErr >= lo.threshold && absErr <= hi.threshold) {
      float t = (absErr - lo.threshold) / (hi.threshold - lo.threshold);
      return {lo.gains.kP + t * (hi.gains.kP - lo.gains.kP),
              lo.gains.kI + t * (hi.gains.kI - lo.gains.kI),
              lo.gains.kD + t * (hi.gains.kD - lo.gains.kD), 
              0.0f,
              lo.gains.slew + t * (hi.gains.slew - lo.gains.slew)};
    }
  }

  return schedules.back().gains;
}

void GainScheduler::clear() { schedules.clear(); }

EncoderKalmanFilter::EncoderKalmanFilter(float process_noise,
                                         float measurement_noise) {
  x.setZero();
  P.setIdentity();
  Q << process_noise, 0, 0, process_noise;
  R = measurement_noise;
  H << 1, 0;
}

float EncoderKalmanFilter::update(float measured_position, float dt) {
  Eigen::Matrix2f F;
  F << 1, dt, 0, 1;
  x = F * x;
  P = F * P * F.transpose() + Q;

  float y = measured_position - H * x;
  float S = (H * P * H.transpose())(0, 0) + R;
  Eigen::Vector2f K = P * H.transpose() / S;

  x = x + K * y;
  P = (Eigen::Matrix2f::Identity() - K * H) * P;
  return x(0);
}

void odomTaskTrampoline(void *param) {
  static_cast<Chassis *>(param)->odometryTask();
}

Chassis::Chassis(pros::Motor fl, pros::Motor fr, pros::Motor bl, pros::Motor br,
                 pros::Imu imu_sensor, ChassisConfig cfg)
    : frontLeft(fl), frontRight(fr), backLeft(bl), backRight(br),
      imu(imu_sensor), config(cfg) {
  if (config.drivetrainWidth <= 0.0f)
    config.drivetrainWidth = config.trackWidth;
  if (config.drivetrainLength <= 0.0f)
    config.drivetrainLength = config.drivetrainWidth;

  auto safePos = [](pros::Motor &m) {
    float v = m.get_position();
    return (std::isinf(v) || std::isnan(v)) ? 0.0f : v;
  };
  prev_fl = safePos(frontLeft);
  prev_fr = safePos(frontRight);
  prev_bl = safePos(backLeft);
  prev_br = safePos(backRight);

  float h = imu.get_rotation();
  prev_heading = (std::isinf(h) || std::isnan(h)) ? 0.0f : h * DEG2RAD;

  motionDistTraveled = 0.0f;

  this->obstacles.setRobotDimensions(config.drivetrainWidth, config.drivetrainLength);
  motion.setOnMotionStart([this]() {
    poseMutex.take();
    motionDistTraveled = 0.0f;
    poseMutex.give();
  });

  pros::Task odom_task(odomTaskTrampoline, this, "Odometry Task");
}

void Chassis::odometryTask() {
  uint32_t now = pros::millis();
  const float d_per_deg = (M_PI * config.wheelDiameter * config.gearRatio) / 360.0f;
  constexpr float wheel_angle = 45.0f * DEG2RAD;
  const float x_component = std::sin(wheel_angle);
  const float y_component = std::cos(wheel_angle);
  const float x_scale = 1.0f / (4.0f * x_component);
  const float y_scale = 1.0f / (4.0f * y_component);
  Eigen::Matrix<float, 2, 4> kinematics;
  kinematics << x_scale, -x_scale, -x_scale, x_scale,
                y_scale,  y_scale,  y_scale, y_scale;

  auto safeEnc = [](pros::Motor &m, float prev) {
    float v = m.get_position();
    return (std::isinf(v) || std::isnan(v)) ? prev : v;
  };

  while (true) {

    float raw_h = imu.get_rotation();
    float current_heading_meas = (std::isinf(raw_h) || std::isnan(raw_h))
                                     ? prev_heading
                                     : raw_h * DEG2RAD;
    float d_theta_meas = std::remainder(current_heading_meas - prev_heading, 2.0f * static_cast<float>(M_PI));
    if (std::isnan(d_theta_meas)) {
        d_theta_meas = 0.0f;
    }

    if (useTrackingWheels) {



      const int n = static_cast<int>(trackingWheelConfigs.size());


      Eigen::VectorXf measured_deltas(n);
      int numVertical = 0;
      int numHorizontal = 0;
      float sumDyVertical = 0.0f;
      float sumDxHorizontal = 0.0f;

      for (int i = 0; i < n; ++i) {

        int32_t raw_pos = trackingWheelSensors[i].get_position();
        float current_pos = static_cast<float>(raw_pos);
        float delta_centideg = current_pos - prevTrackingPositions[i];
        prevTrackingPositions[i] = current_pos;



        const auto& cfg = trackingWheelConfigs[i];
        float delta_inches = (delta_centideg / 36000.0f) * static_cast<float>(M_PI)
                             * cfg.wheelDiameter * cfg.gearRatio;
        measured_deltas(i) = delta_inches;


        if (cfg.orientation == TrackingWheelOrientation::VERTICAL) {


          sumDyVertical += delta_inches + cfg.xOffset * d_theta_meas;
          numVertical++;
        } else {


          sumDxHorizontal += delta_inches - cfg.yOffset * d_theta_meas;
          numHorizontal++;
        }
      }


      float dx_local = 0.0f;
      float dy_local = 0.0f;

      if (numHorizontal > 0) {
        dx_local = sumDxHorizontal / static_cast<float>(numHorizontal);
      }
      if (numVertical > 0) {
        dy_local = sumDyVertical / static_cast<float>(numVertical);
      }


      if (numHorizontal == 0) {
        float raw_fl_tw = safeEnc(frontLeft, prev_fl);
        float raw_fr_tw = safeEnc(frontRight, prev_fr);
        float raw_bl_tw = safeEnc(backLeft, prev_bl);
        float raw_br_tw = safeEnc(backRight, prev_br);
        Eigen::Vector4f raw_enc(raw_fl_tw, raw_fr_tw, raw_bl_tw, raw_br_tw);
        Eigen::Vector4f prev_enc_v(prev_fl, prev_fr, prev_bl, prev_br);
        Eigen::Vector4f wheel_deltas_motor = (raw_enc - prev_enc_v) * d_per_deg;
        Eigen::Vector2f motor_local = kinematics * wheel_deltas_motor;
        dx_local = motor_local.x();
        prev_fl = raw_fl_tw;
        prev_fr = raw_fr_tw;
        prev_bl = raw_bl_tw;
        prev_br = raw_br_tw;
      }

      if (numVertical == 0) {
        float raw_fl_tw = safeEnc(frontLeft, prev_fl);
        float raw_fr_tw = safeEnc(frontRight, prev_fr);
        float raw_bl_tw = safeEnc(backLeft, prev_bl);
        float raw_br_tw = safeEnc(backRight, prev_br);
        Eigen::Vector4f raw_enc(raw_fl_tw, raw_fr_tw, raw_bl_tw, raw_br_tw);
        Eigen::Vector4f prev_enc_v(prev_fl, prev_fr, prev_bl, prev_br);
        Eigen::Vector4f wheel_deltas_motor = (raw_enc - prev_enc_v) * d_per_deg;
        Eigen::Vector2f motor_local = kinematics * wheel_deltas_motor;
        dy_local = motor_local.y();
        prev_fl = raw_fl_tw;
        prev_fr = raw_fr_tw;
        prev_bl = raw_bl_tw;
        prev_br = raw_br_tw;
      }

      poseMutex.take();

      ekf.predict(dx_local, dy_local, d_theta_meas);


      ekf.updateTrackingWheels(trackingWheelConfigs, measured_deltas,
                               dx_local, dy_local, d_theta_meas,
                               trackingWheelMeasNoise);


      float current_w = d_theta_meas / 0.01f;
      float dynamic_R = measurementNoise + std::abs(current_w) * 0.005f;
      ekf.updateIMU(current_heading_meas, dynamic_R);

      float step_dist = std::sqrt(dx_local * dx_local + dy_local * dy_local);
      motionDistTraveled += step_dist;

      currentPose.x = ekf.getX();
      currentPose.y = ekf.getY();
      currentPose.theta = ekf.getTheta();
      if (velocityCalculationsOn) {
        currentPose.velocity.vx = dx_local / 0.01f;
        currentPose.velocity.vy = dy_local / 0.01f;
        currentPose.velocity.w = d_theta_meas / 0.01f;
      }
      poseMutex.give();

    } else {



      float raw_fl = safeEnc(frontLeft, prev_fl);
      float raw_fr = safeEnc(frontRight, prev_fr);
      float raw_bl = safeEnc(backLeft, prev_bl);
      float raw_br = safeEnc(backRight, prev_br);

      Eigen::Vector4f raw_enc(raw_fl, raw_fr, raw_bl, raw_br);
      Eigen::Vector4f prev_enc(prev_fl, prev_fr, prev_bl, prev_br);
      Eigen::Vector4f wheel_deltas = (raw_enc - prev_enc) * d_per_deg;

      float track_radius = (config.drivetrainWidth + config.drivetrainLength) / 2.0f;
      float vt_inches = (wheel_deltas(0) - wheel_deltas(1) + wheel_deltas(2) - wheel_deltas(3)) / 4.0f;
      float d_theta_wheels = vt_inches / (y_component * track_radius);

      Eigen::Vector2f local_delta = kinematics * wheel_deltas;
      poseMutex.take();
      ekf.predict(local_delta.x(), local_delta.y(), d_theta_wheels);

      float current_w = d_theta_meas / 0.01f;
      float dynamic_R = measurementNoise + std::abs(current_w) * 0.005f;
      ekf.updateIMU(current_heading_meas, dynamic_R);

      float step_dist = local_delta.norm();
      motionDistTraveled += step_dist;

      currentPose.x = ekf.getX();
      currentPose.y = ekf.getY();
      currentPose.theta = ekf.getTheta();
      if(velocityCalculationsOn) {
        currentPose.velocity.vx = local_delta.x() / 0.01;
        currentPose.velocity.vy = local_delta.y() / 0.01;
        currentPose.velocity.w = d_theta_meas / 0.01;
      }
      poseMutex.give();
      prev_fl = raw_fl;
      prev_fr = raw_fr;
      prev_bl = raw_bl;
      prev_br = raw_br;
    }

    prev_heading = current_heading_meas;
    pros::Task::delay_until(&now, 10);
  }
}

XDriveVoltages Chassis::calculateHolonomic(float vx, float vy, float vt) {

  constexpr float scale = 12000.0f / 127.0f;

  XDriveVoltages v;
  v.fl = (vy + vx + vt) * scale;
  v.fr = (vy - vx - vt) * scale;
  v.bl = (vy - vx + vt) * scale;
  v.br = (vy + vx - vt) * scale;

  float maxV = std::max({std::abs(v.fl), std::abs(v.fr), std::abs(v.bl),
                         std::abs(v.br), 12000.0f});
  if (maxV > 12000.0f) {
    float r = 12000.0f / maxV;
    v.fl *= r;
    v.fr *= r;
    v.bl *= r;
    v.br *= r;
  }
  return v;
}

void Chassis::setMotorVoltages(XDriveVoltages v) {
  frontLeft.move_voltage((int32_t)v.fl);
  frontRight.move_voltage((int32_t)v.fr);
  backLeft.move_voltage((int32_t)v.bl);
  backRight.move_voltage((int32_t)v.br);
}

void Chassis::brake() {
  frontLeft.brake();
  frontRight.brake();
  backLeft.brake();
  backRight.brake();
}

void Chassis::setPose(float x, float y, float theta) {
  poseMutex.take();
  float theta_rad = theta * DEG2RAD;
  imu.set_rotation(theta);

  currentPose = {x, y, theta_rad};
  prev_heading = theta_rad;

  ekf.setPose(x, y, theta_rad);

  poseMutex.give();
}

void Chassis::setPose(Pose pose) {
  setPose(pose.x, pose.y, pose.theta);
}

Pose Chassis::getPose(bool radians) {
  poseMutex.take();
  Pose p = currentPose;
  poseMutex.give();
  if (!radians)
    p.theta *= RAD2DEG;
  return p;
}

void Chassis::setXGains(std::vector<ScheduledGain> steps) {
  xSched.clear();
  for (auto &s : steps)
    xSched.addStep(s.threshold, s.gains.kP, s.gains.kI, s.gains.kD, s.gains.slew);
}

void Chassis::setYGains(std::vector<ScheduledGain> steps) {
  ySched.clear();
  for (auto &s : steps)
    ySched.addStep(s.threshold, s.gains.kP, s.gains.kI, s.gains.kD, s.gains.slew);
}

void Chassis::setThetaGains(std::vector<ScheduledGain> steps) {
  thetaSched.clear();
  for (auto &s : steps)
    thetaSched.addStep(s.threshold, s.gains.kP, s.gains.kI, s.gains.kD, s.gains.slew);
}

void Chassis::driveControl(float forward, float sideways, float rotation,
                            DriveCurves drivecurves,
                            bool fieldCentric,
                            float headingOffset,
                            DriveCorrection correction)
{
    static bool headingInitialized = false;
    static float targetHeading = 0.0f;
    static PID headingPID(0, 0, 0, 0);
    static uint32_t lastRotationTime = 0;
    static bool wasRotating = false;
    constexpr float MAX_DRIVE_INPUT = 127.0f;
    constexpr float SETTLE_DELAY_MS = 150.0f;
    constexpr float MAX_CORRECTION = 40.0f;

    if (!headingInitialized) {
        targetHeading = getPose(false).theta;
        headingPID.setGains({correction.kP, correction.kI, correction.kD, 0.0, 0.0});
        headingInitialized = true;
    }

    auto applyCurve = [&](float x, const DriveCurve& c) -> float {
        if (std::abs(x) < c.deadzone)
            return 0.0f;

        float sign = (x >= 0.0f) ? 1.0f : -1.0f;
        float normalized =
            (std::abs(x) - c.deadzone) /
            (MAX_DRIVE_INPUT - c.deadzone);

        normalized = std::clamp(normalized, 0.0f, 1.0f);
        normalized =
            std::pow(normalized, c.curve_multipler);

        float output = normalized * MAX_DRIVE_INPUT;
        if (output > 0.0f &&
            output < c.minimum_output)
        {
            output = c.minimum_output;
        }

        return output * sign;
    };
    forward  = applyCurve(forward,  drivecurves.movement);
    sideways = applyCurve(sideways, drivecurves.movement);

    float robotForward  = forward;
    float robotSideways = sideways;
    if (fieldCentric) {

        float adjustedTheta =
            getPose(false).theta - headingOffset;

        float thetaRad = adjustedTheta * DEG2RAD;

        robotSideways =
            sideways * std::cos(thetaRad) -
            forward  * std::sin(thetaRad);

        robotForward =
            sideways * std::sin(thetaRad) +
            forward  * std::cos(thetaRad);
        float inputMagnitude =
            std::sqrt(forward * forward +
                      sideways * sideways);

        float rotatedMagnitude =
            std::sqrt(robotForward * robotForward +
                      robotSideways * robotSideways);

        if (rotatedMagnitude > 0.001f) {

            float scale =
                inputMagnitude / rotatedMagnitude;

            robotForward  *= scale;
            robotSideways *= scale;
        }
    }

    bool isRotating = std::abs(rotation) >= drivecurves.rotation.deadzone;

    if (isRotating) {
        rotation = applyCurve(rotation, drivecurves.rotation);
        targetHeading = getPose(false).theta;
        lastRotationTime = pros::millis();
        wasRotating = true;

    } else {
        uint32_t timeSinceRotation = pros::millis() - lastRotationTime;

        if (wasRotating) {
            targetHeading = getPose(false).theta;
            headingPID.reset();
            headingPID.setGains({correction.kP, correction.kI, correction.kD, 0.0, 0.0});
            wasRotating = false;
        }

        if (timeSinceRotation < (uint32_t)SETTLE_DELAY_MS) {
            rotation = 0.0f;
            targetHeading = getPose(false).theta;
        } else if (correction.correctionOn) {
            float currentHeading = getPose(false).theta;
            float angleError = getAngleError(targetHeading, currentHeading);

            if (std::abs(angleError) < 0.5f) {
                rotation = 0.0f;
            } else {
                rotation = (float)headingPID.update(angleError);
                rotation = std::clamp(rotation, -MAX_CORRECTION, MAX_CORRECTION);
            }
        } else {
            rotation = 0.0f;
        }
    }
    setMotorVoltages(
        calculateHolonomic(
            robotSideways,
            robotForward,
            rotation
        )
    );
}



void Chassis::followPath(const std::vector<PathPoint> &path,
                         float lookaheadDistance,
                         MoveParams params,
                         HeadingMode headingMode,
                         float holdAngleDeg,
                         bool reversed) {

  if (path.size() < 2) {
    std::cout << "[followPathPID] Invalid path." << std::endl;
    return;
  }

  motion.enqueue([=, this]() {

    PID forwardPID(0, 0, 0, 0);
    PID strafePID(0, 0, 0, 0);
    PID thetaPID(0, 0, 0, 0);

    uint32_t start       = pros::millis();
    uint32_t settleStart = 0;
    constexpr uint32_t settleTime = 120;

    const int N = static_cast<int>(path.size());
    std::vector<float> arcLen(N, 0.0f);
    for (int i = 1; i < N; ++i) {
      float dx = path[i].x - path[i-1].x;
      float dy = path[i].y - path[i-1].y;
      arcLen[i] = arcLen[i-1] + std::hypot(dx, dy);
    }
    float totalPathLen = arcLen[N-1];

    float lockedHeading = (headingMode == HeadingMode::HoldAngle)
                              ? holdAngleDeg
                              : getPose(false).theta;

    float pathProgress = 0.0f;
    Pose  prevPose     = getPose(false);

    auto samplePath = [&](float s) -> PathPoint {
      s = std::clamp(s, 0.0f, totalPathLen);
      int lo = 0, hi = N - 2;
      while (lo < hi) {
        int mid = (lo + hi + 1) / 2;
        if (arcLen[mid] <= s) lo = mid;
        else                  hi = mid - 1;
      }
      float segLen = arcLen[lo+1] - arcLen[lo];
      float t      = (segLen > 1e-6f) ? (s - arcLen[lo]) / segLen : 0.0f;
      PathPoint p;
      p.x     = path[lo].x + t * (path[lo+1].x - path[lo].x);
      p.y     = path[lo].y + t * (path[lo+1].y - path[lo].y);
      p.theta = path[lo].theta + t * getAngleError(path[lo+1].theta, path[lo].theta);
      return p;
    };

    auto pathTangentDeg = [&](float s) -> float {
      s = std::clamp(s, 0.0f, totalPathLen);
      int lo = 0, hi = N - 2;
      while (lo < hi) {
        int mid = (lo + hi + 1) / 2;
        if (arcLen[mid] <= s) lo = mid;
        else                  hi = mid - 1;
      }
      float dx = path[lo+1].x - path[lo].x;
      float dy = path[lo+1].y - path[lo].y;
      return std::atan2(dx, dy) * (180.0f / static_cast<float>(M_PI));
    };

    while (pros::millis() - start < params.timeout) {

      Pose curr = getPose(false);

      {
        float dxOdom = curr.x - prevPose.x;
        float dyOdom = curr.y - prevPose.y;
        float dist   = std::hypot(dxOdom, dyOdom);

        if (dist > 1e-4f) {
          float tangentDeg = pathTangentDeg(pathProgress);
          float tangentRad = tangentDeg * (static_cast<float>(M_PI) / 180.0f);
          float tx = std::sin(tangentRad);
          float ty = std::cos(tangentRad);

          float along = dxOdom * tx + dyOdom * ty;

          along = std::max(along, -0.05f * dist);

          pathProgress += along;
          pathProgress  = std::clamp(pathProgress, 0.0f, totalPathLen);
        }

        prevPose = curr;
      }

      {
        PathPoint onPath  = samplePath(pathProgress);
        float     exError = curr.x - onPath.x;
        float     eyError = curr.y - onPath.y;

        float tangentDeg = pathTangentDeg(pathProgress);
        float tangentRad = tangentDeg * (static_cast<float>(M_PI) / 180.0f);
        float tx = std::sin(tangentRad);
        float ty = std::cos(tangentRad);

        float sErr = exError * tx + eyError * ty;

        float correction = std::clamp(sErr * 0.05f,
                                      -lookaheadDistance * 0.25f,
                                      lookaheadDistance * 0.25f);
        pathProgress = std::clamp(pathProgress + correction, 0.0f, totalPathLen);
      }

      float     lookaheadS = std::min(pathProgress + lookaheadDistance, totalPathLen);
      PathPoint lookahead  = samplePath(lookaheadS);

      Eigen::Vector2f robotPos(curr.x, curr.y);
      Eigen::Vector2f lookaheadVec(lookahead.x, lookahead.y);
      Eigen::Vector2f activeTarget = lookaheadVec;

      if (avoidanceMode == AvoidanceMode::On) {
        activeTarget = obstacles.getPotentialFieldTarget(
            robotPos, lookaheadVec, pf_ka, pf_kr,
            pf_influence_radius, getPose(true).theta);
      }

      float headingRad = curr.theta * DEG2RAD;
      float forwardX   =  std::sin(headingRad);
      float forwardY   =  std::cos(headingRad);
      float strafeX    =  std::cos(headingRad);
      float strafeY    = -std::sin(headingRad);

      float globalDX = activeTarget.x() - curr.x;
      float globalDY = activeTarget.y() - curr.y;

      float localForward = globalDX * forwardX + globalDY * forwardY;
      float localStrafe  = globalDX * strafeX  + globalDY * strafeY;

      if (reversed) {
        localForward = -localForward;
        localStrafe  = -localStrafe;
      }

      float distToEnd = totalPathLen - pathProgress;

      float targetHeading;
      switch (headingMode) {

        case HeadingMode::FollowPath: {
          float tangentS = std::min(pathProgress + 2.0f, totalPathLen);
          targetHeading  = pathTangentDeg(tangentS);
          if (reversed) targetHeading += 180.0f;
          break;
        }

        case HeadingMode::HoldAngle:
          targetHeading = lockedHeading;
          break;

        case HeadingMode::CustomAngles:
          targetHeading = lookahead.theta;
          if (reversed) targetHeading += 180.0f;
          break;

        default:
          targetHeading = curr.theta;
          break;
      }

      float angleError = getAngleError(targetHeading, curr.theta);

      if (params.earlyExitRange > 0.0f && distToEnd <= params.earlyExitRange)
        break;

      bool settledPos   = distToEnd        < params.exitRange;
      bool settledAngle = std::abs(angleError) < 2.0f;

      if (settledPos && settledAngle) {
        if (settleStart == 0) settleStart = pros::millis();
        if (pros::millis() - settleStart >= settleTime) break;
      } else {
        if (distToEnd > params.exitRange * 1.5f || std::abs(angleError) > 4.0f)
          settleStart = 0;
      }

      forwardPID.setGains(ySched.getGains(localForward));
      strafePID .setGains(xSched.getGains(localStrafe));
      thetaPID  .setGains(thetaSched.getGains(angleError));

      float forward = static_cast<float>(forwardPID.update(localForward));
      float strafe  = static_cast<float>(strafePID .update(localStrafe));
      float turn    = static_cast<float>(thetaPID  .update(angleError));

      float translationalMag = std::hypot(forward, strafe);

      if (translationalMag > params.maxTranslationSpeed) {
        float scale = params.maxTranslationSpeed / translationalMag;
        forward *= scale;
        strafe  *= scale;
      }

      if (translationalMag > 1e-3f &&
          translationalMag < params.minSpeed &&
          distToEnd > params.exitRange) {
        float scale = params.minSpeed / translationalMag;
        forward *= scale;
        strafe  *= scale;
      }

      turn = std::clamp(turn, -params.maxRotationSpeed, params.maxRotationSpeed);

      float total = std::abs(forward) + std::abs(strafe) + std::abs(turn);
      if (total > params.maxTranslationSpeed) {
        float scale = params.maxTranslationSpeed / total;
        forward *= scale;
        strafe  *= scale;
        turn    *= scale;
      }

      setMotorVoltages(calculateHolonomic(strafe, forward, turn));
      pros::delay(10);
    }

    brake();
  }, params.async);
}

void Chassis::turnToHeading(float targetDeg, MoveParams params) {
  motion.enqueue(
      [=, this]() {
        uint32_t start = pros::millis();
        uint32_t settleStart = 0;
        constexpr uint32_t settleTime = 100;

        PID tPID(0, 0, 0, 0);
        float prevError = 0.0f;

        while (pros::millis() - start < params.timeout) {
          float error = getAngleError(targetDeg, getPose(false).theta);

          if (params.earlyExitRange > 0.0f &&
              std::abs(error) <= params.earlyExitRange)
            return;

          if (std::abs(error) < params.exitRange) {
            if (settleStart == 0)
              settleStart = pros::millis();
            float vel = (error - prevError) / 0.01f;
            if (pros::millis() - settleStart >= settleTime &&
                std::abs(vel) < 0.5f)
              break;
          } else {
            settleStart = 0;
          }

          tPID.setGains(thetaSched.getGains(error));
          float output = (float)tPID.update(error);

          if (std::abs(output) > 1e-3f && std::abs(output) < params.minSpeed)
            output = std::copysign(params.minSpeed, output);
          output = std::clamp(output, -params.maxRotationSpeed,
                              params.maxRotationSpeed);

          setMotorVoltages(calculateHolonomic(0, 0, output));
          prevError = error;
          pros::delay(10);
        }
        brake();
      },
      params.async);
}

void Chassis::turnToPoint(float tx, float ty, MoveParams params) {
  motion.enqueue(
      [=, this]() {
        uint32_t start = pros::millis();
        uint32_t settleStart = 0;
        constexpr uint32_t settleTime = 100;

        PID tPID(0, 0, 0, 0);
        float prevError = 0.0f;

        while (pros::millis() - start < params.timeout) {
          Pose p = getPose(false);
          float ex = tx - p.x;
          float ey = ty - p.y;
          float targetDeg = std::atan2(ex, ey) * RAD2DEG;
          float error = getAngleError(targetDeg, p.theta);

          if (params.earlyExitRange > 0.0f &&
              std::abs(error) <= params.earlyExitRange)
            return;

          if (std::abs(error) < params.exitRange) {
            if (settleStart == 0)
              settleStart = pros::millis();
            float vel = (error - prevError) / 0.01f;
            if (pros::millis() - settleStart >= settleTime &&
                std::abs(vel) < 0.5f)
              break;
          } else {
            settleStart = 0;
          }

          tPID.setGains(thetaSched.getGains(error));
          float output = (float)tPID.update(error);

          if (std::abs(output) > 1e-3f && std::abs(output) < params.minSpeed)
            output = std::copysign(params.minSpeed, output);
          output = std::clamp(output, -params.maxRotationSpeed,
                              params.maxRotationSpeed);

          setMotorVoltages(calculateHolonomic(0, 0, output));
          prevError = error;
          pros::delay(10);
        }
        brake();
      },
      params.async);
}

void Chassis::moveToPoint(float tx, float ty, MoveParams params,
                          bool angleCorrection) {
  motion.enqueue(
      [=, this]() {
        uint32_t start = pros::millis();
        uint32_t settleStart = 0;
        constexpr uint32_t settleTime = 100;

        PID xPID(0, 0, 0, 0), yPID(0, 0, 0, 0), tPID(0, 0, 0, 0);
        float holdHeading = getPose(false).theta;

        while (pros::millis() - start < params.timeout) {
          Pose curr = getPose(false);
          
          float distErr = std::hypot(tx - curr.x, ty - curr.y);

          if (params.earlyExitRange > 0.0f && distErr <= params.earlyExitRange)
            return;

          if (distErr < params.exitRange) {
            if (settleStart == 0)
              settleStart = pros::millis();
            if (pros::millis() - settleStart >= settleTime)
              break;
          } else {
            settleStart = 0;
          }

          Eigen::Vector2f robotPos(curr.x, curr.y);
          Eigen::Vector2f targetPos(tx, ty);
          Eigen::Vector2f activeTarget = targetPos;

          if (avoidanceMode == AvoidanceMode::On) {
            activeTarget = obstacles.getPotentialFieldTarget(robotPos, targetPos, pf_ka, pf_kr, pf_influence_radius, getPose(true).theta);
          }

          float ex_g = activeTarget.x() - curr.x;
          float ey_g = activeTarget.y() - curr.y;

          float targetHeading =
              angleCorrection ? std::atan2(ex_g, ey_g) * RAD2DEG : holdHeading;
          float angleError = getAngleError(targetHeading, curr.theta);

          float rad = curr.theta * DEG2RAD;
          float cosH = std::cos(rad), sinH = std::sin(rad);

          float ex_local = ex_g * cosH - ey_g * sinH;
          float ey_local = ex_g * sinH + ey_g * cosH;

          xPID.setGains(xSched.getGains(std::abs(ex_local)));
          yPID.setGains(ySched.getGains(std::abs(ey_local)));
          tPID.setGains(thetaSched.getGains(angleError));

          float outX_local = (float)xPID.update(ex_local);
          float outY_local = (float)yPID.update(ey_local);
          float outT = angleCorrection ? (float)tPID.update(angleError) : 0.0f;

          float mag = std::hypot(outX_local, outY_local);
          if (mag > 1e-3f && mag < params.minSpeed) {
            float s = params.minSpeed / mag;
            outX_local *= s;
            outY_local *= s;
          }
          if (mag > params.maxTranslationSpeed) {
            float s = params.maxTranslationSpeed / mag;
            outX_local *= s;
            outY_local *= s;
          }
          if (distErr < 2.0f)
            outT = 0.0f;
          outT = std::clamp(outT, -params.maxRotationSpeed,
                            params.maxRotationSpeed);

          setMotorVoltages(calculateHolonomic(outX_local, outY_local, outT));
          pros::delay(10);
        }
        brake();
        
      },
      params.async);
}

void Chassis::moveRelative(float forward, float sideways, MoveParams params,
                           bool holdHeading) {
  motion.enqueue(
      [=, this]() {
        Pose start = getPose(false);
        float headingRad = start.theta * DEG2RAD;

        float targetX = start.x + forward * std::sin(headingRad) +
                        sideways * std::cos(headingRad);
        float targetY = start.y + forward * std::cos(headingRad) -
                        sideways * std::sin(headingRad);

        uint32_t startTime = pros::millis();
        uint32_t settleStart = 0;
        constexpr uint32_t settleTime = 120;

        PID xPID(0, 0, 0, 0), yPID(0, 0, 0, 0), tPID(0, 0, 0, 0);

        while (pros::millis() - startTime < params.timeout) {
          Pose curr = getPose(false);
          
          float distErr = std::hypot(targetX - curr.x, targetY - curr.y);
          float angleError = getAngleError(start.theta, curr.theta);

          if (params.earlyExitRange > 0.0f && distErr <= params.earlyExitRange)
            return;

          if (distErr < params.exitRange) {
            if (settleStart == 0)
              settleStart = pros::millis();
            if (pros::millis() - settleStart >= settleTime)
              break;
          } else {
            settleStart = 0;
          }

          Eigen::Vector2f robotPos(curr.x, curr.y);
          Eigen::Vector2f targetPos(targetX, targetY);
          Eigen::Vector2f activeTarget = targetPos;

        
          if (avoidanceMode == AvoidanceMode::On) {
            activeTarget = obstacles.getPotentialFieldTarget(robotPos, targetPos, pf_ka, pf_kr, pf_influence_radius, getPose(true).theta);
          }

          float ex = activeTarget.x() - curr.x;
          float ey = activeTarget.y() - curr.y;

          xPID.setGains(xSched.getGains(ex));
          yPID.setGains(ySched.getGains(ey));
          tPID.setGains(thetaSched.getGains(angleError));

          float outX_g = (float)xPID.update(ex);
          float outY_g = (float)yPID.update(ey);
          float outT = holdHeading ? (float)tPID.update(angleError) : 0.0f;

          float mag = std::hypot(outX_g, outY_g);
          if (mag > 1e-3f && mag < params.minSpeed) {
            float s = params.minSpeed / mag;
            outX_g *= s;
            outY_g *= s;
          }
          if (mag > params.maxTranslationSpeed) {
            float s = params.maxTranslationSpeed / mag;
            outX_g *= s;
            outY_g *= s;
          }
          outT = std::clamp(outT, -params.maxRotationSpeed,
                            params.maxRotationSpeed);

          float rad = curr.theta * DEG2RAD;
          float cosH = std::cos(rad), sinH = std::sin(rad);

          float outX_local = outX_g * cosH - outY_g * sinH;
          float outY_local = outX_g * sinH + outY_g * cosH;

          setMotorVoltages(calculateHolonomic(outX_local, outY_local, outT));
          pros::delay(10);
        }
        brake();
        
      },
      params.async);
}

void Chassis::moveDistance(float distance, MoveParams params,
                           bool holdHeading) {
  moveRelative(distance, 0.0f, params, holdHeading);
}

void Chassis::strafeDistance(float distance, MoveParams params,
                             bool holdHeading) {
  moveRelative(0.0f, distance, params, holdHeading);
}

void Chassis::moveToPose(float tx, float ty, float targetThetaDeg,
                         MoveParams params) {
  motion.enqueue(
      [=, this]() {
        uint32_t start = pros::millis();
        uint32_t settleStart = 0;
        constexpr uint32_t settleTime = 120;
        constexpr float angleExitDeg = 2.0f;

        PID xPID(0, 0, 0, 0), yPID(0, 0, 0, 0), tPID(0, 0, 0, 0);

        while (pros::millis() - start < params.timeout) {
          Pose curr = getPose(false);
          float ex = tx - curr.x;
          float ey = ty - curr.y;
          float distErr = std::hypot(ex, ey);
          float angleError = getAngleError(targetThetaDeg, curr.theta);

          if (params.earlyExitRange > 0.0f && distErr <= params.earlyExitRange)
            return;

          bool posSettled = distErr < params.exitRange;
          bool angleSettled = std::abs(angleError) < angleExitDeg;
          if (posSettled && angleSettled) {
            if (settleStart == 0)
              settleStart = pros::millis();
            if (pros::millis() - settleStart >= settleTime)
              break;
          } else {
            settleStart = 0;
          }

          xPID.setGains(xSched.getGains(ex));
          yPID.setGains(ySched.getGains(ey));
          tPID.setGains(thetaSched.getGains(angleError));

          float outX_g = (float)xPID.update(ex);
          float outY_g = (float)yPID.update(ey);
          float outT = (float)tPID.update(angleError);

          float mag = std::hypot(outX_g, outY_g);
          if (!posSettled && mag > 1e-3f && mag < params.minSpeed) {
            float s = params.minSpeed / mag;
            outX_g *= s;
            outY_g *= s;
          }
          if (mag > params.maxTranslationSpeed) {
            float s = params.maxTranslationSpeed / mag;
            outX_g *= s;
            outY_g *= s;
          }
          outT = std::clamp(outT, -params.maxRotationSpeed,
                            params.maxRotationSpeed);

          float rad = curr.theta * DEG2RAD;
          float cosH = std::cos(rad), sinH = std::sin(rad);

          float outX_local = outX_g * cosH - outY_g * sinH;
          float outY_local = outX_g * sinH + outY_g * cosH;

          setMotorVoltages(calculateHolonomic(outX_local, outY_local, outT));
          pros::delay(10);
        }
        brake();
      },
      params.async);
}


enum class SwingSide {
  Left,
  Right
};

void Chassis::swingTurn(float targetThetaDeg, SwingSide lockedSide, MoveParams params) {
  motion.enqueue(
      [=, this]() {
        uint32_t start = pros::millis();
        uint32_t settleStart = 0;
        constexpr uint32_t settleTime = 120;
        constexpr float angleExitDeg = 2.0f;
        PID tPID(0, 0, 0, 0);
        float maxRotation = std::min(params.maxRotationSpeed, 127.0f); 

        while (pros::millis() - start < params.timeout) {
          Pose curr = getPose(false);
          float angleError = getAngleError(targetThetaDeg, curr.theta);

          bool angleSettled = std::abs(angleError) < angleExitDeg;
          if (angleSettled) {
            if (settleStart == 0)
              settleStart = pros::millis();
            if (pros::millis() - settleStart >= settleTime)
              break;
          } else {
            settleStart = 0;
          }

          tPID.setGains(thetaSched.getGains(angleError));
          float outT = (float)tPID.update(angleError);
          if (!angleSettled && std::abs(outT) < params.minSpeed) {
            outT = std::copysign(params.minSpeed, outT);
          }
          outT = std::clamp(outT, -maxRotation, maxRotation);

          float outX_local = 0.0f;
          float outY_local = 0.0f;
          if (lockedSide == SwingSide::Left) {
            outY_local = -outT;
          } else {
            outY_local = outT;
          }
          setMotorVoltages(calculateHolonomic(outX_local, outY_local, outT));
          pros::delay(10);
        }
        brake();
      },
      params.async);
}

void Chassis::waitUntilDone() {
  motion.waitUntilDone();
}

void Chassis::waitUntil(float dist) {
  uint32_t targetId = motion.getLastEnqueuedId();
  while (true) {
    poseMutex.take();
    float currentDist = motionDistTraveled;
    poseMutex.give();

    uint32_t runningId = motion.getCurrentRunningId();
    bool empty = motion.isQueueEmpty();
    if (empty && runningId < targetId) {
      break;
    }

    if (runningId >= targetId) {
      if (runningId > targetId || currentDist >= dist) {
        break;
      }
    }

    pros::delay(10);
  }
}

void Chassis::cancelMotion() {
  motion.cancelMotion();
  brake();
}

void Chassis::cancelAllMotions() {
  motion.cancelAll();
  brake();
}

float Chassis::getDistanceTraveled(bool convertToMeters) {
  poseMutex.take();
  float dist = motionDistTraveled;
  poseMutex.give();
  if (convertToMeters) return dist * 0.0254f;
  return dist;
}

void Chassis::setEKFGains(float xProcessNoise, float yProcessNoise, float thetaProcessNoise, float measurementNoise) {
  ekf.setProcessNoise(xProcessNoise, yProcessNoise, thetaProcessNoise, measurementNoise);
}


void Chassis::setVelocityCalculations(bool state)
{
  velocityCalculationsOn = state;
}


bool Chassis::detectCollision() {
    const int32_t TARGET_VOLTAGE_THRESHOLD = 3000;   
    const uint32_t DEBOUNCE_TIME_MS = 250;      

    if (last_collision_check_time == 0) last_collision_check_time = pros::millis();
    
    uint32_t now = pros::millis();
    uint32_t dt = now - last_collision_check_time;
    last_collision_check_time = now;
    auto is_wheel_slipping = [&](pros::Motor& motor) {
        int32_t commanded_voltage = std::abs(motor.get_voltage());
        if (commanded_voltage < TARGET_VOLTAGE_THRESHOLD) {
            return false;
        }

        double actual = std::abs(motor.get_actual_velocity());
        int32_t current = motor.get_current_draw();
        double temp = motor.get_temperature();     
        
        int32_t dynamic_current_threshold = 1200; 
        if (temp > 50.0) {
            dynamic_current_threshold = 900;    
        }
        
        bool speed_deficit = actual < 30.0;
        bool heavy_load = current > dynamic_current_threshold;

        return speed_deficit && heavy_load;
    };
    int slip_count = 0;
    if (is_wheel_slipping(frontLeft)) slip_count++;
    if (is_wheel_slipping(frontRight)) slip_count++;
    if (is_wheel_slipping(backLeft)) slip_count++;
    if (is_wheel_slipping(backRight)) slip_count++;
    bool physically_blocked = (slip_count >= 2);

    if (physically_blocked) {
        stall_accumulator_ms += dt;
    } else {
        stall_accumulator_ms = 0; 
    }

    return stall_accumulator_ms >= DEBOUNCE_TIME_MS;
}


void Chassis::openLoop(float forward, float sideways, float rotation) {
  setMotorVoltages(calculateHolonomic(sideways, forward, rotation));
}

float Chassis::radToDeg(float rad)
{
  return rad * RAD2DEG;
}

float Chassis::degToRad(float deg)
{
  return deg * DEG2RAD;
}

void Chassis::setEKFstate(bool state)
{
  this->config.kfEnabled = state;
}

void Chassis::addTrackingWheel(TrackingWheelConfig config) {
  trackingWheelConfigs.push_back(config);
  trackingWheelSensors.emplace_back(config.port);

  int32_t initPos = trackingWheelSensors.back().get_position();
  prevTrackingPositions.push_back(static_cast<float>(initPos));
  useTrackingWheels = true;




  ekf.setTrackingWheelNoise(0.0003f, 0.0003f, 0.001f);
  trackingWheelMeasNoise = 0.0005f;

  std::cout << "[Chassis] Added tracking wheel on port " << static_cast<int>(config.port)
            << (config.orientation == TrackingWheelOrientation::HORIZONTAL ? " (horizontal)" : " (vertical)")
            << " offset=(" << config.xOffset << ", " << config.yOffset << ")"
            << " dia=" << config.wheelDiameter << " ratio=" << config.gearRatio
            << std::endl;
}

void Chassis::clearTrackingWheels() {
  trackingWheelConfigs.clear();
  trackingWheelSensors.clear();
  prevTrackingPositions.clear();
  useTrackingWheels = false;


  ekf.setProcessNoise(0.001f, 0.001f, 0.003f, 0.0001f);
  std::cout << "[Chassis] Tracking wheels cleared, reverted to motor encoder odometry" << std::endl;
}


#include <map>

// You can place this struct just above the function in your .cpp file
struct ButtonRecord {
  pros::controller_digital_e_t button;
  uint32_t duration_ms;
  uint32_t timestamp_ms;
};

void Chassis::getControllerInput(pros::Controller master) {
  // 1. Define the buttons you want to track locally

  // 2. Use 'static' so these variables remember their state between loops
  static std::map<pros::controller_digital_e_t, uint32_t> pressStartTimes;
  static std::map<pros::controller_digital_e_t, bool> prevStates;
  static std::vector<ButtonRecord> buttonHistory;

  uint32_t currentTime = pros::millis();

  for (auto &btn : controllerButtons) {
    pros::controller_digital_e_t btn_enum = btn.button;
    bool isPressed = master.get_digital(btn_enum);
    bool wasPressed = prevStates[btn_enum];

    if (isPressed && !wasPressed) {
      // --- RISING EDGE: Button was just pressed down ---
      pressStartTimes[btn_enum] = currentTime;
      if (btn.callback) {
        btn.callback();
      }
    } 
    else if (!isPressed && wasPressed) {
      // --- FALLING EDGE: Button was just released ---
      uint32_t duration = currentTime - pressStartTimes[btn_enum];
      
      // Log the record into the static history vector
      buttonHistory.push_back({btn_enum, duration, pressStartTimes[btn_enum]});

      // Optional: Print to terminal to verify it's working
      std::cout << "Button " << btn_enum << " held for " << duration << "ms\n";
    }

    // Update the previous state for the next loop iteration
    prevStates[btn_enum] = isPressed;
  }
}

void Chassis::logReplayData(pros::Controller master, int timeout_ms)
{
  pros::Task log_task([=, this]() {
    Pose lastPose = getPose();
    
    // Clamp the delay to a minimum of 20ms to prevent RTOS starvation 
    // and serial flooding.
    int safe_timeout = (timeout_ms < 20) ? 20 : timeout_ms;
    
    while(true) {
      Pose pose = getPose(); 
      
      double deltaX = std::abs(pose.x - lastPose.x);
      double deltaY = std::abs(pose.y - lastPose.y);
      double deltaTheta = std::abs(pose.theta - lastPose.theta);
      
      if(deltaX > 0.5 || deltaY > 0.5 || deltaTheta > 1.0)
      {
        // Use printf and \n. This avoids the expensive buffer flush of std::endl
        // Limits the output to 2 decimal places to save serial bandwidth.
        printf("%.2f,%.2f,%.2f\n", pose.x, pose.y, pose.theta);
        
        lastPose = pose; 
      }
      
      //getControllerInput(master);
      
      pros::delay(safe_timeout);
    }
  });
}

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

  for (const auto& seg : segments) {
    if (seg.size() >= 2) {
      followPath(seg, lookahead, defaultParams, HeadingMode::CustomAngles, 0.0f, is_reversed);
      waitUntilDone();
      is_reversed = !is_reversed; 
    }
  }
}