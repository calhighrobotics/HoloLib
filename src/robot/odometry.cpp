#include "chassis.h"
#include <cmath>
#include <iostream>

static constexpr float DEG2RAD = M_PI / 180.0f;

/**
 *@brief Trampoline function for the odometry task.
 *@param param The chassis to pass to the task.
 */
void odomTaskTrampoline(void *param) {
  static_cast<Chassis *>(param)->odometryTask();
}

/**
 *@brief Constructs a Chassis.
 *@param fl The front left motor.
 *@param fr The front right motor.
 *@param bl The back left motor.
 *@param br The back right motor.
 *@param imu_sensor The IMU sensor.
 *@param cfg The chassis configuration.
 *@note Do not use the ekf feature if you do not know how to tune it, or if you
 * are utilizing more than one tracking wheel for a dimension.
 */
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

  this->obstacles.setRobotDimensions(config.drivetrainWidth,
                                     config.drivetrainLength);
  motion.setOnMotionStart([this]() {
    poseMutex.take();
    motionDistTraveled = 0.0f;
    poseMutex.give();
  });

  pros::Task odom_task(odomTaskTrampoline, this, "Odometry Task");
}

/**
 *@brief Updates the odometry task.
 *@note This function should be called in a separate task.
 */
void Chassis::odometryTask() {
  uint32_t now = pros::millis();
  const float d_per_deg =
      (M_PI * config.wheelDiameter * config.gearRatio) / 360.0f;
  constexpr float wheel_angle = 45.0f * DEG2RAD;
  const float x_component = std::sin(wheel_angle);
  const float y_component = std::cos(wheel_angle);
  const float x_scale = 1.0f / (4.0f * x_component);
  const float y_scale = 1.0f / (4.0f * y_component);
  Eigen::Matrix<float, 2, 4> kinematics;
  kinematics << x_scale, -x_scale, -x_scale, x_scale, y_scale, y_scale, y_scale,
      y_scale;

  auto safeEnc = [](pros::Motor &m, float prev) {
    float v = m.get_position();
    return (std::isinf(v) || std::isnan(v)) ? prev : v;
  };

  while (true) {

    float raw_h = imu.get_rotation();
    float current_heading_meas = (std::isinf(raw_h) || std::isnan(raw_h))
                                     ? prev_heading
                                     : raw_h * DEG2RAD;
    float d_theta_meas = std::remainder(current_heading_meas - prev_heading,
                                        2.0f * static_cast<float>(M_PI));
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

        const auto &cfg = trackingWheelConfigs[i];
        float delta_inches = (delta_centideg / 36000.0f) *
                             static_cast<float>(M_PI) * cfg.wheelDiameter *
                             cfg.gearRatio;
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

      ekf.updateTrackingWheels(trackingWheelConfigs, measured_deltas, dx_local,
                               dy_local, d_theta_meas, trackingWheelMeasNoise);

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

      float track_radius =
          (config.drivetrainWidth + config.drivetrainLength) / 2.0f;
      float vt_inches = (wheel_deltas(0) - wheel_deltas(1) + wheel_deltas(2) -
                         wheel_deltas(3)) /
                        4.0f;
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
      if (velocityCalculationsOn) {
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

/**
 *@brief Adds a tracking wheel to the chassis.
 *@param config The tracking wheel configuration.
 *@return void
 */
void Chassis::addTrackingWheel(TrackingWheelConfig config) {
  trackingWheelConfigs.push_back(config);
  trackingWheelSensors.emplace_back(config.port);

  int32_t initPos = trackingWheelSensors.back().get_position();
  prevTrackingPositions.push_back(static_cast<float>(initPos));
  useTrackingWheels = true;

  ekf.setTrackingWheelNoise(0.0003f, 0.0003f, 0.001f);
  trackingWheelMeasNoise = 0.0005f;

  std::cout << "[Chassis] Added tracking wheel on port "
            << static_cast<int>(config.port)
            << (config.orientation == TrackingWheelOrientation::HORIZONTAL
                    ? " (horizontal)"
                    : " (vertical)")
            << " offset=(" << config.xOffset << ", " << config.yOffset << ")"
            << " dia=" << config.wheelDiameter << " ratio=" << config.gearRatio
            << std::endl;
}

/**
 *@brief Clears all tracking wheels.
 *@return void
 */
void Chassis::clearTrackingWheels() {
  trackingWheelConfigs.clear();
  trackingWheelSensors.clear();
  prevTrackingPositions.clear();
  useTrackingWheels = false;

  ekf.setProcessNoise(0.001f, 0.001f, 0.003f, 0.0001f);
  std::cout
      << "[Chassis] Tracking wheels cleared, reverted to motor encoder odometry"
      << std::endl;
}
