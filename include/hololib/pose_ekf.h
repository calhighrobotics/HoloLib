#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"
#include "hololib/tracking_wheel.h"
#include <cmath>
#include <vector>

/**
 * @brief Extended Kalman Filter implementation for maintaining the robot's
 * pose.
 *
 * Fuses IMU heading and tracking wheel (or motor encoder) deltas to estimate
 * X, Y, and Theta coordinates on the field.
 */
class PoseEKF {
private:
  Eigen::Vector3f x;
  Eigen::Matrix3f P;
  Eigen::Matrix<float, 1, 3> H;
  float measurementNoise;
  float xProcessNoise = 0.01f, yProcessNoise = 0.01f,
        thetaProcessNoise = 0.001f;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Set the process noise values for the Kalman filter matrices.
   *
   * @param xNoise Process noise in the X direction.
   * @param yNoise Process noise in the Y direction.
   * @param thetaNoise Process noise for the heading.
   * @param measNoise Measurement noise parameter.
   */
  void setProcessNoise(float xNoise, float yNoise, float thetaNoise,
                       float measNoise) {
    xProcessNoise = xNoise;
    yProcessNoise = yNoise;
    thetaProcessNoise = thetaNoise;
    measurementNoise = measNoise;
  }

  /**
   * @brief Construct a new Pose EKF object.
   *
   * @param initial_x Starting X position.
   * @param initial_y Starting Y position.
   * @param initial_theta Starting heading.
   */
  PoseEKF(float initial_x, float initial_y, float initial_theta) {
    x << initial_x, initial_y, initial_theta;
    P.setZero();
    H << 0, 0, 1;
    measurementNoise = 0.00005f;
  }

  /**
   * @brief The prediction step of the EKF based on local relative movements.
   *
   * @param dx_local Change in local X position.
   * @param dy_local Change in local Y position.
   * @param dtheta Change in heading.
   */
  void predict(float dx_local, float dy_local, float dtheta) {
    float s, c;
    if (std::abs(dtheta) < 1e-4f) {
      s = 1.0f - (dtheta * dtheta) / 6.0f;
      c = dtheta / 2.0f;
    } else {
      s = std::sin(dtheta) / dtheta;
      c = (1.0f - std::cos(dtheta)) / dtheta;
    }

    Eigen::Vector2f local_d(dx_local, dy_local);
    Eigen::Matrix2f R_arc;
    R_arc << s, c, -c, s;
    Eigen::Vector2f arc_d = R_arc * local_d;

    float prev_theta = x(2);
    float cos_t = std::cos(prev_theta);
    float sin_t = std::sin(prev_theta);

    Eigen::Matrix2f R_global;
    R_global << cos_t, sin_t, -sin_t, cos_t;
    Eigen::Vector2f global_d = R_global * arc_d;

    x.head<2>() += global_d;
    x(2) += dtheta;
    x(2) = std::remainder(x(2), 2.0f * M_PI);

    Eigen::Matrix3f F = Eigen::Matrix3f::Identity();
    F(0, 2) = global_d.y();
    F(1, 2) = -global_d.x();

    Eigen::Matrix3f Q_step = Eigen::Matrix3f::Zero();
    float var_x = xProcessNoise * std::abs(dx_local) + 1e-6f;
    float var_y = yProcessNoise * std::abs(dy_local) + 1e-6f;
    Eigen::Matrix2f Q_local;
    Q_local << var_x, 0, 0, var_y;

    Q_step.block<2, 2>(0, 0) = R_global * Q_local * R_global.transpose();
    Q_step(2, 2) = thetaProcessNoise * std::abs(dtheta) + 1e-7f;

    P = F * P * F.transpose() + Q_step;
  }

  /**
   * @brief Update the state using absolute heading measured by an IMU.
   *
   * @param measured_theta Heading read from the IMU.
   * @param dynamic_R Dynamic measurement noise.
   */
  void updateIMU(float measured_theta, float dynamic_R) {
    float y = measured_theta - x(2);
    y = std::remainder(y, 2.0f * M_PI);
    float S = (H * P * H.transpose())(0, 0) + dynamic_R;
    Eigen::Vector3f K = P * H.transpose() / S;
    x = x + K * y;
    x(2) = std::remainder(x(2), 2.0f * M_PI);
    P = (Eigen::Matrix3f::Identity() - K * H) * P;
  }

  /**
   * @brief Update the state using unpowered tracking wheels data.
   *
   * @param configs Vector of tracking wheel configurations.
   * @param measured_deltas Measurement differences for each wheel since the
   * last step.
   * @param dx_local Estimated local X change.
   * @param dy_local Estimated local Y change.
   * @param dtheta Estimated heading change.
   * @param wheel_noise Sensor noise for the tracking wheels.
   */
  void updateTrackingWheels(const std::vector<TrackingWheelConfig> &configs,
                            const Eigen::VectorXf &measured_deltas,
                            float dx_local, float dy_local, float dtheta,
                            float wheel_noise) {
    const int n = static_cast<int>(configs.size());
    if (n == 0 || measured_deltas.size() != n)
      return;
    float theta = x(2);
    float cos_t = std::cos(theta);
    float sin_t = std::sin(theta);

    Eigen::MatrixXf H_tw(n, 3);
    Eigen::VectorXf z_pred(n);

    for (int i = 0; i < n; ++i) {
      float ox = configs[i].xOffset;
      float oy = configs[i].yOffset;

      if (configs[i].orientation == TrackingWheelOrientation::VERTICAL) {
        z_pred(i) = dy_local - ox * dtheta;
        H_tw(i, 0) = -sin_t;
        H_tw(i, 1) = cos_t;
        H_tw(i, 2) = -ox;
      } else {

        z_pred(i) = dx_local + oy * dtheta;
        H_tw(i, 0) = cos_t;
        H_tw(i, 1) = sin_t;
        H_tw(i, 2) = oy;
      }
    }

    Eigen::VectorXf y_innov = measured_deltas - z_pred;
    Eigen::MatrixXf R_tw = Eigen::MatrixXf::Identity(n, n) * wheel_noise;
    Eigen::MatrixXf S = H_tw * P * H_tw.transpose() + R_tw;
    Eigen::MatrixXf K = P * H_tw.transpose() * S.inverse();
    x += K * y_innov;
    x(2) = std::remainder(x(2), 2.0f * static_cast<float>(M_PI));

    Eigen::Matrix3f I3 = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f IKH = I3 - K * H_tw;
    P = IKH * P * IKH.transpose() + K * R_tw * K.transpose();
  }

  /**
   * @brief Set specific process noises when using tracking wheels.
   *
   * @param xNoise Process noise in X direction.
   * @param yNoise Process noise in Y direction.
   * @param thetaNoise Process noise for the heading.
   */
  void setTrackingWheelNoise(float xNoise, float yNoise, float thetaNoise) {
    xProcessNoise = xNoise;
    yProcessNoise = yNoise;
    thetaProcessNoise = thetaNoise;
  }

  /**
   * @brief Get current X position.
   * @return float The X position.
   */
  float getX() const { return x(0); }

  /**
   * @brief Get current Y position.
   * @return float The Y position.
   */
  float getY() const { return x(1); }

  /**
   * @brief Get current heading angle.
   * @return float The heading in radians.
   */
  float getTheta() const { return x(2); }

  /**
   * @brief Reset the state to a specific pose.
   *
   * @param new_x New X position.
   * @param new_y New Y position.
   * @param new_theta New heading angle.
   */
  void setPose(float new_x, float new_y, float new_theta) {
    x << new_x, new_y, new_theta;
    P.setZero();
  }
};
