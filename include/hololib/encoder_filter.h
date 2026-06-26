#pragma once

#include "Eigen/Core"
#include "Eigen/Dense"

/**
 * @brief Standard 1D Kalman filter for smoothing raw motor encoder values.
 */
class EncoderKalmanFilter {
public:
  /**
   * @brief Construct an Encoder Kalman Filter.
   *
   * @param process_noise Estimated variability of the process.
   * @param measurement_noise Noise characteristics of the encoder.
   */
  EncoderKalmanFilter(float process_noise = 0.077f,
                      float measurement_noise = 7.2f);

  /**
   * @brief Updates the filter with a new measurement.
   *
   * @param measured_position The raw encoder position.
   * @param dt Time elapsed since last update.
   * @return float The filtered/smoothed position.
   */
  float update(float measured_position, float dt);

private:
  Eigen::Vector2f x;
  Eigen::Matrix2f P, Q;
  Eigen::RowVector2f H;
  float R;
};
