#include "hololib/encoder_filter.h"

/**
 *@brief Constructs an EncoderKalmanFilter.
 *@param process_noise The process noise for the filter.
 *@param measurement_noise The measurement noise for the filter.
 */
EncoderKalmanFilter::EncoderKalmanFilter(float process_noise,
                                         float measurement_noise) {
  x.setZero();
  P.setIdentity();
  Q << process_noise, 0, 0, process_noise;
  R = measurement_noise;
  H << 1, 0;
}

/**
 *@brief Updates the filter with a measured position.
 *@param measured_position The measured position.
 *@param dt The time step.
 *@return float The filtered position.
 */
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
