#pragma once

#include <cmath>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Dense"

namespace hololib {

struct TrackingWheelConfig;
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
    float xProcessNoise = 0.01f, yProcessNoise = 0.01f, thetaProcessNoise = 0.001f;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct a new Pose EKF object.
     *
     * @param initial_x Starting X position.
     * @param initial_y Starting Y position.
     * @param initial_theta Starting heading.
     */
    PoseEKF(float initial_x, float initial_y, float initial_theta);

    /**
     * @brief Set the process noise values for the Kalman filter matrices.
     *
     * @param xNoise Process noise in the X direction.
     * @param yNoise Process noise in the Y direction.
     * @param thetaNoise Process noise for the heading.
     * @param measNoise Measurement noise parameter.
     */
    void setProcessNoise(float xNoise, float yNoise, float thetaNoise, float measNoise);

    /**
     * @brief The prediction step of the EKF based on local relative movements.
     *
     * @param dx_local Change in local X position.
     * @param dy_local Change in local Y position.
     * @param dtheta Change in heading.
     */
    void predict(float dx_local, float dy_local, float dtheta);

    /**
     * @brief Update the state using absolute heading measured by an IMU.
     *
     * @param measured_theta Heading read from the IMU.
     * @param dynamic_R Dynamic measurement noise.
     */
    void updateIMU(float measured_theta, float dynamic_R);

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
    void updateTrackingWheels(const std::vector<TrackingWheelConfig>& configs, const Eigen::VectorXf& measured_deltas,
                              float dx_local, float dy_local, float dtheta, float wheel_noise);

    /**
     * @brief Set specific process noises when using tracking wheels.
     *
     * @param xNoise Process noise in X direction.
     * @param yNoise Process noise in Y direction.
     * @param thetaNoise Process noise for the heading.
     */
    void setTrackingWheelNoise(float xNoise, float yNoise, float thetaNoise);

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
    void setPose(float new_x, float new_y, float new_theta);
};
} // namespace hololib