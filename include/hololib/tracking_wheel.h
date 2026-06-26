#pragma once

#include <cstdint>

/**
 * @brief Defines the orientation of an unpowered tracking wheel.
 */
enum class TrackingWheelOrientation { HORIZONTAL, VERTICAL };

/**
 * @brief Configuration parameters for a tracking wheel.
 */
struct TrackingWheelConfig {
  int8_t port; /**< Sensor port number */
  TrackingWheelOrientation
      orientation;     /**< Orientation (horizontal or vertical) */
  float xOffset;       /**< Offset from tracking center on the X axis */
  float yOffset;       /**< Offset from tracking center on the Y axis */
  float wheelDiameter; /**< Diameter of the tracking wheel */
  float gearRatio;     /**< Gear ratio between the wheel and the sensor */
};
