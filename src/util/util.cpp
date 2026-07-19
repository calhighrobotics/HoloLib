#include "hololib/util/util.hpp"

namespace hololib {

float getAngleError(float target, float current) { return std::remainder(target - current, 360.0f); }

} // namespace hololib