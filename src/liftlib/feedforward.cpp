#include "liftlib/feedforward.hpp"

#include <cmath>
#include <numbers>

namespace liftlib {

Feedforward Feedforward::constant(float kG) {
	Feedforward feedforward;
	feedforward.model = GravityModel::Constant;
	feedforward.kG = kG;
	return feedforward;
}

Feedforward Feedforward::cosine(float kG, float horizontal, float degreesPerUnit) {
	Feedforward feedforward;
	feedforward.model = GravityModel::Cosine;
	feedforward.kG = kG;
	feedforward.horizontal = horizontal;
	feedforward.degreesPerUnit = degreesPerUnit;
	return feedforward;
}

float Feedforward::calculate(float position) const {
	switch (model) {
		case GravityModel::Constant:
			return kG;

		case GravityModel::Cosine: {
			const float degrees = (position - horizontal) * degreesPerUnit;
			const float radians = degrees * std::numbers::pi_v<float> / 180.0f;
			return kG * std::cos(radians);
		}

		case GravityModel::None:
		default:
			return 0;
	}
}

void Feedforward::setModel(GravityModel model) {
	this->model = model;
}

GravityModel Feedforward::getModel() const {
	return model;
}

void Feedforward::setKG(float kG) {
	this->kG = kG;
}

float Feedforward::getKG() const {
	return kG;
}

void Feedforward::setHorizontal(float horizontal) {
	this->horizontal = horizontal;
}

float Feedforward::getHorizontal() const {
	return horizontal;
}

void Feedforward::setDegreesPerUnit(float degreesPerUnit) {
	this->degreesPerUnit = degreesPerUnit;
}

float Feedforward::getDegreesPerUnit() const {
	return degreesPerUnit;
}

bool Feedforward::isEnabled() const {
	return model != GravityModel::None && kG != 0;
}

}  
