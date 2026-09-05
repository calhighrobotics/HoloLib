#pragma once

namespace liftlib {

/** How the holding output varies with position. */
enum class GravityModel {
	/** No gravity term. */
	None,
	/**
	 * A constant push, for mechanisms whose load does not change with position:
	 * a DR4B, a cascade stage, anything travelling straight up.
	 */
	Constant,
	/**
	 * Scaled by the cosine of the arm angle, for a pivoting arm where the load
	 * peaks when the arm is horizontal.
	 */
	Cosine,
};

/**
 * The output needed to hold a mechanism against gravity, added to the PID so the
 * controller does not have to build up error before it pushes back.
 */
class Feedforward {
   private:
	GravityModel model = GravityModel::None;
	float kG = 0;

	/** Position reported when a Cosine arm is horizontal. */
	float horizontal = 0;

	/** Position units per degree, for converting a Cosine arm's position. */
	float degreesPerUnit = 1;

   public:
	Feedforward() = default;

	/** A constant holding output. */
	static Feedforward constant(float kG);

	/**
	 * A holding output scaled by cos(angle).
	 *
	 * horizontal is the position at which the arm is level, and degreesPerUnit
	 * converts the subsystem's position units into degrees of arm rotation.
	 */
	static Feedforward cosine(float kG, float horizontal = 0, float degreesPerUnit = 1);

	/** The holding output at a position, before the PID term is added. */
	float calculate(float position) const;

	void setModel(GravityModel model);
	GravityModel getModel() const;

	void setKG(float kG);
	float getKG() const;

	void setHorizontal(float horizontal);
	float getHorizontal() const;

	void setDegreesPerUnit(float degreesPerUnit);
	float getDegreesPerUnit() const;

	bool isEnabled() const;
};

}  // namespace liftlib
