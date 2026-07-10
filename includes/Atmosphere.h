#pragma once
/**
	\class Atmosphere
	Atmosphere is a class that defines the atmospheric conditions
	occuring on the aircraft.
**/
class Atmosphere
{

public:

	Atmosphere() = default;

	/**
		\fn getRho()
		\brief gets the atmosphere the air density (kg/m^3)
	**/

	double getRho() const;

	/**
		\fn getGravity()
		\brief gets the Earth gravitational acceleration
	**/
	double getGravity() const;

private:
	static constexpr double rho_ = 1.225;
	static constexpr double g_ = 9.81;

};

