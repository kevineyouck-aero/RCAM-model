#include "../includes/Aerodynamics.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <numbers>



bool near(double a, double b, double tol = 1e-6) {
	return abs(a - b) < tol;
}


/*int main() {


	Atmosphere atmosphere;
	AircraftGeometry geometry;
	Aerodynamics aero(atmosphere, geometry);

	AircraftState s{};

	s.u = 3;
	s.v = 4;
	s.w = 12;

	std::cout << "----------Airspeed--------------" << std::endl;
	assert(near(aero.calculateAirspeed(s), 13.0));
	std::cout << "The Airspeed is " << aero.calculateAirspeed(s) << " m/s." << std::endl;

	std::cout << std::endl << "----------Angle of attack--------------" << std::endl;
	assert(near(aero.calculateAlpha(s), atan2(12.0, 3.0)));
	std::cout << "The angle of attack is " << aero.calculateAlpha(s) << " rad ";
	std::cout << "or " << (180 / std::numbers::pi) * aero.calculateAlpha(s) << " degrees." << std::endl;

	std::cout << std::endl << "----------Angle of sideslipe--------------" << std::endl;
	assert(near(aero.calculateBeta(s), asin(4.0 / 13.0)));
	std::cout << "The angle of sideslipe is " << aero.calculateBeta(s) << " rad ";
	std::cout << "or " << (180 / std::numbers::pi) * aero.calculateBeta(s) << " degrees." << std::endl;

	return 0;
}*/