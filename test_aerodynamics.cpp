#include "Aerodynamics.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <numbers>

using namespace std;


bool near(double a, double b, double tol = 1e-6) {
	return abs(a - b) < tol;
}


int main() {


	Atmosphere atmosphere;
	Aerodynamics aero(atmosphere);

	AircraftState s{};

	s.u = 3;
	s.v = 4;
	s.w = 12;

	cout << "----------Airspeed--------------" << endl;
	assert(near(aero.calculateAirspeed(s), 13.0));
	cout << "The Airspeed is " << aero.calculateAirspeed(s) << " m/s." << endl;

	cout << endl << "----------Angle of attack--------------" << endl;
	assert(near(aero.calculateAlpha(s), atan2(12.0, 3.0)));
	cout << "The angle of attack is " << aero.calculateAlpha(s) << " rad ";
	cout << "or " << (180 / numbers::pi) * aero.calculateAlpha(s) << " degrees." << endl;

	cout << endl << "----------Angle of sideslipe--------------" << endl;
	assert(near(aero.calculateBeta(s), asin(4.0 / 13.0)));
	cout << "The angle of sideslipe is " << aero.calculateBeta(s) << " rad ";
	cout << "or " << (180 / numbers::pi) * aero.calculateBeta(s) << " degrees." << endl;

	return 0;
}