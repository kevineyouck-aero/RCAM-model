#include "../includes/FlightDynamics.h"
#include <iostream>



/*int main() {
	
	const AircraftState trimState{ 84.9905,0,1.2713,0,0,0,0,0.0149,0,0,0,500 };
	const ControlInputs trimInput{ 0,-0.1780,0,0.0821,0.0821 };

	const Atmosphere atmosphere;
	const AircraftGeometry geometry;
	const Aerodynamics aero(atmosphere, geometry);

	const FlightConditions fc = aero.computeFlightConditions(trimState);
	std::cout << "Flight Conditions: \n";
	std::cout << fc.Va << " m/s \n";
	std::cout << fc.qbar << " Pascals \n";
	std::cout << fc.alpha << " rad\n";
	std::cout << fc.beta << " rad\n\n";

	const RCAM_model rcam(trimState, trimInput);
	const Gravity gravity(atmosphere, rcam);
	const Propulsion propulsion(atmosphere, rcam, geometry);

	const FlightDynamics flightDynamics(aero, atmosphere, gravity, propulsion, rcam);
	const AerodynamicLoads aeroLoads = aero.computeAerodynamicLoads(fc, trimInput, trimState);
	assert(aeroLoads.force_body(2) < 0.0);

	StateVector dx = flightDynamics.computeStateDerivatives(trimState, trimInput);
	std::cout <<"xdot:\n" << dx << std::endl;
	return 0;
}*/