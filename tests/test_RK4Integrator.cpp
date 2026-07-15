#include "../includes/RK4Integrator.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>

static void writeAircraftStateResults(std::ofstream& o, const StateVector& vector, double time) {
	o << time;
	for (int i = 0; i < vector.size(); i++) {
		o << "," << vector(i);
	}
	o << "\n";
}


/*int main() {
	
	RK4Integrator rk4;
	const AircraftState trimState{ 84.9905,0,1.2713,0,0,0,0,0.0149,0,0,0,500 };
	const ControlInputs trimInput{ 0,-0.1780,0,0.0821,0.0821 };

	StateVector aircraftState = rk4.toStateVector(trimState);
	
	const Atmosphere atmosphere;
	const AircraftGeometry geometry;
	const Aerodynamics aero(atmosphere, geometry);

	const RCAM_model rcam(trimState, trimInput);
	const Gravity gravity(atmosphere, rcam);
	const Propulsion propulsion(atmosphere, rcam, geometry);

	const FlightDynamics flightDynamics(aero, atmosphere, gravity, propulsion, rcam);
	constexpr double dt = 0.02;

	const std::string fileName = "AircraftStateResult.csv";
	std::ofstream aircraftFile(fileName, std::ios::out | std::ios::trunc);
	aircraftFile << "time,u,v,w,p,q,r,phi,theta,psi,pn,pe,pd\n";
	std::cout << "0s" << aircraftState.transpose() << std::endl;
	writeAircraftStateResults(aircraftFile, aircraftState, 0.0);
	for (int i = 0; i < 9000; i++)
	{
		aircraftState = rk4.step(flightDynamics,
								 aircraftState,
								 trimInput,
								 dt);
		if ((i+1) % 100 == 0) {
			std::cout << (i+1) * dt << "s" << aircraftState.transpose() << std::endl;
			writeAircraftStateResults(aircraftFile, aircraftState, (i+1) * dt);
		}
	}
	aircraftFile.close();
	return 0;
}*/