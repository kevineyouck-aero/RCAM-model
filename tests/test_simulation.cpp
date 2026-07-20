#include "../includes/Aerodynamics.h"
#include "../includes/AircraftGeometry.h"
#include "../includes/Atmosphere.h"
#include "../includes/CSVLogger.h"
#include "../includes/FlightDynamics.h"
#include "../includes/Gravity.h"
#include "../includes/Propulsion.h"
#include "../includes/RCAM_model.h"
#include "../includes/RK4Integrator.h"
#include "../includes/Simulation.h"


int main() {
	const AircraftState initState = { 85,0,0,0,0,0,0,0.1,0,0,0,500 };
	const ControlInputs initInput = { 0,-0.1,0,0.08,0.08 };

	const AircraftState trimState = { 84.9905,0,1.2713,0,0,0,0,0.0149,0,0,0,500 };
	const ControlInputs trimInput = { 0,-0.1780,0,0.0821,0.0821 };

	const Atmosphere atmosphere;
	const AircraftGeometry geometry;
	const Aerodynamics aero(atmosphere, geometry);
	const RCAM_model rcam(initState,initInput);
	const Gravity gravity(atmosphere, rcam);
	const Propulsion prop(atmosphere, rcam, geometry);
	const RK4Integrator rk4;
	const std::string filename = "AircraftStateTrimResults.csv";

	CSVLogger aircraftFile(filename);

	const FlightDynamics dynamics(aero, atmosphere, gravity, prop, rcam);

	Simulation sim(dynamics, rk4.toStateVector(trimState), rk4);
	
	sim.run(aircraftFile, trimInput, 250, 0.1);

}