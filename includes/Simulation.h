#pragma once
#include "FlightDynamics.h"
#include "RK4Integrator.h"
#include "CSVLogger.h"


class Simulation
{
public:
	Simulation(const FlightDynamics& dynamics, StateVector state, const RK4Integrator& rk4);
	void run(CSVLogger& logger,
			 const ControlInputs& input, 
		     double simTime, 
		     double dt, 
		     double loggingPeriod = 1.0);

private:
	const FlightDynamics dynamics_;
	StateVector state_;
	const RK4Integrator rk4_;
};