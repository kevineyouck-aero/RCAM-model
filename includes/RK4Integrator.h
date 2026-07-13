#pragma once
#include "FlightDynamics.h"






class RK4Integrator
{
public:
	RK4Integrator() = default;

	AircraftState toAircraftState(const StateVector& vector) const;

	StateVector toStateVector(const AircraftState& aircraft) const;

	StateVector step(const FlightDynamics& dynamics,
				     const StateVector& state, 
					 const ControlInputs& input,
					 double stepTime) const;	
private:

};