#pragma once
#include "Aerodynamics.h"
#include "Gravity.h"
#include "Propulsion.h"

struct StateDerivatives
{
	double u_dot, v_dot, w_dot;
	double p_dot, q_dot, r_dot;
	double phi_dot, theta_dot, psi_dot;
	double pn_dot, pe_dot, pd_dot;
};

class FlightDynamics
{
public:
	FlightDynamics(const Aerodynamics& aero,
				   const Atmosphere& atmosphere,
				   const Gravity& gravity,
				   const Propulsion& propulsion,
				   const RCAM_model& rcam);
	StateDerivatives bodyAircraftAccelerationState(const AircraftState& state, const ControlInputs& input) const;
	StateDerivatives bodyAircraftAngularAccelerationState(const AircraftState& state, const ControlInputs& input) const;
	StateDerivatives bodyAircraftAngularVelocity(const AircraftState& state, const ControlInputs& input) const;
	StateDerivatives NEDtranslationalVelocity(const AircraftState& state, const ControlInputs& input) const;
	StateDerivatives computeStateDerivatives(const AircraftState& state, const ControlInputs& input) const;

private:
	const Aerodynamics& aero_;
	const Atmosphere& atmosphere_;
	const Gravity& gravity_;
	const Propulsion& engine_;
	const RCAM_model& rcam_;
};

