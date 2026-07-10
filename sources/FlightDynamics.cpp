#include "../includes/FlightDynamics.h"


FlightDynamics::FlightDynamics(const Aerodynamics& aero, 
							   const Atmosphere& atmosphere, 
	                           const Gravity& gravity, 
	                           const Propulsion& engine,
							   const RCAM_model& rcam):
	aero_(aero),
	atmosphere_(atmosphere),
	gravity_(gravity),
	engine_(engine),
	rcam_(rcam)
{
}

StateDerivatives FlightDynamics::bodyAircraftAccelerationState(const AircraftState& state, const ControlInputs& input) const
{
	StateDerivatives accelerationStateDerivatives{};
	FlightConditions fc = aero_.computeFlightConditions(state);
	double aircraftMass = rcam_.getAircraftMass();

	Eigen::Vector3d bodyAngularVelocity = { state.p, state.q, state.r };
	Eigen::Vector3d bodyTranslationalVelocity = { state.u, state.v, state.w };

	Eigen::Vector3d gravityBodyForce = gravity_.computeGravityLoads(state).force_body;
	Eigen::Vector3d engineBodyForce = engine_.computePropulsionLoads(input).force_body;
	Eigen::Vector3d aeroBodyForce = aero_.computeAerodynamicLoads(fc, input, state).force_body;
	Eigen::Vector3d totalForceBody = aeroBodyForce + engineBodyForce + gravityBodyForce;


	Eigen::Vector3d accelerationState = (1 / aircraftMass) * totalForceBody - bodyAngularVelocity.cross(bodyTranslationalVelocity);

	accelerationStateDerivatives.u_dot = accelerationState(0);
	accelerationStateDerivatives.v_dot = accelerationState(1);
	accelerationStateDerivatives.w_dot = accelerationState(2);

	return accelerationStateDerivatives;
}

StateDerivatives FlightDynamics::computeStateDerivatives(const AircraftState& state, const ControlInputs& input) const
{
	
	return StateDerivatives();
}
