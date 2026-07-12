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

Eigen::Vector3d FlightDynamics::bodyAircraftAccelerationState(const AerodynamicLoads& aeroLoads,
															  const PropulsionLoads& engineLoads,
															  const GravityLoads& gravityLoads,
															  const AircraftState& state 
) const
{
	const double aircraftMass = rcam_.getAircraftMass();

	const Eigen::Vector3d bodyAngularRate = { state.p, state.q, state.r }; // Gets the angular rate vector
	const Eigen::Vector3d bodyTranslationalVelocity = { state.u, state.v, state.w }; // Gets the body aircraft's velocity

	const Eigen::Vector3d gravityBodyForce = gravityLoads.force_body;
	const Eigen::Vector3d engineBodyForce = engineLoads.force_body;
	const Eigen::Vector3d aeroBodyForce = aeroLoads.force_body;
	const Eigen::Vector3d totalForceBody = aeroBodyForce + engineBodyForce + gravityBodyForce; // Gets the total force applied on the aircraft

	// Calculates the body aircraft's acceleration 
	const Eigen::Vector3d accelerationState = (1.0 / aircraftMass) * totalForceBody - bodyAngularRate.cross(bodyTranslationalVelocity);
		
	return accelerationState;
}

Eigen::Vector3d FlightDynamics::bodyAircraftAngularAccelerationState(const AerodynamicLoads& aeroLoads, 
																	 const PropulsionLoads& engineLoads,
																	 const AircraftState& state 
) const
{
	
	const Eigen::Matrix3d bodyFrameInertiaTensor = rcam_.getBodyInertiaTensorMatrix();	
	const Eigen::Matrix3d bodyFrameInvInertiaTensor = rcam_.getInvBodyInertiaTensorMatrix();  

	const Eigen::Vector3d totalBodyMomentCG = aeroLoads.moment_body + engineLoads.moment_body; // Calculates the total moment applied on the aircraft
	const Eigen::Vector3d bodyAngularRate = { state.p, state.q, state.r }; // Gets the angular rate vector
	
	// Calculates the body aircraft's angular acceleration
	const Eigen::Vector3d bodyAngularAcceleration = bodyFrameInvInertiaTensor * (totalBodyMomentCG - bodyAngularRate.cross(bodyFrameInertiaTensor * bodyAngularRate));

	return bodyAngularAcceleration;
}

Eigen::Vector3d FlightDynamics::bodyAircraftEulerAngleRates(const AircraftState& state) const
{
	double theta = state.theta;
	double phi = state.phi;

	Eigen::Matrix3d H_phi_theta; 
	H_phi_theta << 1, std::sin(phi) * std::tan(theta), std::cos(phi) * std::tan(theta),
				   0,			std::cos(phi),				 -std::sin(phi),
				   0, std::sin(phi) / std::cos(theta), std::cos(phi) / std::cos(theta);
	const Eigen::Vector3d bodyAngularRate = { state.p, state.q, state.r };

	const Eigen::Vector3d bodyEulerAngleRates = H_phi_theta * bodyAngularRate; // Calculates the Euler angle rates

	return bodyEulerAngleRates;
}

Eigen::Vector3d FlightDynamics::nedAircraftVelocity(const AircraftState& state) const
{
	double phi = state.phi;
	double theta = state.theta;
	double psi = state.psi;

	// Designs the DCM rotation matrices 
	Eigen::Matrix3d C1v;
	C1v <<  std::cos(phi), std::sin(phi), 0,
		   -std::sin(phi), std::cos(phi), 0,
				  0,           0,         1 ;
	Eigen::Matrix3d C21;
	C21 << std::cos(theta), 0, -std::sin(theta),
			   0,           1,			 0,
		   std::sin(theta), 0,  std::cos(theta);
	Eigen::Matrix3d Cb2;
	Cb2 <<  1,			0,			 0,
			0,  std::cos(psi), std::sin(psi),
			0, -std::sin(psi), std::cos(psi);

	const Eigen::Matrix3d DCM_matrix_Cbv = Cb2 * C21 * C1v; // Reference order matters phi -> theta -> psi
	const Eigen::Vector3d bodyTranslationalVelocity = { state.u, state.v, state.w };

	// Applies a change of frame from the body frame to the vehicle-carried NED frame
	const Eigen::Vector3d NED_velocity = DCM_matrix_Cbv.transpose() * bodyTranslationalVelocity;

	return NED_velocity;
}

StateVector FlightDynamics::computeStateDerivatives(const AircraftState& state, const ControlInputs& input) const
{
	// Computes the aircraft flight conditions
	const FlightConditions fc = aero_.computeFlightConditions(state);

	/*
		Compute all the necessary aircraft data to get all aerodynamic, 
		propulsive and gravitational loads applied to it.
	*/
	const AerodynamicLoads aeroLoads = aero_.computeAerodynamicLoads(fc, input, state);
	const PropulsionLoads engineLoads = engine_.computePropulsionLoads(input);
	const GravityLoads gravityLoads = gravity_.computeGravityLoads(state);

	/*
		Build 3D state vector to get both translational and rotational 
		acceleration, Euler angle rates and the aircraft's velocity 
		expressed in the vehicle-carried NED frame.
	*/
	const Eigen::Vector3d uvw_dot = bodyAircraftAccelerationState(aeroLoads, engineLoads, gravityLoads, state);
	const Eigen::Vector3d pqr_dot = bodyAircraftAngularAccelerationState(aeroLoads, engineLoads, state);
	const Eigen::Vector3d phi_theta_psi_dot = bodyAircraftEulerAngleRates(state);
	const Eigen::Vector3d pn_pe_pd_dot = nedAircraftVelocity(state);
	
	StateVector stateDerivative;

	// Build a state derivative vector which is a concatenation of all 4 3D vectors 
	stateDerivative.segment<3>(0) = uvw_dot;
	stateDerivative.segment<3>(3) = pqr_dot;
	stateDerivative.segment<3>(6) = phi_theta_psi_dot;
	stateDerivative.segment<3>(9) = pn_pe_pd_dot;
	return stateDerivative;
}
