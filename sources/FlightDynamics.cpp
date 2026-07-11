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
															  const GravityLoads& gravity,
															  const AircraftState& state 
) const
{
	const double aircraftMass = rcam_.getAircraftMass();

	const Eigen::Vector3d bodyAngularRate = { state.p, state.q, state.r };
	const Eigen::Vector3d bodyTranslationalVelocity = { state.u, state.v, state.w };

	const Eigen::Vector3d gravityBodyForce = gravity.force_body;
	const Eigen::Vector3d engineBodyForce = engineLoads.force_body;
	const Eigen::Vector3d aeroBodyForce = aeroLoads.force_body;
	const Eigen::Vector3d totalForceBody = aeroBodyForce + engineBodyForce + gravityBodyForce;


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

	const Eigen::Vector3d totalBodyMomentCG = aeroLoads.moment_body + engineLoads.moment_body;
	const Eigen::Vector3d bodyAngularRate = { state.p, state.q, state.r };
	
	const Eigen::Vector3d bodyAngularAcceleration = bodyFrameInvInertiaTensor * (totalBodyMomentCG - bodyAngularRate.cross(bodyFrameInertiaTensor * bodyAngularRate));

	return bodyAngularAcceleration;
}

Eigen::Vector3d FlightDynamics::bodyAircraftEulerAngleRates(const AircraftState& state) const
{
	double theta = state.theta;
	double phi = state.phi;

	const Eigen::Matrix3d H_phi_theta = { 1, std::sin(phi) * std::tan(theta), std::cos(phi) * std::tan(theta),
										  0,			std::cos(phi),				 -std::sin(phi),
										  0, std::sin(phi) / std::cos(theta), std::cos(phi) / std::cos(theta) };
	const Eigen::Vector3d bodyAngularRate = { state.p, state.q, state.r };

	const Eigen::Vector3d bodyAngularVelocity = H_phi_theta * bodyAngularRate;

	return bodyAngularVelocity;
}

Eigen::Vector3d FlightDynamics::nedAircraftVelocity(const AircraftState& state) const
{
	double phi = state.phi;
	double theta = state.theta;
	double psi = state.psi;

	const Eigen::Matrix3d C1v = { std::cos(phi), std::sin(phi), 0,
								 -std::sin(phi), std::cos(phi), 0,
									    0,           0,         1 };
	const Eigen::Matrix3d C21 = { std::cos(theta), 0, -std::sin(theta),
									  0,           1,			 0,
								  std::sin(theta), 0,  std::cos(theta) };
	const Eigen::Matrix3d Cb2 = { 1,			0,			 0,
								  0,  std::cos(psi), std::sin(psi),
								  0, -std::sin(psi), std::cos(psi)};

	const Eigen::Matrix3d DCM_matrix_Cbv = Cb2 * C21 * C1v;
	const Eigen::Vector3d bodyTranslationalVelocity = { state.u, state.v, state.w };
	const Eigen::Vector3d NED_velocity = DCM_matrix_Cbv.transpose() * bodyTranslationalVelocity;

	return NED_velocity;
}

StateVector FlightDynamics::computeStateDerivatives(const AircraftState& state, const ControlInputs& input) const
{
	const FlightConditions fc = aero_.computeFlightConditions(state);
	const AerodynamicLoads aeroLoads = aero_.computeAerodynamicLoads(fc, input, state);
	const PropulsionLoads engineLoads = engine_.computePropulsionLoads(input);
	const GravityLoads gravityLoads = gravity_.computeGravityLoads(state);

	const Eigen::Vector3d uvw_dot = bodyAircraftAccelerationState(aeroLoads, engineLoads, gravityLoads, state);
	const Eigen::Vector3d pqr_dot = bodyAircraftAngularAccelerationState(aeroLoads, engineLoads, state);
	const Eigen::Vector3d phi_theta_psi_dot = bodyAircraftEulerAngleRates(state);
	const Eigen::Vector3d pn_pe_pd_dot = nedAircraftVelocity(state);
	
	StateVector stateDerivative{};
	stateDerivative <<	uvw_dot, 
						pqr_dot, 
						phi_theta_psi_dot, 
						pn_pe_pd_dot;

	return stateDerivative;
}
