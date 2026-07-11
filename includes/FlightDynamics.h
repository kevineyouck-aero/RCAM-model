#pragma once
#include "Aerodynamics.h"
#include "Gravity.h"
#include "Propulsion.h"

using StateVector = Eigen::Matrix<double, 12, 1>;


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
	Eigen::Vector3d bodyAircraftAccelerationState(const AerodynamicLoads& aeroLoads,
												  const PropulsionLoads& engineLoads, 
												  const GravityLoads& gravity,
												  const AircraftState& state
											     ) const;

	Eigen::Vector3d bodyAircraftAngularAccelerationState(const AerodynamicLoads& aeroLoads,
														 const PropulsionLoads& engineLoads,
														 const AircraftState& state
														 ) const;

	Eigen::Vector3d bodyAircraftEulerAngleRates(const AircraftState& state) const;
	Eigen::Vector3d nedAircraftVelocity(const AircraftState& state) const;
	StateVector computeStateDerivatives(const AircraftState& state, const ControlInputs& input) const;

private:
	const Aerodynamics& aero_;
	const Atmosphere& atmosphere_;
	const Gravity& gravity_;
	const Propulsion& engine_;
	const RCAM_model& rcam_;
};

