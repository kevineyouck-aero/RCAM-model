#pragma once
#include "Aerodynamics.h"
#include "Gravity.h"
#include "Propulsion.h"


/**
	* 12-element aircraft state derivative vector.
	
**/
using StateVector = Eigen::Matrix<double, 12, 1>;


/**
	* @struct StateDerivatives
	* 
	* Stores the aircraft state derivatives.
**/
struct StateDerivatives
{
	double u_dot, v_dot, w_dot;
	double p_dot, q_dot, r_dot;
	double phi_dot, theta_dot, psi_dot;
	double pn_dot, pe_dot, pd_dot;
};

/**
	* @class FlightDynamics
	* 
	* Computes the rigid-body aircraft dynamics using the Newton–Euler equations of motion. 
**/

class FlightDynamics
{
public:
	FlightDynamics(const Aerodynamics& aero,
				   const Atmosphere& atmosphere,
				   const Gravity& gravity,
				   const Propulsion& propulsion,
				   const RCAM_model& rcam);

	/**
		* Computes the body aircraft's acceleration
		* 
		* @param aeroLoads Structure that contains both aerodynamic forces and moments
		* @param engineLoads Structure that contains both propulsive forces and moments
		* @param gravityLoads Structure that contains both gravitational forces and moments 
		* @param Current aircraft state
		* @return Body-frame translational acceleration (m/s²)
	**/
	Eigen::Vector3d bodyAircraftAccelerationState(const AerodynamicLoads& aeroLoads,
												  const PropulsionLoads& engineLoads, 
												  const GravityLoads& gravityLoads,
												  const AircraftState& state
											     ) const;

	/**
		* Computes the body-frame angular acceleration
		* 
		* @param aeroLoads Structure that contains both aerodynamic forces and moments
		* @param engineLoads Structure that contains both propulsive forces and moments
		* @param Current aircraft state
		* @return Body-frame angular acceleration (rad/s²)
	**/
	Eigen::Vector3d bodyAircraftAngularAccelerationState(const AerodynamicLoads& aeroLoads,
														 const PropulsionLoads& engineLoads,
														 const AircraftState& state
														 ) const;

	/**
		* Computes the Euler angle rates
		* 
		* @param Current aircraft state
		* @return Euler angle rates (rad/s)
	**/
	Eigen::Vector3d bodyAircraftEulerAngleRates(const AircraftState& state) const;

	/**
		* Computes the aircraft's vehicle-carried NED velocity
		* 
		* @param Current aircraft state
		* @return Aircraft velocity expressed in the vehicle-carried NED frame (m/s)
	**/
	Eigen::Vector3d nedAircraftVelocity(const AircraftState& state) const;

	/**
		* Computes the state derivative of the aircraft
		* 
		* @param Current aircraft state
		* @param input Surface control deflections [aileron, elevator, rudder, Throttles]
		* @return A state derivative vector


	**/
	StateVector computeStateDerivatives(const AircraftState& state, const ControlInputs& input) const;

private:
	const Aerodynamics& aero_;
	const Atmosphere& atmosphere_;
	const Gravity& gravity_;
	const Propulsion& engine_;
	const RCAM_model& rcam_;
};

