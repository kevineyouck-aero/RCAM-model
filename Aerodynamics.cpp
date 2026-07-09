#include "Aerodynamics.h"
#include <cmath>
#include <iostream>
#include <algorithm>


Aerodynamics::Aerodynamics(const Atmosphere& atmosphere):atmosphere_(atmosphere)
{
}

double Aerodynamics::calculateAirspeed(const AircraftState& state)
{
	double u2 = state.u * state.u;
	double v2 = state.v * state.v;
	double w2 = state.w * state.w;
	return std::sqrt(u2 + v2 + w2); // computes the aircraft true airspeed in body-axis velocity components
}

double Aerodynamics::calculateAlpha(const AircraftState& state)
{
	return std::atan2(state.w, state.u); // returns the plane angle of attack
}

double Aerodynamics::calculateBeta(const AircraftState& state)
{
	double Va = calculateAirspeed(state);	
	if (Va < 1e-06)
		return 0.0;
	return std::asin(std::clamp(state.v / Va, -1.0, 1.0)); // return the plane sideslipe angle 
}

FlightConditions Aerodynamics::computeFlightConditions(const AircraftState& state)
{
	FlightConditions fc{};
	fc.Va = calculateAirspeed(state);						// Calculates the airspeed at the current state
	fc.alpha = calculateAlpha(state);						// Calculates the current state angle of attack
	fc.beta = calculateBeta(state);							// Calculates the current state sideslipe angle
	fc.qbar = 0.5 * atmosphere_.getRho() * fc.Va * fc.Va;	// Calculates the dynamic pressure of the plane
	return fc;
}

double Aerodynamics::calculateLiftCoefficient(const FlightConditions& fc,
											  const ControlInputs& input, 
											  const AircraftState& state
)
{
	double Va = std::max(fc.Va, 1e-6);
	double alpha = fc.alpha;
	double wing_lift_coefficient = 0.0;
	double alpha2 = alpha * alpha;
	double alpha3 = alpha2 * alpha;
	/*
		Defines an interval set by (alpha_switch) to dress up the wing lift 
		coefficient that follows a linear pattern when the angle of attack 
		(alpha) is below (alpha_switch). As soon (alpha) reaches over 
		(alpha_switch) the wing lift coefficient begins to follow a 
		polynomial pattern.	
	*/
	if (alpha <= alpha_switch)
		wing_lift_coefficient = n * (alpha - alpha_zero_lift_angle_of_attack);
	else	
		wing_lift_coefficient = a3 * alpha3 + a2 * alpha2 + a1 * alpha + a0;
	
	double tail_lift_coefficient = 0.0;
	double epsilon = depsda * (alpha - alpha_zero_lift_angle_of_attack);
	double alpha_tail = alpha - epsilon + input.elevator + (1.3 * state.q * length_tail) / Va;
	tail_lift_coefficient = 3.1 * (tail_platform_area / wing_platform_area) * alpha_tail;

	return wing_lift_coefficient + tail_lift_coefficient;
}

double Aerodynamics::calculateDragCoefficient(const FlightConditions& fc)
{
	double alpha = fc.alpha;
	double t = 5.5 * alpha + 0.654;
	return 0.13 + 0.07 * t * t;
}

double Aerodynamics::calculateSideForceCoefficient(const FlightConditions& fc, const ControlInputs& input)
{
	double beta = fc.beta;
	return -1.6 * beta + 0.24 * input.rudder;
}

AeroCoefficients Aerodynamics::computeAeroCoefficients(const FlightConditions& fc, 
													   const ControlInputs& input,
													   const AircraftState& state
)
{
	AeroCoefficients aero{};
	aero.CD = calculateDragCoefficient(fc);
	aero.CL = calculateLiftCoefficient(fc, input, state);
	aero.CY = calculateSideForceCoefficient(fc, input);

	double St = tail_platform_area;
	double lt = length_tail;
	double S = wing_platform_area;
	double epsilon = depsda * (fc.alpha - alpha_zero_lift_angle_of_attack);
	double dCMdx_normalization_scalar = cbar / max(fc.Va, 1e-6);

	Eigen::Vector3d eta;
	eta << -1.4 * fc.beta,
		   -0.59 - (3.1 * St * lt / (S * cbar)) * (fc.alpha - epsilon),
			(1 - fc.alpha * (180 / (15 * pi)) * fc.beta);

	//RCAM aerodynamic derivatives (NASA RCAM model)
	Eigen::Matrix3d dCMdx_matrix;
	dCMdx_matrix << -11, 0, 5,
				     0, -4.03 * (St * lt * lt) / (S * cbar * cbar), 0,
				     1.7, 0, -11.5;

	Eigen::Matrix3d dCMdx;
	dCMdx = dCMdx_normalization_scalar * dCMdx_matrix;

	// Angular rates vector [roll, pitch, yaw] expressed in the body frame
	Eigen::Vector3d rates; 
	rates << state.p,
			 state.q,
			 state.r;

	// RCAM aerodynamic derivatives (NASA RCAM model)
	Eigen::Matrix3d dCMdu;
	dCMdu << -0.6, 0, 0.22,
			  0, -3.1 * St * lt / (S * cbar), 0,
			  0, 0, -0.63;

	// Control surface deflections [aileron, elevator, rudder]
	Eigen::Vector3d Inputs;
	Inputs << input.aileron,
			  input.elevator,
			  input.rudder;

	Eigen::Vector3d CM_ac_body;
	CM_ac_body = eta + dCMdx * rates + dCMdu * Inputs;

	aero.Cl = CM_ac_body(0);
	aero.Cm = CM_ac_body(1);
	aero.Cn = CM_ac_body(2);

	return aero;
}

Eigen::Vector3d Aerodynamics::computeAerodynamicForce(const FlightConditions& fc, const AeroCoefficients& coeff)
{	
	double S = wing_platform_area;
	Eigen::Vector3d Fa_stability;

	Fa_stability << -coeff.CD * fc.qbar * S,
					 coeff.CY * fc.qbar * S,
					 coeff.CL * fc.qbar * S;

	// Rotation from the stabilty fram to the body frame
	Eigen::Matrix3d Cbs;
	Cbs << std::cos(fc.alpha), 0, -std::sin(fc.alpha),
		         0,			   1,        0,
		   std::sin(fc.alpha), 0,  std::cos(fc.alpha);

	Eigen::Vector3d Fa_body = Cbs * Fa_stability; // Calculates aerodynamic force in body frame

	return Fa_body;
}

Eigen::Vector3d Aerodynamics::computeAerodynamicMoments(const FlightConditions& fc,  
														const AeroCoefficients& coeff,
	                                                    const Eigen::Vector3d& forceBody)
{
	
	Eigen::Vector3d CM_ac_body;
	CM_ac_body << coeff.Cl,
				  coeff.Cm,
		          coeff.Cn;

	double S = wing_platform_area;
	
	// Calculates the body moment about the aerodynamic center
	Eigen::Vector3d M_ac_body = CM_ac_body * fc.qbar * S * cbar;
	
	// Applies a moment transfer to shift body moment to the center of gravity
	Eigen::Vector3d M_cg_body =  M_ac_body + (r_ac - r_cg).cross(forceBody);
	
	return M_cg_body;
}

AerodynamicLoads Aerodynamics::computeAerodynamicLoads(const FlightConditions& fc,
													   const ControlInputs& input, 
													   const AircraftState& state
)
{
	AerodynamicLoads loads{};
	AeroCoefficients coeff = computeAeroCoefficients(fc, input, state);
	loads.force_body = computeAerodynamicForce(fc, coeff);
	loads.moment_body = computeAerodynamicMoments(fc, coeff, loads.force_body);

	return loads; // Returns the body aerodynamic moments and forces all at once
}