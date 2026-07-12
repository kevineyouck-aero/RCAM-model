#pragma once
#include "RCAM_model.h"
#include "Atmosphere.h"
#include "AircraftGeometry.h"
#include <Eigen/Dense>

/**	
	* @struct FlightConditions
	* 
	* Stores the current aircraft flight conditions.
**/
struct FlightConditions
{
	double Va;
	double alpha;
	double beta;
	double qbar;

};

/**
	* @struct AeroCoefficients
	* 
	* Stores the aerodynamic force and moment coefficients of the RCAM model
	* 
	* CL, CY, CD aerodynamic force coefficients
	* Cl, Cm, Cn aerodynamic moment coefficients
**/
struct AeroCoefficients
{
	double CL;
	double CY;
	double CD;

	double Cl = 0.0;
	double Cm = 0.0;
	double Cn = 0.0;
};

/**
	* struct AerodynamicLoads
	* 
	* Contains a 3D-vector for both aerodynamic	forces and moments expressed in the body frame.
**/
struct AerodynamicLoads
{
	Eigen::Vector3d force_body;
	Eigen::Vector3d moment_body;	
};

/**
	* @class Aerodynamics
	* 
	* Aerodynamics is a class that defines the aircraft's aerodynamics
**/

class Aerodynamics
{
public:
	explicit Aerodynamics(const Atmosphere& atmosphere, const AircraftGeometry& geometry);

	/**
		* Calculates the aircraft true airspeed
		* 
		* @param Current aircraft state
		* @return True airspeed (m/s)
	**/
	double calculateAirspeed(const AircraftState& state) const;

	/**
		* Calculates the aircraft's angle of attack
		* 
		* @param Current aircraft state
		* @return Aircraft angle of attack (rad)
	**/
	double calculateAlpha(const AircraftState& state) const;

	/**
		* Calculates the aircraft's sideslip angle 
		* 
		* @param Current aircraft state
		* @return Aircraft sideslip angle (rad)
	**/
	double calculateBeta(const AircraftState& state) const;

	/**
		* Computes the aircraft flight conditions
		* 
		* @param Current aircraft state
		* @return Computed flight conditions
	**/
	FlightConditions computeFlightConditions(const AircraftState& state) const;

	/**
		* Calculates the dimensionless aircraft lift coefficient
		* 
		* @param fc Current aircraft flight conditions
		* @param input Control surface deflections
		* @param Current aircraft state
		* @return Dimensionless lift coefficient
	**/
	double calculateLiftCoefficient(const FlightConditions& fc, 
									const ControlInputs& input, 
									const AircraftState& state) const;

	/**
		* Calculates the dimensionless aircraft drag coefficient
		* 
		* @param fc Current aircraft flight conditions
		* @return Dimensionless drag coefficient 
	**/
	double calculateDragCoefficient(const FlightConditions& fc) const;

	/**
		* Calculates the dimensionless aircraft sideforce coefficient
		* 
		* @param fc Current aircraft flight conditions
		* @param input Control surface deflections
		* @return Dimensionless side force coefficient
	**/
	double calculateSideForceCoefficient(const FlightConditions& fc, const ControlInputs& input) const;

	/**
		
		* Calculates all the aerodynamic force and moment coefficients
		* 
		* @param fc Current aircraft flight conditions
		* @param input Control surface deflections
		* @param Current aircraft state
		* @return Aerodynamic forces and moment coefficients 
	**/
	AeroCoefficients computeAeroCoefficients(const FlightConditions& fc, 
											 const ControlInputs& input, 
											 const AircraftState& state) const;

	/**
		* Computes the aerodynamic forces acting on the aircraft
		* 
		* @param fc Current aircraft flight conditions
		* @param coeff Aerodynamic coefficients 
		* @return Body-frame aerodynamic force

	**/
	Eigen::Vector3d computeAerodynamicForce(const FlightConditions& fc,	const AeroCoefficients& coeff) const;

	/**
		* Computes the aerodynamic moment acting on the aircraft
		* 
		* @param fc Current aircraft flight conditions
		* @param coeff Aerodynamic coefficients 	
		* @param forceBody 3D-vector expressed in the body frame.
		* @return Body-frame aerodynamic moment
	**/
	Eigen::Vector3d computeAerodynamicMoments(const FlightConditions& fc, 
											  const AeroCoefficients& coeff, 
										      const Eigen::Vector3d& forceBody) const;

	/**
		* Computes the aerodynamic loads acting on the aircraft
		* 
		* @param fc	Current flight conditions
		* @param input Control surface deflections
		* @param state Current aircraft state
		* @return Aerodynamic loads expressed in the body frame
	**/
	AerodynamicLoads computeAerodynamicLoads(const FlightConditions& fc, 
											 const ControlInputs& input, 
											 const AircraftState& state											 
										     ) const;
private:
	const AircraftGeometry& geometry_;
	const Atmosphere& atmosphere_;
	double pi = std::numbers::pi;

	const double alpha_switch = 14.5 * pi / 180;
	const double alpha_zero_lift_angle_of_attack = -11.5 * pi / 180;

	static constexpr double length_tail = 24.8;
	static constexpr double wing_platform_area = 260;
	static constexpr double tail_platform_area = 64;
	
	
	static constexpr double depsda = 0.25;
	static constexpr double n = 5.5;
	static constexpr double a3 = -768.5;
	static constexpr double a2 = 609.2;
	static constexpr double a1 = -155.2;
	static constexpr double a0 = 15.212;
};