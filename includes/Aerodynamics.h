#pragma once
#include "RCAM_model.h"
#include "Atmosphere.h"
#include "AircraftGeometry.h"
#include <Eigen/Dense>

/**	
	\struct FlightConditions
	FlightConditions is a structure design to address the aircraft parameters such as 
	the true airspeed, the angle of attack the angle of sideslip and the dynamic
	pressure.
**/
struct FlightConditions
{
	double Va;
	double alpha;
	double beta;
	double qbar;

};

/**
	\struct AeroCoeffiecient
	AeroCoeffiecient is a structure design to get all the coefficients specific the RCAM
	aircraft model. CL (Lift coefficient), CY (Sideforce coefficient) and CD (Drag 
	coefficient) define the force coefficients. Cl (rolling moment coefficient), Cm 
	(Pitching moment coefficient) and Cn (Yawing moment coefficient) define the moment
	coefficients.
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
	\struct AerodynamicLoads
	AerodynamicLoads is a structure design to define a 3D-vector for both aerodynamic
	forces and moments.
**/
struct AerodynamicLoads
{
	Eigen::Vector3d force_body;
	Eigen::Vector3d moment_body;	
};

/**
	\class Aerodynamics
	Aerodynamics is a class that defines the aircraft's aerodynamics
**/

class Aerodynamics
{
public:
	explicit Aerodynamics(const Atmosphere& atmosphere, const AircraftGeometry& geometry);

	/**
		\fn calculateAirspeed(const AircraftState& state)
		\brief calculates the aircraft true airspeed (m/s)
		\param state 
		given aircraft current state
	**/
	double calculateAirspeed(const AircraftState& state) const;

	/**
		\fn calculateAlpha(const AircraftState& state)
		\brief calculates the aircraft's angle of attack (rad)
		\param state
		given aircraft current state
	**/
	double calculateAlpha(const AircraftState& state) const;

	/**
		\fn calculateBeta(const AircraftState& state)
		\brief calculates the aircraft's sideslip angle (rad)
		\param state
		given aircraft current state
	**/
	double calculateBeta(const AircraftState& state) const;

	/**
		\fn computeFlightConditions(const AircraftState& state)
		\brief gets the aircraft's flight conditions at a given state in time 
		\param state
		given aircraft current state
	**/
	FlightConditions computeFlightConditions(const AircraftState& state) const;

	/**
		\fn calculateLiftCoefficient(const FlightConditions& fc, 
									const ControlInputs& input, 
									const AircraftState& state) const

		\brief calculates the dimensionless aircraft lift coefficient
		\param fc
		given given aircraft flight conditions
		\param input
		given control surface deflections 
		\param state
		given aircraft current state
	**/
	double calculateLiftCoefficient(const FlightConditions& fc, 
									const ControlInputs& input, 
									const AircraftState& state) const;

	/**
		\fn calculateDragCoefficient(const FlightConditions& fc) const
		\brief calculates the dimensionless aircraft drag coefficient
		\param fc
		given given aircraft flight conditions
	**/
	double calculateDragCoefficient(const FlightConditions& fc) const;

	/**
		\fn calculateSideForceCoefficient(const FlightConditions& fc, const ControlInputs& input) const
		\brief calculates the dimensionless aircraft sideforce coefficient
		\param fc
		given given aircraft flight conditions
		\param input
		given control surface deflections
	**/
	double calculateSideForceCoefficient(const FlightConditions& fc, const ControlInputs& input) const;

	/**
		\fn computeAeroCoefficients(const FlightConditions& fc, 
											 const ControlInputs& input, 
											 const AircraftState& state) const
		\brief calculates all the aerodynamic force and moment coefficients
		\param fc
		given given aircraft flight conditions
		\param input
		given control surface deflections
		\param state
		given aircraft current state
	**/
	AeroCoefficients computeAeroCoefficients(const FlightConditions& fc, 
											 const ControlInputs& input, 
											 const AircraftState& state) const;

	/**
		\fn computeAerodynamicForce(const FlightConditions& fc,	const AeroCoefficients& coeff) const
		\brief return a 3D-vector representing the aerodynamic forces in the body frame
		\param fc
		given given aircraft flight conditions
		\param coeff
		calculated aerodynamic coefficients by the given flight conditions, control surface deflections and aircraft's 
		current state in time.
	**/
	Eigen::Vector3d computeAerodynamicForce(const FlightConditions& fc,	const AeroCoefficients& coeff) const;

	/**
		\fn computeAerodynamicMoments(const FlightConditions& fc, 
											  const AeroCoefficients& coeff, 
										      const Eigen::Vector3d& forceBody) const
		\brief return a 3D-vector representing the aerodynamic moments in the body frame
		\param fc
		given given aircraft flight conditions
		\param coeff
		calculated aerodynamic coefficients by the given flight conditions, control surface deflections and aircraft's
		current state in time.	
		\param forceBody
		3D-vector expressed in the body frame.
	**/
	Eigen::Vector3d computeAerodynamicMoments(const FlightConditions& fc, 
											  const AeroCoefficients& coeff, 
										      const Eigen::Vector3d& forceBody) const;

	/**
		\fn computeAerodynamicLoads(const FlightConditions& fc, 
											 const ControlInputs& input, 
											 const AircraftState& state											 
										     ) const
		\brief returns loads for both aerodynamic force and moments
		\param fc
		given given aircraft flight conditions
		\param input
		given control surface deflections
		\param state
		given aircraft current state
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