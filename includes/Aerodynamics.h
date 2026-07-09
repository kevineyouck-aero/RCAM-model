#pragma once
#include "RCAM_model.h"
#include "Atmosphere.h"
#include "AircraftGeometry.h"
#include <Eigen/Dense>


struct FlightConditions
{
	double Va;
	double alpha;
	double beta;
	double qbar;

};

struct AeroCoefficients
{
	double CL;
	double CY;
	double CD;

	double Cl = 0.0;
	double Cm = 0.0;
	double Cn = 0.0;
};

struct AerodynamicLoads
{
	Eigen::Vector3d force_body;
	Eigen::Vector3d moment_body;	
};

class Aerodynamics
{
public:
	Aerodynamics() = default;
	explicit Aerodynamics(const Atmosphere& atmosphere, const AircraftGeometry& geometry);

	double calculateAirspeed(const AircraftState& state);
	double calculateAlpha(const AircraftState& state);
	double calculateBeta(const AircraftState& state);
	FlightConditions computeFlightConditions(const AircraftState& state);

	double calculateLiftCoefficient(const FlightConditions& fc, 
									const ControlInputs& input, 
									const AircraftState& state);

	double calculateDragCoefficient(const FlightConditions& fc);
	double calculateSideForceCoefficient(const FlightConditions& fc, const ControlInputs& input);

	AeroCoefficients computeAeroCoefficients(const FlightConditions& fc, 
											 const ControlInputs& input, 
											 const AircraftState& state);

	Eigen::Vector3d computeAerodynamicForce(const FlightConditions& fc,	const AeroCoefficients& coeff);

	Eigen::Vector3d computeAerodynamicMoments(const FlightConditions& fc, 
											  const AeroCoefficients& coeff, 
										      const Eigen::Vector3d& forceBody);

	AerodynamicLoads computeAerodynamicLoads(const FlightConditions& fc, 
											 const ControlInputs& input, 
											 const AircraftState& state											 
										     );
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

