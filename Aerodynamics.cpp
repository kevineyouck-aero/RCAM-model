#include "Aerodynamics.h"
#include <cmath>
#include <iostream>
#include <algorithm>

using namespace std;

Aerodynamics::Aerodynamics(const Atmosphere& atmosphere):atmosphere_(atmosphere)
{
}

double Aerodynamics::calculateAirspeed(const AircraftState& state)
{
	return sqrt(pow(state.u, 2) + pow(state.v, 2) + pow(state.w, 2));
}

double Aerodynamics::calculateAlpha(const AircraftState& state)
{
	return atan2(state.w, state.u);
}

double Aerodynamics::calculateBeta(const AircraftState& state)
{
	double Va = this->calculateAirspeed(state);
	double ratio = state.v / Va;
	ratio = clamp(ratio, -1.0, 1.0);	
	if (Va < 1e-06)
		return 0.0;
	return asin(ratio);

}

double Aerodynamics::calculateDynamicPressure(const AircraftState& state)
{
	double airspeed = calculateAirspeed(state);
	return 0.5 * atmosphere_.getRho() * pow(airspeed, 2);
}

FlightConditions Aerodynamics::computeFlightCondtions(const AircraftState& state)
{
	FlightConditions fc{};
	fc.Va = this->calculateAirspeed(state);
	fc.alpha = this->calculateAlpha(state);
	fc.beta = this->calculateBeta(state);
	fc.qbar = this->calculateDynamicPressure(state);
	return fc;
}

double Aerodynamics::calculateLiftCoefficient(const FlightConditions& fc,
											  const ControlInputs& input, 
											  const AircraftState& state
)
{
	double Va = max(fc.Va, 1e-6);
	double alpha = fc.alpha;
	double wing_lift_coefficient = 0.0;
	double alpha2 = alpha * alpha;
	double alpha3 = alpha2 * alpha;

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
	return 0.13 + 0.07*pow((5.5*alpha+0.654),2);
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
	aero.CD = this->calculateDragCoefficient(fc);
	aero.CL = this->calculateLiftCoefficient(fc, input, state);
	aero.CY = this->calculateSideForceCoefficient(fc, input);
	return aero;
}



