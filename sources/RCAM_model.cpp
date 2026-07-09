#include "RCAM_model.h"
#include <iostream>
#include <numbers>
#include <utility>

using namespace std;

RCAM_model::RCAM_model(const AircraftState& state, const ControlInputs& input):
	state_(state),
	input_(input)
{
}

double RCAM_model::getAircraftMass() const
{
	return mass_;
}




