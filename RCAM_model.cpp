#include "RCAM_model.h"
#include <iostream>
#include <numbers>
#include <utility>

using namespace std;

RCAM_model::RCAM_model(AircraftState state, ControlInputs input):
	state_(state), input_(input)
{
}




