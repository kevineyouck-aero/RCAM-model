#include "../includes/RCAM_model.h"
#include <iostream>
#include <numbers>
#include <utility>

RCAM_model::RCAM_model(const AircraftState& state, const ControlInputs& input):
	state_(state),
	input_(input)
{
}

double RCAM_model::getAircraftMass() const
{
	return mass_;
}

Eigen::Matrix3d RCAM_model::getBodyInertiaTensorMatrix() const
{
	return mass_ * inertiaMatrix;
}

Eigen::Matrix3d RCAM_model::getInvBodyInertiaTensorMatrix() const
{
	return (1.0 / mass_) * invInertiaMatrix;
}




