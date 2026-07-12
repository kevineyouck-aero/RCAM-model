#include "../includes/RCAM_model.h"
#include <iostream>
#include <numbers>
#include <utility>

RCAM_model::RCAM_model(const AircraftState& state, const ControlInputs& input):
	state_(state),
	input_(input)
{
	inertiaMatrix_ << 40.07, 0.0, 2.098,
						0.0, 64.0, 0.0,
					  2.098, 0.0, 99.92;
	invInertiaMatrix_ << 0.0249836, 0.0, 0.000523151,
							0.0, 0.015625, 0.0,
						 0.000523151, 0.0, 0.010019;
}

double RCAM_model::getAircraftMass() const
{
	return mass_;
}

Eigen::Matrix3d RCAM_model::getBodyInertiaTensorMatrix() const
{
	return mass_ * inertiaMatrix_;
}

Eigen::Matrix3d RCAM_model::getInvBodyInertiaTensorMatrix() const
{
	return (1.0 / mass_) * invInertiaMatrix_;
}




