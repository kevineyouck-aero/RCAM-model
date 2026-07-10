#pragma once
#include "Atmosphere.h"
#include "RCAM_model.h"
#include <Eigen/Dense>


struct GravityLoads
{
	Eigen::Vector3d force_body;
	Eigen::Vector3d moment_body;

};
class Gravity
{

public:
	Gravity(const Atmosphere& atmosphere, const RCAM_model& rcam);
	Eigen::Vector3d gravityForceBody(const AircraftState& aircraft) const;
	GravityLoads computeGravityLoads(const AircraftState& aircraft) const;

private:
	const Atmosphere& atmosphere_;
	const RCAM_model& rcam_;
};

