#include "../includes/Gravity.h"
#include <cmath>

Gravity::Gravity(const Atmosphere& atmosphere, const RCAM_model& rcam):
	atmosphere_(atmosphere),
	rcam_(rcam)
{
}

Eigen::Vector3d Gravity::gravityForceBody(const AircraftState& aircraft) const
{
	double m = rcam_.getAircraftMass();
	double g = atmosphere_.getGravity();
	Eigen::Vector3d gravity_body = { -std::sin(aircraft.theta), 
									  std::sin(aircraft.phi) * std::cos(aircraft.theta),
									  std::cos(aircraft.phi) * std::cos(aircraft.theta)
									};
	Eigen::Vector3d gravity_force_body = m * g * gravity_body;
	return gravity_force_body;
}

GravityLoads Gravity::computeGravityLoads(const AircraftState& aircraft) const
{
	GravityLoads gLoads{};
	gLoads.force_body = gravityForceBody(aircraft);
	gLoads.moment_body.setZero();
	return gLoads;
}
