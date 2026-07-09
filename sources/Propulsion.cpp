#include "Propulsion.h"

Propulsion::Propulsion(const Atmosphere& atmosphere, const RCAM_model& rcam, const AircraftGeometry& geometry):
	atmosphere_(atmosphere),
	rcam_(rcam),
	geometry_(geometry)
{
}

PropulsionLoads Propulsion::computePropulsionLoads(const ControlInputs& input) const
{
	PropulsionLoads propLoads{};
	double u4 = input.throttle1;
	double u5 = input.throttle2;


	double m = rcam_.getAircraftMass();
	double g = atmosphere_.getGravity();

	Eigen::Vector3d	engine1ThrustBody = { u4 * m * g, 0, 0 };
	Eigen::Vector3d	engine2ThrustBody = { u5 * m * g, 0, 0 };

	Eigen::Vector3d r_cg = geometry_.getCenterOfGravity();

	// RCAM convention: mu is defined according to the reference video/model.
	// Do not replace by (engine_position - cg_position) unless the moment equation is also changed.
	Eigen::Vector3d mu1 = { r_cg(0) - x_apt1, 
							y_apt1  - r_cg(1),
							r_cg(2) - z_apt1 };

	Eigen::Vector3d mu2 = { r_cg(0) - x_apt2, 
							y_apt2 - r_cg(1), 
							r_cg(2) - z_apt2 };

	propLoads.force_body = engine1ThrustBody + engine2ThrustBody;

	Eigen::Vector3d momentEngine1_cg_body = mu1.cross(engine1ThrustBody);
	Eigen::Vector3d momentEngine2_cg_body = mu2.cross(engine2ThrustBody);

	propLoads.moment_body = momentEngine1_cg_body + momentEngine2_cg_body;

	return propLoads;
}
