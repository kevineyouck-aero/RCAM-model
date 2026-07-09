#pragma once
#include <Eigen/Dense>
#include "Atmosphere.h"
#include "Aerodynamics.h"
#include "RCAM_model.h"


struct PropulsionLoads
{
	Eigen::Vector3d force_body;
	Eigen::Vector3d moment_body;
};

class Propulsion
{
public:
	Propulsion() = default;
	Propulsion(const Atmosphere& atmosphere, const RCAM_model& rcam, const Aerodynamics& aero);

	PropulsionLoads computePropulsionLoads(const ControlInputs& input) const;


private:
	const Atmosphere& atmosphere_;
	const RCAM_model& rcam_;
	const Aerodynamics& aero_;
	static constexpr double x_apt1 = 0;
	static constexpr double y_apt1 = -7.94;
	static constexpr double z_apt1 = -1.9;

	static constexpr double x_apt2 = 0;
	static constexpr double y_apt2 = 7.94;
	static constexpr double z_apt2 = -1.9;

	const Eigen::Vector3d mu1 = {};

};

