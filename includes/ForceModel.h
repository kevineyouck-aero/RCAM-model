#include "RCAM_model.h"
#include <Eigen/Dense>


struct ForcesMoments {
	Eigen::Vector3d force_body;
	Eigen::Vector3d moment_body;
};

class ForceModel
{
	virtual ~ForceModel() = default;
	virtual ForcesMoments compute(const AircraftState& state, const ControlInputs& input) const = 0;
};
