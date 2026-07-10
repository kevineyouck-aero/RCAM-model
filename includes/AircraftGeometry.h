#include <Eigen/Dense>
class AircraftGeometry
{
public:
	AircraftGeometry() = default;
	Eigen::Vector3d getCenterOfGravity() const { return r_cg; }
	Eigen::Vector3d getAerodynamicCenter() const { return r_ac; }
	double getCbar() const { return cbar; }

private:

	static constexpr double cbar = 6.6;
	static constexpr double x_cg = 0.23 * cbar;
	static constexpr double y_cg = 0;
	static constexpr double z_cg = 0.1 * cbar;

	static constexpr double x_ac = 0.12 * cbar;
	static constexpr double y_ac = 0;
	static constexpr double z_ac = 0;


	const Eigen::Vector3d r_cg = { x_cg,y_cg,z_cg };
	const Eigen::Vector3d r_ac = { x_ac,y_ac,z_ac };

};
