#pragma once
#include <vector>
#include <numbers>
#include <Eigen/Dense>


struct AircraftState
{
	double u, v, w;				// Body axis velocities (m/s)
	double p, q, r;				// Body axis angular rate (rad/s)
	double psi, theta, phi;		// Euler angles (rad)
	double pn, pe, pd;			// NED position (m)

};

struct ControlInputs {
	double aileron;
	double elevator;
	double rudder;
	double throttle1;
	double throttle2;
};
class RCAM_model
{
public:
	RCAM_model(const AircraftState& state, const ControlInputs& inputs);
	double getAircraftMass() const;
	Eigen::Matrix3d getBodyInertiaTensorMatrix() const;
	Eigen::Matrix3d getInvBodyInertiaTensorMatrix() const;
	//void step(double dt, const vector<double>& controls);
	
	
private:
	AircraftState state_;
	ControlInputs input_;
	static constexpr double mass_ = 120000.0;		// Aircraft's nominal mass according to RCAM documentation (kg)	

	//void calculate_forces_and_moments(const AircraftState & s, double* forces, double* moments);
	double pi = std::numbers::pi;
	std::pair<double, double> limit_aileron_angle = { -25 * pi / 180.0, 25 * pi / 180.0 };
	std::pair<double, double> limit_elevation_angle = { -25 * pi / 180.0, 10 * pi / 180.0 };
	std::pair<double, double> limit_rudder_angle = { -30 * pi / 180.0, 30 * pi / 180.0 };
	std::pair<double, double> limit_throttle1 = { 0.5 * pi / 180.0, 10 * pi / 180.0 };
	std::pair<double, double> limit_throttle2 = { 0.5 * pi / 180.0, 10 * pi / 180.0 };

	Eigen::Matrix3d inertiaMatrix_;
	// Refer to RCAM documentation for this matrix

	Eigen::Matrix3d invInertiaMatrix_; 
	// Refer to Christopher Lum's RCAM design video about the hardcoded inverse Inertia tensor matrix
};

