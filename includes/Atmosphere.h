#pragma once

class Atmosphere
{

public:
	Atmosphere() = default;
	double getRho() const;
	double getGravity() const;

private:
	static constexpr double rho_ = 1.225;
	static constexpr double g_ = 9.81;

};

