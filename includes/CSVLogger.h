#pragma once
#include "FlightDynamics.h"
#include <fstream>


class CSVLogger
{
public:
	CSVLogger(const std::string& filename);
	void writeHeader();
	void write(const StateVector& vector, double time);

private:
	std::ofstream file_;
};

