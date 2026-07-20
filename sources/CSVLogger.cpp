#include "../includes/CSVLogger.h"

CSVLogger::CSVLogger(const std::string& filename):
	file_(filename, std::ios::out | std::ios::trunc)
{
	if (!file_) {
		throw std::runtime_error(
			"Unable to open CSV file"
		);
	}
}

void CSVLogger::writeHeader()
{
	file_ << "time,u,v,w,p,q,r,phi,theta,psi,pn,pe,pd\n";
}

void CSVLogger::write(const StateVector& vector, double time)
{
	file_ << time;
	for (Eigen::Index i = 0; i < vector.size(); i++)	
		file_ << "," << vector(i);
	file_ << "\n";
}
