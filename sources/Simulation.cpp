#include "../includes/Simulation.h"
#include "../includes/CSVLogger.h"


Simulation::Simulation(const FlightDynamics& dynamics, StateVector state, const RK4Integrator& rk4):
	dynamics_(dynamics),
	state_(state),
	rk4_(rk4)
{
}

void Simulation::run(CSVLogger& logger,
				     const ControlInputs& input, 
					 double simTime, 
					 double dt,
					 double loggingPeriod)
{
	const int numberOfSteps = static_cast<int>(simTime / dt);
	const int loggingInterval = static_cast<int>(loggingPeriod / dt);
	logger.writeHeader();
	logger.write(state_, 0.0);
	for (int i = 0; i < numberOfSteps; i++) {
		state_ = rk4_.step(dynamics_,
						   state_,
						   input,
						   dt);
		if ((i + 1) % loggingInterval == 0) {
			logger.write(state_, (i + 1) * dt);
		}
	}
		
}