#include "../includes/RK4Integrator.h"


AircraftState RK4Integrator::toAircraftState(const StateVector& vector) const
{
    AircraftState aircraftStateVector{};
    aircraftStateVector.u = vector(0);
    aircraftStateVector.v = vector(1);
    aircraftStateVector.w = vector(2);

    aircraftStateVector.p = vector(3);
    aircraftStateVector.q = vector(4);
    aircraftStateVector.r = vector(5);

    aircraftStateVector.phi = vector(6);
    aircraftStateVector.theta = vector(7);
    aircraftStateVector.psi = vector(8);

    aircraftStateVector.pn = vector(9);
    aircraftStateVector.pe = vector(10);
    aircraftStateVector.pd = vector(11);

    return aircraftStateVector;
}

StateVector RK4Integrator::toStateVector(const AircraftState& aircraft) const
{
    StateVector aircraftStateVector;
    aircraftStateVector << aircraft.u,
                           aircraft.v,
                           aircraft.w,
                           aircraft.p,
                           aircraft.q,
                           aircraft.r,
                           aircraft.phi,
                           aircraft.theta,
                           aircraft.psi,
                           aircraft.pn,
                           aircraft.pe,
                           aircraft.pd;

    return aircraftStateVector;
}

StateVector RK4Integrator::step(const FlightDynamics& dynamics,
                                const StateVector& state,
                                const ControlInputs& input,
                                double dt) const
{

    const StateVector k1 = dynamics.computeStateDerivatives(toAircraftState(state), input);

    const StateVector estimatedState2 = state + 0.5 * dt * k1;
    const StateVector k2 = dynamics.computeStateDerivatives(toAircraftState(estimatedState2),input);

    const StateVector estimatedState3 = state + 0.5 * dt * k2;
    const StateVector k3 = dynamics.computeStateDerivatives(toAircraftState(estimatedState3), input);

    const StateVector estimatedState4 = state + dt * k3;
    const StateVector k4 = dynamics.computeStateDerivatives(toAircraftState(estimatedState4), input);

    return  state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}
