#include "../includes/RK4Integrator.h"


AircraftState RK4Integrator::toAircraftState(const StateVector& vector) const
{
    AircraftState aircraft{};
    aircraft.u = vector(0);
    aircraft.v = vector(1);
    aircraft.w = vector(2);

    aircraft.p = vector(3);
    aircraft.q = vector(4);
    aircraft.r = vector(5);

    aircraft.phi = vector(6);
    aircraft.theta = vector(7);
    aircraft.psi = vector(8);

    aircraft.pn = vector(9);
    aircraft.pe = vector(10);
    aircraft.pd = vector(11);

    return aircraft;
}

StateVector RK4Integrator::toStateVector(const AircraftState& aircraft) const
{
    StateVector state;
    state << aircraft.u,
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
    return state;
}

StateVector RK4Integrator::step(const FlightDynamics& dynamics,
                                const StateVector& state,
                                const ControlInputs& input,
                                double dt) const
{

    const StateVector k1 = dynamics.computeStateDerivatives(toAircraftState(state), input);
    const StateVector k2 = dynamics.computeStateDerivatives(toAircraftState(state + 0.5 * dt * k1),input);
    const StateVector k3 = dynamics.computeStateDerivatives(toAircraftState(state + 0.5 * dt * k2), input);
    const StateVector k4 = dynamics.computeStateDerivatives(toAircraftState(state + dt * k3), input);

    return  state + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}
