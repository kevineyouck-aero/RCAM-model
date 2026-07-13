# RCAM 6-DOF Flight Dynamics Simulator 

![Language](https://img.shields.io/badge/C%2B%2B-17-blue)

![Status](https://img.shields.io/badge/status-under%20development-orange)

![Linear Algebra](https://img.shields.io/badge/Eigen-3-success)

## Overview
The main objective of this project is to implement a nonlinear 6-DOF Research Civil Aircraft Model (RCAM) within a C++ framework to reproduce as faithfully as possible the aircraft dynamics to support future control laws, trimming, linearization, and autopilot design. This project is intended to strengthen practical experience in aircraft flight dynamics, stability and control and aerospace software development.

## Mathematical Model

The simulator implements a nonlinear 6-DOF equations of motion using the Newton-Euler formulation.

### State vector
x = [u v w p q r φ θ ψ pN pE pD]ᵀ

### Control Inputs
u = [δa δe δr δT1 δT2]ᵀ

The simulator numerically evaluates 

ẋ = f(x,u)

at every simulation step


## Features
### Implemented
- [x] International standard atmosphere
- [x] Aircraft geometry model
- [x] Nonlinear aerodynamic model
- [x] Propulsion model
- [x] Gravity model
- [x] 6-DOF Newton-Euler equations
- [x] Flight condition computation
- [x] Modular object-oriented architecture
- [x] Eigen-based linear algebra

### Planned
- [ ] RK4 integrator
- [ ] Complete simulation loop
- [ ] Aircraft trimming
- [ ] Linearization
- [ ] Flight control laws 
- [ ] Autopilot
	
## Software Architecture

```text
                         AircraftState
                               │
                               ▼
                     FlightDynamics
            ┌──────────┼───────────┐
            │          │           │
            ▼          ▼           ▼
      Aerodynamics  Propulsion  Gravity
            │
            ▼
       Atmosphere
            │
            ▼
    AircraftGeometry
```



##  Project Structure
```text
includes/
    Header files defining all class interfaces

sources/
    Source files implementing the project

tests/
    Unit and validation test programs

README.md
```

## Validation
The current implementation has been validated by reproducing the trim condition and comparing the computed flight conditions and state derivatives with the original MATLAB implementation of the RCAM model.
Validation includes:

- Flight condition computation

- Aerodynamic force and moment generation

- Gravity model

- Propulsion model

- 6-DOF Newton-Euler equations

- Trim condition verification

## Build 
- C++ Visual Studio 2026
- Eigen 3
- C++17	

Build instructions will be added as the project progresses.

## Roadmap
- [x] Atmosphere
- [x] Aircraft Geometry
- [x] Aerodynamics
- [x] Propulsion
- [x] Gravity
- [x] Flight Dynamics

- [ ] RK4 Integrator
- [ ] Simulation Loop
- [ ] Linearization
- [ ] Flight Control Laws

## References:
	
	1. **Research Civil Aircraft Model (RCAM)** 
	https://garteur.org/wp-content/reports/FM/FM_AG-08_TP-088-3.pdf

	2. **Flight Dynamics and Control - Dr Christopher Lum (University of Washington)**
	 https://www.youtube.com/playlist?list=PLxdnSsBqCrrFlrvFM7YV7PTb7TUEYf3_0

	3. Stevens, B. L., Lewis, F. L., and Johnson, E. N. Aircraft Control and Simulation 3rd Edition Wiley






















