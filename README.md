# Nonlinear and Linearized Quadrotor Simulation in MATLAB

## Overview

This project focuses on the mathematical modeling, simulation, stability analysis, and control system design of a quadrotor UAV using MATLAB and Control System Toolbox.

The main objective is to develop a nonlinear quadrotor model, obtain a linearized state-space representation around an operating point, compare the nonlinear and linearized responses, and investigate several control strategies, including state-feedback control, Linear Quadratic Regulator (LQR), state observation, and integral control.

The project provides a structured workflow for understanding how a quadrotor model evolves from nonlinear equations of motion into a linear state-space system that can be analyzed and controlled using classical control theory.

## Project Objectives

- Develop and simulate a 12-state nonlinear quadrotor dynamic model.
- Define physical parameters, input constraints, state constraints, and operating points.
- Linearize the nonlinear dynamics around the hover equilibrium using numerical Jacobians.
- Compare nonlinear and linearized system responses under different input perturbations.
- Analyze open-loop stability using eigenvalues and time constants.
- Evaluate controllability and observability using their corresponding matrices.
- Design state-feedback controllers using pole placement and LQR.
- Simulate closed-loop responses and evaluate tracking performance.
- Design a Luenberger observer to estimate system states.
- Investigate integral control and combine observer-based feedback with integral action.

## System Model

The quadrotor is represented by a 12-dimensional state vector:

\[
x = [x,y,z,\phi,\theta,\psi,v_x,v_y,v_z,p,q,r]^T
\]

where the states describe position, Euler angles, translational velocities, and angular rates.

The control input is represented by four variables:

\[
u = [u_1,u_2,u_3,u_4]^T
\]

These inputs represent collective thrust and the three rotational control moments under the conventions used in the model.

The nonlinear equations account for the coupling between attitude, thrust, translational acceleration, and rotational dynamics. The model parameters are defined in the MATLAB source code, including mass, gravitational acceleration, arm length, and moments of inertia.

The linearized model is obtained around a selected operating point using the state-space approximation:

\[
\Delta\dot{x}=A\Delta x+B\Delta u
\]

where \(A\) and \(B\) are the state and input Jacobian matrices, respectively.

## Repository Structure

The project is organized into separate MATLAB files, with each file responsible for a particular part of the modeling and control workflow.

| File | Purpose |
|---|---|
| `main.m` | Main entry point. Initializes the model, computes the linearized matrices, and calls the analysis and controller-design functions. |
| `system_model.m` | Defines the quadrotor parameters, nonlinear equations of motion, state and input constraints, operating points, and initial simulations. |
| `system_jacobian.m` | Computes the numerical Jacobian matrices \(A\) and \(B\) around the selected operating point using central finite differences. |
| `linearization_check.m` | Compares nonlinear and linearized simulations under several thrust perturbations and calculates state-wise RMS errors. |
| `task6_7_stability_controllability.m` | Performs open-loop eigenvalue analysis and evaluates controllability and observability. |
| `controller_design.m` | Implements pole-placement state feedback and LQR, simulates a closed-loop response, and calculates performance metrics. |
| `observer_design.m` | Designs a Luenberger observer and compares actual states, estimated states, and estimation errors. |
| `combined_simulation.m` | Combines state feedback, observer-based estimation, and integral control in a closed-loop simulation. |

### 1. `main.m` — Main Execution Script

This script coordinates the main project workflow. It initializes the nonlinear model, calculates the linearized matrices at the hover operating point, and passes these matrices to the subsequent analysis, controller-design, observer-design, and combined-simulation routines.

### 2. `system_model.m` — Nonlinear Modeling and Operating Points

This file contains the physical parameters and nonlinear equations of motion. It also defines the system constraints and two operating points: hover and a vertical-motion condition.

The initial simulation illustrates selected state responses, including altitude and attitude angles.

### 3. `system_jacobian.m` — Numerical Linearization

This function approximates the Jacobian matrices using central finite differences. It can calculate the linearized model around either the hover or the climb operating point.

The resulting matrices are used by the remaining scripts for stability analysis and controller design.

### 4. `linearization_check.m` — Nonlinear vs. Linearized Simulation

This script simulates the nonlinear model and its linear approximation under different collective-thrust perturbations.

It compares selected states, including altitude, roll angle, and translational velocities, and calculates root-mean-square errors to help assess the accuracy of the linear approximation.

### 5. `task6_7_stability_controllability.m` — System Analysis

This script examines the open-loop eigenvalues and time constants of the linearized system. It also constructs the controllability and observability matrices and checks their ranks.

These analyses provide a foundation for determining whether the modeled states can be controlled or reconstructed under the selected system representation.

### 6. `controller_design.m` — State-Feedback Control

Two state-feedback design approaches are implemented:

- **Pole placement:** Selects desired closed-loop poles based on specified damping and settling-time parameters.
- **Linear Quadratic Regulator (LQR):** Computes an optimal feedback gain using selected state and input weighting matrices.

The script compares the resulting closed-loop poles and simulates a reference response, reporting rise time, settling time, overshoot, and steady-state error.

### 7. `observer_design.m` — State Estimation

This script designs a Luenberger observer by assigning observer poles and calculating the observer gain matrix.

The simulation begins with different initial conditions for the actual system and observer, allowing the convergence of estimated states and the corresponding estimation errors to be examined.

### 8. `combined_simulation.m` — Observer-Based and Integral Control

This script brings together state-feedback control, observer-based state estimation, and integral action.

It first designs a state-feedback controller and observer, then augments the system with an integral state associated with altitude. Finally, it simulates the combined closed-loop dynamics and plots altitude, estimated altitude, integral state, and control input.

## How to Run

1. Install MATLAB with the **Control System Toolbox**.
2. Download or clone this repository.
3. Place all project `.m` files in the same directory.
4. Set that directory as the MATLAB Current Folder.
5. Run the main script:

```matlab
main
```

The script calls the modeling, linearization, stability-analysis, controller-design, observer-design, and combined-simulation functions in sequence.

The simulations generate plots and display numerical results in the MATLAB Command Window. Some routines may also be run independently for more focused analysis.

## Main Outcomes

The project establishes a workflow for:

- Constructing a nonlinear quadrotor model from its equations of motion.
- Obtaining a local linear approximation around an operating point.
- Investigating the limitations of linearization by comparing simulated responses.
- Analyzing system stability and the controllability and observability properties of the linearized model.
- Comparing pole-placement and LQR feedback design approaches.
- Evaluating observer convergence and estimation error.
- Exploring the integration of state estimation, feedback control, and integral action.

The numerical performance of each method depends on the selected parameters, operating point, controller gains, and simulation conditions. The project code generates the relevant plots and metrics; quantitative conclusions should be based on the actual MATLAB outputs.

## Limitations and Future Improvements

Potential extensions include:

- Validating the model against a more detailed physical quadrotor model or experimental data.
- Incorporating actuator saturation, motor dynamics, and external disturbances.
- Improving reference tracking and steady-state error handling.
- Comparing controller performance under identical simulation conditions.
- Adding automated tests and reproducible numerical results.
- Extending the simulation to trajectory tracking and three-dimensional flight paths.

## Technologies

- MATLAB
- Control System Toolbox
- Nonlinear ordinary differential equation simulation
- State-space modeling and numerical linearization
- Classical feedback control
- Pole placement and LQR
- Luenberger state observers

## Author

**Amirali Bagheral**

This project was developed as an academic study of nonlinear system modeling, linearization, and control design for quadrotor dynamics.
