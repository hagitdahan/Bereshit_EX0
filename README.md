# Bereshit_101 – Lunar Landing Simulation

This project simulates the autonomous descent of the Bereshit spacecraft onto the Moon using Java and PID controllers. The simulation models realistic spacecraft behavior including gravity, mass loss, thrust, and orientation.

## Mission Overview
Bereshit was Israel’s first lunar lander, launched in 2019. The mission ended in a crash due to a series of navigation and communication failures. This project recreates a soft landing scenario with a control system that avoids such issues.

## Features
- Realistic physics modeling of thrust, gravity, and mass
- PID control of vertical speed, horizontal speed, and angle
- Piecewise-linear interpolation of desired descent profiles
- Data logging and analysis using Python

## Files
- `Bereshit_101.java` – Main simulator logic
- `PID.java` – Control system
- `TrajectoryInterpolator.java` – Target profile interpolator
- `LandingStates.java` – Landing phase manager
- `Moon.java` – Lunar physics
- `bereshit_log.csv` – Output telemetry
- `plot_bereshit_graphs.py` – Graph generation script
- `bereshitReport.docx` – detailed report containing explanation of the simulation, PID system, interpolation, graphs, and technical background of the Bereshit crash

## How to Run
1. Compile and run `Bereshit_101.java`
2. Open the resulting `bereshit_log.csv` with `plot_bereshit_graphs.py` to visualize:
    - Vertical speed vs desired
    - Thrust over time (NN)
    - Altitude and horizontal speed

## Goal
Land successfully with vertical and horizontal speeds below 2.5 m/s and minimal fuel usage.
