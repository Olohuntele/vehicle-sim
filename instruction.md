# Repository Instructions: Vehicle Simulation (AEB)

## Project Overview
C++ Autonomous Emergency Braking (AEB) simulation using Raylib and CMake.

## Code Standards & Guidelines
- Language Standard: C++17
- Framework: Raylib (Keep rendering logic strictly separated from core physics/logic)
- Architecture: Decouple physical dynamic models (e.g., dynamic friction, aerodynamic drag, TTC calculations) from drawing loops.
- Build System: CMake (Do not alter structure unless requested).

## Target Execution Environment
- Headless GitHub Codespaces utilizing Xvfb / noVNC on DISPLAY :99.
