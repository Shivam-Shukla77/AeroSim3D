# AeroSim 3D - High-Fidelity Aerospace Flight Simulation Engine

AeroSim 3D is a sophisticated 6DOF (Six Degrees of Freedom) aerospace simulation environment engineered by **Shivam Shukla**. The project demonstrates the integration of low-level systems programming, classical mechanics, and autonomous control theory within a custom C++ framework.

## Project Motivation & Learning Objectives

AeroSim 3D was developed as a personal engineering challenge to bridge the gap between software development and complex physical systems. My primary objective was to push beyond standard coursework and build a high-performance application from scratch to deepen my understanding of systems-level programming. 

Through architecting this engine, I specifically targeted and enhanced my skills in:
* **Advanced C++ & Memory:** Managing complex application states and optimizing a real-time simulation loop.
* **Applied Mathematics:** Translating theoretical physics (RK4 numerical integration, kinematics) into deterministic, functioning code.
* **Build Systems & Tooling:** Mastering the GCC/MinGW compiler pipeline, managing dependencies, and resolving complex dynamic/static linking architectures.
* **Network Concurrency:** Implementing asynchronous HTTP networking (libcurl) without blocking the main rendering thread.

## Core Engineering Specifications

- **Numerical Integration:** Employs a deterministic 4th-order Runge-Kutta (RK4) solver to ensure high-precision state vectors and orbital stability.
- **Autonomous Flight Control (FCS):** Implements a PD/PID control architecture for three-axis stabilization and propulsive landing (Hoverslam) algorithms.
- **Dynamic Variable-Mass Systems:** Real-time simulation of propellant consumption, resulting in dynamic shifts in Center of Mass (CoM) and Moment of Inertia.
- **Atmospheric Physics:** Simulated aerodynamic drag, True Dynamic Pressure (Max-Q), and stochastic crosswinds utilizing procedural noise.
- **AI Mission Director:** Utilizes asynchronous HTTP networking to interface with a Large Language Model for real-time telemetry analysis and mission diagnostics.

## AI Analysis Requirements

To utilize the integrated AI Mission Director and telemetry analysis features, the host system must meet the following requirements:
* **Ollama Framework:** The Ollama runtime must be installed and active.
* **Model Requirement:** The **Gemma-2b** model must be pulled and available (`ollama run gemma:2b`) for the simulation to process asynchronous analysis requests.

## System Architecture

The engine is developed with a modular architecture:
- `main.cpp`: Core application loop, memory management, and state initialization.
- `physics.cpp`: Fundamental RK4 integrator, dynamic aerospace physics, and autonomous flight control logic (PID controllers & Hoverslam algorithm).
- `ui.cpp`: Hybrid 2D/3D telemetry dashboard featuring hardware-accelerated Attitude ViewCube.
- `ai_director.cpp`: Asynchronous network layer for LLM interfacing.

## Technical Stack

- **Primary Language:** C++ (MinGW-w64)
- **Graphics Pipeline:** Raylib 5.0
- **Networking Interface:** libcurl
- **Data Serialization:** nlohmann/json

## Known Technical Debt & Development Roadmap

AeroSim 3D is under active, iterative development. While the core RK4 physics engine and UI pipeline are stable, code profiling has identified two areas requiring mathematical and algorithmic optimization:

* **Terminal Descent (Hoverslam) Kinematics:** The autonomous landing algorithm occasionally over-corrects. Profiling reveals this is due to the kinematic predictor calculating maximum deceleration (`a_max`) using instantaneous mass. Because propellant mass depletes rapidly during the suicide burn, the engine thrust-to-weight ratio overperforms the initial prediction. **Roadmap:** Transition from a static kinematic predictor to a dynamic closed-loop controller that accounts for continuous mass depletion.
* **Main Thread Render Bottlenecks:** Under specific conditions, the simulation drops below 60 FPS. Code profiling identifies two primary bottlenecks on the main thread:
    1.  **Trajectory Prediction Overhead:** The simulation recalculates a 50-step RK4 future trajectory every single frame. 
    2.  **GPU Overdraw:** The exhaust particle system generates thousands of overlapping alpha-blended textures near the ground.
    **Roadmap:** Move the trajectory predictor to an asynchronous background thread (`std::async` or `std::thread`), and implement hardware instancing or alpha-culling for the exhaust particle system.

## Portability & Distribution

AeroSim 3D is distributed as a standalone, portable binary. All necessary runtime dependencies, including standard C++ libraries and external middleware (DLLs), are packaged within the primary distribution directory. 

1. Ensure `AeroSim3D.exe`, the associated `.dll` files, and the `resources/` directory are co-located.
2. Execute `AeroSim3D.exe` to initialize the simulation.

## Acknowledgements & Licenses

This project integrates several open-source libraries and Creative Commons assets:
* **nlohmann/json**: MIT License (Copyright © 2013-2022 Niels Lohmann)
* **libcurl**: curl License (Copyright © 1996-2024 Daniel Stenberg)
* **FastNoiseLite**: MIT License (Copyright © 2020 Jordan Peck)
* **Raylib**: zlib License (Copyright © 2013-2024 Ramon Santamaria)
* **3D Assets**: Sourced via Sketchfab under CC BY 4.0 (Attributions to A.O., Sander, and The Smithsonian Institution).
* **Seamless Rocket Booster Roar & Crackle**: Pixabay Content License (Free to use, modify, and distribute within compiled software).
* **Nasalization Regular**: Freeware / Typodermic Desktop License.

---
**Lead Developer:** Shivam Shukla
**Location:** Kanpur, Uttar Pradesh, India