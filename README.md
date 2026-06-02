# Dual-Rate Robotic Manipulator Simulator

A highly optimized, multi-process robotics simulation environment built in C++ and Python. This project serves as a testbed for comparing traditional, deterministic control schemes (like Computed Torque Control) against asynchronous, machine-learning-based controllers (Physics-Informed Neural Networks).

## Systems Architecture

To accurately simulate real-world robotic hardware limitations, this project abandons standard single-threaded simulation loops in favor of a **Distributed Multi-Process Architecture** utilizing POSIX Shared Memory (`/dev/shm`).

The system operates in three completely decoupled silos:

1. **The Real-Time Control Loop (C++)**
   * Paced strictly at **1000Hz** using POSIX `clock_nanosleep` and `CLOCK_MONOTONIC`.
   * Enforces `SCHED_FIFO` OS-level priority to guarantee microsecond deadline execution.
   * Calculates Lagrangian dynamics, integrates physics, and executes traditional control laws natively.
2. **The Graphics Visualization Engine (C++ / OpenGL)**
   * A standalone executable running at 60Hz.
   * Utilizes 3D instanced rendering to visualize the manipulator's kinematic state, the desired path, and the actual traced path.
   * Achieves **Total Fault Isolation**: A graphics driver crash will not interrupt the real-time physics loop.
3. **The ML Inference Engine (Python / JAX) [WIP]**
   * Asynchronously polls the robot state at 50-100Hz.
   * Executes neural network forward passes via XLA to compute compensation torques.

### Lock-Free Inter-Process Communication (IPC)

To prevent the 1000Hz control loop from ever blocking or waiting on the UI/ML threads, the system utilizes a **Dual-IPC** strategy:

* **High-Speed Telemetry:** Lock-free data transfer utilizing an atomic **Sequence Lock (Seqlock)**.
* **Bulk Trajectory Data:** Event-driven transfer of massive path arrays utilizing a cross-process hardware mutex (`pthread_mutexattr_setpshared`).

## Current State & Features

* **Computed Torque Control (CTC):** Fully operational traditional controller validating the baseline physics engine.
* **Continuous Trajectory Generation:** Capable of generating non-stop, randomized cubic Hermite splines (Catmull-Rom) for continuous testing.
* **High-Fidelity Telemetry Logging:** Pre-allocated, zero-allocation CSV dumping for post-simulation frequency and tracking analysis.

## Roadmap / Future Work

* **Physics-Informed Neural Network (PINN):** Finalize the Python IPC bridge to inject ML compensation torques into the real-time C++ plant.
* **Sensor Noise Injection:** Introduce Gaussian noise and filtering (State Estimation) to transition from a "perfect" simulation to real-world conditions.
* **Comparative Study Metrics:** Automate the calculation of Tracking Error (RMSE) and Total Control Effort between CTC, SMC, and PINN under mass-mismatch disturbances.
* **Trajectory Upgrades:** Upgrade to Quintic splines / Minimum Jerk optimization to guarantee $C^2$ continuous acceleration (eliminating feedforward torque spikes).

## Build and Run

### Dependencies

* **C++:** CMake (>=3.10), GCC (C++17), Eigen3, Boost.Interprocess, GLFW, OpenGL.
* **Python:** Poetry, JAX, NumPy.
* **OS:** Linux (Requires POSIX compliance for shared memory and real-time scheduling).

### Compilation

```bash
mkdir build && cd build
cmake ..
make
```

### Execution

Due to strict real-time OS scheduling, the control loop must be executed with elevated privileges. The startup order must be strictly followed to ensure shared memory initialization.

```bash
# Terminal 1: Start the Real-Time Master (Creates IPC)
sudo ./RoboticSim

# Terminal 2: Start the OpenGL Visualizer
./GraphicsApp

# Terminal 3: Start the ML Inference (coming soon)
poetry run python pinn_inference.py
```
