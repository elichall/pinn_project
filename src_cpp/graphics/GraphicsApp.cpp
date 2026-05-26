#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "GraphicsEngine.h"
#include "SharedMemoryLink.h"
#include "robot.h"
#include "kinematics.h"
#include "config.h"

#include <iostream>
#include <vector>
#include <fcntl.h>
#include <sys/mman.h>

int main() {
    std::cout << "Starting GraphicsApp..." << std::endl;

    // --- IPC Initialization ---
    int fd_path = shm_open("/pinn_manip_path", O_RDWR, 0666);
    if (fd_path == -1) {
        std::cerr << "Failed to open shared memory. Is the control loop running?" << std::endl;
        return 1;
    }
    IPC::PathIPC *pathIPC = (IPC::PathIPC *)mmap(
        NULL, sizeof(IPC::PathIPC), PROT_READ | PROT_WRITE, MAP_SHARED, fd_path, 0);

    int fd_telemetry = shm_open("/pinn_manip_telemetry", O_RDWR, 0666);
    if (fd_telemetry == -1) {
        std::cerr << "Failed to open telemetry shared memory." << std::endl;
        return 1;
    }
    IPC::TelemetryIPC *telemetryIPC = (IPC::TelemetryIPC *)mmap(
        NULL, sizeof(IPC::TelemetryIPC), PROT_READ | PROT_WRITE, MAP_SHARED, fd_telemetry, 0);

    std::cout << "Successfully connected to POSIX shared memory." << std::endl;

    // --- Engine Initialization ---
    // CUBE instance shape, 5 objects, 800x800 window, wireframe on, sphere LOD {1,1}, lines on, 5000 max line points, static color
    Engine::GraphicsEngine engine(Engine::CUBE, INSTANCE_OBJECTS_NUMBER_EST, MY_WINDOW_AREA, true, {1, 1}, true, MAX_PATH_POINTS, false);
    if (!engine.init()) {
        std::cerr << "Engine init failed!" << std::endl;
        return -1;
    }
    Engine::RenderPayload payload;

    // --- Robot System Setup ---
    RobotSystem robot;
    
    // Trace lines
    std::vector<Engine::LineData> traceLines(2);
    
    // traceLines[0]: Desired Path (White)
    traceLines[0].color = glm::vec3(1.0f, 1.0f, 1.0f);
    traceLines[0].drawCount = 0;
    
    // traceLines[1]: Actual Path Snail Trail (Red)
    traceLines[1].color = glm::vec3(1.0f, 0.0f, 0.0f);
    traceLines[1].drawCount = 0;
    traceLines[1].points.resize(MAX_PATH_POINTS, glm::vec3(0.0f));

    payload.spatialMats = &robot.spatialMats;
    payload.colors = &robot.colors;
    payload.lines = &traceLines;

    uint32_t localPathVersion = 0;

    // --- Render Loop ---
    while (!engine.shouldClose()) {
        
        // --- 1. Path Data (Slow Channel) ---
        uint32_t currentPathVersion = telemetryIPC->pathVersion.load(std::memory_order_acquire);
        if (currentPathVersion > localPathVersion) {
            pthread_mutex_lock(&pathIPC->mutex);
            
            int numPoints = pathIPC->num_points;
            if (numPoints > MAX_PATH_POINTS) numPoints = MAX_PATH_POINTS;
            
            traceLines[0].points.resize(numPoints);
            for (int i = 0; i < numPoints; ++i) {
                traceLines[0].points[i] = glm::vec3(pathIPC->pathX[i], pathIPC->pathY[i], pathIPC->pathZ[i]);
            }
            traceLines[0].drawCount = numPoints;
            
            pthread_mutex_unlock(&pathIPC->mutex);
            localPathVersion = currentPathVersion;
            
            // Reset actual path trail when a new desired path arrives
            traceLines[1].drawCount = 0;
        }

        // --- 2. Telemetry Data (Fast Channel) ---
        uint32_t seq1, seq2;
        double q[3];
        do {
            seq1 = telemetryIPC->sequenceCounter.load(std::memory_order_acquire);
            q[0] = telemetryIPC->q[0];
            q[1] = telemetryIPC->q[1];
            q[2] = telemetryIPC->q[2];
            seq2 = telemetryIPC->sequenceCounter.load(std::memory_order_acquire);
        } while (seq1 & 1 || seq1 != seq2);

        // --- 3. Kinematics Update ---
        RobotState state;
        state.time = 0.0f; // Unused for rendering the current frame
        state.theta1 = static_cast<float>(q[0]);
        state.d = static_cast<float>(q[1]);
        state.theta2 = static_cast<float>(q[2]);
        
        updateManipulatorKinematics(robot, state);

        // Update Actual Path (Snail Trail)
        // Only add if we haven't hit the max points yet
        if (traceLines[1].drawCount < MAX_PATH_POINTS) {
            glm::vec3 endEffectorPos = glm::vec3(robot.spatialMats[4][3]); // 4th column is translation
            traceLines[1].points[traceLines[1].drawCount] = endEffectorPos;
            traceLines[1].drawCount++;
        }

        // --- 4. Render ---
        engine.renderFrame(payload);
    }

    return 0;
}
