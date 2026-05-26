#include "SharedMemoryLink.h"
#include <iostream>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

int main() {
    std::cout << "Starting GraphicsApp..." << std::endl;

    // Connect to the shared memory path created by the main control loop
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

    // TODO: Initialize graphics engine and start 60Hz read loop

    return 0;
}
