#include "SharedMemoryLink.h"
#include <fcntl.h>    // For O_CREAT, O_RDWR
#include <sys/mman.h> // For shm_open, mmap, PROT_READ, PROT_WRITE
#include <unistd.h>   // For ftruncate
#include <iostream>

namespace IPC {

TelemetryIPC* initTelemetryIPC() {
  // Create objects which will be the SharedMemoryLink
  int fd_telemetry = shm_open("/pinn_manip_telemetry", O_CREAT | O_RDWR, 0666);

  // allocate the space in ram
  ftruncate(fd_telemetry, sizeof(TelemetryIPC));

  // maps the memory to cpp pointers
  TelemetryIPC *telemetryIPC =
      (TelemetryIPC *)mmap(NULL, sizeof(TelemetryIPC), PROT_READ | PROT_WRITE,
                           MAP_SHARED, fd_telemetry, 0);

  // initalize the counters to zero
  telemetryIPC->sequenceCounter.store(0);
  telemetryIPC->pathVersion.store(0);

  return telemetryIPC;
}

PathIPC* initPathIPC() {
  int fd_path = shm_open("/pinn_manip_path", O_CREAT | O_RDWR, 0666);

  // allocate the space in ram
  ftruncate(fd_path, sizeof(PathIPC));

  PathIPC *pathIPC = (PathIPC *)mmap(
      NULL, sizeof(PathIPC), PROT_READ | PROT_WRITE, MAP_SHARED, fd_path, 0);

  // Set access privilages
  // Configure Mutex Attributes
  pthread_mutexattr_t attr;
  pthread_mutexattr_init(&attr);
  // CRITICAL: Tells the kernel this mutex must be respected across process
  // boundaries
  pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);

  // Initialize the mutex located at path_ipc->mutex using these attributes
  pthread_mutex_init(&pathIPC->mutex, &attr);
  // clean up memory
  pthread_mutexattr_destroy(&attr);

  return pathIPC;
}

void writeTelemetry(TelemetryIPC* ipc, const double* q, const double* qdot, const double* tau) {
  // on odd python and graphics won't grab data
  // if squenceCounter is different after fetching, they discard
  ipc->sequenceCounter.fetch_add(1, std::memory_order_release);
  for (int i = 0; i < 3; ++i) {
    ipc->q[i] = q[i];
    ipc->qdot[i] = qdot[i];
    ipc->tauPINN[i] = tau[i];
  } // increment to even so other programs know they can read
  ipc->sequenceCounter.fetch_add(1, std::memory_order_release);
}

void writePath(PathIPC* pathIPC, TelemetryIPC* telemetryIPC) {
  // unlock mutex
  pthread_mutex_lock(&pathIPC->mutex);

  /*
  write waypoints to ipc
  */

  // lock again and increment path flag to let graphics know there is a change
  pthread_mutex_unlock(&pathIPC->mutex);
  telemetryIPC->pathVersion.fetch_add(1, std::memory_order_relaxed);
}

} // namespace IPC
