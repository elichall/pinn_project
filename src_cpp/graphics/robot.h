#ifndef ROBOT_SYSTEM_H
#define ROBOT_SYSTEM_H

#include <vector>
#include <glm/glm.hpp>

// The pure physical state at any given microsecond
struct RobotState {
    float time;
    float theta1; // Base Rotation
    float d;      // Prismatic Extension
    float theta2; // End Effector Rotation
};

// memory layout for the instanced renderer engine
struct RobotSystem {
    int numLinks = 5;
    
    // memory blocks for the GPU
    std::vector<glm::mat4> spatialMats;
    std::vector<glm::vec3> colors;

    RobotSystem() {
        spatialMats.resize(numLinks, glm::mat4(1.0f));
        
        colors = {
            {0.8f, 0.2f, 0.2f}, // Base: Red
            {0.2f, 0.3f, 0.9f}, // Link 1: Blue
            {0.2f, 0.8f, 0.2f}, // Joint 1: Green
            {0.2f, 0.3f, 0.9f}, // Link 2: Blue
            {0.2f, 0.8f, 0.2f}  // End Effector: Green
        };
    }
};

#endif