#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include "robot.h"
#include "config.h"

inline glm::mat4 calculateDHMatrix(float theta, float a, float alpha, float d) {
    float Cth = cosf(theta); float Sth = sinf(theta);
    float Cal = cosf(alpha); float Sal = sinf(alpha);

    return glm::mat4(
        Cth,      Sth,       0,    0,   // Column 0
        -Sth*Cal, Cth*Cal,   Sal,  0,   // Column 1
        Sth*Sal,  -Cth*Sal,  Cal,  0,   // Column 2
        a*Cth,    a*Sth,     d,    1    // Column 3
    );
}

inline void updateManipulatorKinematics(RobotSystem& sys, const RobotState& state) {
    float q[3] = {state.theta1, state.d, state.theta2};

    // D-H table: theta, a, alpha, d
    glm::vec4 tableDH[5] = { 
        {q[0], 0, 0, 0}, 
        {0, q[1] / 2.0f, 0, 0}, 
        {0, q[1] / 2.0f, 0, 0}, 
        {q[2], link2Length / 2.0f, 0, 0}, 
        {0, link2Length / 2.0f, 0, 0} 
    };

    glm::vec3 scales[5] = {
        {baseHeight,  baseWidth, baseWidth},
        {q[1],        linkWidth, linkWidth},
        {jointSize,   jointSize, jointSize},
        {link2Length, linkWidth, linkWidth},
        {jointSize,   jointSize, jointSize}
    };

    glm::mat4 runningMat = glm::mat4(1.0f);
    
    for (int i = 0; i < 5; i++) {
        glm::mat4 stepMat = runningMat * calculateDHMatrix(tableDH[i][0], tableDH[i][1], tableDH[i][2], tableDH[i][3]);
        sys.spatialMats[i] = stepMat * glm::scale(glm::mat4(1.0f), scales[i]);
        runningMat = stepMat;
    }
}

#endif