#ifndef CONFIG_H
#define CONFIG_H

#include <array>

// Engine settings constants
const int INSTANCE_OBJECTS_NUMBER_EST = 5;
const std::array<int,2> MY_WINDOW_AREA = {800, 800};
const int LINE_DATA_LENGTH_EST = 2000;

// Physical Constants
const float link2Length = 0.8f; 
const float linkWidth = 0.1f;
const float baseWidth = 0.2f;
const float baseHeight = 0.3f;
const float jointSize = 0.15f;

#endif