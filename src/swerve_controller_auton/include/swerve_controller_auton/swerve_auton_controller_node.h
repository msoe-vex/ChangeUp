#pragma once

#include "nlohmann/json.hpp"
#include "iostream.h"
#include "Eigen/Dense"

class SwerveAutonControllerNode {
public:
    bool LoadPathsText(string text);

    bool LoadPaths(json loadedJson, vector<Eigen::Vector2d>* pos_vectors);
}