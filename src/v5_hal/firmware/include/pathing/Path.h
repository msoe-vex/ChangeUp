#pragma once

#include "eigen/Eigen/Dense"
#include "pathing/PathPoint.h"
#include "math/Pose.h"
#include <vector>
#include "3rdparty/json.hpp"

using namespace std;
using namespace nlohmann;

class Path {
public:
    Path(json jsonToLoad);

    Pose update(float time);

    vector<PathPoint> getPathPoints();

private:
    vector<PathPoint> m_pathPoints;
};