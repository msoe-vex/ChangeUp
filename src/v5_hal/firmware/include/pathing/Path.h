#pragma once

#include "eigen/Eigen/Dense"
#include "pathing/PathPoint.h"
#include "math/Pose.h"
#include <vector>

using namespace std;

class Path {
public:
    Path();

    Path(vector<PathPoint> pathPoints);

    Pose* update(float time);

    vector<PathPoint> getPathPoints();

private:
    vector<PathPoint> m_pathPoints;
};