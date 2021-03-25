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

    Pose update(float time);

    vector<PathPoint> getPathPoints();

    bool isComplete();

private:
    vector<PathPoint> m_pathPoints;
    PathPoint m_last_point;
    bool m_is_complete;
};