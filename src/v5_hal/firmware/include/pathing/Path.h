#pragma once

#include "eigen/Eigen/Dense"
#include "pathing/PathPoint.h"
#include "math/Pose.h"
#include <vector>

class Path {
public:
    Path();
    static Path fromJson();

    Pose update(float time);

private:
    vector<PathPoint> m_pathPoints;
};