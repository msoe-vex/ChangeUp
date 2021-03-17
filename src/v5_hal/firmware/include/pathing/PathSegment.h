#pragma once

#include "api.h"
#include "eigen/Eigen/Dense"
#include "pathing/Waypoint.h"

class PathSegment {
private:
    Waypoint m_start_waypoint;
    Waypoint m_end_waypoint;
    float m_length_m;
    float m_duration_ms;

public:
    PathSegment(Waypoint start_waypoint, Waypoint end_waypoint);

    void initialize();

    float getLength();

    float getDuration();
};