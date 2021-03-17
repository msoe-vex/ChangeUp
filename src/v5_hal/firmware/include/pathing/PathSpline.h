#pragma once

#include <vector>

#include "api.h"
#include "eigen/Eigen/Dense"
#include "pathing/Waypoint.h"

class PathSpline {
private:
    Waypoint m_start_waypoint;
    Waypoint m_end_waypoint;
    float m_length_m;
    float m_duration_ms;

    Rotation2Dd m_path_start_angle;
    Rotation2Dd m_path_end_angle;

    std::vector<Waypoint> m_calculated_waypoints;

    void m_generateSpline();

public:
    PathSpline(Waypoint start_waypoint, Waypoint end_waypoint);

    void calculate();

    float getLength();

    float getDuration();

    Waypoint getWaypointAtPercentage(float percentage);

    Waypoint getWaypointAtTime(float time);
};