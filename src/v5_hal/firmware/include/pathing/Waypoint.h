#pragma once

#include "api.h"
#include "eigen/Eigen/Dense"
#include "math/Pose.h"

class Waypoint {
private:
    Pose m_position;
    Vector2d m_velocity;

public:
    Waypoint(Pose position, Vector2d velocity);

    Pose getPosition();

    Vector2d getVelocity();
};