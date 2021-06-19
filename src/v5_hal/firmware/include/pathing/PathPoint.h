#pragma once

#include "eigen/Eigen/Dense"
#include "math/Pose.h"
// #include "util/Logger.h"

class PathPoint {
public:
    PathPoint(float time, Pose pose, Vector2d linear_velocity, float rotational_velocity);

    float getTime();

    Pose getPose();

    Vector2d getLinearVelocity();

    float getRotationalVelocity();

    PathPoint interpolateTo(PathPoint other, float time);

    bool equals(PathPoint* that);
private:
    Pose m_pose;
    float m_time;
    Vector2d m_linear_velocity;
    float m_rotational_velocity;
};