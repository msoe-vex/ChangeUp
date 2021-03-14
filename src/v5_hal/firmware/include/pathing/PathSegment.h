#pragma once

#include "api.h"
#include "eigen/Eigen/Dense"
#include "math/Pose.h"

class PathSegment {
private:
    float m_length_m;
    float m_duration_ms;
    float m_start_velocity;
    float m_end_velocity;
public:
    PathSegment(Pose start_pose, Pose end_pose, float start_velocity, float end_velocity);

    void Initialize();

    float getLength();

    float getDuration();
};