#pragma once

#include "api.h"
#include "eigen/Eigen/Dense"
#include "pathing/Path.h"
#include "util/Timer.h"
#include "util/PID.h"
#include "util/Constants.h"
#include "math/Pose.h"

using namespace Eigen;

class HolonomicPosePursuit {
private:
    Pose m_target_pose;
    Timer m_timer;
    float m_previous_time;
    PID m_x_pid;
    PID m_y_pid;
    PID m_theta_pid;

public:
    struct TargetVelocity {
        Vector2d linear_velocity;
        float rotational_velocity;
        bool is_within_end_tolerance;
    };

    HolonomicPosePursuit(Pose target_pose, Timer timer=Timer());

    void startPursuit();

    TargetVelocity getTargetVelocity(Pose current_pose);

    ~HolonomicPosePursuit();
};