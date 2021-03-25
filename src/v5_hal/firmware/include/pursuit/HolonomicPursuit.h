#pragma once

#include "api.h"
#include "eigen/Eigen/Dense"
#include "pathing/Path.h"
#include "util/Timer.h"
#include "util/PID.h"
#include "util/Constants.h"
#include "math/Pose.h"

using namespace Eigen;

class HolonomicPursuit {
private:
    Path m_path;
    Timer m_timer;
    float m_previous_time;
    PID m_x_pid;
    PID m_y_pid;
    PID m_theta_pid;

public:
    struct TargetVelocity {
        Vector2d linear_velocity;
        float rotational_velocity;
    };

    HolonomicPursuit(Path path, Timer timer=Timer());

    void startPursuit();

    TargetVelocity getTargetVelocity(Pose current_pose);

    ~HolonomicPursuit();
};