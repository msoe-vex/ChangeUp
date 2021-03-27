#pragma once

#include "api.h"
#include "eigen/Eigen/Dense"
#include "pathing/Path.h"
#include "util/Timer.h"
#include "util/PID.h"
#include "util/Constants.h"
#include "math/Pose.h"
#include "adaptive_pursuit_controller/Logger.h"

using namespace Eigen;

class HolonomicPursuit {
private:
    Path m_path;
    Timer m_timer;
    PID m_x_pid;
    PID m_y_pid;
    PID m_theta_pid;

public:
    struct TargetVelocity {
        Vector2d linear_velocity;
        float rotational_velocity;
        bool end_of_path;
    };

    HolonomicPursuit(Path path, Timer timer=Timer());

    void startPursuit();

    TargetVelocity getTargetVelocity(Pose current_pose);

    ~HolonomicPursuit();
};