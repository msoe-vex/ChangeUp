#include "pursuit/holonomic_pursuit/HolonomicPosePursuit.h"

HolonomicPosePursuit::HolonomicPosePursuit(Pose target_pose, Timer timer) : 
        m_target_pose(target_pose),
        m_timer(timer),
        m_x_pid(0.1, 0., 0., 0.),
        m_y_pid(0.1, 0., 0., 0.),
        m_theta_pid(0.1, 0., 0., 0.) {
    
}

void HolonomicPosePursuit::startPursuit() {
    m_timer.Start();
}

HolonomicPosePursuit::TargetVelocity HolonomicPosePursuit::getTargetVelocity(Pose current_pose) {
    float current_time = m_timer.Get();

    m_previous_time = m_timer.Get();

    Vector2d linear_error = m_target_pose.position - current_pose.position;
    float theta_error = m_target_pose.angle.angle() - current_pose.angle.angle();

    // Determine the feedback of each movement component to get to our new position
    float x_feedback = m_x_pid.calculate(linear_error.x());
    float y_feedback = m_y_pid.calculate(linear_error.y());
    float theta_feedback = m_theta_pid.calculate(theta_error);

    // Return the target velocities, and whether the path is at the end point
    TargetVelocity target_velocity = {
        Vector2d(x_feedback * MAX_VELOCITY, y_feedback * MAX_VELOCITY), 
        theta_feedback * MAX_VELOCITY,
        linear_error.norm() < POSE_PURSUIT_LINEAR_THRESHOLD && theta_error < POSE_PURSUIT_THETA_THRESHOLD
    };
    
    return target_velocity;
}

HolonomicPosePursuit::~HolonomicPosePursuit() {

}