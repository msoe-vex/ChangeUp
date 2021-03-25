#include "HolonomicPursuit.h"

HolonomicPursuit::HolonomicPursuit(Path path, Timer timer) : 
        m_path(path),
        m_timer(timer),
        m_x_pid(1., 0., 0., 0.),
        m_y_pid(1., 0., 0., 0.),
        m_theta_pid(1., 0., 0., 0.) {
    
}

void HolonomicPursuit::startPursuit() {
    m_timer.Start();
}

HolonomicPursuit::TargetVelocity HolonomicPursuit::getTargetVelocity(Pose current_pose) {
    float current_time = m_timer.Get();

    Pose next_pose = m_path.update(current_time);

    m_previous_time = m_timer.Get();

    Vector2d linear_error = next_pose.position - current_pose.position;
    float theta_error = next_pose.angle.angle() - current_pose.angle.angle();

    // Determine the feedback of each movement component to get to our new position
    float x_feedback = m_x_pid.calculate(linear_error.x());
    float y_feedback = m_y_pid.calculate(linear_error.y());
    float theta_feedback = m_theta_pid.calculate(theta_error);

    // Return the target velocities, and whether the path is at the end point
    TargetVelocity target_velocity = {
        Vector2d(x_feedback * MAX_VELOCITY, y_feedback * MAX_VELOCITY), 
        theta_feedback * MAX_VELOCITY, 
        m_path.isComplete()
    };
    
    return target_velocity;
}

HolonomicPursuit::~HolonomicPursuit() {

}