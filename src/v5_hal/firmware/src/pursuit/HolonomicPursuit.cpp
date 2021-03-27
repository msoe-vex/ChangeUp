#include "pursuit/HolonomicPursuit.h"

HolonomicPursuit::HolonomicPursuit(Path path, Timer timer) : 
        m_path(path),
        m_timer(timer),
        m_x_pid(0.1, 0., 0., 0.),
        m_y_pid(0.1, 0., 0., 0.),
        m_theta_pid(0.1, 0., 0., 0.) {
    
}

void HolonomicPursuit::startPursuit() {
    m_timer.Start();
}

HolonomicPursuit::TargetVelocity HolonomicPursuit::getTargetVelocity(Pose current_pose) {
    Pose next_pose = m_path.update(m_timer.Get());

    Logger::logInfo("Next pose | x: " + std::to_string(next_pose.position.x()) + " | y: " + std::to_string(next_pose.position.y()));

    Vector2d linear_error = next_pose.position - current_pose.position;
    float theta_error = next_pose.angle.angle() - current_pose.angle.angle();

    // Determine the feedback of each movement component to get to our new position
    float x_feedback = m_x_pid.calculate(linear_error.x());
    float y_feedback = m_y_pid.calculate(linear_error.y());
    float theta_feedback = m_theta_pid.calculate(theta_error);

    // Return the target velocities, and whether the path is at the end point
    TargetVelocity target_velocity = {
        Vector2d(x_feedback * MAX_VELOCITY, y_feedback * MAX_VELOCITY), 
        0., // TODO CHANGE BACK 
        m_path.isComplete()
    };
    
    return target_velocity;
}

HolonomicPursuit::~HolonomicPursuit() {

}