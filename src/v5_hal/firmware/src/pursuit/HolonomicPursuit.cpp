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

    float x_percent = m_x_pid.calculate(linear_error(0));
    float y_percent = m_y_pid.calculate(linear_error(1));
    float theta_percent = m_theta_pid.calculate(theta_error);

    TargetVelocity target_velocity = {
        Vector2d(x_percent * MAX_VELOCITY, y_percent * MAX_VELOCITY), 
        theta_percent * MAX_VELOCITY};
    return target_velocity;
}

HolonomicPursuit::~HolonomicPursuit() {

}