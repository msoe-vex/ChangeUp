#include "Actions/DriveAction.h"
#include <math.h>

DriveAction::DriveAction(TankDriveNode* tank_drive_node, double distance, double max_velocity, 
        double max_accel) :
        m_tank_drive_node(tank_drive_node), m_distance(distance), 
        m_max_velocity(max_velocity), m_max_accel(max_accel), m_lastSpeed(0) {

}

void DriveAction::ActionInit() {
    m_timer.Start();
    m_tank_drive_node->resetEncoders();
}

AutonAction::actionStatus DriveAction::Action() {
    double dt = m_timer.Get() - m_lastTime;
    double speed = m_max_velocity;

    double accel = (m_max_velocity - m_lastSpeed) / dt;

    if (accel < -m_max_accel) {
        speed = m_lastSpeed - m_max_accel * dt;
    } else if (accel > m_max_accel) {
        speed = m_lastSpeed + m_max_accel * dt;
    }

    // Subtract the found offset of 3 inches to shorten the path
    double remainingDistance = max(m_distance - ((m_tank_drive_node->getLeftDistancePID() / 900.0) * (M_PI * 4.0625)) - 3, 0.);

    double maxAllowedSpeed = sqrt(2 * m_max_accel * remainingDistance);
    if (fabs(speed) > maxAllowedSpeed) {
        speed = std::copysign(maxAllowedSpeed, speed);
    }

    m_lastSpeed = speed;
    m_lastTime = m_timer.Get();

    // m_actual_left_distance = (m_tank_drive_node->getLeftDistancePID() / (M_PI * 4.0625)) * 900;
    // m_actual_right_distance = (m_tank_drive_node->getRightDistancePID() / (M_PI * 4.0625)) * 900;

    std::string log_str = "Remaining: " + to_string(remainingDistance) + "\nSpeed: " + to_string(speed);
    m_tank_drive_node->m_handle->logwarn(log_str.c_str());

    if (remainingDistance < 0.5) {
        return END;
    } else {
        m_tank_drive_node->setDriveVelocity(speed, speed);
        return CONTINUE;
    }
}

void DriveAction::ActionEnd() {
    m_tank_drive_node->setDriveVelocity(0, 0);
}