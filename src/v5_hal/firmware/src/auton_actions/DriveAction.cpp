#include "DriveAction.h"
#include <math.h>

DriveAction::DriveAction(TankDriveNode* tank_drive_node, double distance, double max_velocity, 
        double max_accel) :
        m_tank_drive_node(tank_drive_node), m_distance(distance), 
        m_max_velocity(max_velocity), m_max_accel(max_accel), m_lastSpeed(0), m_feedForward(4.91) {

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
    double remainingDistance = max(fabs(m_distance) - fabs(((m_tank_drive_node->getLeftDistancePID() / 900.0) * (M_PI * 4.0625))) - 3, 0.);

    double maxAllowedSpeed = sqrt(2 * m_max_accel * remainingDistance);
    if (fabs(speed) > maxAllowedSpeed) {
        speed = std::copysign(maxAllowedSpeed, speed);
    }

    speed = max(speed, m_feedForward);

    m_lastSpeed = speed;
    m_lastTime = m_timer.Get();

    if (remainingDistance < 0.5) {
        return END;
    } else {
        if (m_distance > 0) {
            m_tank_drive_node->setDriveVelocity(speed, speed);
        } else {
            m_tank_drive_node->setDriveVelocity(-speed, -speed);
        }
        
        return CONTINUE;
    }
}

void DriveAction::ActionEnd() {
    m_tank_drive_node->setDriveVelocity(0, 0);
}