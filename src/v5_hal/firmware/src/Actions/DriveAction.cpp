#include "Actions/DriveAction.h"
#include <math.h>

DriveAction::DriveAction(TankDriveNode* tank_drive_node, double distance_left, double distance_right, int max_velocity) :
        m_tank_drive_node(tank_drive_node), m_distance_left(distance_left), m_distance_right(distance_right), m_max_velocity(max_velocity) {

}

void DriveAction::actionInit() {
    m_tank_drive_node->setDriveDistancePID(m_distance_left, m_distance_right, m_max_velocity);
}

AutonAction::actionStatus DriveAction::action() {
    m_actual_left_distance = m_tank_drive_node->getLeftDistancePID();
    m_actual_right_distance = m_tank_drive_node->getRightDistancePID();

    if(fabs(m_actual_left_distance - m_distance_left) < 128 &&
        fabs(m_actual_right_distance - m_distance_right) < 128) {
        return END;
    } else {
        return CONTINUE;
    }
}

void DriveAction::actionEnd() {
    
}