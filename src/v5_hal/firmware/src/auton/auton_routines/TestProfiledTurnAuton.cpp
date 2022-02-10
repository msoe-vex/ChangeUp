#include "auton/auton_routines/TestProfiledTurnAuton.h"

TestProfiledTurnAction::TestProfiledTurnAction(IDriveNode* drive_node, InertialSensorNode* inertial_sensor_node) : 
        Auton("Test ProfiledTurnAction Node"), m_drive_node(drive_node), m_inertial_sensor_node(inertial_sensor_node) {
    
}

void TestProfiledTurnAction::AddNodes() {
    Eigen::Rotation2Dd target_angle(M_PI_2);

    m_turn_node = new AutonNode(10, new ProfiledTurnAction(m_drive_node, m_inertial_sensor_node, target_angle, 1.0, 1.0));
    Auton::AddFirstNode(m_turn_node);
}