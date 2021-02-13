#include "Autons/TestTurnAuton.h"

TestTurnAuton::TestTurnAuton(TankDriveNode* chassis_node, InertialSensorNode* inertial_sensor_node) : 
        Auton("Test Turn Node"), m_chassis_node(chassis_node), m_inertial_sensor_node(inertial_sensor_node) {
    

}

void TestTurnAuton::AddNodes() {
    Eigen::Rotation2Dd target_angle(M_PI);

    m_turn_node = new AutonNode(10, new TurnToAngleAction(m_chassis_node, m_inertial_sensor_node, target_angle));
    Auton::AddFirstNode(m_turn_node);
}