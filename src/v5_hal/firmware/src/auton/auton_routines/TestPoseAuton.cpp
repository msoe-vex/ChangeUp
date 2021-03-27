#include "auton/auton_routines/TestPoseAuton.h"

TestPoseAuton::TestPoseAuton(IDriveNode* drive_node, InertialSensorNode* inertial_sensor_node) : 
        Auton("Test Pose Node"), m_drive_node(drive_node), m_inertial_sensor_node(inertial_sensor_node) {

}

void TestPoseAuton::AddNodes() {
    Eigen::Rotation2Dd target_angle(M_PI);

    m_turn_node = new AutonNode(10, new TurnToAngleAction(m_drive_node, m_inertial_sensor_node, target_angle));
    Auton::AddFirstNode(m_turn_node);
}