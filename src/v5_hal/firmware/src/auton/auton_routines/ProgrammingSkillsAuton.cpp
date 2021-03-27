#include "auton/auton_routines/ProgrammingSkillsAuton.h"

ProgrammingSkillsAuton::ProgrammingSkillsAuton(IDriveNode* drive_node, OdometryNode* odom_node,
        ConveyorNode* conveyor_node, IntakeNode* intake_node, InertialSensorNode* inertial_sensor_node) : Auton("Programming Skills"), 
        m_drive_node(drive_node), 
        m_odom_node(odom_node), 
        m_inertial_sensor_node(inertial_sensor_node),
        m_conveyor_node(conveyor_node),
        m_intake_node(intake_node) {
    
}

void ProgrammingSkillsAuton::AddNodes() {
    // Define Auton nodes, actions, and sequences here
    m_deploy_node = new AutonNode(1, new DeployAction(m_conveyor_node));


	// Sequence nodes here
    Auton::AddFirstNode(m_deploy_node);
}