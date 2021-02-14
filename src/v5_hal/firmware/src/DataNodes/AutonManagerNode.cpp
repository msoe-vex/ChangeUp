#include "DataNodes/AutonManagerNode.h"

AutonManagerNode::AutonManagerNode(NodeManager* node_manager, TankDriveNode* tank_drive_node, OdometryNode* odometry_node,
    ConveyorNode* conveyor_node, InertialSensorNode* inertial_sensor_node) : Node(node_manager, 50) {
    m_programming_skills_auton = new ProgrammingSkillsAuton(tank_drive_node, odometry_node, conveyor_node, inertial_sensor_node);
    m_selected_auton = m_programming_skills_auton;
}

void AutonManagerNode::initialize() {
    PathManager::GetInstance()->LoadPathsFile("/usd/path.json");
    m_selected_auton->AutonInit();
}

void AutonManagerNode::autonPeriodic() {
    if(!m_selected_auton->Complete()) {
        m_selected_auton->AutonPeriodic();
    }
}