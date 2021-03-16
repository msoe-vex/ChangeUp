#include "nodes/auton_nodes/AutonManagerNode.h"

AutonManagerNode::AutonManagerNode(NodeManager* node_manager, IDriveNode* drive_node, OdometryNode* odometry_node,
    ConveyorNode* conveyor_node, InertialSensorNode* inertial_sensor_node) : Node(node_manager, 50) {
    m_programming_skills_auton = new ProgrammingSkillsAuton(drive_node, odometry_node, conveyor_node, inertial_sensor_node);
    selected_auton = m_programming_skills_auton;
}

void AutonManagerNode::initialize() {
    PathManager::GetInstance()->LoadPathsFile("/usd/path.json");
    selected_auton->AutonInit();
}

void AutonManagerNode::autonPeriodic() {
    if(!selected_auton->Complete()) {
        selected_auton->AutonPeriodic();
    }
}