#include "nodes/auton_nodes/AutonManagerNode.h"

AutonManagerNode::AutonManagerNode(NodeManager* node_manager, IDriveNode* drive_node, ConveyorNode* conveyor_node, 
        IntakeNode* intake_node, OdometryNode* odometry_node, InertialSensorNode* inertial_sensor_node) : Node(node_manager, 50) {
    m_test_path_auton = new TestPathAuton(drive_node, odometry_node);
    selected_auton = m_test_path_auton;
}

void AutonManagerNode::initialize() {
    PathManager::GetInstance()->LoadPathsFile("/usd/path.json");
}

void AutonManagerNode::autonPeriodic() {
    if(!selected_auton->Complete()) {
        selected_auton->AutonPeriodic();
    }
}