#pragma once

#include "NodeManager.h"
#include "ProgrammingSkillsAuton.h"
#include "data_nodes/TankDriveNode.h"
#include "data_nodes/OdometryNode.h"
#include "data_nodes/ConveyorNode.h"
#include "data_nodes/InertialSensorNode.h"
#include "api.h"

class AutonManagerNode : public Node {
private:
    Auton* m_programming_skills_auton;

public:
    AutonManagerNode(NodeManager* node_manager, TankDriveNode* tank_drive_node, OdometryNode* odometry_node, 
        ConveyorNode* conveyor_node, InertialSensorNode* inertial_sensor_node);

    Auton* selected_auton;

    void initialize();

    void autonPeriodic();
};
