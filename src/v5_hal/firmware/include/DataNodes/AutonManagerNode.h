#pragma once

#include "NodeManager.h"
#include "ProgrammingSkillsAuton.h"
#include "DataNodes/TankDriveNode.h"
#include "DataNodes/OdometryNode.h"
#include "DataNodes/ConveyorNode.h"
#include "api.h"

class AutonManagerNode : public Node {
private:
    Auton * m_programming_skills_auton;
    Auton * m_selected_auton;

public:
    AutonManagerNode(NodeManager* node_manager, TankDriveNode* tank_drive_node, OdometryNode* odometry_node, 
        ConveyorNode* conveyor_node);

    void initialize();

    void autonPeriodic();
};
