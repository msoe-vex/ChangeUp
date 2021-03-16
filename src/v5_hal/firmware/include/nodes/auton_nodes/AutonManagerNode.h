#pragma once

#include "nodes/NodeManager.h"
#include "auton/auton_routines/ProgrammingSkillsAuton.h"
#include "nodes/subsystems/drivetrain_nodes/AbstractDriveNode.h"
#include "nodes/odometry_nodes/OdometryNode.h"
#include "nodes/subsystems/ConveyorNode.h"
#include "nodes/sensor_nodes/InertialSensorNode.h"
#include "api.h"

class AutonManagerNode : public Node {
private:
    Auton* m_programming_skills_auton;

public:
    AutonManagerNode(NodeManager* node_manager, AbstractDriveNode* drive_node, OdometryNode* odometry_node, 
        ConveyorNode* conveyor_node, InertialSensorNode* inertial_sensor_node);

    Auton* selected_auton;

    void initialize();

    void autonPeriodic();
};
