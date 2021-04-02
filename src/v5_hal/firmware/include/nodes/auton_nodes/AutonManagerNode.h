#pragma once

#include "nodes/NodeManager.h"
#include "auton/auton_routines/ProgrammingSkillsAuton.h"
#include "auton/auton_routines/TestPathAuton.h"
#include "auton/auton_routines/TestPoseAuton.h"
#include "auton/auton_routines/TestTurnAuton.h"
#include "nodes/subsystems/drivetrain_nodes/IDriveNode.h"
#include "nodes/odometry_nodes/OdometryNode.h"
#include "nodes/subsystems/ConveyorNode.h"
#include "nodes/subsystems/IntakeNode.h"
#include "nodes/sensor_nodes/InertialSensorNode.h"
#include "pathing/PathManager.h"
#include "api.h"

class AutonManagerNode : public Node {
private:
    Auton* m_programming_skills_auton;

public:
    AutonManagerNode(NodeManager* node_manager, IDriveNode* drive_node, ConveyorNode* conveyor_node, IntakeNode* intake_node, 
        OdometryNode* odometry_node, InertialSensorNode* inertial_sensor_node);

    Auton* selected_auton;

    void initialize();

    void autonPeriodic();
};
