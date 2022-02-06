#pragma once

#include "auton/Auton.h"
#include "pathing/PathManager.h"
#include "nodes/subsystems/drivetrain_nodes/IDriveNode.h"
#include "nodes/odometry_nodes/OdometryNode.h"
#include "nodes/subsystems/ConveyorNode.h"
#include "nodes/subsystems/IntakeNode.h"
#include "nodes/sensor_nodes/InertialSensorNode.h"
#include "auton/auton_actions/HolonomicFollowPathAction.h"
#include "auton/auton_actions/DeployAction.h"
#include "auton/auton_actions/IntakeAction.h"
#include "auton/auton_actions/UpdateConveyorStateAction.h"
#include "auton/auton_actions/TurnToAngleAction.h"
#include "auton/auton_actions/OpenIntakesAction.h"
#include "auton/auton_actions/DriveStraightAction.h"
#include "auton/auton_actions/ScoreSingleBallAction.h"
#include "eigen/Eigen/Dense"
#include "util/Constants.h"
#include "auton/AutonSequencePresets.h"

class ProgrammingSkillsAuton : public Auton {
public:
    ProgrammingSkillsAuton(IDriveNode* drive_node, OdometryNode* odom_node, ConveyorNode* conveyer_node,
        IntakeNode* intake_node, InertialSensorNode* inertial_sensor_node);

    void AddNodes() override;

private:
    IDriveNode* m_drive_node;
    OdometryNode* m_odom_node;
    ConveyorNode* m_conveyor_node;
    IntakeNode* m_intake_node;
    InertialSensorNode* m_inertial_sensor_node;
};