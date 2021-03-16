#pragma once

#include "auton/Auton.h"
#include "adaptive_pursuit_controller/PathManager.h"
#include "nodes/subsystems/drivetrain_nodes/AbstractDriveNode.h"
#include "nodes/odometry_nodes/OdometryNode.h"
#include "nodes/subsystems/ConveyorNode.h"
#include "nodes/sensor_nodes/InertialSensorNode.h"
#include "auton/auton_actions/FollowPathAction.h"
#include "auton/auton_actions/DeployAction.h"
#include "auton/auton_actions/IntakeAction.h"
#include "auton/auton_actions/BottomConveyorAction.h"
#include "auton/auton_actions/TopConveyorAction.h"
#include "auton/auton_actions/TurnToAngleAction.h"
#include "auton/auton_actions/OpenIntakesAction.h"
#include "auton/auton_actions/DriveAction.h"
#include "eigen/Eigen/Dense"
#include "util/Constants.h"

class ProgrammingSkillsAuton : public Auton {
public:
    ProgrammingSkillsAuton(AbstractDriveNode* drive_node, OdometryNode* odom_node, ConveyorNode* conveyer_node,
        InertialSensorNode* inertial_sensor_node);

    void AddNodes() override;

private:
    AbstractDriveNode* m_drive_node;
    OdometryNode* m_odom_node;
    ConveyorNode* m_conveyor_node;
    InertialSensorNode* m_inertial_sensor_node;

    AutonNode* m_deploy_node;

    AutonNode* m_turn_1;
    AutonNode* m_forward_1;
    AutonNode* m_wait_1;
    AutonNode* m_reverse_1;
    AutonNode* m_turn_2;  
    AutonNode* m_forward_3;
    AutonNode* m_score_1;
    AutonNode* m_descore_1;
    AutonNode* m_reverse_2;
    AutonNode* m_outtake_1;
    AutonNode* m_turn_3;
    AutonNode* m_forward_4;
    AutonNode* m_delay_close_intakes_1;
    AutonNode* m_close_intakes;
    AutonNode* m_turn_4;
    AutonNode* m_forward_5;
    AutonNode* m_descore_2;
    AutonNode* m_score_2;
    AutonNode* m_reverse_3;
    AutonNode* m_turn_5;
    AutonNode* m_forward_6;
    AutonNode* m_eject_1;
    AutonNode* m_stop_eject_1;
    AutonNode* m_capture_ball_1;
    AutonNode* m_forward_7;
    AutonNode* m_turn_6;
    AutonNode* m_forward_8;
    AutonNode* m_score_3;
    AutonNode* m_descore_3;
    AutonNode* m_forward_9;
};