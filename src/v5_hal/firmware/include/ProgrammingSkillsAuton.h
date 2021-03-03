#pragma once

#include "Auton.h"
#include "adaptive_pursuit_controller/PathManager.h"
#include "data_nodes/TankDriveNode.h"
#include "data_nodes/OdometryNode.h"
#include "data_nodes/ConveyorNode.h"
#include "data_nodes/InertialSensorNode.h"
#include "auton_actions/FollowPathAction.h"
#include "auton_actions/DeployAction.h"
#include "auton_actions/IntakeAction.h"
#include "auton_actions/BottomConveyorAction.h"
#include "auton_actions/TopConveyorAction.h"
#include "auton_actions/TurnToAngleAction.h"
#include "auton_actions/OpenIntakesAction.h"
#include "auton_actions/DriveAction.h"
#include "eigen/Eigen/Dense"

class ProgrammingSkillsAuton : public Auton {
public:
    ProgrammingSkillsAuton(TankDriveNode* tank_drive_node, OdometryNode* odom_node, ConveyorNode* conveyer_node,
        InertialSensorNode* inertial_sensor_node);

    void AddNodes() override;

private:
    TankDriveNode* m_tank_drive_node;
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