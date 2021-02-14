#pragma once

#include "Auton.h"
#include "adaptive_pursuit_controller/PathManager.h"
#include "DataNodes/TankDriveNode.h"
#include "DataNodes/OdometryNode.h"
#include "DataNodes/ConveyorNode.h"
#include "DataNodes/InertialSensorNode.h"
#include "Actions/FollowPathAction.h"
#include "Actions/DeployAction.h"
#include "Actions/IntakeAction.h"
#include "Actions/BottomConveyorAction.h"
#include "Actions/TopConveyorAction.h"
#include "Actions/TurnToAngleAction.h"
#include "Actions/OpenIntakesAction.h"
#include "Actions/DriveAction.h"
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

    AutonNode* m_path_1;
    AutonNode* m_turn_1;
    AutonNode* m_forward_1;
    AutonNode* m_turn_2;

    AutonNode* m_wait_to_close_intakes;
    AutonNode* m_close_intakes;
    
    
    AutonNode* m_path_2;
    AutonNode* m_path_3;
};