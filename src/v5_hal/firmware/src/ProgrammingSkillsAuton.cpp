#include "ProgrammingSkillsAuton.h"

ProgrammingSkillsAuton::ProgrammingSkillsAuton(TankDriveNode* tank_drive_node, OdometryNode* odom_node,
        ConveyorNode* conveyor_node, InertialSensorNode* inertial_sensor_node) : Auton("Programming Skills"), 
        m_tank_drive_node(tank_drive_node), m_odom_node(odom_node), m_inertial_sensor_node(inertial_sensor_node),
        m_conveyor_node(conveyor_node) {
    
}

void ProgrammingSkillsAuton::AddNodes() {
    m_deploy_node = new AutonNode(5, new DriveAction(m_tank_drive_node, 48, 49.1, 40));

    Auton::AddFirstNode(m_deploy_node);
    
    // m_turn_1 = new AutonNode(1.5, new TurnToAngleAction(m_tank_drive_node, m_inertial_sensor_node, Rotation2Dd(1.2)));
    
    // m_turn_1->AddAction(new IntakeAction(m_conveyor_node));
    // m_turn_1->AddAction(new BottomConveyorAction(m_conveyor_node, false));
    // m_turn_1->AddAction(new TopConveyorAction(m_conveyor_node, ConveyorNode::HOLDING));
    
    // m_deploy_node->AddNext(m_turn_1);    

    // m_forward_1 = new AutonNode(5, new DriveAction(m_tank_drive_node, 45.6, 45.6, 150));
    // m_forward_1->AddAction(new OpenIntakesAction(m_conveyor_node));
    // m_turn_1->AddNext(m_forward_1);

    // m_wait_1 = new AutonNode(1, new WaitAction(1));
    // m_wait_1->AddAction(new OpenIntakesAction(m_conveyor_node, false));

    // m_forward_1->AddNext(m_wait_1);

    // m_reverse_1 = new AutonNode(3, new DriveAction(m_tank_drive_node, -20, -20, 100));
    // m_reverse_1->AddAction(new BottomConveyorAction(m_conveyor_node, false, 0));

    // m_wait_1->AddNext(m_reverse_1);

    // m_turn_2 = new AutonNode(3, new TurnToAngleAction(m_tank_drive_node, m_inertial_sensor_node, Rotation2Dd(3 * M_PI_4)));
    // m_reverse_1->AddNext(m_turn_2);
}