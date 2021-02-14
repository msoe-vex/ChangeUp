#include "ProgrammingSkillsAuton.h"

ProgrammingSkillsAuton::ProgrammingSkillsAuton(TankDriveNode* tank_drive_node, OdometryNode* odom_node,
        ConveyorNode* conveyor_node, InertialSensorNode* inertial_sensor_node) : Auton("Programming Skills"), 
        m_tank_drive_node(tank_drive_node), m_odom_node(odom_node), m_inertial_sensor_node(inertial_sensor_node),
        m_conveyor_node(conveyor_node) {
    
}

void ProgrammingSkillsAuton::AddNodes() {
    m_deploy_node = new AutonNode(1, new DeployAction(m_conveyor_node));

    Auton::AddFirstNode(m_deploy_node);
    
    m_turn_1 = new AutonNode(5, new TurnToAngleAction(m_tank_drive_node, m_inertial_sensor_node, Rotation2Dd(1.)));
    
    m_turn_1->AddAction(new IntakeAction(m_conveyor_node));
    m_turn_1->AddAction(new BottomConveyorAction(m_conveyor_node, false));
    m_turn_1->AddAction(new TopConveyorAction(m_conveyor_node, ConveyorNode::HOLDING));
    
    m_deploy_node->AddNext(m_turn_1);    

    m_forward_1 = new AutonNode(5, new DriveAction(m_tank_drive_node, 46, 49.1, 40));
    m_forward_1->AddAction(new OpenIntakesAction(m_conveyor_node));
    m_turn_1->AddNext(m_forward_1);

    m_wait_1 = new AutonNode(1, new WaitAction(1));
    m_wait_1->AddAction(new OpenIntakesAction(m_conveyor_node, false));

    m_forward_1->AddNext(m_wait_1);

    m_reverse_1 = new AutonNode(3, new DriveAction(m_tank_drive_node, -14, 49.1, 40));

    m_wait_1->AddNext(m_reverse_1);

    m_turn_2 = new AutonNode(3, new TurnToAngleAction(m_tank_drive_node, m_inertial_sensor_node, Rotation2Dd(3.2 * M_PI_4)));
    m_turn_2->AddAction(new IntakeAction(m_conveyor_node, 0));
    m_reverse_1->AddNext(m_turn_2);

    // Move toward corner goal
    m_forward_3 = new AutonNode(2, new DriveAction(m_tank_drive_node, 22, 150, 40));
    m_forward_3->AddAction(new IntakeAction(m_conveyor_node, 12000));
    m_forward_3->AddAction(new BottomConveyorAction(m_conveyor_node, false, 0));
    m_turn_2->AddNext(m_forward_3);

    m_score_1 = new AutonNode(1.5, new TopConveyorAction(m_conveyor_node, ConveyorNode::SCORING, 1.5));
    m_forward_3->AddNext(m_score_1);

    m_descore_1 = new AutonNode(0.2, new WaitAction(0.2));
    m_descore_1->AddAction(new TopConveyorAction(m_conveyor_node, ConveyorNode::STOPPED));
    m_descore_1->AddAction(new IntakeAction(m_conveyor_node, 12000));
    m_descore_1->AddAction(new BottomConveyorAction(m_conveyor_node, false, 12000));
    m_score_1->AddNext(m_descore_1);

    m_reverse_2 = new AutonNode(2, new DriveAction(m_tank_drive_node, -31, 150, 40));
    m_descore_1->AddNext(m_reverse_2);
    
    m_outtake_1 = new AutonNode(1, new IntakeAction(m_conveyor_node, -10000, 1));
    m_outtake_1->AddAction(new BottomConveyorAction(m_conveyor_node, false, -10000, 1));
    m_outtake_1->AddAction(new TopConveyorAction(m_conveyor_node, ConveyorNode::REVERSE, 1));
    m_reverse_2->AddNext(m_outtake_1);

    m_turn_3 = new AutonNode(1.5, new TurnToAngleAction(m_tank_drive_node, m_inertial_sensor_node, Rotation2Dd(-0.9)));
    m_outtake_1->AddNext(m_turn_3);

    m_forward_4 = new AutonNode(3, new DriveAction(m_tank_drive_node, 37.8, 200, 40));
    m_forward_4->AddAction(new OpenIntakesAction(m_conveyor_node));
    m_forward_4->AddAction(new IntakeAction(m_conveyor_node));
    m_forward_4->AddAction(new BottomConveyorAction(m_conveyor_node, false));
    m_forward_4->AddAction(new TopConveyorAction(m_conveyor_node, ConveyorNode::HOLDING));
    
    m_delay_close_intakes_1 = new AutonNode(1, new WaitAction(1));
    m_close_intakes = new AutonNode(0.5, new OpenIntakesAction(m_conveyor_node, false));
    m_delay_close_intakes_1->AddNext(m_close_intakes);

    m_turn_3->AddNext(m_forward_4);
    m_turn_3->AddNext(m_delay_close_intakes_1);

    m_turn_4 = new AutonNode(1.5, new TurnToAngleAction(m_tank_drive_node, m_inertial_sensor_node, Rotation2Dd(2.9)));
    m_forward_4->AddNext(m_turn_4);

    m_forward_5 = new AutonNode(2, new DriveAction(m_tank_drive_node, 33, 150, 40));
    m_forward_5->AddAction(new OpenIntakesAction(m_conveyor_node));
    m_turn_4->AddNext(m_forward_5);

    m_score_2 = new AutonNode(1.5, new TopConveyorAction(m_conveyor_node, ConveyorNode::SCORING, 1.5));
    m_forward_5->AddNext(m_score_2);

    m_descore_2 = new AutonNode(1.5, new BottomConveyorAction(m_conveyor_node, true, 0, 1.5));
    m_descore_2->AddAction(new OpenIntakesAction(m_conveyor_node, false));
    m_score_2->AddNext(m_descore_2);

    m_reverse_3 = new AutonNode(1.5, new DriveAction(m_tank_drive_node, -9, 150, 40));
    m_reverse_3->AddAction(new BottomConveyorAction(m_conveyor_node, true, 12000));
    m_reverse_3->AddAction(new TopConveyorAction(m_conveyor_node, ConveyorNode::REVERSE));
    m_descore_2->AddNext(m_reverse_3);

    m_turn_5 = new AutonNode(1, new TurnToAngleAction(m_tank_drive_node, m_inertial_sensor_node, Rotation2Dd(-1.3)));
    m_reverse_3->AddNext(m_turn_5);

    m_eject_1 = new AutonNode(2, new BottomConveyorAction(m_conveyor_node, true, 12000, 2));
    m_eject_1->AddAction(new TopConveyorAction(m_conveyor_node, ConveyorNode::REVERSE));
    m_turn_5->AddNext(m_eject_1);

    m_forward_6 = new AutonNode(2.5, new DriveAction(m_tank_drive_node, 28, 150, 40));
    m_forward_6->AddAction(new OpenIntakesAction(m_conveyor_node));
    m_forward_6->AddAction(new BottomConveyorAction(m_conveyor_node, false, 12000));
    m_forward_6->AddAction(new TopConveyorAction(m_conveyor_node, ConveyorNode::HOLDING));

    m_eject_1->AddNext(m_forward_6);

    m_capture_ball_1 = new AutonNode(1, new WaitAction(1));
    m_capture_ball_1->AddAction(new OpenIntakesAction(m_conveyor_node, false));

    m_forward_6->AddNext(m_capture_ball_1);

    m_forward_7 = new AutonNode(1, new DriveAction(m_tank_drive_node, 8, 150, 40));
    m_capture_ball_1->AddNext(m_forward_7);

    m_turn_6 = new AutonNode(1, new TurnToAngleAction(m_tank_drive_node, m_inertial_sensor_node, Rotation2Dd(-2.35)));
    m_turn_6->AddAction(new BottomConveyorAction(m_conveyor_node, false, 0));

    m_forward_7->AddNext(m_turn_6);

    m_forward_8 = new AutonNode(1.5, new DriveAction(m_tank_drive_node, 24, 150, 40));
    m_turn_6->AddNext(m_forward_8);

    m_score_3 = new AutonNode(1.5, new TopConveyorAction(m_conveyor_node, ConveyorNode::SCORING, 1.5));
    m_forward_8->AddNext(m_score_3);

    m_descore_3 = new AutonNode(0.75, new BottomConveyorAction(m_conveyor_node, false, 12000, 0.75));
    m_score_3->AddNext(m_descore_3);

    m_forward_9 = new AutonNode(1.5, new DriveAction(m_tank_drive_node, -14, 150, 40));
    m_descore_3->AddNext(m_forward_9);
=======
    m_deploy_node = new AutonNode(5, new DeployAction(m_conveyor_node));

    Auton::AddFirstNode(m_deploy_node);

    m_path_1 = new AutonNode(10, new FollowPathAction(m_tank_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Path1")));

    m_deploy_node->AddNext(m_path_1);
    m_path_1->AddAction(new IntakeAction(m_conveyor_node));
    m_path_1->AddAction(new BottomConveyorAction(m_conveyor_node, false));
    m_path_1->AddAction(new TopConveyorAction(m_conveyor_node, ConveyorNode::HOLDING));

    // Configure the position for the first path
    Waypoint first_waypoint = PathManager::GetInstance()->GetPath("Path1").getFirstWaypoint();
    Vector2d initial_pos(first_waypoint.position.getY(), -first_waypoint.position.getX());
    Rotation2Dd initial_rot(M_PI);
    Pose initial_pose(initial_pos, initial_rot);
    m_odom_node->setCurrentPose(initial_pose);

    m_path_2 = new AutonNode(10, new FollowPathAction(m_tank_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Path2"), true));

    m_path_1->AddNext(m_path_2);
    m_path_2->AddAction(new IntakeAction(m_conveyor_node, 0));
    m_path_2->AddAction(new BottomConveyorAction(m_conveyor_node, false, 0));
>>>>>>> Added basic node sequencing for auton
}