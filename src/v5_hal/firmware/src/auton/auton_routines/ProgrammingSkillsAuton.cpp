#include "auton/auton_routines/ProgrammingSkillsAuton.h"

ProgrammingSkillsAuton::ProgrammingSkillsAuton(IDriveNode* drive_node, OdometryNode* odom_node,
        ConveyorNode* conveyor_node, IntakeNode* intake_node, InertialSensorNode* inertial_sensor_node) : Auton("Programming Skills"), 
        m_drive_node(drive_node), 
        m_odom_node(odom_node), 
        m_inertial_sensor_node(inertial_sensor_node),
        m_conveyor_node(conveyor_node),
        m_intake_node(intake_node) {
    
}

void ProgrammingSkillsAuton::AddNodes() {
    // Define Auton nodes, actions, and sequences here
    m_deploy_node = new AutonNode(1, new DeployAction(m_conveyor_node, m_intake_node));

    // (╯°□°)╯︵ ┻━┻
    AutonNode* path1 = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("TestPath")));
    path1->AddAction(new IntakeAction(m_intake_node, MAX_MOTOR_VOLTAGE));
    path1->AddAction(new OpenIntakesAction(m_intake_node, true));
    path1->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::HOLDING));

    AutonNode* path2 = new AutonNode(10, new TurnToAngleAction(m_drive_node, m_inertial_sensor_node, Eigen::Rotation2Dd(toRadians(75))));
    path2->AddAction(new IntakeAction(m_intake_node, 0));

    AutonNode* path3 = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Path3")));

    path2->AddNext(path3);
    path1->AddNext(path2);
    m_deploy_node->AddNext(path1);

    AutonNode* goal_sequence1 = getGoalScoringSequence(path3, m_intake_node, m_conveyor_node);

    AutonNode* path4 = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Path4")));
    path4->AddAction(new IntakeAction(m_intake_node, -MAX_MOTOR_VOLTAGE));
    path4->AddAction(new OpenIntakesAction(m_intake_node, true));
    path4->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::REVERSE));

    AutonNode* path5 = new AutonNode(10, new TurnToAngleAction(m_drive_node, m_inertial_sensor_node, Eigen::Rotation2Dd(toRadians(135))));
    path5->AddAction(new IntakeAction(m_intake_node, MAX_MOTOR_VOLTAGE));
    path5->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::HOLDING));

    AutonNode* forbidden_path = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("ForbiddenPath")));

    AutonNode* forbidden_turn = new AutonNode(10, new TurnToAngleAction(m_drive_node, m_inertial_sensor_node, Eigen::Rotation2Dd(toRadians(-90))));

    AutonNode* path6 = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Path6")));
    
    AutonNode* path7 = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Path7")));
    path7->AddAction(new IntakeAction(m_intake_node, 0));

    path6->AddNext(path7);
    forbidden_turn->AddNext(path6);
    forbidden_path->AddNext(forbidden_turn);
    path5->AddNext(forbidden_path);
    path4->AddNext(path5);
    goal_sequence1->AddNext(path4);

    AutonNode* goal_sequence2 = getGoalScoringSequence(path7, m_intake_node, m_conveyor_node);

    AutonNode* path8 = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Path8")));

    AutonNode* path9 = new AutonNode(10, new TurnToAngleAction(m_drive_node, m_inertial_sensor_node, Eigen::Rotation2Dd(toRadians(-45))));
    path9->AddAction(new OpenIntakesAction(m_intake_node, true));
    path9->AddAction(new IntakeAction(m_intake_node, -MAX_MOTOR_VOLTAGE, 1.0));
    path9->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::REVERSE));

    AutonNode* path11 = new AutonNode(10, new TurnToAngleAction(m_drive_node, m_inertial_sensor_node, Eigen::Rotation2Dd(toRadians(165))));
    
    AutonNode* path12 = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Path12")));
    path5->AddAction(new IntakeAction(m_intake_node, MAX_MOTOR_VOLTAGE));
    path5->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::HOLDING));
    
    AutonNode* path13 = new AutonNode(10, new TurnToAngleAction(m_drive_node, m_inertial_sensor_node, Eigen::Rotation2Dd(toRadians(-75))));

    AutonNode* path14 = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Path14")));
    path14->AddAction(new IntakeAction(m_intake_node, 0));

    path13->AddNext(path14);
    path12->AddNext(path13);
    path11->AddNext(path12);
    path9->AddNext(path11);
    path8->AddNext(path9);
    goal_sequence2->AddNext(path8);

    AutonNode* goal_sequence3 = getGoalScoringSequence(path14, m_intake_node, m_conveyor_node);

    AutonNode* path15 = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("FinalPath")));
    path15->AddAction(new IntakeAction(m_intake_node, -MAX_MOTOR_VOLTAGE));
    path15->AddAction(new OpenIntakesAction(m_intake_node, true));
    path15->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::REVERSE));

    // AutonNode* path16 = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, Path()));
    // path16->AddAction(new IntakeAction(m_intake_node, MAX_MOTOR_VOLTAGE));
    // path16->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::HOLDING));

    // AutonNode* path17 = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, Path()));
    
    // AutonNode* path18 = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, Path()));
    
    // AutonNode* path19 = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, Path()));
    
    // AutonNode* path20 = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, Path()));
    // path20->AddAction(new IntakeAction(m_intake_node, 0));

    // path19->AddNext(path20);
    // path18->AddNext(path19);
    // path17->AddNext(path18);
    // path16->AddNext(path17);
    // path15->AddNext(path16);
    goal_sequence3->AddNext(path15);

    // AutonNode* goal_sequence4 = getGoalScoringSequence(path20, m_intake_node, m_conveyor_node);

	// Sequence nodes here
    Auton::AddFirstNode(m_deploy_node);
}