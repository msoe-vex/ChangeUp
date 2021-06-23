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
    AutonNode* deploy = new AutonNode(0.5, new DeployAction(m_conveyor_node, m_intake_node));
    Auton::AddFirstNode(deploy);

    AutonNode* path1 = new AutonNode(3.5, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("DeployWall2ToGoal4"), true));
    deploy->AddNext(path1);
    path1->AddAction(new IntakeAction(m_intake_node, MAX_MOTOR_VOLTAGE));
    path1->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::HOLDING));

    AutonNode* descoreGoal4 = getSingleGoalScoringSequence(path1, m_intake_node, m_conveyor_node);
    // AutonNode* scoreSingleBall = new AutonNode(5.0, new ScoreSingleBallAction(m_conveyor_node, true));
    // path1->AddNext(scoreSingleBall);

    AutonNode* path2 = new AutonNode(5.5, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Goal4ToGoal1")));
    descoreGoal4->AddNext(path2);

    AutonNode* delayedOuttake = new AutonNode(1.0, new WaitAction(1.0));
    descoreGoal4->AddNext(delayedOuttake);
    
    AutonNode* outtake = new AutonNode(1.0, new IntakeAction(m_intake_node, -MAX_MOTOR_VOLTAGE, 1.0));
    outtake->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::REVERSE));
    delayedOuttake->AddNext(outtake);

    AutonNode* delayedOpenIntake = new AutonNode(1.0, new WaitAction(1.0));
    delayedOpenIntake->AddAction(new IntakeAction(m_intake_node));
    delayedOpenIntake->AddAction(new OpenIntakesAction(m_intake_node));
    delayedOpenIntake->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::HOLDING));
    outtake->AddNext(delayedOpenIntake);

    AutonNode* closeIntakes = new AutonNode(0.1, new OpenIntakesAction(m_intake_node, false));
    delayedOpenIntake->AddNext(closeIntakes);

    AutonNode* descoreGoal1 = getSingleGoalScoringSequence(path2, m_intake_node, m_conveyor_node);
    //AutonNode* scoreSingleBall2 = new AutonNode(5.0, new ScoreSingleBallAction(m_conveyor_node, true));

    AutonNode* path3 = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Goal1ToGoal2")));
    descoreGoal1->AddNext(path3);
    
}