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

    AutonNode* path_deployWall2ToGoal4 = new AutonNode(1.75, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("DeployWall2ToGoal4"), true));
    deploy->AddNext(path_deployWall2ToGoal4);

    AutonNode* getPreloadToTop = new AutonNode(0.6, new WaitAction(0.6));
    getPreloadToTop->AddAction(new IntakeAction(m_intake_node, MAX_MOTOR_VOLTAGE));
    getPreloadToTop->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::HOLDING));
    deploy->AddNext(getPreloadToTop);

    AutonNode* stopBottomRollerBeforeGoal4 = new AutonNode(0.1, new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::HOLDING_TOP));
    getPreloadToTop->AddNext(stopBottomRollerBeforeGoal4);

    AutonNode* descoreGoal4 = getSingleGoalScoringSequence(path_deployWall2ToGoal4, m_intake_node, m_conveyor_node);
    // AutonNode* scoreSingleBall = new AutonNode(5.0, new ScoreSingleBallAction(m_conveyor_node, true));
    // path1->AddNext(scoreSingleBall);

    AutonNode* path_goal4ToGoal1 = new AutonNode(6.0, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("V2-Goal4ToGoal1")));
    descoreGoal4->AddNext(path_goal4ToGoal1);

    addActionsToPath_Goal4ToGoal1(descoreGoal4, m_intake_node, m_conveyor_node);

    AutonNode* descoreGoal1 = getDoubleGoalScoringSequence(path_goal4ToGoal1, m_intake_node, m_conveyor_node);


    AutonNode* path_goal1ToGoal2 = new AutonNode(6.5, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("V2-Goal1ToGoal2")));
    descoreGoal1->AddNext(path_goal1ToGoal2);

    addActionsToPath_Goal1ToGoal2(descoreGoal1, m_intake_node, m_conveyor_node);

    AutonNode* descoreGoal2 = getDoubleGoalScoringSequence(path_goal1ToGoal2, m_intake_node, m_conveyor_node);


    AutonNode* path_goal2ToGoal3 = new AutonNode(6.0, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("V2-Goal2ToGoal3")));
    descoreGoal2->AddNext(path_goal2ToGoal3);

    addActionsToPath_Goal2ToGoal3(descoreGoal2, m_intake_node, m_conveyor_node);

    AutonNode* descoreGoal3 = getDoubleGoalScoringSequence(path_goal2ToGoal3, m_intake_node, m_conveyor_node);


    AutonNode* path_goal3ToGoal6 = new AutonNode(5.0, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("V2-Goal3ToGoal6"), true));
    descoreGoal3->AddNext(path_goal3ToGoal6);

    addActionsToPath_Goal3ToGoal6(descoreGoal3, m_intake_node, m_conveyor_node);

    AutonNode* descoreGoal6 = getSingleGoalScoringSequence(path_goal3ToGoal6, m_intake_node, m_conveyor_node);


    AutonNode* path_goal6ToGoal9 = new AutonNode(5.0, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("V2-Goal6ToGoal9"), true));
    descoreGoal6->AddNext(path_goal6ToGoal9);

    addActionsToPath_Goal6ToGoal9(descoreGoal6, m_intake_node, m_conveyor_node);

    AutonNode* descoreGoal9 = getDoubleGoalScoringSequence(path_goal6ToGoal9, m_intake_node, m_conveyor_node);


    AutonNode* path_goal9ToGoal8 = new AutonNode(6.0, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("V2-Goal9ToGoal8"), true));
    descoreGoal9->AddNext(path_goal9ToGoal8);

    addActionsToPath_Goal9ToGoal8(descoreGoal9, m_intake_node, m_conveyor_node);

    AutonNode* descoreGoal8 = getDoubleGoalScoringSequence(path_goal9ToGoal8, m_intake_node, m_conveyor_node);


    AutonNode* path_goal8ToGoal7 = new AutonNode(5.5, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("V2-Goal8ToGoal7"), true));
    descoreGoal8->AddNext(path_goal8ToGoal7);

    addActionsToPath_Goal8ToGoal7(descoreGoal8, m_intake_node, m_conveyor_node);

    AutonNode* descoreGoal7 = getDoubleGoalScoringSequence(path_goal8ToGoal7, m_intake_node, m_conveyor_node);


    AutonNode* path_goal7ToGoal5 = new AutonNode(7.0, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("V2-Goal7ToGoal5"), true));
    descoreGoal7->AddNext(path_goal7ToGoal5);

    addActionsToPath_Goal7ToGoal5(descoreGoal7, m_intake_node, m_conveyor_node);

    // AutonNode* backUp = new AutonNode(2.0, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("V2-BackUp"), true));
    // path_goal7ToGoal5->AddNext(backUp);

    // AutonNode* flipDownGoalPlate = new AutonNode(0.5, new WaitAction(0.5));
    // flipDownGoalPlate->AddAction(new LiftGoalPlateAction(m_intake_node, false));
    // backUp->AddNext(flipDownGoalPlate);

    // AutonNode* driveForwardToGoal = new AutonNode(7.0, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("V2-Foward")));
    // flipDownGoalPlate->AddNext(driveForwardToGoal);

    AutonNode* descoreGoal5 = getMiddleGoalScoringSequence(path_goal7ToGoal5, m_intake_node, m_conveyor_node);

    AutonNode* backAwayAndPrayToGod = new AutonNode(7.0, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("V2-BackUp")));
    descoreGoal5->AddNext(backAwayAndPrayToGod);

    // AutonNode* path_goal4ToGoal1 = new AutonNode(4.75, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Goal4ToGoal1")));
    // descoreGoal4->AddNext(path_goal4ToGoal1);

    // AutonNode* delayedOuttakeGoal4 = new AutonNode(1.2, new WaitAction(1.2));
    // descoreGoal4->AddNext(delayedOuttakeGoal4);
    
    // AutonNode* outtakeGoal4 = new AutonNode(1.0, new IntakeAction(m_intake_node, -MAX_MOTOR_VOLTAGE, 1.0));
    // outtakeGoal4->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::REVERSE));
    // delayedOuttakeGoal4->AddNext(outtakeGoal4);

    // AutonNode* delayedOpenIntakeAfterGoal4 = new AutonNode(0.45, new WaitAction(0.45));
    // delayedOpenIntakeAfterGoal4->AddAction(new IntakeAction(m_intake_node));
    // delayedOpenIntakeAfterGoal4->AddAction(new OpenIntakesAction(m_intake_node));
    // delayedOpenIntakeAfterGoal4->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::HOLDING));
    // outtakeGoal4->AddNext(delayedOpenIntakeAfterGoal4);

    // AutonNode* closeIntakesOnBallAfterGoal4 = new AutonNode(0.85, new WaitAction(0.85));
    // closeIntakesOnBallAfterGoal4->AddAction(new OpenIntakesAction(m_intake_node, false));
    // delayedOpenIntakeAfterGoal4->AddNext(closeIntakesOnBallAfterGoal4);

    // AutonNode* stopBottomRollerBeforeGoal1 = new AutonNode(0.1, new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::HOLDING_TOP));
    // closeIntakesOnBallAfterGoal4->AddNext(stopBottomRollerBeforeGoal1);

    // AutonNode* descoreGoal1 = getSingleGoalScoringSequence(path_goal4ToGoal1, m_intake_node, m_conveyor_node);
    // //AutonNode* scoreSingleBall2 = new AutonNode(5.0, new ScoreSingleBallAction(m_conveyor_node, true));

    // AutonNode* path_goal1ToGoal2 = new AutonNode(7.5, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Goal1ToGoal2")));
    // descoreGoal1->AddNext(path_goal1ToGoal2);
    
    // AutonNode* delayedOuttakeAfterGoal1 = new AutonNode(1.0, new WaitAction(1.0));
    // descoreGoal1->AddNext(delayedOuttakeAfterGoal1);

    // AutonNode* outtakeAfterGoal1 = new AutonNode(1.0, new IntakeAction(m_intake_node, -MAX_MOTOR_VOLTAGE, 1.0));
    // outtakeAfterGoal1->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::REVERSE));
    // delayedOuttakeAfterGoal1->AddNext(outtakeAfterGoal1);

    // AutonNode* openIntakesForBallAfterGoal1 = new AutonNode(0.5, new WaitAction(0.5));
    // openIntakesForBallAfterGoal1->AddAction(new OpenIntakesAction(m_intake_node));
    // openIntakesForBallAfterGoal1->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::HOLDING));
    // outtakeAfterGoal1->AddNext(openIntakesForBallAfterGoal1);

    // AutonNode* waitToCloseIntakesOnBallAfterGoal1 = new AutonNode(0.25, new WaitAction(0.25));
    // waitToCloseIntakesOnBallAfterGoal1->AddAction(new IntakeAction(m_intake_node));
    // openIntakesForBallAfterGoal1->AddNext(waitToCloseIntakesOnBallAfterGoal1);

    // AutonNode* closeIntakesOnBallAfterGoal1 = new AutonNode(0.35, new WaitAction(0.35));
    // closeIntakesOnBallAfterGoal1->AddAction(new OpenIntakesAction(m_intake_node, false));
    // waitToCloseIntakesOnBallAfterGoal1->AddNext(closeIntakesOnBallAfterGoal1);

    // AutonNode* openIntakesOnBallBeforeGoal2 = new AutonNode(1.0, new WaitAction(1.0));
    // openIntakesOnBallBeforeGoal2->AddAction(new OpenIntakesAction(m_intake_node));
    // closeIntakesOnBallAfterGoal1->AddNext(openIntakesOnBallBeforeGoal2);

    // AutonNode* closeIntakesOnBallBeforeGoal2 = new AutonNode(0.1, new OpenIntakesAction(m_intake_node, false));
    // openIntakesOnBallBeforeGoal2->AddNext(closeIntakesOnBallBeforeGoal2);

    // AutonNode* descoreGoal2 = getDoubleGoalScoringSequence(path_goal1ToGoal2, m_intake_node, m_conveyor_node);

    // // Temp
    // //AutonNode* descoreGoal2 = getDoubleGoalScoringSequence(deploy, m_intake_node, m_conveyor_node);

    // // Reset pose on goal 2 before continuing
    // AutonNode* path_goal2ToGoal3 = new AutonNode(7.0, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("Goal2ToGoal3"), true));
    // descoreGoal2->AddNext(path_goal2ToGoal3);

    // AutonNode* delayAfterGoal2 = new AutonNode(0.5, new WaitAction(0.5));
    // descoreGoal2->AddNext(delayAfterGoal2);

    // AutonNode* ejectBallsFromGoal2 = new AutonNode(1.0, new IntakeAction(m_intake_node, -MAX_MOTOR_VOLTAGE, 1.0));
    // ejectBallsFromGoal2->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::REVERSE));
    // delayAfterGoal2->AddNext(ejectBallsFromGoal2);

    // AutonNode* intakeAfterGoal2Eject = new AutonNode(1.0, new WaitAction(1.0));
    // intakeAfterGoal2Eject->AddAction(new OpenIntakesAction(m_intake_node));
    // intakeAfterGoal2Eject->AddAction(new IntakeAction(m_intake_node));
    // intakeAfterGoal2Eject->AddAction(new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::HOLDING));
    // ejectBallsFromGoal2->AddNext(intakeAfterGoal2Eject);

    // AutonNode* closeIntakeAfterGoal2 = new AutonNode(1.25, new WaitAction(1.25));
    // closeIntakeAfterGoal2->AddAction(new OpenIntakesAction(m_intake_node, false));
    // intakeAfterGoal2Eject->AddNext(closeIntakeAfterGoal2);

    // AutonNode* openIntakeForBallBeforeGoal3 = new AutonNode(0.5, new WaitAction(0.5));
    // openIntakeForBallBeforeGoal3->AddAction(new OpenIntakesAction(m_intake_node));
    // closeIntakeAfterGoal2->AddNext(openIntakeForBallBeforeGoal3);

    // AutonNode* closeIntakeForBallBeforeGoal3 = new AutonNode(0.5, new WaitAction(0.5));
    // closeIntakeForBallBeforeGoal3->AddAction(new OpenIntakesAction(m_intake_node, false));
    // openIntakeForBallBeforeGoal3->AddNext(closeIntakeForBallBeforeGoal3);

    // AutonNode* stopBottomRollerForGoal3 = new AutonNode(0.1, new UpdateConveyorStateAction(m_conveyor_node, ConveyorNode::HOLDING_TOP));
    // closeIntakeForBallBeforeGoal3->AddNext(stopBottomRollerForGoal3);

    // AutonNode* descoreGoal3 = getDoubleGoalScoringSequence(path_goal2ToGoal3, m_intake_node, m_conveyor_node);
}