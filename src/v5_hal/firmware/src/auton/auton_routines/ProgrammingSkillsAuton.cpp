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


    AutonNode* path_goal7ToGoal5 = new AutonNode(3.5, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("V2-Goal7ToGoal5"), true));
    descoreGoal7->AddNext(path_goal7ToGoal5);

    addActionsToPath_Goal7ToGoal5(descoreGoal7, m_intake_node, m_conveyor_node);

    AutonNode* descoreGoal5 = getMiddleGoalScoringSequence(path_goal7ToGoal5, m_intake_node, m_conveyor_node);

    AutonNode* backAwayAndPrayToGod = new AutonNode(7.0, new FollowPathAction(m_drive_node, m_odom_node, PathManager::GetInstance()->GetPath("V2-BackUp")));
    descoreGoal5->AddNext(backAwayAndPrayToGod);
}