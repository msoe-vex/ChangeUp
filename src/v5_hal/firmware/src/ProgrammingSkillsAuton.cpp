#include "ProgrammingSkillsAuton.h"

ProgrammingSkillsAuton::ProgrammingSkillsAuton(TankDriveNode* tankDriveNode, OdometryNode* odom_node,
 ConveyorNode* conveyor_node) : Auton("Programming Skills"), m_tankDriveNode(tankDriveNode), m_odom_node(odom_node),
 m_conveyor_node(conveyor_node) {
    
}

void ProgrammingSkillsAuton::AddNodes() {
    testNode = new AutonNode(10, new FollowPathAction(m_tankDriveNode, m_odom_node, PathManager::GetInstance()->GetPath("TestPath")));
    //testNode = new AutonNode(10, new DeployAction(m_conveyor_node));
    Auton::AddFirstNode(testNode);

    Waypoint first_waypoint = PathManager::GetInstance()->GetPath("TestPath").getFirstWaypoint();

    Vector2d initial_pos(first_waypoint.position.getX(), first_waypoint.position.getY());

    Rotation2Dd initial_rot(first_waypoint.rotation.getRadians());

    Pose initial_pose(initial_pos, initial_rot);

    m_odom_node->setCurrentPose(initial_pose);
}