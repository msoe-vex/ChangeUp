#include "ProgrammingSkillsAuton.h"

ProgrammingSkillsAuton::ProgrammingSkillsAuton(TankDriveNode* tankDriveNode, OdometryNode* odom_node) : Auton("Programming Skills"), 
    m_tankDriveNode(tankDriveNode), m_odom_node(odom_node) {
    

}

void ProgrammingSkillsAuton::AddNodes() {
    testNode = new AutonNode(10, new FollowPathAction(m_tankDriveNode, m_odom_node, PathManager::GetInstance()->GetPath("TestPath")));
    Auton::AddFirstNode(testNode);
}