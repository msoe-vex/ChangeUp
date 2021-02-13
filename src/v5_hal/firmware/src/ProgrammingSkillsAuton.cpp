#include "ProgrammingSkillsAuton.h"

ProgrammingSkillsAuton::ProgrammingSkillsAuton(TankDriveNode * tankDriveNode) : Auton("Programming Skills"), 
    m_tankDriveNode(tankDriveNode) {
    

}

void ProgrammingSkillsAuton::AddNodes() {
    testNode = new AutonNode(10, new FollowPathAction(m_tankDriveNode, PathManager::GetInstance()->GetPath("TestPath")));
    Auton::AddFirstNode(testNode);
}