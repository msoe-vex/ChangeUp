#include "auton/auton_routines/TestPathAuton.h"

TestPathAuton::TestPathAuton(IDriveNode* drive_node, OdometryNode* odom_node) : 
        Auton("Test Path Node"), 
        m_drive_node(drive_node), 
        m_odom_node(odom_node),
        m_path_manager(PathManager::GetInstance()) {

}

void TestPathAuton::AddNodes() {
    Path path = m_path_manager->GetPath("TestPath");

    m_path_node = new AutonNode(20, new FollowPathAction(m_drive_node, m_odom_node, path, true));

    Auton::AddFirstNode(m_path_node);
}