#include "auton/auton_routines/TestPathAuton.h"

TestPathAuton::TestPathAuton(IDriveNode* drive_node, OdometryNode* odom_node) : 
        Auton("Test Path Node"), 
        m_drive_node(drive_node), 
        m_odom_node(odom_node),
        m_path_manager(PathManager::GetInstance()) {

}

void TestPathAuton::AddNodes() {
    Path path = m_path_manager->GetPath("TestPath");
    Pose start_pose = path.getPathPoints().at(0).getPose();

    m_odom_node->setCurrentPose(start_pose);
    
    Logger::logInfo("Set current pose of the robot to x:" + std::to_string(start_pose.position.x()) + 
                    " | y:" + std::to_string(start_pose.position.y()) + " | angle:" + std::to_string(start_pose.angle.angle()));

    m_path_node = new AutonNode(10, new FollowPathAction(m_drive_node, m_odom_node, path, true));

    Auton::AddFirstNode(m_path_node);
}