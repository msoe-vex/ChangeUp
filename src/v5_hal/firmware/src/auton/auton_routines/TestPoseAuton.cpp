#include "auton/auton_routines/TestPoseAuton.h"

TestPoseAuton::TestPoseAuton(IDriveNode* drive_node, OdometryNode* odom_node) : 
        Auton("Test Pose Node"), 
        m_drive_node(drive_node), 
        m_odom_node(odom_node) {

}

void TestPoseAuton::AddNodes() {
    Pose pose(Vector2d(0., 10.), Rotation2Dd(0.));

    m_pose_node = new AutonNode(10, new DriveToPoseAction(m_drive_node, m_odom_node, pose));
    Auton::AddFirstNode(m_pose_node);
}