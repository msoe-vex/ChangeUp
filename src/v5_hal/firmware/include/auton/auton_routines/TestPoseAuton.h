#include "auton/Auton.h"
#include "nodes/subsystems/drivetrain_nodes/IDriveNode.h"
#include "nodes/odometry_nodes/OdometryNode.h"
#include "auton/auton_actions/DriveToPoseAction.h"
#include "eigen/Eigen/Dense"

class TestPoseAuton : public Auton {
public:
    TestPoseAuton(IDriveNode* drive_node, OdometryNode* odom_node);

    void AddNodes();

private:
    IDriveNode* m_drive_node;
    OdometryNode* m_odom_node;
    AutonNode* m_pose_node;
};