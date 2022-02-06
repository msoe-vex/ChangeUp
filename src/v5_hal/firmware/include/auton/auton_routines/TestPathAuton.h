#include "auton/Auton.h"
#include "nodes/subsystems/drivetrain_nodes/IDriveNode.h"
#include "nodes/odometry_nodes/OdometryNode.h"
#include "auton/auton_actions/TurnToAngleAction.h"
#include "auton/auton_actions/HolonomicFollowPathAction.h"
#include "pathing/PathManager.h"
#include "pathing/Path.h"
#include "eigen/Eigen/Dense"

class TestPathAuton : public Auton {
public:
    TestPathAuton(IDriveNode* drive_node, OdometryNode* odom_node);

    void AddNodes();

private:
    IDriveNode* m_drive_node;
    OdometryNode* m_odom_node;
    AutonNode* m_path_node;
    PathManager* m_path_manager;
};