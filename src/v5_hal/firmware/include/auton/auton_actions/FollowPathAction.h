#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "nodes/subsystems/drivetrain_nodes/IDriveNode.h"
#include "nodes/odometry_nodes/OdometryNode.h"
#include "pursuit/HolonomicPursuit.h"
#include "math/Pose.h"
#include "adaptive_pursuit_controller/Logger.h"

class FollowPathAction : public AutonAction {
private:
    IDriveNode* m_drive_node;
    OdometryNode* m_odom_node;
    HolonomicPursuit m_holonomic_pursuit;
    Path m_path;
    bool m_reset_pose;
    Timer m_timer;

public:
    FollowPathAction(IDriveNode* drive_node, OdometryNode* odom_node, Path path, bool reset_pose=false);

    void ActionInit();

    actionStatus Action();

    void ActionEnd();
};