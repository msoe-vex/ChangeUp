#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "nodes/subsystems/drivetrain_nodes/IDriveNode.h"
#include "nodes/odometry_nodes/OdometryNode.h"
#include "pursuit/holonomic_pursuit/HolonomicPursuit.h"
#include "math/Pose.h"
#include "util/Logger.h"

class TankFollowPathAction : public AutonAction {
private:
    IDriveNode* m_drive_node;
    OdometryNode* m_odom_node;
    HolonomicPursuit m_holonomic_pursuit;
    Path m_path;
    bool m_reset_pose;
    Timer m_timer;

public:
    TankFollowPathAction(IDriveNode* drive_node, OdometryNode* odom_node, Path path, bool reset_pose=false);

    void ActionInit();

    actionStatus Action();

    void ActionEnd();
};