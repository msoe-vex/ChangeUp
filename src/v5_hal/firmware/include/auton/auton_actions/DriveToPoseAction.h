#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "nodes/subsystems/drivetrain_nodes/IDriveNode.h"
#include "nodes/odometry_nodes/OdometryNode.h"
#include "math/Pose.h"

class DriveToPoseAction : public AutonAction {
private:
    IDriveNode* m_drive_node;
    OdometryNode* m_odom_node;
    Timer m_timer;
    Pose m_end_pose;

public:
    DriveToPoseAction(IDriveNode* drive_node, OdometryNode* odom_node, Pose end_pose);

    void ActionInit();
    actionStatus Action();
    void ActionEnd();
};