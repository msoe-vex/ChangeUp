#pragma once

#include "auton/Auton.h"
#include "nodes/subsystems/drivetrain_nodes/TankDriveNode.h"
#include "nodes/odometry_nodes/OdometryNode.h"
#include "adaptive_pursuit_controller/AdaptivePursuit.h"

class FollowPathAction : public AutonAction {
private:
    TankDriveNode* m_tank_drive;
    OdometryNode* m_odom_node;

    AdaptivePursuit m_controller;

    string m_printString;

public:
    FollowPathAction(TankDriveNode* tank_drive, OdometryNode* odom_node, Path path, bool reversed = false, double wheelDiameter = 4.0625, 
        double fixedLookahead = 10, double pathCompletionTolerance = 0.25, bool gradualStop = true);

    void ActionInit();
    actionStatus Action();
    void ActionEnd();
};