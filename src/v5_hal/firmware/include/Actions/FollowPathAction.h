#pragma once

#include "Auton.h"
#include "DataNodes/TankDriveNode.h"
#include "DataNodes/OdometryNode.h"
#include "adaptive_pursuit_controller/AdaptivePursuit.h"

class FollowPathAction : public AutonAction {
private:
    TankDriveNode* m_tank_drive;
    OdometryNode* m_odom_node;

    AdaptivePursuit m_controller;

    string m_printString;

public:
    FollowPathAction(TankDriveNode* tank_drive, OdometryNode* odom_node, Path path, double wheelDiameter = 4.0625, 
        bool reversed = false, double fixedLookahead = 6, double pathCompletionTolerance = 0.25, bool gradualStop = true);

    void ActionInit();
    actionStatus Action();
    void ActionEnd();
};