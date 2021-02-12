#pragma once

#include "Auton.h"
#include "DataNodes/TankDriveNode.h"
#include "adaptive_pursuit_controller/AdaptivePursuit.h"

class FollowPathAction : AutonAction {
private:
    TankDriveNode* m_tank_drive;
    AdaptivePursuit m_controller;
public:
    FollowPathAction(TankDriveNode* tank_drive, Path path, double maxAccel, double wheelDiameter, bool reversed = false,
                     double fixedLookahead = 10, double pathCompletionTolerance = 0.1, bool gradualStop = true);

    void actionInit();
    actionStatus action();
    void actionEnd();
};