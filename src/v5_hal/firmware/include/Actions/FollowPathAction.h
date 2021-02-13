#pragma once

#include "Auton.h"
#include "DataNodes/TankDriveNode.h"
#include "adaptive_pursuit_controller/AdaptivePursuit.h"

class FollowPathAction : public AutonAction {
private:
    TankDriveNode* m_tank_drive;
    AdaptivePursuit m_controller;
public:
    FollowPathAction(TankDriveNode* tank_drive, Path path, double wheelDiameter = 4.0625, bool reversed = false,
                     double fixedLookahead = 10, double pathCompletionTolerance = 0.1, bool gradualStop = true);

    void ActionInit();
    actionStatus Action();
    void ActionEnd();
};