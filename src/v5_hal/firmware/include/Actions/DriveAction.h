#pragma once

#include "api.h"
#include "Auton.h"
#include "Timer.h"
#include "DataNodes/TankDriveNode.h"

class DriveAction : public DriveAction {
private:
    TankDriveNode* m_tank_drive_node;
    Timer m_timer;
public:
    DriveAction(TankDriveNode* tank_drive_node, double distance);

    void actionInit();
    actionStatus action();
    void actionEnd();
};