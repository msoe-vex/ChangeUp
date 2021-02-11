#pragma once

#include "api.h"
#include "Auton.h"
#include "DataNodes/TankDriveNode.h"

class DriveAction : public AutonAction {
private:
    TankDriveNode* m_tank_drive_node;
    double m_distance_left;
    double m_distance_right;
    int m_max_velocity;
    int m_actual_left_distance;
    int m_actual_right_distance;

public:
    DriveAction(TankDriveNode* tank_drive_node, double distance_left, double distance_right, int max_velocity);

    void actionInit();
    actionStatus action();
    void actionEnd();
};