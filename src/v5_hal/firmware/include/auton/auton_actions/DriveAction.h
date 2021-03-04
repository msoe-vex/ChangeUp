#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "nodes/subsystems/drivetrain_nodes/TankDriveNode.h"

class DriveAction : public AutonAction {
private:
    TankDriveNode* m_tank_drive_node;
    Timer m_timer;
    double m_distance;
    double m_max_velocity;
    double m_max_accel;
    int m_actual_left_distance;
    int m_actual_right_distance;
    double m_lastSpeed;
    double m_lastTime;
    double m_feedForward;

public:
    DriveAction(TankDriveNode* tank_drive_node, double distance, double max_velocity, 
        double max_accel);

    void ActionInit();
    actionStatus Action();
    void ActionEnd();
};