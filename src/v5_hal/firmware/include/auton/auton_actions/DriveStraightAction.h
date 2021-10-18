#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "nodes/subsystems/drivetrain_nodes/IDriveNode.h"

class DriveStraightAction : public AutonAction {
private:
    IDriveNode* m_drive_node;
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
    DriveStraightAction(IDriveNode* drive_node, double distance, double max_velocity, 
        double max_accel);

    void ActionInit();
    actionStatus Action();
    void ActionEnd();
};