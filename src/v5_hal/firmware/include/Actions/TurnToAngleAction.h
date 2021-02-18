#pragma once

#include "api.h"
#include "Auton.h"
#include "Timer.h"
#include "DataNodes/TankDriveNode.h"
#include "DataNodes/InertialSensorNode.h"
#include "eigen/Eigen/Dense"

class TurnToAngleAction : public AutonAction {
private:
    TankDriveNode* m_tank_drive;
    InertialSensorNode* m_inertial_sensor;
    Eigen::Rotation2Dd m_target_angle;
    Timer m_turn_timer;
    
    double m_total_error;
    double m_previous_error;
    double m_feed_forward;

    double kP = 3.;
    double kI = 0.;
    double kD = 0.;

public:
    TurnToAngleAction(TankDriveNode* tank_drive, InertialSensorNode* inertial_sensor, 
        Eigen::Rotation2Dd target_angle);

    void ActionInit();

    actionStatus Action();

    void ActionEnd();
};