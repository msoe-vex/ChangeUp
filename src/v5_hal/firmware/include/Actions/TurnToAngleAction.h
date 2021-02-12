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
    InertialSensorNode* m_inertial_sensor
    Eigen::Rotation2Dd m_angle;

public:
    TurnToAngleAction(TankDriveNode* tank_drive, InertialSensorNode* inertial_sensor, Eigen::Rotation2Dd angle);

    void actionInit();

    actionStatus action();

    void actionEnd();

};