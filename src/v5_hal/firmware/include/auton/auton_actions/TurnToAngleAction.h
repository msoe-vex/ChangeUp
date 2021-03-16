#pragma once

#include "api.h"
#include "auton/Auton.h"
#include "util/Timer.h"
#include "util/Constants.h"
#include "nodes/subsystems/drivetrain_nodes/AbstractDriveNode.h"
#include "nodes/sensor_nodes/InertialSensorNode.h"
#include "eigen/Eigen/Dense"

class TurnToAngleAction : public AutonAction {
private:
    AbstractDriveNode* m_drive_node;
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
    TurnToAngleAction(AbstractDriveNode* drive_node, InertialSensorNode* inertial_sensor, 
        Eigen::Rotation2Dd target_angle);

    void ActionInit();

    actionStatus Action();

    void ActionEnd();
};