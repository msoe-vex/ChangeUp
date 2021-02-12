#include "Actions/TurnToAngleAction.h"

TurnToAngleAction::TurnToAngleAction(TankDriveNode* tank_drive, InertialSensorNode* inertial_sensor, Eigen::Rotation2Dd angle) : 
    m_tank_drive(tank_drive), m_inertial_sensor(inertial_sensor), m_angle(angle) {

}

void actionInit() {
    m_timer.Start();
}

actionStatus action() {

}

void actionEnd() {

}