#include "Actions/TurnToAngleAction.h"

TurnToAngleAction::TurnToAngleAction(TankDriveNode* tank_drive, InertialSensorNode* inertial_sensor, Eigen::Rotation2Dd target_angle) : 
    m_tank_drive(tank_drive), m_inertial_sensor(inertial_sensor), m_target_angle(target_angle) {

}

void TurnToAngleAction::actionInit() {
    m_timer.Start();
}

AutonAction::actionStatus TurnToAngleAction::action() {    
    Eigen::Rotation2Dd current_angle(m_inertial_sensor->getYaw());

    double delta_angle = (current_angle * m_target_angle.inverse()).smallestAngle();

    double turning_power = delta_angle / M_PI;

    double total_turning_power = kP * turning_power;
    
    if (total_turning_power > 1) {
        total_turning_power = 1;
    } else if (total_turning_power < -1) {
        total_turning_power = -1;
    }

    m_tank_drive->setDriveVoltage((int)(total_turning_power * 12000), (int)(total_turning_power * -12000));

    if (!m_inertial_sensor->isAtAngle(m_target_angle.angle())) {
        return CONTINUE;
    } else {
        return END;
    }
}

void TurnToAngleAction::actionEnd() {
    m_tank_drive->setDriveVoltage(0, 0);
}