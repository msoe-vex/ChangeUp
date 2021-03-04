#include "auton/auton_actions/TurnToAngleAction.h"

TurnToAngleAction::TurnToAngleAction(TankDriveNode* tank_drive, InertialSensorNode* inertial_sensor, 
        Eigen::Rotation2Dd target_angle) : 
        m_tank_drive(tank_drive), m_inertial_sensor(inertial_sensor), 
        m_target_angle(target_angle), m_feed_forward(0.1) {

}

void TurnToAngleAction::ActionInit() {
    m_turn_timer.Reset();
}

AutonAction::actionStatus TurnToAngleAction::Action() {    
    Eigen::Rotation2Dd current_angle(m_inertial_sensor->getYaw());

    // std::string log_str = "Gyro Angle: " + to_string(current_angle.angle()) + "\nTarget Angle: " + to_string(m_target_angle.angle());
    // m_tank_drive->m_handle->logwarn(log_str.c_str());

    double delta_angle = (m_target_angle.inverse() * current_angle).smallestAngle();

    double turning_power = delta_angle / M_PI * -1;

    // Integral
    m_total_error += turning_power;

    //Derivative
    double delta_error = turning_power - m_previous_error;

    // Update previous value
    m_previous_error = turning_power;

    double total_turning_power = (kP * turning_power) + (kI * m_total_error) + (kD * delta_error);

    total_turning_power = std::copysign(min(fabs(total_turning_power) + m_feed_forward, 1.0), total_turning_power);

    m_tank_drive->setDriveVoltage((int)(total_turning_power * -12000), (int)(total_turning_power * 12000));

    if (m_inertial_sensor->isAtAngle(m_target_angle) && m_turn_timer.Get() == 0) {
        m_turn_timer.Start();
    } else {
        m_turn_timer.Stop();
    }

    if (m_turn_timer.Get() < 0.5) {
        return CONTINUE;
    } else {
        return END;
    }
}

void TurnToAngleAction::ActionEnd() {
    m_tank_drive->setDriveVoltage(0, 0);
}