#include "kinematics/XDriveKinematics.h"

XDriveKinematics::XDriveKinematics() {

}

HolonomicDriveNode::HolonomicDriveMotorPowers XDriveKinematics::InverseKinematics(float x_velocity, float y_velocity, float theta_velocity) {
    double front_left = (double)(y_velocity + x_velocity + theta_velocity);
    double back_left = (double)(y_velocity - x_velocity + theta_velocity);
    double front_right = (double)(y_velocity - x_velocity - theta_velocity);
    double back_right = (double)(y_velocity + x_velocity - theta_velocity);

    double max_val = std::max({front_left, back_left, front_right, back_right, 127.0});

    HolonomicDriveNode::HolonomicDriveMotorPowers motor_powers {
        (front_left / max_val) * MAX_MOTOR_VOLTAGE,
        (back_left / max_val) * MAX_MOTOR_VOLTAGE,
        (front_right / max_val) * MAX_MOTOR_VOLTAGE,
        (back_right / max_val) * MAX_MOTOR_VOLTAGE
    };

    return motor_powers;
}

XDriveKinematics::~XDriveKinematics() {

}
