#include "SwerveController.h"


SwerveController::SwerveController(Eigen::Vector2d left_module_location, Eigen::Vector2d right_module_location, Eigen::Vector2d rear_module_location,
    double rotation_angle_threshold, double max_velocity, double max_rotation_velocity, double kP, double kI, double kD) : 
    leftSwerveModule(left_module_location, rotation_angle_threshold, max_velocity, max_rotation_velocity, kP, kI, kD),
    rightSwerveModule(right_module_location, rotation_angle_threshold, max_velocity, max_rotation_velocity, kP, kI, kD),
    rearSwerveModule(rear_module_location, rotation_angle_threshold, max_velocity, max_rotation_velocity, kP, kI, kD) {
}

<<<<<<< Updated upstream
void SwerveController::assignActualAngle(int left_pot, int right_pot, int rear_pot) {
    left_actual_angle = Eigen::Rotation2Dd((left_pot - LEFT_POT_OFFSET) % 4095);
    right_actual_angle = Eigen::Rotation2Dd((right_pot - RIGHT_POT_OFFSET) % 4095);
    rear_actual_angle = Eigen::Rotation2Dd((rear_pot - REAR_POT_OFFSET) % 4095);
=======
void SwerveController::assignActualAngle(const float left_msg, const float right_msg, const float rear_msg) {
    left_actual_angle = Eigen::Rotation2Dd((((float)left_msg / 4095.0) * M_PI) * 2.0);
    right_actual_angle = Eigen::Rotation2Dd((((float)right_msg / 4095.0) * M_PI) * 2.0);
    rear_actual_angle = Eigen::Rotation2Dd((((float)rear_msg / 4095.0) * M_PI) * 2.0);
>>>>>>> Stashed changes
}

MotorPowers SwerveController::calculateLeftModule(Eigen::Vector2d target_velocity, double rotation_velocity) {
    MotorPowers motor_powers = leftSwerveModule.InverseKinematics(target_velocity, rotation_velocity, left_actual_angle);

    //printf("Motor Powers %i %i\n",  motor_powers.left_motor_power, motor_powers.right_motor_power);

    return motor_powers;
}

MotorPowers SwerveController::calculateRightModule(Eigen::Vector2d target_velocity, double rotation_velocity) {
    right_motor_powers = rightSwerveModule.InverseKinematics(target_velocity, rotation_velocity, right_actual_angle);

    return right_motor_powers;
}

MotorPowers SwerveController::calculateRearModule(Eigen::Vector2d target_velocity, double rotation_velocity) {
    rear_motor_powers = rearSwerveModule.InverseKinematics(target_velocity, rotation_velocity, rear_actual_angle);

    return rear_motor_powers;
}

