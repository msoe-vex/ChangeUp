#include "eigen/Eigen/Dense"
#include "math.h"
#include "SwerveModule"

class SwerveController(){
    Eigen::Vector2d target_velocity;
    Eigen::Rotation2Dd left_actual_angle;
    Eigen::Rotation2Dd right_actual_angle;
    Eigen::Rotation2Dd rear_actual_angle;
    Eigen::Vector2d module_location;

    double rotation_velocity;
    double x;
    double y; 
    double rotation_angle_threshold;
    double max_velocity;
    double max_rotation_velocity;

    SwerveController(Eigen::Vector2d left_module_location, Eigen::Vector2d right_module_location, Eigen::Vector2d rear_module_location,
        double rotation_angle_threshold, double max_velocity, double max_rotation_velocity, double kP, double kI, double kD) : 
        leftSwerveModule(left_module_location, rotation_angle_threshold, max_velocity, max_rotation_velocity, kP, kI, kD),
        rightSwerveModule(right_module_location, rotation_angle_threshold, max_velocity, max_rotation_velocity, kP, kI, kD),
        rearSwerveModule(rear_module_location, rotation_angle_threshold, max_velocity, max_rotation_velocity, kP, kI, kD) {
        
    }

    void assignTargetVelocity(const geometry_msgs::Vector3& msg) {
        target_velocity(0) = msg.x;
        target_velocity(1) = msg.y;
    }

    void assignRotationVelocity(const float msg) {
        rotation_velocity = msg;
    }

    void assignActualAngle(const float left_msg, const float right_msg, const float rear_msg) {
        left_actual_angle = Eigen::Rotation2Dd(left_msg);
        right_actual_angle = Eigen::Rotation2Dd(right_msg);
        rear_actual_angle = Eigen::Rotation2Dd(rear_msg);
    }


    MotorPowers calculate() {
        MotorPowers motor_powers = swerveModule.InverseKinematics(target_velocity, rotation_velocity, actual_angle);

        return motor_powers;
    }
}
