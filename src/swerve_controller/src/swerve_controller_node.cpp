#include <ros/ros.h>
#include <Eigen/Dense>
#include <math.h>
#include <ros/console.h>

#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int8.h"

#include "swerve_controller/SwerveModule.h"

Eigen::Vector2d target_velocity;
Eigen::Rotation2Dd actual_angle;
Eigen::Vector2d module_location;

double rotation_velocity;
double x;
double y; 
double rotation_angle_threshold;
double max_velocity;
double max_rotation_velocity;

void assignTargetVelocity(const geometry_msgs::Vector3& msg) {
    target_velocity(0) = msg.x;
    target_velocity(1) = msg.y;
}

void assignRotationVelocity(const std_msgs::Float32& msg) {
    rotation_velocity = msg.data;
}

void assignActualAngle(const std_msgs::Float32& msg) {
    actual_angle = Eigen::Rotation2Dd(msg.data);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "swerve_controller");

    ros::NodeHandle handle;

    handle.param("module_x", x, 5.0);
    handle.param("module_y", y, 5.0);
    module_location(0) = x;
    module_location(1) = y;

    handle.param("rotation_angle_threshold", rotation_angle_threshold, (M_PI / 3));
    handle.param("max_velocity", max_velocity, 1.31);
    handle.param("max_rotation_velocity", max_rotation_velocity, 100.0);
    handle.param("kP", kP, 0.0);
    handle.param("kI", kI, 0.0);
    handle.param("kD", kD, 0.0);

    SwerveModule swerveModule(module_location, rotation_angle_threshold, max_velocity, max_rotation_velocity, kP, kI, kD);

    ros::Subscriber swerve_controller_tf_sub = handle.subscribe("swerveCommandTf", 10, assignTargetVelocity);
    ros::Subscriber swerve_controller_rotate_sub = handle.subscribe("swerveCommandRotate", 10, assignRotationVelocity);
    ros::Subscriber swerve_controller_angle_sub = handle.subscribe("swerveCommandActAngle", 10, assignActualAngle);

    ros::Publisher swerve_controller_left_motor_pub = handle.advertise<std_msgs::Int8>("leftMotor", 10);
    ros::Publisher swerve_controller_right_motor_pub = handle.advertise<std_msgs::Int8>("rightMotor", 10);

    ros::Rate loop_rate(50);

    while (ros::ok()) {
        MotorPowers motor_powers = swerveModule.InverseKinematics(target_velocity, rotation_velocity, actual_angle);

        std_msgs::Int8 left_motor_power;
        std_msgs::Int8 right_motor_power;

        left_motor_power.data = motor_powers.left_motor_power;
        right_motor_power.data = motor_powers.right_motor_power;

        swerve_controller_left_motor_pub.publish(left_motor_power);
        swerve_controller_right_motor_pub.publish(right_motor_power);

        ros::spinOnce();
        loop_rate.sleep();
    }
}