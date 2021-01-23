#include <ros/ros.h>
#include <Eigen/Dense>
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include <math.h>
#include "std_msgs/Int8.h"

#include "swerve_controller/SwerveModule.cpp"

Eigen::Vector2d target_velocity;
double rotation_velocity;
Eigen::Rotation2Dd actual_angle;

void assignTargetVelocity(const geometry_msgs::Vector3& msg) {
    target_velocity(0) = msg.x;
    target_velocity(1) = msg.y;
}

void assignRotationVelocity(const std_msgs::Float32& msg) {
    rotation_velocity = msg.data;
}

void assignActualAngle(const std_msgs::Float32& msg) {
    actual_angle = Eigen::Rotation2Dd (msg.data);
}

int main(int argc, char **argv) {
    ros::NodeHandle nh;
    ros::init(argc, argv, "swerve_controller");

    double x, y, rotation_angle_threshold, max_velocity, max_rotation_velocity;

    nh.param("left_module_x", x, 5.0);
    nh.param("left_module_y", y, 5.0);
    Eigen::Vector2d module_location(x, y);

    nh.param("rotation_angle_threshold", rotation_angle_threshold, M_PI);
    nh.param("max_velocity", max_velocity, 1.31);
    nh.param("max_rotation_velocity", max_rotation_velocity, 100.0);


    SwerveModule swerve_module(module_location, rotation_angle_threshold, max_velocity, max_rotation_velocity);

    ros::Subscriber swerve_controller_tf_sub = nh.subscribe("swerve_command_tf", 10, assignTargetVelocity);
    ros::Subscriber swerve_controller_rotate_sub = nh.subscribe("swerve_command_rotate", 10, assignRotationVelocity);
    ros::Subscriber swerve_controller_angle_sub = nh.subscribe("swerve_command_act_angle", 10, assignActualAngle);

    ros::Publisher swerve_controller_left_motor_pub = nh.advertise<std_msgs::Int8>("left_motor", 10);
    ros::Publisher swerve_controller_right_motor_pub = nh.advertise<std_msgs::Int8>("right_motor", 10);

    ros::Rate loop_rate(100);

    while (ros::ok()) {
        motorPowers* motor_powers = swerve_module.InverseKinematics(target_velocity, rotation_velocity, actual_angle);
        std_msgs::Int8 left_motor_power;
        std_msgs::Int8 right_motor_power;
        left_motor_power.data = motor_powers->left_motor_power;
        right_motor_power.data = motor_powers->right_motor_power;


        swerve_controller_left_motor_pub.publish(left_motor_power);
        swerve_controller_right_motor_pub.publish(right_motor_power);

        ros::spinOnce();
        loop_rate.sleep();
    }
}