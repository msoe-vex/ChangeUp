#include <ros/ros.h>
#include <Eigen/Dense>
#include "geometry_msgs/Vector3.h"
#include <math.h>

#include "swerve_controller/SwerveModule.h"

Eigen::Vector2d target_velocity;

void processCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    target_velocity(0) = msg->x;
    target_velocity(1) = msg->y;
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

    ros::Subscriber sub = nh.subscribe("swerve_command_tf", 10, processCallback);
}