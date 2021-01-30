#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/Int16.h"

class DriverControlNode : public Node {
private:

    std::string m_handle_name;
    ros::Publisher* m_publisher;

    Eigen::Vector2d target_velocity;
    double rotation_velocity;
    double rotation_angle_threshold = (M_PI / 3);
    double max_velocity = 1.31;
    double max_rotation_velocity = 100.0;
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    Eigen::Vector2d left_module_location;
    Eigen::Vector2d right_module_location;
    Eigen::Vector2d rear_module_location;
    double left_module_location_x = -5.25;
    double left_module_location_y = 5.55;
    double right_module_location_x = 5.25;
    double right_module_location_y = 5.55;
    double rear_module_location_x = 0.0;
    double rear_module_location_y = -5.21;

public:
    DriverControlNode(NodeManager* node_manager, 
        std::string handle_name);

    void initialize();

    void assignActualAngle(const float left_msg, const float right_msg, const float rear_msg);

    MotorPowers calculateLeftModule(Eigen::Vector2d target_velocity, double rotation_velocity);

    MotorPowers calculateRightModule(Eigen::Vector2d target_velocity, double rotation_velocity);

    MotorPowers calculateRearModule(Eigen::Vector2d target_velocity, double rotation_velocity);

    void periodic();

    ~DriverControlNode();
};
