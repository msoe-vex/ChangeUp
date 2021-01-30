#pragma once

#include "NodeManager.h"
#include "eigen/Eigen/Dense"
#include "math.h"
#include "SwerveController.h"

class DriverControlNode : public Node {
private:
    
    SwerveController swerveController;

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

    pros::Motor left_swerve_1;
    pros::Motor left_swerve_2;
    pros::Motor right_swerve_1;
    pros::Motor right_swerve_2;
    pros::Motor rear_swerve_1;
    pros::Motor rear_swerve_2;
    pros::ADIAnalogIn left_swerve_pot;
    pros::ADIAnalogIn right_swerve_pot;
    pros::ADIAnalogIn rear_swerve_pot;

    pros::Controller controller_primary;

public:
    DriverControlNode(NodeManager* node_manager, pros::Motor left_swerve_1, pros::Motor left_swerve_2, 
        pros::ADIAnalogIn left_swerve_pot, pros::Motor right_swerve_1, pros::Motor right_swerve_2, 
        pros::ADIAnalogIn right_swerve_pot, pros::Motor rear_swerve_1, pros::Motor rear_swerve_2, 
        pros::ADIAnalogIn rear_swerve_pot, pros::Controller controller_primary);

    void initialize();

    void assignActualAngle(const float left_msg, const float right_msg, const float rear_msg);

    MotorPowers calculateLeftModule(Eigen::Vector2d target_velocity, double rotation_velocity);

    MotorPowers calculateRightModule(Eigen::Vector2d target_velocity, double rotation_velocity);

    MotorPowers calculateRearModule(Eigen::Vector2d target_velocity, double rotation_velocity);

    void periodic();

    ~DriverControlNode();
};
