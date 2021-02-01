#pragma once

#include "NodeManager.h"
#include "eigen/Eigen/Dense"
#include "math.h"
#include "SwerveController.h"
#include "DataNodes/MotorNode.h"
#include "DataNodes/ADIAnalogInNode.h"
#include "DataNodes/ControllerNode.h"
#include "DataNodes/InertialSensorNode.h"
#include "ros_lib/v5_hal/RollPitchYaw.h"

class DriverControlNode : public Node {
private:
    SwerveController swerveController;

    std::string m_handle_name;
    ros::Publisher* m_publisher;
    ros::Subscriber<v5_hal::RollPitchYaw, DriverControlNode>* m_navx_sub;

    Eigen::Vector2d controller_target_velocity;
    Eigen::Vector2d field_target_velocity;
    Eigen::Rotation2Dd robot_angle = Eigen::Rotation2Dd(0);
    double rotation_velocity;
    double rotation_angle_threshold = (M_PI / 3);
    double max_velocity = 1.31;
    double max_rotation_velocity = 0.17;
    double kP = 0.8;
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

    MotorNode* left_swerve_1;
    MotorNode* left_swerve_2;
    MotorNode* right_swerve_1;
    MotorNode* right_swerve_2;
    MotorNode* rear_swerve_1;
    MotorNode* rear_swerve_2;
    MotorNode* left_intake;
    MotorNode* right_intake;
    MotorNode* bottom_rollers;
    MotorNode* ejection_roller;
    MotorNode* top_rollers;
    ADIAnalogInNode* left_swerve_pot;
    ADIAnalogInNode* right_swerve_pot;
    ADIAnalogInNode* rear_swerve_pot;
    pros::Controller* controller_primary;

    void m_navxDataCallback(const v5_hal::RollPitchYaw& msg);

    void m_spinIntakesVoltage(int voltage);

    void m_spinMainRollersVoltage(int voltage);

    void m_spinEjectionRollerVoltage(int voltage);

public:
    DriverControlNode(NodeManager* node_manager, MotorNode* left_swerve_1, MotorNode* left_swerve_2, 
        ADIAnalogInNode* left_swerve_pot, MotorNode* right_swerve_1, MotorNode* right_swerve_2, 
        ADIAnalogInNode* right_swerve_pot, MotorNode* rear_swerve_1, MotorNode* rear_swerve_2, 
        ADIAnalogInNode* rear_swerve_pot, MotorNode* left_intake, MotorNode* right_intake, MotorNode* bottom_rollers,
        MotorNode* ejection_roller, MotorNode* top_rollers, ControllerNode* controller_primary);

    void initialize();

    void assignActualAngle(const float left_msg, const float right_msg, const float rear_msg);

    MotorPowers calculateLeftModule(Eigen::Vector2d target_velocity, double rotation_velocity);

    MotorPowers calculateRightModule(Eigen::Vector2d target_velocity, double rotation_velocity);

    MotorPowers calculateRearModule(Eigen::Vector2d target_velocity, double rotation_velocity);

    void periodic();

    ~DriverControlNode();
};
