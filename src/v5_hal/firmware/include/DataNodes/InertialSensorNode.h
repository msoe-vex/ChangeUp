#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/RollPitchYaw.h"
#include "eigen/Eigen/Dense"

class InertialSensorNode : public Node {
public:
    enum SensorConfig {
        V5, ROS
    };

    InertialSensorNode(NodeManager* node_manager, std::string handle_name, 
        int sensor_port);

    InertialSensorNode(NodeManager* node_manager, std::string handle_name, 
        std::string subscribe_handle);
    
    void initialize();
    
    Eigen::Rotation2Dd getYaw();

    bool isAtAngle(Eigen::Rotation2Dd angle);
    
    void teleopPeriodic();

    void autonPeriodic();
    
    ~InertialSensorNode();

private:
    pros::Imu* m_inertial_sensor = nullptr;
    std::string m_handle_name;
    std::string m_sub_inertial_sensor_name;
    double turning_threshold = 0.15;
    SensorConfig m_config;
    Eigen::Rotation2Dd m_yaw;
    Eigen::Rotation2Dd m_gyro_offset_angle; 

    ros::Subscriber<v5_hal::RollPitchYaw, InertialSensorNode>* m_inertial_sensor_sub;

    void m_handleSensorMsg(const v5_hal::RollPitchYaw& msg);
};