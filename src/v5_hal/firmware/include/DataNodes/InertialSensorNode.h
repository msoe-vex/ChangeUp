#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/RollPitchYaw.h"

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
    
    double getYaw();

    bool isAtAngle(double angle);
    
    void teleopPeriodic();

    void autonPeriodic();
    
    ~InertialSensorNode();

private:
    pros::Imu* m_inertial_sensor = nullptr;
    std::string m_handle_name;
    std::string m_sub_inertial_sensor_name;
    double turning_threshold = 0.1;
    SensorConfig m_config;
    double m_yaw;

    ros::Subscriber<v5_hal::RollPitchYaw, InertialSensorNode>* m_inertial_sensor_sub = nullptr;

    void m_handleSensorMsg(const v5_hal::RollPitchYaw& msg);
};