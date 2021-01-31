#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"

class InertialSensorNode : public Node {
private:
    pros::Imu m_inertial_sensor;
    std::string m_handle_name;

public:
    InertialSensorNode(NodeManager* node_manager, int sensor_port, 
        std::string handle_name);
    
    void initialize();
    
    double getYaw();
    
    void periodic();
    
    ~InertialSensorNode();
};