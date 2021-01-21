#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/V5RotationSensor.h"

class RotationSensorNode : public Node {
private:
    ros::Publisher m_publisher;
    pros::Rotation m_rotation;
    v5_hal::V5RotationSensor m_rotation_msg;
    std::string* m_handle_name;

    void populateyMessage();

public:
    RotationSensorNode(NodeManager* nodeManager, std::string handldName, int rotationPort);
    void initialize();
    void periodic();
    ~RotationSensorNode();

};