#pragma once

#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/V5RotationSensor.h"

class RotationSesnorNode : public Node () {
private:
    ros::NodeHandle* m_handle;
    ros::Publisher* m_publisher;
    pros::Rotation* m_rotation;
    std::string m_handle_name;
    v5_hal::V5RotationSensor* m_rotation_msg;
    void m_populateRotationMsg();

public:
    RotationSesnorNode(NodeManager* nodeManager, std::string handldName, int rotationPort);
    void initialize();
    void periodic();
    ~RotationSesnorNode();

};