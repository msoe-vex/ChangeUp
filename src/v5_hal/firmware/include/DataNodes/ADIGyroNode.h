#pragma once

#include "api.h"
#include "NodeManager.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/ADIGyroData.h"

class ADIGyroNode : public Node {
private:
    pros::ADIGyro* m_gyro;
    v5_hal::ADIGyroData* m_gyro_msg;
    ros::NodeHandle* m_handle;
    ros::Publisher* m_publisher;
    std::string m_handle_name;

    void populateGyroMsg();

public:
    ADIGyroNode(NodeManager* nodeManager, int port, double multiplier, std::string handleName);

    void initialize();

    void periodic();

    ~ADIGyroNode();
};