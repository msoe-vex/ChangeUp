#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/ADIGyroData.h"

class ADIGyroNode : public Node {
private:
    pros::ADIGyro m_gyro;
    v5_hal::ADIGyroData m_gyro_msg;
    ros::Publisher m_publisher;
    std::string* m_handle_name;

    void populateMessage();

public:
    ADIGyroNode(NodeManager* nodeManager, int port, double multiplier,
        std::string* handleName);

    void initialize();

    void periodic();

    ~ADIGyroNode();
};
