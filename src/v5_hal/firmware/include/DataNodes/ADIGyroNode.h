#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/Float32.h"

class ADIGyroNode : public Node {
private:
    pros::ADIGyro m_gyro;
    std_msgs::Float32 m_gyro_msg;
    std::string m_handle_name;
    ros::Publisher* m_publisher;

    void m_populateMessage();

public:
    ADIGyroNode(NodeManager* node_manager, int port, double multiplier,
        std::string* handle_name);

    void initialize();

    void periodic();

    ~ADIGyroNode();
};
