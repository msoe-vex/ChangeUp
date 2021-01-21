#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/V5Battery.h"

class BatteryNode : public Node {
private:
    ros::Publisher m_publisher;
    v5_hal::V5Battery m_battery_msg;
    std::string* m_handle_name;

    void populateMessage();

public:
    BatteryNode(NodeManager* nodeManager, std::string* handleName);

    void initialize();

    void periodic();

    ~BatteryNode();
};