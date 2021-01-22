#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/V5Battery.h"

class BatteryNode : public Node {
private:
    v5_hal::V5Battery m_battery_msg;
    std::string m_handle_name;
    ros::Publisher* m_publisher;

    void m_populateMessage();

public:
    BatteryNode(NodeManager* node_manager, std::string* handle_name);

    void initialize();

    void periodic();

    ~BatteryNode();
};