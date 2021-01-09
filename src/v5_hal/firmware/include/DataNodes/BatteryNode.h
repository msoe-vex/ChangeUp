#pragma once

#include "api.h"
#include "NodeManager.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/V5Battery.h"

class BatteryNode : public Node {
private:
    ros::NodeHandle* m_handle;
    ros::Publisher* m_publisher;
    v5_hal::V5Battery* m_battery_msg;
    std::string m_handle_name;

    void m_populateBatteryMsg();

public:
    BatteryNode(NodeManager* nodeManager, std::string handleName);

    void initialize();

    void periodic();

    ~BatteryNode();
};