#pragma once

#include "api.h"
#include "NodeManager.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/CompetitionStatus.h"
#include "pros/misc.hpp"

class CompetitionStatusNode : public Node {
private: 
    ros::NodeHandle* m_handle;
    v5_hal::CompetitionStatus* m_comp_status_msg;
    ros::Publisher* m_publisher;
    std::string m_handle_name;

    void m_populateCompStatusMsg();

public:
    CompetitionStatusNode(NodeManager* nodeManager, std::string handleName);

    void initialize();

    void periodic();

    ~CompetitionStatusNode();
};