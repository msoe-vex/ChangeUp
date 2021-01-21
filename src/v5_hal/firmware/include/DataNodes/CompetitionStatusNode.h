#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/CompetitionStatus.h"
#include "pros/misc.h"

class CompetitionStatusNode : public Node {
private: 
    v5_hal::CompetitionStatus m_comp_status_msg;
    ros::Publisher m_publisher;
    std::string* m_handle_name;

    void populateMessage();

public:
    CompetitionStatusNode(NodeManager* nodeManager, std::string* handleName);

    void initialize();

    void periodic();

    ~CompetitionStatusNode();
};