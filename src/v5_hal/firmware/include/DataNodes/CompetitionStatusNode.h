#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/CompetitionStatus.h"
#include "pros/misc.h"

class CompetitionStatusNode : public Node {
private: 
    v5_hal::CompetitionStatus m_comp_status_msg;
    std::string m_handle_name;
    ros::Publisher* m_publisher;

    void m_populateMessage();

public:
    CompetitionStatusNode(NodeManager* node_manager, std::string handle_name);

    void initialize();

    void periodic();

    ~CompetitionStatusNode();
};