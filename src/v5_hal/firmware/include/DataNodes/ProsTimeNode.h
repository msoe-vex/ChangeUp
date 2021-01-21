#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/v5_hal/ProsTime.h"

class ProsTimeNode : public Node {
private:
    ros::Publisher m_publisher;
    v5_hal::ProsTime m_prosTime_msg;
    std::string* m_handle_name;

    void populateMessage();
    
public:
    ProsTimeNode(NodeManager* nodeHandle, std::string* handleName);

    void initialize();
    
    void periodic();

    ~ProsTimeNode();
};