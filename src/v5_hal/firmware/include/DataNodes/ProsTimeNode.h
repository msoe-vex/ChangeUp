#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/UInt32.h"

class ProsTimeNode : public Node {
private:
    std_msgs::UInt32 m_pros_time_msg;
    std::string m_handle_name;
    ros::Publisher* m_publisher;

    void m_populateMessage();
    
public:
    ProsTimeNode(NodeManager* node_handle, std::string handle_name);

    void initialize();
    
    void periodic();

    ~ProsTimeNode();
};