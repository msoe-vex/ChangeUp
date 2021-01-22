#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/String.h"

class HelloWorldNode : public Node {
private:
    std_msgs::String m_string_msg;
    std::string m_handle_name;
    ros::Publisher* m_publisher;

    void m_populateMessage();

public:
    HelloWorldNode(NodeManager* node_manager, std::string handle_name);

    void initialize();

    void periodic();

    ~HelloWorldNode();
};
