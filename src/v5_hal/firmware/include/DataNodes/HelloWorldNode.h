#pragma once

#include "NodeManager.h"
#include "api.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/String.h"

class HelloWorldNode : public Node {
private:
    std_msgs::String m_string_msg;
    ros::Publisher m_publisher;
    std::string* m_handle_name;

    void populateMessage();

public:
    HelloWorldNode(NodeManager* nodeManager, std::string* handleName);

    void initialize();

    void periodic();

    ~HelloWorldNode();
};
