#pragma once

#include "api.h"
#include "nodes/NodeManager.h"
#include "ros_lib/ros.h"
#include "ros_lib/std_msgs/Bool.h"
#include "ros_lib/std_msgs/Empty.h"

class ADIDigitalOutNode : public Node {
private:
    pros::ADIDigitalOut m_digital_out;
    std::string m_handle_name;
    std::string m_sub_publish_data_name;
    std::string m_sub_digital_out_name;
    ros::Subscriber<std_msgs::Bool, ADIDigitalOutNode>* m_digital_out_sub;
    ros::Subscriber<std_msgs::Empty, ADIDigitalOutNode>* m_publish_data_sub;

    void m_setValue(const std_msgs::Bool& msg);

    void m_publishData(const std_msgs::Empty& msg);

public:
    ADIDigitalOutNode(NodeManager* node_manager, std::string handle_name,
    int port, bool initial_state = false);

    void initialize();

    void setValue(int value);

    ~ADIDigitalOutNode();
};