#pragma once

#include "api.h"
#include "NodeManager.h"

class ADIDigitalOutNode : public Node {
private:
    pros::ADIDigitalOut m_digital_out;
    std::string m_handle_name;
    ros::Subscriber<std_msgs::Bool, ADIDigitalOutNode>* m_digital_out_sub;
    std::string m_sub_digital_out_name;

    void m_setValue ();

public:
    ADIDigitalOutNode (NodeManager* node_manager, std::string handle_name,
    int port, bool initial_state = false);

    void initialize ();

    ~ADIDigitalOutNode ();
};