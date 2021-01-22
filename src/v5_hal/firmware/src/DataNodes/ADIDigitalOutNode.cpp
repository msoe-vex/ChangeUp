#include "DataNodes/ADIDigitalOutNode.h"

ADIDigitalOutNode::ADIDigitalOutNode (NodeManager* node_manager, std::string handle_name,
    int port, bool initial_state = false) : Node (nodeManager, 20), 
    m_digital_out (port, intial_state) {
    m_handle_name = handle_name.insert(0, "output/");
    m_sub_digital_out_name = m_handle_name + "/digitalOut";
    m_digital_out_sub = new ros::Subscriber<std_msgs::Bool, ADIDigitalOutNode>
        m_sub_digital_out_name.c_str(), &ADIDigitlOutNode::m_setValue, this);
    }

void ADIDigitlOutNode::m_setValue (const std_msgs::Bool& msg) {
    m_digital_out.set_value((int)msg.data);
}

void ADIDigitalOutNode::initialize () {
    Node::m_handle->initNode();
    Node::m_handle->subscribe(*m_digital_out_sub);
}

ADIDigitalOutNode::~ADIDigitalOutNode () {
    delete m_digital_out_sub;
}