#include "DataNodes/ProsTimeNode.h"

ProsTimeNode::ProsTimeNode(NodeManager* node_manager, std::string handle_name) 
    : Node (node_manager, 20) {
    m_handle_name = handle_name.insert(0, "prosTime/");

    m_publisher = new ros::Publisher(m_handle_name.c_str(), &m_pros_time_msg);
}

void ProsTimeNode::initialize() {
    Node::m_handle->initNode();
    Node::m_handle->advertise(*m_publisher);
}

void ProsTimeNode::periodic() {
    m_populateMessage();
    m_publisher->publish(&m_pros_time_msg);
    Node::m_handle->spinOnce();
}

void ProsTimeNode::m_populateMessage() {
    m_pros_time_msg.data = pros::millis();
}

ProsTimeNode::~ProsTimeNode() {
    delete m_publisher;
}