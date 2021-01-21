#include "DataNodes/ProsTimeNode.h"

ProsTimeNode::ProsTimeNode(NodeManager* nodeManager, std::string handleName) 
    : Node (nodeManager, 200), m_publisher(handleName->insert(0, "motor_").c_str(), &m_motor_msg) {
    m_handle_name = handleName;
}

void ProsTimeNode::initialize() {
    Node::m_handle->initNode();
    Node::m_handle->advertise(m_publisher);
}

void ProsTimeNode::periodic() {
    populateMessage();
    m_publisher->publish(&m_prostime_msg);
    Node::m_handle->spinOnce();
}

void ProsTimeNode::populateMessage() {
    m_prostime_msg.millis_since_boot = pros::millis();
}

ProsTimeNode::~ProsTimeNode() {
    delete m_handle_name;
}