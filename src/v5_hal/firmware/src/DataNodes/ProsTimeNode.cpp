#include "DataNodes/ProsTimeNode.h"

ProsTimeNode::ProsTimeNode(NodeManager* nodeManager, std::string* handleName) 
    : Node (nodeManager, 20), m_publisher(handleName->insert(0, "prosTime/").c_str(), &m_prosTime_msg) {
    m_handle_name = handleName;
}

void ProsTimeNode::initialize() {
    Node::m_handle->initNode();
    Node::m_handle->advertise(m_publisher);
}

void ProsTimeNode::periodic() {
    populateMessage();
    m_publisher.publish(&m_prosTime_msg);
    Node::m_handle->spinOnce();
}

void ProsTimeNode::populateMessage() {
    m_prosTime_msg.millis_since_boot = pros::millis();
}

ProsTimeNode::~ProsTimeNode() {
    delete m_handle_name;
}