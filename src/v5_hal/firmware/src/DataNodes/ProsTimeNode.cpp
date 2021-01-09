#include "DataNodes/ProsTimeNode.h"

ProsTimeNode::ProsTimeNode(NodeManager* nodeManager, std::string handleName) 
    : Node (nodeManager, 200) {
    m_handle = new ros::NodeHandle();
    m_handle_name = handleName;
    m_prostime_msg = new v5_hal::ProsTime();
}

void ProsTimeNode::initialize() {
    m_publisher = new ros::Publisher(m_handle_name.c_str(), m_prostime_msg);

    m_handle->advertise(*m_publisher);
}

void ProsTimeNode::periodic() {
    populateProsTimeMsg();
    m_publisher->publish(m_prostime_msg);
    m_handle->spinOnce();
}

void ProsTimeNode::populateProsTimeMsg() {
    m_prostime_msg->millis_since_boot = pros::millis();
}

ProsTimeNode::~ProsTimeNode() {
    delete m_handle;
    delete m_prostime_msg;
    delete m_publisher;
}