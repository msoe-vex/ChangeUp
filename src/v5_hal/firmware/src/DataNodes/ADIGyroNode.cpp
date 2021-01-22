#include "DataNodes/ADIGyroNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
ADIGyroNode::ADIGyroNode(NodeManager* node_manager, int port, double multiplier,
    std::string handle_name) : Node(node_manager, 20), m_gyro(port, multiplier) {
    m_handle_name = handle_name.insert(0, "sensor/");

    m_publisher = new ros::Publisher(m_handle_name.c_str(), &m_gyro_msg);
}

void ADIGyroNode::initialize() {
    // Initialize the handler, and set up data to publish
    Node::m_handle->initNode();
    Node::m_handle->advertise(*m_publisher);
}

void ADIGyroNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    m_populateMessage();
    m_publisher->publish(&m_gyro_msg);
    Node::m_handle->spinOnce();
}

void ADIGyroNode::m_populateMessage() { 
    m_gyro_msg.data = m_gyro.get_value(); 
}

ADIGyroNode::~ADIGyroNode() { 
    delete m_publisher;
}