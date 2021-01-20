#include "DataNodes/ADIGyroNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
ADIGyroNode::ADIGyroNode(NodeManager* nodeManager, int port, double multiplier,
    std::string* handleName)
    : Node(nodeManager, 20),
    m_gyro(port, multiplier),
    m_publisher(handleName->insert(0, "sensor_").c_str(), &m_gyro_msg) {
    m_handle_name = handleName;
}

void ADIGyroNode::initialize() {
    // Initialize the handler, and set up data to publish
    Node::m_handle->initNode();
    Node::m_handle->advertise(m_publisher);
}

void ADIGyroNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    populateMessage();
    m_publisher.publish(&m_gyro_msg);
    Node::m_handle->spinOnce();
}

void ADIGyroNode::populateMessage() { m_gyro_msg.degrees = m_gyro.get_value(); }

ADIGyroNode::~ADIGyroNode() { delete m_handle_name; }