#include "DataNodes/ADIUltrasonicNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
ADIUltrasonicNode::ADIUltrasonicNode(NodeManager* nodeManager, int port_ping,
                                     int port_echo, std::string* handleName)
    : Node(nodeManager, 20),
      m_ultrasonic(port_ping, port_echo),
      m_publisher(handleName->c_str(), &m_ultrasonic_msg) {
    m_handle_name = handleName;
}

void ADIUltrasonicNode::initialize() {
    // Initialize the handler, and set up data to publish
    m_handle->initNode();
    m_handle->advertise(m_publisher);
}

void ADIUltrasonicNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    populateMessage();
    m_publisher.publish(&m_ultrasonic_msg);
    m_handle->spinOnce();
}

void ADIUltrasonicNode::populateMessage() {
    m_ultrasonic_msg.distance = m_ultrasonic.get_value();
}

ADIUltrasonicNode::~ADIUltrasonicNode() { delete m_handle_name; }