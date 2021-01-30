#include "DataNodes/ADIUltrasonicNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
ADIUltrasonicNode::ADIUltrasonicNode(NodeManager* node_manager, int port_ping,
    int port_echo, std::string handle_name) : Node(node_manager, 50),
    m_ultrasonic(port_ping, port_echo) {
    m_handle_name = handle_name.insert(0, "sensor/");

    m_publisher = new ros::Publisher(m_handle_name.c_str(), &m_ultrasonic_msg);
}

void ADIUltrasonicNode::initialize() {
    // Initialize the handler, and set up data to publish
    m_handle->initNode();
    m_handle->advertise(*m_publisher);
}

int ADIUltrasonicNode::getValue() {
    return m_ultrasonic.get_value();
}

void ADIUltrasonicNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    m_populateMessage();
    m_publisher->publish(&m_ultrasonic_msg);
    m_handle->spinOnce();
}

void ADIUltrasonicNode::m_populateMessage() {
    m_ultrasonic_msg.data = getValue();
}

ADIUltrasonicNode::~ADIUltrasonicNode() { 
    delete m_publisher;
}