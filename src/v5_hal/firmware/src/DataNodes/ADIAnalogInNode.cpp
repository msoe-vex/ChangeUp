#include "ADIAnalogInNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
ADIAnalogInNode::ADIAnalogInNode(NodeManager* node_manager, int port,
    std::string handle_name, bool reverse) : Node(node_manager, 50), 
    m_analog_in(port), m_is_reversed(reverse) {
    m_handle_name = handle_name.insert(0, "sensor/");

    m_publisher = new ros::Publisher(m_handle_name.c_str(), &m_analog_in_msg);
}

void ADIAnalogInNode::initialize() {
    // Initialize the handler, and set up data to publish
    Node::m_handle->initNode();
    Node::m_handle->advertise(*m_publisher);
}

int ADIAnalogInNode::getValue() {
    return m_analog_in.get_value();
}

void ADIAnalogInNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    m_populateMessage();
    m_publisher->publish(&m_analog_in_msg);
    Node::m_handle->spinOnce();
}

void ADIAnalogInNode::m_populateMessage() {
    if (m_is_reversed) {
        m_analog_in_msg.data = (4096 - m_analog_in.get_value());
    } else {
        m_analog_in_msg.data = m_analog_in.get_value();
    }
}

ADIAnalogInNode::~ADIAnalogInNode() { 
    delete m_publisher; 
}