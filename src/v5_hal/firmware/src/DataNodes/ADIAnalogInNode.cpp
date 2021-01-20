#include "ADIAnalogInNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
ADIAnalogInNode::ADIAnalogInNode(NodeManager* nodeManager, int port,
    std::string* handleName)
    : Node(nodeManager, 20),
    m_analog_in(port),
    m_publisher(handleName->insert(0, "sensor_").c_str(), &m_analog_in_msg) {
    m_handle_name = handleName;
}

void ADIAnalogInNode::initialize() {
    // Initialize the handler, and set up data to publish
    Node::m_handle->initNode();
    Node::m_handle->advertise(m_publisher);
}

void ADIAnalogInNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    populateMessage();
    m_publisher.publish(&m_analog_in_msg);
    Node::m_handle->spinOnce();
}

void ADIAnalogInNode::populateMessage() {
    m_analog_in_msg.value = m_analog_in.get_value();
}

ADIAnalogInNode::~ADIAnalogInNode() { delete m_handle_name; }