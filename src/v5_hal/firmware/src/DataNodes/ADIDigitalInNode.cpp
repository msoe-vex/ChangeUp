#include "DataNodes/ADIDigitalInNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
ADIDigitalInNode::ADIDigitalInNode(NodeManager* nodeManager, int port,
    std::string* handleName)
    : Node(nodeManager, 20),
    m_digital_in(port),
    m_publisher(handleName->insert(0, "sensor_").c_str(), &m_digital_in_msg) {
    m_handle_name = handleName;
}

void ADIDigitalInNode::initialize() {
    // Initialize the handler, and set up data to publish
    Node::m_handle->initNode();
    Node::m_handle->advertise(m_publisher);
}

void ADIDigitalInNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    populateMessage();
    m_publisher.publish(&m_digital_in_msg);
    Node::m_handle->spinOnce();
}

void ADIDigitalInNode::populateMessage() {
    m_digital_in_msg.value = m_digital_in.get_value();
}

ADIDigitalInNode::~ADIDigitalInNode() { delete m_handle_name; }