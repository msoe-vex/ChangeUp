#include "DataNodes/ADIDigitalInNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
ADIDigitalInNode::ADIDigitalInNode(NodeManager* node_manager, int port,
    std::string handle_name) : Node(node_manager, 50), m_digital_in(port) {
    m_handle_name = handle_name.insert(0, "sensor/");

    m_publisher = new ros::Publisher(m_handle_name.c_str(), &m_digital_in_msg);
}

void ADIDigitalInNode::initialize() {
    // Initialize the handler, and set up data to publish
    Node::m_handle->initNode();
    Node::m_handle->advertise(*m_publisher);
}

int ADIDigitalInNode::getValue() {
    return m_digital_in.get_value();
}

void ADIDigitalInNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    m_populateMessage();
    m_publisher->publish(&m_digital_in_msg);
    Node::m_handle->spinOnce();
}

void ADIDigitalInNode::m_populateMessage() {
    // By defalt, C++ maps 0 to false and 1 to true
    m_digital_in_msg.data = (bool)getValue();
}

ADIDigitalInNode::~ADIDigitalInNode() { 
    delete m_publisher;
 }