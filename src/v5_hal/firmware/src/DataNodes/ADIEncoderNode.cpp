#include "DataNodes/ADIEncoderNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
ADIEncoderNode::ADIEncoderNode(NodeManager* node_manager, int port_top,
    int port_bottom, std::string handle_name, bool reverse) : Node(node_manager, 50), 
    m_encoder(port_top, port_bottom, reverse) {
    m_handle_name = handle_name.insert(0, "sensor/");

    m_publisher = new ros::Publisher(m_handle_name.c_str(), &m_encoder_msg);
}

void ADIEncoderNode::initialize() {
    // Initialize the handler, and set up data to publish
    Node::m_handle->advertise(*m_publisher);
}

void ADIEncoderNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    m_populateMessage();
    m_publisher->publish(&m_encoder_msg);
}

void ADIEncoderNode::m_populateMessage() {
    m_encoder_msg.data = m_encoder.get_value();
}

ADIEncoderNode::~ADIEncoderNode() { 
    delete m_publisher;
}