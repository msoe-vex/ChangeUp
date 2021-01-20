#include "DataNodes/ADIEncoderNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
ADIEncoderNode::ADIEncoderNode(NodeManager* nodeManager, int port_top,
                               int port_bottom, bool reverse,
                               std::string* handleName)
    : Node(nodeManager, 20),
      m_encoder(port_top, port_bottom, reverse),
      m_publisher(handleName->c_str(), &m_encoder_msg) {
    m_handle_name = handleName;
}

void ADIEncoderNode::initialize() {
    // Initialize the handler, and set up data to publish
    Node::m_handle->initNode();
    Node::m_handle->advertise(m_publisher);
}

void ADIEncoderNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    populateMessage();
    m_publisher.publish(&m_encoder_msg);
    Node::m_handle->spinOnce();
}

void ADIEncoderNode::populateMessage() {
    m_encoder_msg.ticks = m_encoder.get_value();
}

ADIEncoderNode::~ADIEncoderNode() { delete m_handle_name; }