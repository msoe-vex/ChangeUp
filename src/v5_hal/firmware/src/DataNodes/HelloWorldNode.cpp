#include "DataNodes/HelloWorldNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
HelloWorldNode::HelloWorldNode(NodeManager* nodeManager,
    std::string* handleName)
    : Node(nodeManager, 20),
    m_publisher(handleName->insert(0, "debug_").c_str(), &m_string_msg) {
    m_handle_name = handleName;
}

void HelloWorldNode::initialize() {
    // Initialize the handler, and set up data to publish
    Node::m_handle->initNode();
    Node::m_handle->advertise(m_publisher);
}

void HelloWorldNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    populateMessage();
    m_publisher.publish(&m_string_msg);
    Node::m_handle->spinOnce();
}

void HelloWorldNode::populateMessage() {
    std::string debug_message = "Diagnostic: Hello World Node";
    m_string_msg.data = debug_message.c_str();
}

HelloWorldNode::~HelloWorldNode() { delete m_handle_name; }