#include "DataNodes/HelloWorldNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
HelloWorldNode::HelloWorldNode(NodeManager* node_manager,
    std::string handle_name) : Node(node_manager, 1000) {
    m_handle_name = handle_name.insert(0, "debug/");

    m_publisher = new ros::Publisher(m_handle_name.c_str(), &m_string_msg);
}

void HelloWorldNode::initialize() {
    // Initialize the handler, and set up data to publish
    Node::m_handle->initNode();
    Node::m_handle->advertise(*m_publisher);
}

void HelloWorldNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    m_populateMessage();
    m_publisher->publish(&m_string_msg);
    Node::m_handle->spinOnce();
}

void HelloWorldNode::m_populateMessage() {
    std::string debug_message = "Diagnostic: Hello World Node";
    m_string_msg.data = debug_message.c_str();
}

HelloWorldNode::~HelloWorldNode() { 
    delete m_publisher;
 }