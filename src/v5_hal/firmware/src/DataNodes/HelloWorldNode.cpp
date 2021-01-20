#include "DataNodes/HelloWorldNode.h"

// By default, this constructor calls the constructor for the Node object in NodeManager.h
HelloWorldNode::HelloWorldNode(NodeManager* nodeManager, std::string* handleName):Node(nodeManager, 20UL),
        m_string_msg(),
        m_publisher(handleName->c_str(), &m_string_msg) {
    m_handle_name = handleName;
}

void HelloWorldNode::initialize() {    
    // Initialize the handler, and set up data relating to what this node publishes
    Node::m_handle->initNode();
    Node::m_handle->advertise(m_publisher);
}

void HelloWorldNode::periodic() {
    populateMessage();
    m_publisher.publish(&m_string_msg);
    Node::m_handle->spinOnce();
}

void HelloWorldNode::populateMessage() {
    std::string debug_message = "Diagnostic: Hello World Node";
    m_string_msg.data = debug_message.c_str();
}

HelloWorldNode::~HelloWorldNode() {
    delete m_handle_name;
}