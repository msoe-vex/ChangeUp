#include "ADIAnalogInNode.h"

// By default, this constructor calls the constructor for the Node object in NodeManager.h
ADIAnalogInNode::ADIAnalogInNode(NodeManager* nodeManager, int port, std::string handleName):Node(nodeManager, 200) {
    m_analog_in = new pros::ADIAnalogIn(port);
    m_analog_in_msg = new v5_hal::ADIAnalogInData();
    m_handle = new ros::NodeHandle();
    m_handle_name = handleName;
}

void ADIAnalogInNode::initialize() {
    // Define a string handle for the analog input
    std::string analog_in_handle = "AnalogIn_" + m_handle_name;

    // Create a publisher with the custom title, and message location
    m_publisher = new ros::Publisher(analog_in_handle.c_str(), m_analog_in_msg);
    
    // Initialize the handler, and set up data relating to what this node publishes
    m_handle->initNode();
    m_handle->advertise(*m_publisher);
}

void ADIAnalogInNode::periodic() {
    populateAnalogInMsg();
    m_publisher->publish(m_analog_in_msg);
    
    m_handle->spinOnce();
}

void ADIAnalogInNode::populateAnalogInMsg() {
    m_analog_in_msg->value = m_analog_in->get_value();
}

ADIAnalogInNode::~ADIAnalogInNode() {
    delete m_analog_in;
    delete m_analog_in_msg;
    delete m_handle;
}