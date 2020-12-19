#include "DataNodes/ADIAnalogIn.h"

// By default, this constructor calls the constructor for the Node object in NodeManager.h
ADIAnalogIn::ADIAnalogIn(NodeManager* nodeManager, int port, std::string handleName):Node(nodeManager, 200) {
    m_analog_in = new pros::ADIAnalogIn::ADIAnalogIn(port);
    m_analog_in_msg = new v5_hal::ADIAnalogIn();
    m_handle = new ros::NodeHandle();
    m_handle_name = handleName;
}

void ADIAnalogIn::initialize() {
    // Define a string handle for the analog input
    std::string analog_in_handle = "AnalogIn_" + m_handle_name;

    // Create a publisher with the custom title, and message location
    m_publisher = new ros::Publisher(analog_in_handle.c_str(), m_analog_in_msg);
    
    // Initialize the handler, and set up data relating to what this node publishes
    m_handle->initNode();
    m_handle->advertise(*m_publisher);
}

void ADIAnalogIn::periodic() {
    populateAnalogInMsg();
    m_publisher->publish(m_analog_in_msg);
    
    m_handle->spinOnce();
}

void ADIAnalogIn::populateAnalogInMsg() {
    m_analog_in_msg->value = m_analog_in->get_value();
}

ADIAnalogIn::~ADIAnalogIn() {
    delete m_analog_in;
    delete m_analog_in_msg;
    delete m_handle;
}