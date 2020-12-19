#include "DataNodes/ADIDigitalIn.h"

// By default, this constructor calls the constructor for the Node object in NodeManager.h
ADIDigitalIn::ADIDigitalIn(NodeManager* nodeManager, int port, std::string handleName):Node(nodeManager, 200) {
    m_digital_in = new pros::ADIDigitalIn::ADIDigitalIn(port);
    m_digital_in_msg = new v5_hal::ADIDigitalIn();
    m_handle = new ros::NodeHandle();
    m_handle_name = handleName;
}

void ADIDigitalIn::initialize() {
    // Define a string handle for the digital input
    std::string digital_in_handle = "DigitalIn_" + m_handle_name;

    // Create a publisher with the custom title, and message location
    m_publisher = new ros::Publisher(digital_in_handle.c_str(), m_digital_in_msg);
    
    // Initialize the handler, and set up data relating to what this node publishes
    m_handle->initNode();
    m_handle->advertise(*m_publisher);
}

void ADIDigitalIn::periodic() {
    populateDigitalInMsg();
    m_publisher->publish(m_digital_in_msg);

    m_handle->spinOnce();
}

void ADIDigitalIn::populateDigitalInMsg() {
    m_digital_in_msg->value = m_digital_in->get_value();
}

ADIDigitalIn::~ADIDigitalIn() {
    delete m_digital_in;
    delete m_digital_in_msg;
    delete m_handle;
}