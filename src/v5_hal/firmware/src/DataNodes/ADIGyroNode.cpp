#include "DataNodes/ADIGyroNode.h"

// By default, this constructor calls the constructor for the Node object in NodeManager.h
ADIGyroNode::ADIGyroNode(NodeManager* nodeManager, int port, double multiplier, 
 std::string handleName):Node(nodeManager, 200) {
    m_gyro = new pros::ADIGyro::ADIGyro(port, multiplier);
    m_gyro_msg = new v5_hal::ADIGyroData();
    m_handle = new ros::NodeHandle();
    m_handle_name = handleName;
}

void ADIGyroNode::initialize() {
    // Define a string handle for the gyro sensor
    std::string gyro_handle = "Gyro_" + m_handle_name;

    // Create a publisher with the custom title, and message location
    m_publisher = new ros::Publisher(gyro_handle.c_str(), m_gyro_msg);
    
    // Initialize the handler, and set up data relating to what this node publishes
    m_handle->initNode();
    m_handle->advertise(*m_publisher);
}

void ADIGyroNode::periodic() {
    populateGyroMsg();
    m_publisher->publish(m_gyro_msg);

    m_handle->spinOnce();
}

void ADIGyroNode::populateGyroMsg() {
    m_gyro_msg->degrees = m_gyro->get_value();
}

ADIGyroNode::~ADIGyroNode() {
    delete m_gyro;
    delete m_gyro_msg;
    delete m_handle;
}