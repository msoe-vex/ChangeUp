#include "DataNodes/ADIEncoderNode.h"

// By default, this constructor calls the constructor for the Node object in NodeManager.h
ADIEncoderNode::ADIEncoderNode(NodeManager* nodeManager, int port_top, int port_bottom, bool reverse,
 std::string handleName):Node(nodeManager, 200) {
    m_encoder = new pros::ADIEncoder::ADIEncoder(port_top, port_bottom, reverse);
    m_encoder_msg = new v5_hal::ADIEncoderData();
    m_handle = new ros::NodeHandle();
    m_handle_name = handleName;
}

void ADIEncoderNode::initialize() {
    // Define a string handle for the encoder
    std::string encoder_handle = "Encoder_" + m_handle_name;

    // Create a publisher with the custom title, and message location
    m_publisher = new ros::Publisher(encoder_handle.c_str(), m_encoder_msg);
    
    // Initialize the handler, and set up data relating to what this node publishes
    m_handle->initNode();
    m_handle->advertise(*m_publisher);
}

void ADIEncoderNode::periodic() {
    populateEncoderMsg();
    m_publisher->publish(m_encoder_msg);

    m_handle->spinOnce();
}

void ADIEncoderNode::populateEncoderMsg() {
    m_encoder_msg->ticks = m_encoder->get_value();
}

ADIEncoderNode::~ADIEncoderNode() {
    delete m_encoder;
    delete m_encoder_msg;
    delete m_handle;
}