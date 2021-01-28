#include "DataNodes/RotationSensorNode.h"

RotationSensorNode::RotationSensorNode(NodeManager* node_manager, 
    std::string handle_name, int rotation_port) : Node (node_manager, 50), 
    m_rotation_sensor(rotation_port) {
    m_handle_name = handle_name.insert(0, "sensor/");

    m_publisher = new ros::Publisher(m_handle_name.c_str(), &m_rotation_msg);
}

void RotationSensorNode::initialize() {
    Node::m_handle->advertise(*m_publisher);
}

void RotationSensorNode::periodic() {
    m_populateMessage();
    m_publisher->publish(&m_rotation_msg);
}

void RotationSensorNode::m_populateMessage() {
    m_rotation_msg.encoder_angle = m_rotation_sensor.get_angle();
    m_rotation_msg.encoder_position = m_rotation_sensor.get_position();
    m_rotation_msg.encoder_velocity = m_rotation_sensor.get_velocity();
}

RotationSensorNode::~RotationSensorNode () {
    delete m_publisher;
}