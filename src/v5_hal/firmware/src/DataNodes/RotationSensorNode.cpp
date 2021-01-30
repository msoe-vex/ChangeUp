#include "DataNodes/RotationSensorNode.h"

RotationSensorNode::RotationSensorNode(NodeManager* node_manager, 
    int rotation_port, std::string handle_name) : Node (node_manager, 20), 
    m_rotation_sensor(rotation_port) {
    m_handle_name = handle_name.insert(0, "sensor/");

    m_publisher = new ros::Publisher(m_handle_name.c_str(), &m_rotation_msg);
}

void RotationSensorNode::initialize() {
    Node::m_handle->advertise(*m_publisher);
}

int RotationSensorNode::getAngle() {
    return m_rotation_sensor.get_angle();
}

int RotationSensorNode::getPosition() {
    return m_rotation_sensor.get_position();
}

int RotationSensorNode::getVelocity() {
    return m_rotation_sensor.get_velocity();
}

void RotationSensorNode::periodic() {
    m_populateMessage();
    // m_publisher->publish(&m_rotation_msg);
}

void RotationSensorNode::m_populateMessage() {
    m_rotation_msg.encoder_angle = getAngle();
    m_rotation_msg.encoder_position = getPosition();
    m_rotation_msg.encoder_velocity = getVelocity();
}

RotationSensorNode::~RotationSensorNode () {
    delete m_publisher;
}