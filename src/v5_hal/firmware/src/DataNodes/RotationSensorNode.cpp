#include "DataNodes/RotationSensorNode.h"

RotationSensorNode::RotationSensorNode(NodeManager* nodeManager, std::string handleName, int rotationPort)
    : Node (nodeManager, 200), m_rotation(rotationPort), m_publisher(handleName->insert(0, "motor_").c_str(), &m_motor_msg) {
    m_handle_name = handleName;
}

void RotationSensorNode::initialize() {
    Node::m_handle->initNode();
    Node::m_handle->advertise(m_publisher);
}

void RotationSensorNode::periodic() {
    populateMessage();
    m_publisher->publish(&m_rotation_msg);
    Node::m_handle->spinOnce();
}

void RotationSensorNode::m_populateRotationMsg() {
    m_rotation_msg.encoder_angle = m_rotation->get_angle();
    m_rotation_msg.encoder_position = m_rotation->get_position();
    m_rotation_msg.encoder_velocity = m_rotation->get_velocity();
}

RotationSensorNode::~RotationSensorNode () {
    delete m_handle_name;
}