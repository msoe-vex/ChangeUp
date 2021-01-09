#include "DataNodes/RotationSensorNode.h"

RotationSensorNode::RotationSensorNode(NodeManager* nodeManager, std::string handleName, int rotationPort)
    : Node (nodeManager, 200) {
    m_rotation = new pros::Rotation(rotationPort);
    m_handle = new ros::NodeHandle();
    m_handle_name = handleName;
    m_rotation_msg = new v5_hal::V5RotationSensor();
}

void RotationSensorNode::initialize() {
    std::string rotationHandle = "Rotation" + m_handle_name;

    m_publisher = new ros::Publisher(rotationHandle.c_str(), m_rotation_msg);

    m_handle->initNode();
    m_handle->advertise(*m_publisher);
}

void RotationSensorNode::periodic() {
    m_populateRotationMsg();
    m_publisher->publish(m_rotation_msg);
    m_handle->spinOnce();
}

void RotationSensorNode::m_populateRotationMsg() {
    m_rotation_msg->encoder_angle = m_rotation->get_angle();
    m_rotation_msg->encoder_position = m_rotation->get_position();
    m_rotation_msg->encoder_velocity = m_rotation->get_velocity();
}

RotationSensorNode::~RotationSensorNode () {
    delete m_rotation;
    delete m_handle;
    delete m_rotation_msg;
    delete m_publisher;
}