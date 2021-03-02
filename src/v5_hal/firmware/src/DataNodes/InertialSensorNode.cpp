#include "DataNodes/InertialSensorNode.h"

InertialSensorNode::InertialSensorNode(NodeManager* node_manager, 
        std::string handle_name, int sensor_port) : Node(node_manager, 20) {
    m_handle_name = handle_name.insert(0, "sensor/");
    m_config = V5;

    m_inertial_sensor = new pros::Imu(sensor_port);
}

InertialSensorNode::InertialSensorNode(NodeManager* node_manager, std::string handle_name, 
        std::string subscribe_handle) : Node(node_manager, 20), 
        m_sub_inertial_sensor_name(subscribe_handle) {
    m_handle_name = handle_name.insert(0, "sensor/");
    m_config = ROS;

    m_inertial_sensor_sub = new ros::Subscriber<v5_hal::RollPitchYaw, InertialSensorNode>
        (m_sub_inertial_sensor_name.c_str(), &InertialSensorNode::m_handleSensorMsg, this);
}

void InertialSensorNode::m_handleSensorMsg(const v5_hal::RollPitchYaw& msg) {
    m_rpy.roll = msg.roll;
    m_rpy.pitch = msg.pitch;
    m_rpy.yaw = msg.yaw;
}

void InertialSensorNode::initialize() {
    switch (m_config) {
        case V5:
            m_inertial_sensor->reset();
            break;
        case ROS:
            Node::m_handle->subscribe(*m_inertial_sensor_sub);
            break;
        default:
            Node::m_handle->logerror("Error initializing inertial sensor");
    }
}

InertialSensorNode::RPY InertialSensorNode::getRPY() {
    return m_rpy;
}

bool InertialSensorNode::isAtAngle(double angle) {
    return ((m_rpy.yaw - turning_threshold < angle) && (angle < m_rpy.yaw + turning_threshold));
}

void InertialSensorNode::teleopPeriodic() {
    switch (m_config) {
        case V5:
            if (!m_inertial_sensor->is_calibrating()) {
                m_rpy.roll = m_inertial_sensor->get_roll();
                m_rpy.pitch = m_inertial_sensor->get_pitch();
                m_rpy.yaw = m_inertial_sensor->get_yaw();
            }
    }
}

void InertialSensorNode::autonPeriodic() {
    switch (m_config) {
        case V5:
            if (!m_inertial_sensor->is_calibrating()) {
                m_rpy.roll = m_inertial_sensor->get_roll();
                m_rpy.pitch = m_inertial_sensor->get_pitch();
                m_rpy.yaw = m_inertial_sensor->get_yaw();
            }
    }
}

InertialSensorNode::~InertialSensorNode () {
    switch (m_config) {
        case V5:
            delete m_inertial_sensor;
        case ROS:
            delete m_inertial_sensor_sub;
    }
}