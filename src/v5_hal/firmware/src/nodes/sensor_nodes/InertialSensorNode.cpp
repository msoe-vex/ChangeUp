#include "nodes/sensor_nodes/InertialSensorNode.h"

InertialSensorNode::InertialSensorNode(NodeManager* node_manager, 
        std::string handle_name, int sensor_port) : Node(node_manager, 20), 
        m_yaw(0), m_gyro_offset_angle(-M_PI_2) {
    m_handle_name = handle_name.insert(0, "sensor/");
    m_config = V5;

    m_inertial_sensor = new pros::Imu(sensor_port);
}

InertialSensorNode::InertialSensorNode(NodeManager* node_manager, std::string handle_name, 
        std::string subscribe_handle) : Node(node_manager, 20), m_yaw(0), m_gyro_offset_angle(M_PI_2),
        m_sub_inertial_sensor_name(subscribe_handle) {
    m_handle_name = handle_name.insert(0, "sensor/");
    m_config = ROS;

    m_inertial_sensor_sub = new ros::Subscriber<v5_hal::RollPitchYaw, InertialSensorNode>
        (m_sub_inertial_sensor_name.c_str(), &InertialSensorNode::m_handleSensorMsg, this);
}

void InertialSensorNode::m_handleSensorMsg(const v5_hal::RollPitchYaw& msg) {
    Eigen::Rotation2Dd current_angle(msg.yaw  * (M_PI/180));
    m_yaw = (current_angle.inverse()) * m_gyro_offset_angle;
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

Eigen::Rotation2Dd InertialSensorNode::getYaw() {
    return m_yaw;
}

bool InertialSensorNode::isAtAngle(Eigen::Rotation2Dd angle) {
    return fabs((m_yaw * angle.inverse()).smallestAngle()) < turning_threshold;
}

void InertialSensorNode::teleopPeriodic() {
    switch (m_config) {
        case V5:
            if (!(m_inertial_sensor->is_calibrating())) {
                Eigen::Rotation2Dd current_angle(m_inertial_sensor->get_yaw() * -(M_PI/180));
                m_yaw = current_angle * m_gyro_offset_angle.inverse();
            }
    }

    // std::string msg = m_handle_name + " value: " + std::to_string(getYaw().angle());
    // m_handle->logwarn(msg.c_str());
}

void InertialSensorNode::autonPeriodic() {
    switch (m_config) {
        case V5:
            if (!m_inertial_sensor->is_calibrating()) {
                Eigen::Rotation2Dd current_angle(m_inertial_sensor->get_yaw() * -(M_PI/180));
                m_yaw = current_angle * m_gyro_offset_angle.inverse();
            }
    }

    Logger::logInfo("Robot angle: " + std::to_string(getYaw().angle()));
}

InertialSensorNode::~InertialSensorNode () {
    switch (m_config) {
        case V5:
            delete m_inertial_sensor;
        case ROS:
            delete m_inertial_sensor_sub;
    }
}