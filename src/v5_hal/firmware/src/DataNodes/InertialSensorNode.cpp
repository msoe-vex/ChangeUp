#include "DataNodes/InertialSensorNode.h"

InertialSensorNode::InertialSensorNode(NodeManager* node_manager, 
    int sensor_port, std::string handle_name) : Node(node_manager, 20), 
    m_inertial_sensor(sensor_port) {
    m_handle_name = handle_name.insert(0, "sensor/");
}

void InertialSensorNode::initialize() {

}

double InertialSensorNode::getYaw() {
    return m_inertial_sensor.get_yaw();
}

void InertialSensorNode::periodic() {

}

InertialSensorNode::~InertialSensorNode () {

}