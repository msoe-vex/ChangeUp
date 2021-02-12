#include "DataNodes/InertialSensorNode.h"

InertialSensorNode::InertialSensorNode(NodeManager* node_manager, 
    int sensor_port, std::string handle_name) : Node(node_manager, 20), 
    m_inertial_sensor(sensor_port) {
    m_handle_name = handle_name.insert(0, "sensor/");
}

void InertialSensorNode::initialize() {
    m_inertial_sensor.reset();
    while(m_inertial_sensor.is_calibrating()){
        pros::delay(10);
    }
}

double InertialSensorNode::getYaw() {
    return m_inertial_sensor.get_yaw() * (M_PI/180);
}

bool InertialSensorNode::isAtAngle(double angle) {
    return ((getYaw() - turning_threshold < angle) && (angle < getYaw() + turining_threshold));
}

void InertialSensorNode::teleopPeriodic() {

}

void InertialSensorNode::autonPeriodic() {

}

InertialSensorNode::~InertialSensorNode () {

}