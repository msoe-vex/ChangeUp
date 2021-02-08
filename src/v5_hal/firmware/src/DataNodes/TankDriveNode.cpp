#include "DataNodes/TankDriveNode.h"

TankDriveNode::TankDriveNode(NodeManager* node_manager, std::string handle_name, MotorNode* left_front_motor, 
    MotorNode* left_rear_motor, MotorNode* right_front_motor, MotorNode* right_rear_motor) : Node(node_manager, 10),
    m_left_front_motor(left_front_motor), m_left_rear_motor(left_rear_motor), m_right_front_motor(right_front_motor),
    m_right_rear_motor(right_rear_motor) {
    m_handle_name = handle_name.insert(0, "drivetrain/");
}

void TankDriveNode::m_resetEncoders() {
    m_left_front_motor->resetEncoder();
    m_left_rear_motor->resetEncoder();
    m_right_front_motor->resetEncoder();
    m_right_rear_motor->resetEncoder();
}

void TankDriveNode::m_setLeftVoltage(int voltage) {
    m_left_front_motor->moveVoltage(voltage);
    m_left_rear_motor->moveVoltage(voltage);
}

void TankDriveNode::m_setRightVoltage(int voltage) {
    m_right_front_motor->moveVoltage(voltage);
    m_right_rear_motor->moveVoltage(voltage);
}

void TankDriveNode::m_setLeftDistancePID(double distance, int max_velocity) {
    m_left_front_motor->moveAbsolute(distance, max_velocity);
    m_left_rear_motor->moveAbsolute(distance, max_velocity);
}

void TankDriveNode::m_setRightDistancePID(double distance, int max_velocity) {
    m_right_front_motor->moveAbsolute(distance, max_velocity);
    m_right_rear_motor->moveAbsolute(distance, max_velocity);
}

void TankDriveNode::initialize() {
    m_resetEncoders();
}

void TankDriveNode::setDriveVoltage(int left_voltage, int right_voltage) {
    m_setLeftVoltage(left_voltage);
    m_setRightVoltage(right_voltage);
}

void TankDriveNode::setDriveDistancePID(double left_distance, double right_distance, int max_velocity) {
    m_resetEncoders();
    m_setLeftDistancePID(left_distance, max_velocity);
    m_setRightDistancePID(right_distance, max_velocity);
}

void TankDriveNode::periodic() {

}

TankDriveNode::~TankDriveNode() {
    delete m_left_front_motor;
    delete m_left_rear_motor;
    delete m_right_front_motor;
    delete m_right_rear_motor;
}