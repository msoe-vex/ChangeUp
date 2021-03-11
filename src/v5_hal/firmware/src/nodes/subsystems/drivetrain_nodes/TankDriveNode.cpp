#include "nodes/subsystems/drivetrain_nodes/TankDriveNode.h"

TankDriveNode::TankDriveNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, 
    MotorNode* left_front_motor, MotorNode* left_rear_motor, MotorNode* right_front_motor, 
    MotorNode* right_rear_motor) : Node(node_manager, 10), m_controller(controller->getController()), m_left_front_motor(left_front_motor), 
    m_left_rear_motor(left_rear_motor), m_right_front_motor(right_front_motor), m_right_rear_motor(right_rear_motor) {
    m_handle_name = handle_name.insert(0, "robot/");
}

void TankDriveNode::resetEncoders() {
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

void TankDriveNode::m_setLeftVelocity(float velocity) {
    m_left_front_motor->moveVelocity(velocity);
    m_left_rear_motor->moveVelocity(velocity);
}

void TankDriveNode::m_setRightVelocity(float velocity) {
    m_right_front_motor->moveVelocity(velocity);
    m_right_rear_motor->moveVelocity(velocity);
}

void TankDriveNode::m_setLeftDistancePID(double distance, int max_velocity) {
    m_left_front_motor->moveAbsolute(distance, max_velocity);
    m_left_rear_motor->moveAbsolute(distance, max_velocity);
}

int TankDriveNode::getLeftDistancePID() {
    return m_left_front_motor->getPosition();
}

int TankDriveNode::getRightDistancePID() {
    return m_right_front_motor->getPosition();
}

void TankDriveNode::m_setRightDistancePID(double distance, int max_velocity) {
    m_right_front_motor->moveAbsolute(distance, max_velocity);
    m_right_rear_motor->moveAbsolute(distance, max_velocity);
}

void TankDriveNode::initialize() {
    resetEncoders();
}

void TankDriveNode::setDriveVoltage(int left_voltage, int right_voltage) {
    m_setLeftVoltage(left_voltage);
    m_setRightVoltage(right_voltage);
}

void TankDriveNode::setDriveVelocity(float left_velocity, float right_velocity) { //incoming values should be in m/s so we convert to rpm here
    m_setLeftVelocity(left_velocity / MAX_ROBOT_SPEED * 200);
    m_setRightVelocity(right_velocity / MAX_ROBOT_SPEED * 200);
}

void TankDriveNode::setDriveDistancePID(double left_distance, double right_distance, int max_velocity) {
    resetEncoders();
    m_setLeftDistancePID(left_distance, max_velocity);
    m_setRightDistancePID(right_distance, max_velocity);
}

void TankDriveNode::teleopPeriodic() {
    m_setLeftVoltage(((m_controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) / 127.0) * MAX_MOTOR_VOLTAGE);
    m_setRightVoltage(((m_controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) / 127.0) * MAX_MOTOR_VOLTAGE);
}

void TankDriveNode::autonPeriodic() {

}

TankDriveNode::~TankDriveNode() {
    delete m_left_front_motor;
    delete m_left_rear_motor;
    delete m_right_front_motor;
    delete m_right_rear_motor;
}