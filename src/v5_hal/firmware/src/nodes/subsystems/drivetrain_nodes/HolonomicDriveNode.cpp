#include "nodes/subsystems/drivetrain_nodes/HolonomicDriveNode.h"

HolonomicDriveNode::HolonomicDriveNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller,
    MotorNode* left_front_motor, MotorNode* left_rear_motor, 
    MotorNode* right_front_motor, MotorNode* right_rear_motor) : 
    Node(node_manager, 10), m_controller(controller->getController()),
    m_left_front_motor(left_front_motor), m_left_rear_motor(left_rear_motor), 
    m_right_front_motor(right_front_motor), m_right_rear_motor(right_rear_motor) {
    m_handle_name = handle_name.insert(0, "robot/");
}

void HolonomicDriveNode::resetEncoders() {
    m_left_front_motor->resetEncoder();
    m_left_rear_motor->resetEncoder();
    m_right_front_motor->resetEncoder();
    m_right_rear_motor->resetEncoder();
}

void HolonomicDriveNode::m_setLeftFrontVoltage(int voltage) {
    m_left_front_motor->moveVoltage(voltage);
}

void HolonomicDriveNode::m_setLeftRearVoltage(int voltage) {
    m_left_rear_motor->moveVoltage(voltage);
}

void HolonomicDriveNode::m_setRightFrontVoltage(int voltage) {
    m_right_front_motor->moveVoltage(voltage);
}

void HolonomicDriveNode::m_setRightRearVoltage(int voltage) {
    m_right_rear_motor->moveVoltage(voltage);
}

void HolonomicDriveNode::m_setLeftFrontVelocity(int velocity) {
    m_left_front_motor->moveVelocity(velocity);
}

void HolonomicDriveNode::m_setLeftRearVelocity(int velocity) {
    m_left_rear_motor->moveVelocity(velocity);
}

void HolonomicDriveNode::m_setRightFrontVelocity(int velocity) {
    m_right_front_motor->moveVelocity(velocity);
}

void HolonomicDriveNode::m_setRightRearVelocity(int velocity) {
    m_right_rear_motor->moveVelocity(velocity);
}

int HolonomicDriveNode::getLeftFrontPosition() {
    return m_left_front_motor->getPosition();
}

int HolonomicDriveNode::getLeftRearPosition() {
    return m_left_rear_motor->getPosition();
}

int HolonomicDriveNode::getRightFrontPosition() {
    return m_right_front_motor->getPosition();
}

int HolonomicDriveNode::getRightRearPosition() {
    return m_right_rear_motor->getPosition();
}

void HolonomicDriveNode::initialize() {
    resetEncoders();
}

void HolonomicDriveNode::setDriveVoltage(int left_front_voltage, int left_rear_voltage, 
    int right_front_voltage, int right_rear_voltage) {
    m_setLeftFrontVoltage(left_front_voltage);
    m_setLeftRearVoltage(left_rear_voltage);
    m_setRightFrontVoltage(right_front_voltage);
    m_setRightRearVoltage(right_rear_voltage);    
}

void HolonomicDriveNode::setDriveVelocity(float left_front_velocity, float left_rear_velocity, 
    float right_front_velocity, float right_rear_velocity) { //incoming values should be in m/s so we convert to rpm here
    m_setLeftFrontVelocity(left_front_velocity / MAX_WHEEL_SPEED * 200);
    m_setLeftRearVelocity(left_rear_velocity / MAX_WHEEL_SPEED * 200);
    m_setRightFrontVelocity(right_front_velocity / MAX_WHEEL_SPEED * 200);
    m_setRightRearVelocity(right_rear_velocity / MAX_WHEEL_SPEED * 200);
}

void HolonomicDriveNode::teleopPeriodic() {
    int left_y = m_controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int left_x = m_controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    int right_x = m_controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    double front_left = (double)(left_y + left_x + right_x);
    double back_left = (double)(left_y - left_x + right_x);
    double front_right = (double)(left_y - left_x - right_x);
    double back_right = (double)(left_y + left_x - right_x);

    double max_val = std::max({front_left, back_left, front_right, back_right, 127.0});

    m_setLeftFrontVoltage((front_left / max_val) * MAX_MOTOR_VOLTAGE);
    m_setLeftRearVoltage((back_left / max_val) * MAX_MOTOR_VOLTAGE);
    m_setRightFrontVoltage((front_right / max_val) * MAX_MOTOR_VOLTAGE);
    m_setRightRearVoltage((back_right / max_val) * MAX_MOTOR_VOLTAGE);
}

void HolonomicDriveNode::autonPeriodic() {

}

HolonomicDriveNode::~HolonomicDriveNode() {
    delete m_left_front_motor;
    delete m_left_rear_motor;
    delete m_right_front_motor;
    delete m_right_rear_motor;
}