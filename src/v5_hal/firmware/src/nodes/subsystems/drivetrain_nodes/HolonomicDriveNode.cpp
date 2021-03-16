#include "nodes/subsystems/drivetrain_nodes/HolonomicDriveNode.h"

HolonomicDriveNode::HolonomicDriveNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller,
    MotorNode* left_front_motor, MotorNode* left_rear_motor, 
    MotorNode* right_front_motor, MotorNode* right_rear_motor) : 
    IDriveNode(node_manager), m_controller(controller->getController()),
    m_left_front_motor(left_front_motor), m_left_rear_motor(left_rear_motor), 
    m_right_front_motor(right_front_motor), m_right_rear_motor(right_rear_motor) {
    m_handle_name = handle_name.insert(0, "robot/");
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

void HolonomicDriveNode::m_setLeftFrontVelocity(float velocity) {
    m_left_front_motor->moveVelocity(velocity);
}

void HolonomicDriveNode::m_setLeftRearVelocity(float velocity) {
    m_left_rear_motor->moveVelocity(velocity);
}

void HolonomicDriveNode::m_setRightFrontVelocity(float velocity) {
    m_right_front_motor->moveVelocity(velocity);
}

void HolonomicDriveNode::m_setRightRearVelocity(float velocity) {
    m_right_rear_motor->moveVelocity(velocity);
}

void HolonomicDriveNode::initialize() {
    resetEncoders();
}

void HolonomicDriveNode::resetEncoders() {
    m_left_front_motor->resetEncoder();
    m_left_rear_motor->resetEncoder();
    m_right_front_motor->resetEncoder();
    m_right_rear_motor->resetEncoder();
}

IDriveNode::FourMotorDriveEncoderVals HolonomicDriveNode::getIntegratedEncoderVals() {
    return FourMotorDriveEncoderVals {
        m_left_front_motor->getPosition(),
        m_left_rear_motor->getPosition(),
        m_right_front_motor->getPosition(),
        m_right_rear_motor->getPosition()
    };
}

void HolonomicDriveNode::setDriveVoltage(int x_voltage, int theta_voltage) {
    // TODO use kinematics to map to motor powers
}

void HolonomicDriveNode::setDriveVoltage(int x_voltage, int y_voltage, int theta_voltage) {
    // TODO use kinematics to map to motor powers
}

void HolonomicDriveNode::setDriveVelocity(float x_velocity, float theta_velocity) {
    // TODO use kinematics to map to motor powers
}

void HolonomicDriveNode::setDriveVelocity(float x_velocity, float y_velocity, float theta_velocity) {
    // TODO use kinematics to map to motor powers
}

void HolonomicDriveNode::teleopPeriodic() {
    int y_power = m_controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int x_power = m_controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    int theta_power = m_controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    double front_left = (double)(y_power + x_power + theta_power);
    double back_left = (double)(y_power - x_power + theta_power);
    double front_right = (double)(y_power - x_power - theta_power);
    double back_right = (double)(y_power + x_power - theta_power);

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