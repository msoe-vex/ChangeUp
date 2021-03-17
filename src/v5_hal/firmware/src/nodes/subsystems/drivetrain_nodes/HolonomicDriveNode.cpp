#include "nodes/subsystems/drivetrain_nodes/HolonomicDriveNode.h"

HolonomicDriveNode::HolonomicDriveNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller,
    HolonomicMotors motors, HolonomicDriveKinematics kinematics) : 
    IDriveNode(node_manager), m_controller(controller->getController()),
    m_motors(motors), m_kinematics(kinematics) {
    m_handle_name = handle_name.insert(0, "robot/");
}

void HolonomicDriveNode::m_setLeftFrontVoltage(int voltage) {
    m_motors.left_front_motor->moveVoltage(voltage);
}

void HolonomicDriveNode::m_setLeftRearVoltage(int voltage) {
    m_motors.left_rear_motor->moveVoltage(voltage);
}

void HolonomicDriveNode::m_setRightFrontVoltage(int voltage) {
    m_motors.right_front_motor->moveVoltage(voltage);
}

void HolonomicDriveNode::m_setRightRearVoltage(int voltage) {
    m_motors.right_rear_motor->moveVoltage(voltage);
}

void HolonomicDriveNode::m_setLeftFrontVelocity(float velocity) {
    m_motors.left_front_motor->moveVelocity(velocity);
}

void HolonomicDriveNode::m_setLeftRearVelocity(float velocity) {
    m_motors.left_rear_motor->moveVelocity(velocity);
}

void HolonomicDriveNode::m_setRightFrontVelocity(float velocity) {
    m_motors.right_front_motor->moveVelocity(velocity);
}

void HolonomicDriveNode::m_setRightRearVelocity(float velocity) {
    m_motors.right_rear_motor->moveVelocity(velocity);
}

void HolonomicDriveNode::initialize() {
    resetEncoders();
}

void HolonomicDriveNode::resetEncoders() {
    m_motors.left_front_motor->resetEncoder();
    m_motors.left_rear_motor->resetEncoder();
    m_motors.right_front_motor->resetEncoder();
    m_motors.right_rear_motor->resetEncoder();
}

IDriveNode::FourMotorDriveEncoderVals HolonomicDriveNode::getIntegratedEncoderVals() {
    return FourMotorDriveEncoderVals {
        m_motors.left_front_motor->getPosition(),
        m_motors.left_rear_motor->getPosition(),
        m_motors.right_front_motor->getPosition(),
        m_motors.right_rear_motor->getPosition()
    };
}

void HolonomicDriveNode::setDriveVoltage(int x_voltage, int theta_voltage) {
    setDriveVoltage(x_voltage, 0, theta_voltage);
}

void HolonomicDriveNode::setDriveVoltage(int x_voltage, int y_voltage, int theta_voltage) {
    IDriveKinematics::FourMotorPercentages motor_percentages = 
        m_kinematics.inverseKinematics(x_voltage, y_voltage, theta_voltage, MAX_MOTOR_VOLTAGE);

    m_setLeftFrontVoltage(motor_percentages.left_front_percent * MAX_MOTOR_VOLTAGE);
    m_setLeftRearVoltage(motor_percentages.left_front_percent * MAX_MOTOR_VOLTAGE);
    m_setRightFrontVoltage(motor_percentages.left_front_percent * MAX_MOTOR_VOLTAGE);
    m_setRightRearVoltage(motor_percentages.left_front_percent * MAX_MOTOR_VOLTAGE);
}

void HolonomicDriveNode::setDriveVelocity(float x_velocity, float theta_velocity) {
    setDriveVelocity(x_velocity, 0, theta_velocity);
}

void HolonomicDriveNode::setDriveVelocity(float x_velocity, float y_velocity, float theta_velocity) {
    IDriveKinematics::FourMotorPercentages motor_percentages = 
        m_kinematics.inverseKinematics(x_velocity, y_velocity, theta_velocity, MAX_VELOCITY);

    m_setLeftFrontVelocity(motor_percentages.left_front_percent * MAX_VELOCITY);
    m_setLeftRearVelocity(motor_percentages.left_front_percent * MAX_VELOCITY);
    m_setRightFrontVelocity(motor_percentages.left_front_percent * MAX_VELOCITY);
    m_setRightRearVelocity(motor_percentages.left_front_percent * MAX_VELOCITY);
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