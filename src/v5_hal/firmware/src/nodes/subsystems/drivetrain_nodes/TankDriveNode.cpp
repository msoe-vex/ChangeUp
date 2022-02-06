#include "nodes/subsystems/drivetrain_nodes/TankDriveNode.h"

TankDriveNode::TankDriveNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, 
    InertialSensorNode* inertial_sensor, TankEightMotors motors) : IDriveNode(node_manager), 
    m_controller(controller->getController()), m_motors(motors) {
    m_handle_name = handle_name.insert(0, "robot/");
}

void TankDriveNode::resetEncoders() {
    m_motors.left_motor_1->resetEncoder();
    m_motors.left_motor_2->resetEncoder();
    m_motors.left_motor_3->resetEncoder();
    m_motors.left_motor_4->resetEncoder();
    m_motors.right_motor_1->resetEncoder();
    m_motors.right_motor_2->resetEncoder();
    m_motors.right_motor_3->resetEncoder();
    m_motors.right_motor_4->resetEncoder();
}

IDriveNode::FourMotorDriveEncoderVals TankDriveNode::getIntegratedEncoderVals() {
    return FourMotorDriveEncoderVals {
        m_motors.left_motor_1->getPosition(),
        m_motors.left_motor_3->getPosition(),
        m_motors.right_motor_2->getPosition(),
        m_motors.right_motor_4->getPosition()
    };
}

void TankDriveNode::m_setLeftVoltage(int voltage) {
    m_motors.left_motor_1->moveVoltage(voltage);
    m_motors.left_motor_2->moveVoltage(voltage);
    m_motors.left_motor_3->moveVoltage(voltage);
    m_motors.left_motor_4->moveVoltage(voltage);
}

void TankDriveNode::m_setRightVoltage(int voltage) {
    m_motors.right_motor_1->moveVoltage(voltage);
    m_motors.right_motor_2->moveVoltage(voltage);
    m_motors.right_motor_3->moveVoltage(voltage);
    m_motors.right_motor_4->moveVoltage(voltage);
}

void TankDriveNode::m_setLeftVelocity(float velocity) {
    m_motors.left_motor_1->moveVelocity(velocity);
    m_motors.left_motor_2->moveVelocity(velocity);
    m_motors.left_motor_3->moveVelocity(velocity);
    m_motors.left_motor_4->moveVelocity(velocity);
}

void TankDriveNode::m_setRightVelocity(float velocity) {
    m_motors.right_motor_1->moveVelocity(velocity);
    m_motors.right_motor_2->moveVelocity(velocity);
    m_motors.right_motor_3->moveVelocity(velocity);
    m_motors.right_motor_4->moveVelocity(velocity);
}

void TankDriveNode::m_setLeftDistancePID(double distance, int max_velocity) {
    m_motors.left_motor_1->moveAbsolute(distance, max_velocity);
    m_motors.left_motor_2->moveAbsolute(distance, max_velocity);
    m_motors.left_motor_3->moveAbsolute(distance, max_velocity);
    m_motors.left_motor_4->moveAbsolute(distance, max_velocity);
}

int TankDriveNode::getLeftDistancePID() {
    return m_motors.left_motor_1->getPosition();
}

int TankDriveNode::getRightDistancePID() {
    return m_motors.right_motor_1->getPosition();
}

void TankDriveNode::m_setRightDistancePID(double distance, int max_velocity) {
    m_motors.right_motor_1->moveAbsolute(distance, max_velocity);
    m_motors.right_motor_2->moveAbsolute(distance, max_velocity);
    m_motors.right_motor_3->moveAbsolute(distance, max_velocity);
    m_motors.right_motor_4->moveAbsolute(distance, max_velocity);
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
    delete m_motors.left_motor_1;
    delete m_motors.left_motor_2;
    delete m_motors.left_motor_3;
    delete m_motors.left_motor_4;
    delete m_motors.right_motor_1;
    delete m_motors.right_motor_2;
    delete m_motors.right_motor_3;
    delete m_motors.right_motor_4;
}