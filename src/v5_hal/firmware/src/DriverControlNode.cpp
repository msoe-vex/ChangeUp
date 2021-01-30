#include "DriverControlNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
DriverControlNode::DriverControlNode(NodeManager* node_manager, MotorNode* left_swerve_1, MotorNode* left_swerve_2, 
        ADIAnalogInNode* left_swerve_pot, MotorNode* right_swerve_1, MotorNode* right_swerve_2, 
        ADIAnalogInNode* right_swerve_pot, MotorNode* rear_swerve_1, MotorNode* rear_swerve_2, 
        ADIAnalogInNode* rear_swerve_pot, ControllerNode* controller_primary) : Node(node_manager, 20), 
    swerveController(left_module_location, right_module_location, rear_module_location, rotation_angle_threshold,
    max_velocity, max_rotation_velocity, kP, kI, kD), left_swerve_1(left_swerve_1), left_swerve_2(left_swerve_2),
    left_swerve_pot(left_swerve_pot), right_swerve_1(right_swerve_1), right_swerve_2(right_swerve_2),
    right_swerve_pot(right_swerve_pot), rear_swerve_1(rear_swerve_1), rear_swerve_2(rear_swerve_2),
    rear_swerve_pot(rear_swerve_pot), controller_primary(controller_primary->getController()) {

    left_module_location(0) = left_module_location_x;
    left_module_location(1) = left_module_location_y;
    right_module_location(0) = right_module_location_x;
    right_module_location(1) = right_module_location_y;
    rear_module_location(0) = rear_module_location_x;
    rear_module_location(1) = rear_module_location_y;
}

void DriverControlNode::initialize() {
    Node::m_handle->initNode();
}

void DriverControlNode::periodic() {

    target_velocity(0) = (controller_primary->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X) / 127) * max_velocity;
    target_velocity(0) = (controller_primary->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127) * max_velocity;
    rotation_velocity = (controller_primary->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127) * max_rotation_velocity;

    swerveController.assignActualAngle(left_swerve_pot->getValue(), right_swerve_pot->getValue(), rear_swerve_pot->getValue());
    left_swerve_1->moveVoltage(swerveController.calculateLeftModule(target_velocity, rotation_velocity).left_motor_power);
    left_swerve_2->moveVoltage(swerveController.calculateLeftModule(target_velocity, rotation_velocity).right_motor_power);
    right_swerve_1->moveVoltage(swerveController.calculateRightModule(target_velocity, rotation_velocity).left_motor_power);
    right_swerve_2->moveVoltage(swerveController.calculateRightModule(target_velocity, rotation_velocity).right_motor_power);
    rear_swerve_1->moveVoltage(swerveController.calculateRearModule(target_velocity, rotation_velocity).left_motor_power);
    rear_swerve_2->moveVoltage(swerveController.calculateRearModule(target_velocity, rotation_velocity).right_motor_power);

}

DriverControlNode::~DriverControlNode() { 
    delete m_publisher; 
}
