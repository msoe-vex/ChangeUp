#include "DriverControlNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
DriverControlNode::DriverControlNode(NodeManager* node_manager, pros::Motor left_swerve_1, pros::Motor left_swerve_2, 
        pros::ADIAnalogIn left_swerve_pot, pros::Motor right_swerve_1, pros::Motor right_swerve_2, 
        pros::ADIAnalogIn right_swerve_pot, pros::Motor rear_swerve_1, pros::Motor rear_swerve_2, 
        pros::ADIAnalogIn rear_swerve_pot) : Node(node_manager, 20), 
    swerveController(left_module_location, right_module_location, rear_module_location, rotation_angle_threshold,
    max_velocity, max_rotation_velocity, kP, kI, kD), left_swerve_1(left_swerve_1), left_swerve_2(left_swerve_2),
    left_swerve_pot(left_swerve_pot), right_swerve_1(right_swerve_1), right_swerve_2(right_swerve_2),
    right_swerve_pot(right_swerve_pot), rear_swerve_1(rear_swerve_1), rear_swerve_2(rear_swerve_2),
    rear_swerve_pot(rear_swerve_pot) {

    left_module_location(0) = left_module_location_x;
    left_module_location(1) = left_module_location_y;
    right_module_location(0) = right_module_location_x;
    right_module_location(1) = right_module_location_y;
    rear_module_location(0) = rear_module_location_x;
    rear_module_location(1) = rear_module_location_y;

    left_swerve_1.move(-127);

}

void DriverControlNode::initialize() {
    Node::m_handle->initNode();
}

void DriverControlNode::periodic() {

    swerveController.assignActualAngle(left_swerve_pot.get_value(), right_swerve_pot.get_value(), rear_swerve_pot.get_value());
    left_swerve_1.move(swerveController.calculateLeftModule(target_velocity, rotation_velocity).left_motor_power);
    left_swerve_2.move(swerveController.calculateLeftModule(target_velocity, rotation_velocity).right_motor_power);
    right_swerve_1.move(swerveController.calculateRightModule(target_velocity, rotation_velocity).left_motor_power);
    right_swerve_2.move(swerveController.calculateRightModule(target_velocity, rotation_velocity).right_motor_power);
    rear_swerve_1.move(swerveController.calculateRearModule(target_velocity, rotation_velocity).left_motor_power);
    rear_swerve_2.move(swerveController.calculateRearModule(target_velocity, rotation_velocity).right_motor_power);

}

DriverControlNode::~DriverControlNode() { 
    delete m_publisher; 
}
