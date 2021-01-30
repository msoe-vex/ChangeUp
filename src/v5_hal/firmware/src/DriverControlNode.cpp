#include "eigen/Eigen/Dense"
#include "math.h"
#include "SwerveController.h"
#include "DataNodes/DriverControlNode.h"

// By default, this constructor calls the constructor for the Node object in
// NodeManager.h
DriverControlNode::DriverControlNode(NodeManager* node_manager, 
    std::string handle_name) : Node(node_manager, 20) {
    m_handle_name = handle_name.insert(0, "driveControl/");

    left_module_location(0) = left_module_location_x;
    left_module_location(1) = left_module_location_y;
    right_module_location(0) = right_module_location_x;
    right_module_location(1) = right_module_location_y;
    rear_module_location(0) = rear_module_location_x;
    rear_module_location(1) = rear_module_location_y;

    SwerveController(left_module_location, right_module_location, rear_module_location, rotation_angle_threshold,
        max_velocity, max_rotation_velocity, kP, kI, kD);

}

void DriverControlNode::initialize() {
    // Initialize the handler, and set up data to publish
    Node::m_handle->initNode();
}

void DriverControlNode::periodic() {
    // Publish data when called, and spin the handler to send data to the
    // coprocessor on the published topic
    Node::m_handle->spinOnce();

    assignActualAngle(0.0, 0.0, 0.0);
    calculateLeftModule(target_velocity, rotation_velocity);
    calculateRightModule(target_velocity, rotation_velocity);
    calculateRearModule(target_velocity, rotation_velocity);
}

DriverControlNode::~DriverControlNode() { 
    delete m_publisher; 
}
