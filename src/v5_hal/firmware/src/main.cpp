#include "main.h"
#include "NodeManager.h"
#include "eigen/Eigen/Dense"

NodeManager* node_manager = new NodeManager(pros::millis);

// Declare all nodes here
ControllerNode* primary_controller;

MotorNode* left_front_drive;
MotorNode* left_rear_drive;
MotorNode* right_front_drive;
MotorNode* right_rear_drive;

MotorNode* left_intake;
MotorNode* right_intake;
MotorNode* bottom_rollers;
MotorNode* ejection_roller;
MotorNode* top_rollers;

ADIAnalogInNode* bottom_conveyor_sensor;
ADIAnalogInNode* middle_conveyor_sensor;
ADIAnalogInNode* top_conveyor_sensor;

TankDriveNode* tank_drive_node;
ConveyorNode* conveyor_node;


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// Define all nodes used by the robot here
	primary_controller = new ControllerNode(node_manager, "primary");

	left_front_drive = new MotorNode(node_manager, 3, "leftFrontDrive", false);
	left_rear_drive = new MotorNode(node_manager, 2, "leftRearDrive", true);
	right_front_drive = new MotorNode(node_manager, 1, "rightFrontDrive", true);
	right_rear_drive = new MotorNode(node_manager, 4, "rightRearDrive", false);
	
	left_intake = new MotorNode(node_manager, 13, "leftIntake", true);
	right_intake = new MotorNode(node_manager, 16, "rightIntake", false);
	bottom_rollers = new MotorNode(node_manager, 11, "bottomRollers", false);
	ejection_roller = new MotorNode(node_manager, 15, "ejectionRoller", false);
	top_rollers = new MotorNode(node_manager, 7, "topRollers", true);

	bottom_conveyor_sensor = new ADIAnalogInNode(node_manager, 1, "bottomConveyorSensor");
	middle_conveyor_sensor = new ADIAnalogInNode(node_manager, 2, "middleConveyorSensor");
	top_conveyor_sensor = new ADIAnalogInNode(node_manager, 3, "topConveyorSensor");

	tank_drive_node = new TankDriveNode(node_manager, "drivetrain", primary_controller, 
		left_front_drive, left_rear_drive, right_front_drive, right_rear_drive);

	conveyor_node = new ConveyorNode(node_manager, "conveyor", primary_controller, left_intake,
		right_intake, bottom_rollers, ejection_roller, top_rollers, bottom_conveyor_sensor, middle_conveyor_sensor,
		top_conveyor_sensor);

	// Call the node manager to initialize all of the nodes above
	node_manager->initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

	
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 *
 * NOTE: If custom code is needed outside of the node manager, it should be put
 * into a different task with a wait. Each node has an embedded timing control loop
 * and adding a wait to this thread will disrupt the performance of all nodes.
 */
void opcontrol() {
	while (true) {
		node_manager->executeTeleop();
	}
}
