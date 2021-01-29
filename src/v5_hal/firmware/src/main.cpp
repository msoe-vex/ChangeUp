#include "main.h"
#include "NodeManager.h"
#include "eigen/Eigen/Dense"

NodeManager* node_manager = new NodeManager(pros::millis);

// Declare all nodes here
ControllerNode* primary_controller;
MotorNode* left_module_1;
MotorNode* left_module_2;
ADIAnalogInNode* left_module_pot;
MotorNode* right_module_1;
MotorNode* right_module_2;
ADIAnalogInNode* right_module_pot;
MotorNode* rear_module_1;
MotorNode* rear_module_2;
ADIAnalogInNode* rear_module_pot;
MotorNode* left_intake;
MotorNode* right_intake;
MotorNode* bottom_rollers;
MotorNode* top_rollers;
ADIEncoderNode* x_odometry_encoder;
ADIEncoderNode* y_odometry_encoder;
BatteryNode* battery;
CompetitionStatusNode* competition_status;
ProsTimeNode* pros_time;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// Define all nodes used by the robot here
	primary_controller = new ControllerNode(node_manager, "primary");

	left_module_1 = new MotorNode(node_manager, 1, "leftModule1", true);
	left_module_2 = new MotorNode(node_manager, 2, "leftModule2", true);
	left_module_pot = new ADIAnalogInNode(node_manager, 8, "leftModulePot");

	right_module_1 = new MotorNode(node_manager, 3, "rightModule1", false);
	right_module_2 = new MotorNode(node_manager, 4, "rightModule2", false);
	right_module_pot = new ADIAnalogInNode(node_manager, 7, "rightModulePot");

	rear_module_1 = new MotorNode(node_manager, 5, "rearModule1", false);
	rear_module_2 = new MotorNode(node_manager, 6, "rearModule2", false);
	rear_module_pot = new ADIAnalogInNode(node_manager, 6, "rearModulePot");
	
	left_intake = new MotorNode(node_manager, 7, "leftIntake", false);
	right_intake = new MotorNode(node_manager, 8, "rightIntake", false);
	bottom_rollers = new MotorNode(node_manager, 9, "bottomRollers", false);
	top_rollers = new MotorNode(node_manager, 10, "topRollers", false);
	
	x_odometry_encoder = new ADIEncoderNode(node_manager, 1, 2, "xOdometryEncoder", false);
	y_odometry_encoder = new ADIEncoderNode(node_manager, 3, 4, "yOdometryEncoder", false);
	
	battery = new BatteryNode(node_manager, "v5battery");
	competition_status = new CompetitionStatusNode(node_manager, "competitionStatus");
	pros_time = new ProsTimeNode(node_manager, "prosTime");

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
void autonomous() {}

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
		node_manager->execute();
	}
}
