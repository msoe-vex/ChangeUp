#include "main.h"
#include "NodeManager.h"

NodeManager* nodeManager = new NodeManager(pros::millis);

// Declare all nodes here
MotorNode* leftSwerve1; 
MotorNode* leftSwerve2;
MotorNode* rightSwerve1;
MotorNode* rightSwerve2;
MotorNode* rearSwerve1; 
MotorNode* rearSwerve2; 
MotorNode* leftIntake;
MotorNode* rightIntake;
MotorNode* bottomRollers;
MotorNode* topRollers;
ADIEncoderNode* xOdometryEncoder;
ADIEncoderNode* yOdometryEncoder;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {	
	// Define all nodes used by the robot here
	leftSwerve1 = new MotorNode(nodeManager, 1, new std::string("motor_leftSwerve1"), false);
	leftSwerve2 = new MotorNode(nodeManager, 2, new std::string("motor_leftSwerve2"), false);
	rightSwerve1 = new MotorNode(nodeManager, 3, new std::string("motor_rightSwerve1"), false);
	rightSwerve2 = new MotorNode(nodeManager, 4, new std::string("motor_rightSwerve2"), false);
	rearSwerve1 = new MotorNode(nodeManager, 5, new std::string("motor_rearSwerve1"), false);
	rearSwerve2 = new MotorNode(nodeManager, 6, new std::string("motor_rearSwerve2"), false);

	leftIntake = new MotorNode(nodeManager, 7, new std::string("motor_leftIntake"), false);
	rightIntake = new MotorNode(nodeManager, 8, new std::string("motor_rightIntake"), false);

	bottomRollers = new MotorNode(nodeManager, 9, new std::string("motor_bottomRollers"), false);
	topRollers = new MotorNode(nodeManager, 10, new std::string("motor_topRollers"), false);

	xOdometryEncoder = new ADIEncoderNode(nodeManager, 1, 2, new std::string("sensor_xOdometryEncoder"), false);
	yOdometryEncoder = new ADIEncoderNode(nodeManager, 3, 4, new std::string("sensor_yOdometryEncoder"), false);

	// Call the node manager to initialize all of the nodes above
	nodeManager->initialize();
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
		nodeManager->execute();
	}
}
