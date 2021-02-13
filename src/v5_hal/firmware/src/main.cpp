#include "main.h"

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

ADIEncoderNode* x_odom_encoder;
ADIEncoderNode* y_odom_encoder;

InertialSensorNode* inertial_sensor;

TankDriveNode* tank_drive_node;
ConveyorNode* conveyor_node;

ADIEncoderNode* x_odometry_encoder;
ADIEncoderNode* y_odometry_encoder;

BatteryNode* battery;

CompetitionStatusNode* competition_status;

ProsTimeNode* pros_time;
Auton* programming_skills_auton;

OdometryNode* odom_node;

ConnectionCheckerNode* connection_checker_node;
OdometryNode* odom_node;

AutonManagerNode* auton_manager_node;

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

	inertial_sensor = new InertialSensorNode(node_manager, 9, "inertialSensor");
	
	x_odometry_encoder = new ADIEncoderNode(node_manager, 1, 2, "xOdometryEncoder", false);
	y_odometry_encoder = new ADIEncoderNode(node_manager, 3, 4, "yOdometryEncoder", false);
	
	battery = new BatteryNode(node_manager, "v5battery");
	competition_status = new CompetitionStatusNode(node_manager, "competitionStatus");
	pros_time = new ProsTimeNode(node_manager, "prosTime");

	bottom_conveyor_sensor = new ADIAnalogInNode(node_manager, 1, "bottomConveyorSensor");
	middle_conveyor_sensor = new ADIAnalogInNode(node_manager, 2, "middleConveyorSensor");
	top_conveyor_sensor = new ADIAnalogInNode(node_manager, 3, "topConveyorSensor");


	x_odom_encoder = new ADIEncoderNode(node_manager, 'E', 'F', "xOdomEncoder");
	y_odom_encoder = new ADIEncoderNode(node_manager, 'G', 'H', "yOdomEncoder");

	inertial_sensor = new InertialSensorNode(node_manager, "inertialSensor", 18);

	tank_drive_node = new TankDriveNode(node_manager, "drivetrain", primary_controller, 
		left_front_drive, left_rear_drive, right_front_drive, right_rear_drive);

	// odom_node = new OdometryNode(node_manager, "odometry", tank_drive_node, x_odom_encoder, 
		// y_odom_encoder, inertial_sensor, OdometryNode::FOLLOWER);

	conveyor_node = new ConveyorNode(node_manager, "conveyor", primary_controller, left_intake,
		right_intake, bottom_rollers, ejection_roller, top_rollers, bottom_conveyor_sensor, middle_conveyor_sensor,
		top_conveyor_sensor);

	auton_manager_node = new AutonManagerNode(node_manager, tank_drive_node, odom_node, conveyor_node);

	connection_checker_node = new ConnectionCheckerNode(node_manager);

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
	while(pros::competition::is_autonomous()) {
		node_manager->executeAuton();
	}
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
	while(true) {
		node_manager->executeTeleop();
	}
}
