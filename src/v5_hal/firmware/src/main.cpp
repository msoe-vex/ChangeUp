#include "main.h"

NodeManager* node_manager = new NodeManager(pros::millis);

// Declare all nodes here
ControllerNode* primary_controller;

MotorNode* left_front_drive;
MotorNode* left_rear_drive;
MotorNode* right_front_drive;
MotorNode* right_rear_drive;
HolonomicDriveNode* holonomic_drive_node;

MotorNode* left_intake;
MotorNode* right_intake;
ADIDigitalOutNode* left_intake_pneumatic;
ADIDigitalOutNode* right_intake_pneumatic;
IntakeNode* intake_node;

MotorNode* bottom_conveyor;
MotorNode* top_conveyor;
ADIAnalogInNode* bottom_conveyor_sensor;
ADIAnalogInNode* top_conveyor_sensor;
ConveyorNode* conveyor_node;


ADIEncoderNode* x_odom_encoder;
ADIEncoderNode* y_odom_encoder;

InertialSensorNode* inertial_sensor;

OdometryNode* odom_node;

Auton* programming_skills_auton;

AutonManagerNode* auton_manager_node;

ConnectionCheckerNode* connection_checker_node;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	// Define all nodes used by the robot here
	primary_controller = new ControllerNode(node_manager, "primary");
	
	/* Define the drivetrain components */
	left_front_drive = new MotorNode(node_manager, 1, "leftFrontDrive", true);
	left_rear_drive = new MotorNode(node_manager, 2, "leftRearDrive", true);
	right_front_drive = new MotorNode(node_manager, 3, "rightFrontDrive", false);
	right_rear_drive = new MotorNode(node_manager, 4, "rightRearDrive", false);

    holonomic_drive_node = new HolonomicDriveNode(node_manager, "drivetrain", primary_controller,
	    HolonomicDriveNode::HolonomicMotors { left_front_drive, left_rear_drive, right_front_drive, right_rear_drive },
		HolonomicDriveKinematics(EncoderConfig { 0, 360, 0.08255 }, 
								 HolonomicDriveKinematics::HolonomicWheelLocations { Vector2d(-1, -1), Vector2d(-1, -1), Vector2d(-1, -1), Vector2d(-1, -1) }));

	/* Define the intake components */
	left_intake = new MotorNode(node_manager, 5, "leftIntake", true);
	right_intake = new MotorNode(node_manager, 6, "rightIntake", false);

	left_intake_pneumatic = new ADIDigitalOutNode(node_manager, "leftIntakeOpen", 'H', false);
	right_intake_pneumatic = new ADIDigitalOutNode(node_manager, "rightIntakeOpen", 'G', false);
	
	intake_node = new IntakeNode(node_manager, "intake", primary_controller, left_intake,
		right_intake, left_intake_pneumatic, right_intake_pneumatic);	

	/* Define the conveyor components */
	bottom_conveyor = new MotorNode(node_manager, 11, "bottomConveyor", true);
	top_conveyor = new MotorNode(node_manager, 12, "topConveyor", true, pros::E_MOTOR_GEARSET_06);

	bottom_conveyor_sensor = new ADIAnalogInNode(node_manager, 'E', "bottomConveyorSensor");
	top_conveyor_sensor = new ADIAnalogInNode(node_manager, 'F', "topConveyorSensor");

	conveyor_node = new ConveyorNode(node_manager, "conveyorNode", primary_controller, bottom_conveyor, top_conveyor, 
		bottom_conveyor_sensor, top_conveyor_sensor);

	/* Define the odometry components */
	x_odom_encoder = new ADIEncoderNode(node_manager, 'A', 'B', "xOdomEncoder");
	y_odom_encoder = new ADIEncoderNode(node_manager, 'C', 'D', "yOdomEncoder", true);

	inertial_sensor = new InertialSensorNode(node_manager, "inertialSensor", "/navx/rpy");

	odom_node = new OdometryNode(node_manager, "odometry", x_odom_encoder, 
		y_odom_encoder, inertial_sensor, OdometryNode::FOLLOWER);
	
    /* Define other components */
	connection_checker_node = new ConnectionCheckerNode(node_manager);

	/* Define autonomous components */
	auton_manager_node = new AutonManagerNode(node_manager, holonomic_drive_node, conveyor_node, intake_node, odom_node, inertial_sensor);

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
void competition_initialize() {
	
}

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
	// Reset all nodes to default configuration
	node_manager->reset();

	// Reset the chosen autonomous and initialize
	auton_manager_node->selected_auton->AutonInit();
	
	// Execute autonomous code
	while (pros::competition::is_autonomous()) {
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
	// Reset all nodes to default configuration
	node_manager->reset();
	
	// Execute teleop code
	while (true) {
		node_manager->executeTeleop();
	}
}
