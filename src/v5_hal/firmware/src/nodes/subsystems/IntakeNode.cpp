#include "nodes/subsystems/IntakeNode.h"

IntakeNode::IntakeNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, 
        MotorNode* left_intake, MotorNode* right_intake, ADIDigitalOutNode* goal_plate, 
        ADIDigitalOutNode* intake_open) : Node(node_manager, 10), 
        m_controller(controller->getController()),
        m_left_intake(left_intake),
        m_right_intake(right_intake),
        m_goal_plate(goal_plate),
        m_intake_open(intake_open) {
    m_handle_name = handle_name.insert(0, "robot/");
}

void IntakeNode::setIntakeVoltage(int voltage) {
    m_left_intake->moveVoltage(voltage);
    m_right_intake->moveVoltage(voltage);
}

void IntakeNode::openIntakes(int open) {
    m_intake_open->setValue(open);
}

void IntakeNode::liftGoalPlate(int open) {
    m_goal_plate->setValue(open);
}

void IntakeNode::initialize() {

}

void IntakeNode::teleopPeriodic() {
    if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { 
        setIntakeVoltage(MAX_MOTOR_VOLTAGE);
    } else if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        setIntakeVoltage(-1 * MAX_MOTOR_VOLTAGE);
    } else {
        setIntakeVoltage(0);
    }

    if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        openIntakes(1);
    } else {
        openIntakes(0);
    }

    if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        liftGoalPlate(0);
    } else if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        liftGoalPlate(1);
    }
}

void IntakeNode::autonPeriodic() {
    
}

IntakeNode::~IntakeNode() {
    delete m_left_intake;
    delete m_right_intake;
    delete m_goal_plate;
    delete m_intake_open;
}