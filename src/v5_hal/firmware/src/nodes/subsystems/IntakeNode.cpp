#include "nodes/subsystems/IntakeNode.h"

IntakeNode::IntakeNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, 
        MotorNode* left_intake, MotorNode* right_intake, ADIDigitalOutNode* left_intake_pneumatic, 
        ADIDigitalOutNode* right_intake_pneumatic) : Node(node_manager, 10), 
        m_controller(controller->getController()),
        m_left_intake(left_intake),
        m_right_intake(right_intake),
        m_left_intake_pneumatic(left_intake_pneumatic),
        m_right_intake_pneumatic(right_intake_pneumatic) {
    m_handle_name = handle_name.insert(0, "robot/");
}

void IntakeNode::setIntakeVoltage(int voltage) {
    m_left_intake->moveVoltage(voltage);
    m_right_intake->moveVoltage(voltage);
}

void IntakeNode::openIntakes(int open) {
    m_left_intake_pneumatic->setValue(open);
    m_right_intake_pneumatic->setValue(open);
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

    if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        openIntakes(1);
    } else if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
        openIntakes(0);
    }
}

void IntakeNode::autonPeriodic() {
    
}

IntakeNode::~IntakeNode() {
    delete m_left_intake;
    delete m_right_intake;
    delete m_left_intake_pneumatic;
    delete m_right_intake_pneumatic;
}