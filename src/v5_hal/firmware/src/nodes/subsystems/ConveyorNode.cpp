#include "nodes/subsystems/ConveyorNode.h"

ConveyorNode::ConveyorNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, 
        MotorNode* bottom_conveyor_motor, MotorNode* top_conveyor_motor, ADIAnalogInNode* bottom_conveyor_sensor, 
        ADIAnalogInNode* top_conveyor_sensor) : Node(node_manager, 10), 
        m_controller(controller->getController()),
        m_bottom_conveyor_motor(bottom_conveyor_motor),  
        m_top_conveyor_motor(top_conveyor_motor), 
        m_bottom_conveyor_sensor(bottom_conveyor_sensor),  
        m_top_conveyor_sensor(top_conveyor_sensor) {
    m_handle_name = handle_name.insert(0, "robot/");
}

void ConveyorNode::m_updateConveyorHoldingState() {
    bool is_ball_at_top = (m_top_conveyor_sensor->getValue() <= BALL_PRESENT_THRESHOLD);
    bool is_ball_at_bottom = (m_bottom_conveyor_sensor->getValue() <= BALL_PRESENT_THRESHOLD);

    setTopConveyorVoltage(is_ball_at_top ? 0 : MAX_MOTOR_VOLTAGE);
    setBottomConveyorVoltage(is_ball_at_top && is_ball_at_bottom ? 0 : MAX_MOTOR_VOLTAGE);
}

void ConveyorNode::setBottomConveyorVoltage(int voltage) {
    m_bottom_conveyor_motor->moveVoltage(voltage);
}

void ConveyorNode::setTopConveyorVoltage(int voltage) {
    m_top_conveyor_motor->moveVoltage(voltage);
}

void ConveyorNode::setConveyorVoltage(int voltage) {
    setTopConveyorVoltage(voltage);
    setBottomConveyorVoltage(voltage);
}

void ConveyorNode::setConveyorState(ConveyorState conveyorState) {
    m_current_conveyor_state = conveyorState;
}

int ConveyorNode::getNumBallsStored() {
    int ballsStored = 0;

    if (m_bottom_conveyor_sensor->getValue() <= BALL_PRESENT_THRESHOLD) {
        ballsStored++;
    }

    if (m_top_conveyor_sensor->getValue() <= BALL_PRESENT_THRESHOLD) {
        ballsStored++;
    }

    return ballsStored;
}

void ConveyorNode::initialize() {

}

void ConveyorNode::teleopPeriodic() {
    if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { 
        setBottomConveyorVoltage(MAX_MOTOR_VOLTAGE);
        setTopConveyorVoltage(MAX_MOTOR_VOLTAGE);
    } else if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        setBottomConveyorVoltage(-1 * MAX_MOTOR_VOLTAGE);
        setTopConveyorVoltage(-1 * MAX_MOTOR_VOLTAGE);
    } else if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        setBottomConveyorVoltage(-1 * MAX_MOTOR_VOLTAGE);
        setTopConveyorVoltage(0);
    } else {
        setBottomConveyorVoltage(0);
        setTopConveyorVoltage(0);
    }
}

void ConveyorNode::autonPeriodic() {
    switch(m_current_conveyor_state) {
        case STOPPED:
            setConveyorVoltage(0);
        break;
        case HOLDING:
            m_updateConveyorHoldingState();
        break;
        case SCORING:
            setConveyorVoltage(MAX_MOTOR_VOLTAGE);
        break;
        case REVERSE:
            setConveyorVoltage(-1 * MAX_MOTOR_VOLTAGE);
        break;
    }
}

ConveyorNode::~ConveyorNode() {

}