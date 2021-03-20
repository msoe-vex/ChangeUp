#include "nodes/subsystems/ConveyorNode.h"

ConveyorNode::ConveyorNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, 
    MotorNode* bottom_conveyor_motor, MotorNode* ejection_roller_motor, MotorNode* top_conveyor_motor, 
    ADIAnalogInNode* bottom_conveyor_sensor, ADIAnalogInNode* middle_conveyor_sensor, 
    ADIAnalogInNode* top_conveyor_sensor) : Node(node_manager, 10), 
    m_controller(controller->getController()),
    m_bottom_conveyor_motor(bottom_conveyor_motor), 
    m_ejection_roller_motor(ejection_roller_motor), 
    m_top_conveyor_motor(top_conveyor_motor), 
    m_bottom_conveyor_sensor(bottom_conveyor_sensor), 
    m_middle_conveyor_sensor(middle_conveyor_sensor), 
    m_top_conveyor_sensor(top_conveyor_sensor) {
    m_handle_name = handle_name.insert(0, "robot/");
}

void ConveyorNode::m_updateConveyorHoldingState() {
    if (m_top_conveyor_sensor->getValue() <= BALL_PRESENT_THRESHOLD) {
        // Ball is waiting on top; stop spinning balls up
        setTopConveyorVoltage(0);
    } else if (m_middle_conveyor_sensor->getValue() <= BALL_PRESENT_THRESHOLD &&
        m_bottom_conveyor_sensor->getValue() > BALL_PRESENT_THRESHOLD) {
        // Ball isn't on top, but one is waiting in the middle with nothing behind
        setTopConveyorVoltage(0);
    } else if (m_middle_conveyor_sensor->getValue() <= BALL_PRESENT_THRESHOLD &&
        m_bottom_conveyor_sensor->getValue() <= BALL_PRESENT_THRESHOLD) {
        // Balls are present in the bottom and middle positions
        setTopConveyorVoltage(MAX_MOTOR_VOLTAGE);
    } else {
        setTopConveyorVoltage(MAX_MOTOR_VOLTAGE);
    }
}

void ConveyorNode::setBottomConveyorVoltage(int voltage) {
    m_bottom_conveyor_motor->moveVoltage(voltage);
}

void ConveyorNode::setEjectionRollerVoltage(int voltage) {
    m_ejection_roller_motor->moveVoltage(voltage);
}

void ConveyorNode::setTopConveyorVoltage(int voltage) {
    m_top_conveyor_motor->moveVoltage(voltage);
}

void ConveyorNode::setConveyorState(ConveyorState conveyorState) {
    m_current_conveyor_state = conveyorState;
}

int ConveyorNode::getNumBallsStored() {
    int ballsStored = 0;

    if (m_bottom_conveyor_sensor->getValue() <= BALL_PRESENT_THRESHOLD) {
        ballsStored++;
    }

    if (m_middle_conveyor_sensor->getValue() <= BALL_PRESENT_THRESHOLD) {
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

}

void ConveyorNode::autonPeriodic() {
    
}

ConveyorNode::~ConveyorNode() {

}