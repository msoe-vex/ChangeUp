#include "DataNodes/ConveyorNode.h"

ConveyorNode::ConveyorNode(NodeManager* node_manager, std::string handle_name, ControllerNode* controller, 
    MotorNode* left_intake, MotorNode* right_intake,MotorNode* bottom_conveyor_motor, MotorNode* ejection_roller_motor, 
    MotorNode* top_conveyor_motor, ADIAnalogInNode* bottom_conveyor_sensor, ADIAnalogInNode* middle_conveyor_sensor, 
    ADIAnalogInNode* top_conveyor_sensor, ADIDigitalOutNode* digital_out_node) : Node(node_manager, 10), 
    m_controller(controller->getController()),
    m_left_intake(left_intake),
    m_right_intake(right_intake),
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
        setTopConveyorVoltage(12000);
    } else {
        setTopConveyorVoltage(12000);
    }
}

void ConveyorNode::setIntakeVoltage(int voltage) {
    m_left_intake->moveVoltage(voltage);
    m_right_intake->moveVoltage(voltage);
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

void ConveyorNode::openIntakes(int open) {
    m_intake_pneumatics->setValue(open);
}

void ConveyorNode::initialize() {

}

void ConveyorNode::teleopPeriodic() {
    int intake_voltage = 0;
	int bottom_conveyor_voltage = 0;
    int top_conveyor_voltage = 0;
	int ejection_roller_voltage = 0;

    if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        m_enableStateMachine = !m_enableStateMachine;
        pros::delay(100);
    }

    if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        m_enableStateMachine = false;
        top_conveyor_voltage = 12000;
        ejection_roller_voltage = 12000;

        if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
			ejection_roller_voltage *= -1;
		}
    } else if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        m_enableStateMachine = false;
        top_conveyor_voltage = -12000;
        ejection_roller_voltage = 12000;
    }

	if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        intake_voltage = 12000;
		bottom_conveyor_voltage = 12000;
		ejection_roller_voltage = 12000;

		if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
			ejection_roller_voltage *= -1;
		}
	}
	else if (m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        intake_voltage = -12000;
		bottom_conveyor_voltage = -12000;
		ejection_roller_voltage = -12000;
	}

    openIntakes(m_controller->get_digital(pros::E_CONTROLLER_DIGITAL_UP));

    setIntakeVoltage(intake_voltage);
    setBottomConveyorVoltage(bottom_conveyor_voltage);
	setEjectionRollerVoltage(ejection_roller_voltage);

    if (m_enableStateMachine) {
        m_updateConveyorHoldingState();
    } else {
        setTopConveyorVoltage(top_conveyor_voltage);
    }
}

void ConveyorNode::autonPeriodic() {
    switch (m_current_conveyor_state) {
        case STOPPED:
            // Stop all motors
            setTopConveyorVoltage(0);
        break;
        case HOLDING:
            // Update the state machine controlling the ball storage
            m_updateConveyorHoldingState();
        break;
        case SCORING:
            // Run all motors
            setTopConveyorVoltage(12000);
        break;
    }
}

ConveyorNode::~ConveyorNode() {

}