#include "auton/auton_actions/DeployAction.h"

DeployAction::DeployAction(ConveyorNode* conveyor_node, IntakeNode* intake_node) : 
        m_conveyor_node(conveyor_node),
        m_intake_node(intake_node) {

}

void DeployAction::ActionInit() {
    m_timer.Start();

    m_conveyor_node->setConveyorState(ConveyorNode::DEPLOY);
    m_intake_node->setIntakeVoltage(-MAX_MOTOR_VOLTAGE);
    m_intake_node->openIntakes(1);
    m_intake_node->liftGoalPlate(1);
}

AutonAction::actionStatus DeployAction::Action() {
    if (m_timer.Get() > 0.5) {
        m_conveyor_node->setConveyorState(ConveyorNode::STOPPED);
        m_intake_node->setIntakeVoltage(0);
        m_intake_node->openIntakes(0);
        return END;
    }

    return CONTINUE;
}

void DeployAction::ActionEnd() {
    m_timer.Stop();
    m_conveyor_node->setConveyorState(ConveyorNode::STOPPED);
    m_intake_node->setIntakeVoltage(0);
    m_intake_node->openIntakes(0);
}