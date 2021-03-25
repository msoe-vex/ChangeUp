#include "auton/auton_actions/DeployAction.h"

DeployAction::DeployAction(ConveyorNode* conveyor_node) : 
        m_conveyor_node(conveyor_node) {

}

void DeployAction::ActionInit() {
    m_timer.Start();

    m_conveyor_node->setBottomConveyorVoltage(-1 * MAX_MOTOR_VOLTAGE);
}

AutonAction::actionStatus DeployAction::Action() {
    if (m_timer.Get() > 0.1) {
        m_conveyor_node->setBottomConveyorVoltage(0);
        return END;
    }

    return CONTINUE;
}

void DeployAction::ActionEnd() {
    m_timer.Stop();
    m_conveyor_node->setBottomConveyorVoltage(0);
}