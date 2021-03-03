#include "DeployAction.h"

DeployAction::DeployAction(ConveyorNode* conveyor_node) : 
        m_conveyor_node(conveyor_node) {

}

void DeployAction::ActionInit() {
    m_timer.Start();

    m_conveyor_node->setIntakeVoltage(-12000);
    m_conveyor_node->setBottomConveyorVoltage(-12000);
}

AutonAction::actionStatus DeployAction::Action() {
    if (m_timer.Get() > 0.1) {
        m_conveyor_node->setBottomConveyorVoltage(0);

        if (m_timer.Get() > 1.0) {
            return END;
        }
    }

    return CONTINUE;
}

void DeployAction::ActionEnd() {
    m_timer.Stop();
    m_conveyor_node->setIntakeVoltage(0);
    m_conveyor_node->setBottomConveyorVoltage(0);
}