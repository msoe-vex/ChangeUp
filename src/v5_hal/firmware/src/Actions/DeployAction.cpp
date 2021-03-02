#include "Actions/DeployAction.h"

DeployAction::DeployAction(ConveyorNode* conveyor_node) : 
        m_conveyor_node(conveyor_node) {

}

void DeployAction::ActionInit() {
    m_timer.Start();
}

AutonAction::actionStatus DeployAction::Action() {
    m_conveyor_node->setIntakeVoltage(-12000);
    m_conveyor_node->setBottomConveyorVoltage(-12000);

    if(m_timer.Get() > 1) {
        return END;
    } else {
        return CONTINUE;
    }
}

void DeployAction::ActionEnd() {
    m_timer.Stop();
    m_conveyor_node->setIntakeVoltage(0);
    m_conveyor_node->setBottomConveyorVoltage(0);
}