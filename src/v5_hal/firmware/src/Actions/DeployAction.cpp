#include "Actions/DeployAction.h"
#include "Timer.h"


DeployAction::DeployAction(conveyor_node) : m_conveyor_node(conveyor_node) {

}

void DeployAction::actionInit() {
    m_timer.Start();
}

AutonAction::actionStatus DeployAction::action() {
    m_conveyor_node->setIntakeVoltage(-12000);
    m_conveyor_node->setBottomConveyorVoltage(-12000);

    if(m_timer.Get() > 1) {
        return END;
    } else {
        return CONTINUE;
    }
}

void DeployAction::actionEnd() {
    m_timer.Stop();
    m_conveyor_node->setIntakeVoltage(0);
    m_conveyor_node->setBottomConveyorVoltage(0);
}