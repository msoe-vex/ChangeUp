#include "Actions/IntakeAction.h"

IntakeAction::IntakeAction(ConveyorNode* conveyor_node, int voltage, double time) : 
        m_conveyor_node(conveyor_node), m_voltage(voltage), m_time(time) {
    
}

void IntakeAction::actionInit() {
    m_timer.Start();
}

AutonAction::actionStatus IntakeAction::action() {
    if (m_time <= 0) {
        // Run until the action stops
        m_conveyor_node->setIntakeVoltage(m_voltage);
    } else {
        // Run until elapsed time is reached
        if (m_timer.Get() < m_time) {
            m_conveyor_node->setIntakeVoltage(m_voltage);
            return CONTINUE;
        } else {
            m_conveyor_node->setIntakeVoltage(0);
            return END;
        }
    }
}

void IntakeAction::actionEnd() {

}