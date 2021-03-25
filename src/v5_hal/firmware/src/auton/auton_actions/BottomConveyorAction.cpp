#include "auton/auton_actions/BottomConveyorAction.h"

BottomConveyorAction::BottomConveyorAction(ConveyorNode* conveyor_node, int voltage, double time) : 
        m_conveyor_node(conveyor_node),
        m_time(time), 
        m_voltage(voltage) {
        
}

void BottomConveyorAction::ActionInit() {
    m_timer.Start();
}

AutonAction::actionStatus BottomConveyorAction::Action() {
    if (m_time <= 0) {
        m_conveyor_node->setBottomConveyorVoltage(m_voltage);
        return END;    
    } else {
        if (m_timer.Get() < m_time) {
            m_conveyor_node->setBottomConveyorVoltage(m_voltage);
            return CONTINUE;
        } else {
            m_conveyor_node->setBottomConveyorVoltage(0);
            return END;
        }
    }   
}

void BottomConveyorAction::ActionEnd() {

}