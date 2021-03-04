#include "auton/auton_actions/BottomConveyorAction.h"

BottomConveyorAction::BottomConveyorAction(ConveyorNode* conveyor_node, bool eject, int voltage, double time) : 
        m_conveyor_node(conveyor_node), m_eject(eject), m_time(time), m_voltage(voltage) {
        
}

void BottomConveyorAction::ActionInit() {
    m_timer.Start();

    // Sets the direction of the eject roller
    if (m_eject == false) {
        m_eject_voltage = m_voltage;
    } else {
        m_eject_voltage = -1 * m_voltage;
    }
}

AutonAction::actionStatus BottomConveyorAction::Action() {
    if (m_time <= 0) {
        m_conveyor_node->setBottomConveyorVoltage(m_voltage);
        m_conveyor_node->setEjectionRollerVoltage(m_eject_voltage);
        return END;    
    } else {
        if (m_timer.Get() < m_time) {
            m_conveyor_node->setBottomConveyorVoltage(m_voltage);
            m_conveyor_node->setEjectionRollerVoltage(m_eject_voltage);
            return CONTINUE;
        } else {
            m_conveyor_node->setBottomConveyorVoltage(0);
            m_conveyor_node->setEjectionRollerVoltage(0);
            return END;
        }
    }   
}

void BottomConveyorAction::ActionEnd() {

}