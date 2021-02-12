#include "Actions/BottomConveyorAction.h"

BottomConveyorAction::BottomConveyorAction(ConveyorNode* conveyor_node, bool eject=false, int voltage=12000, double time=0) : 
    m_conveyor_node(conveyor_node), m_eject(eject), m_voltage(voltage), m_time(time) {
        
    }

void BottomConveyorAction::actionInit() {
    m_timer.Start();

    //sets the direction of the eject roller
    if (m_eject == false) {
        m_eject_voltage = m_voltage;
    } else {
        m_eject_voltage = -1 * m_voltage;
    }
}

AutonAction::actionStatus BottomConveyorAction::action() {
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

void BottomConveyorAction::actionEnd() {

}