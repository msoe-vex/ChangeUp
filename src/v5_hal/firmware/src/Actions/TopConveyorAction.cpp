#include "Actions/TopConveyorAction.h"

TopConveyorAction::TopConveyorAction(ConveyorNode* conveyor_node, bool store_balls, int voltage, double time) : 
        m_conveyor_node(conveyor_node), m_store_balls(store_balls), m_voltage(voltage), m_time(time) {
    
}

void TopConveyorAction::actionInit() {
    m_timer.Start();
}

AutonAction::actionStatus TopConveyorAction::action() {
    if (m_time <= 0) {
        
    } else {
        // Run until elapsed time is reached
        if (m_timer.Get() < m_time) {
            return CONTINUE;
        } else {
            return END;
        }
    }
}

void TopConveyorAction::actionEnd() {

}