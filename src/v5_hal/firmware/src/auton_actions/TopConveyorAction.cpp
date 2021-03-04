#include "TopConveyorAction.h"

TopConveyorAction::TopConveyorAction(ConveyorNode* conveyor_node, ConveyorNode::ConveyorState conveyor_state, double time) : 
        m_conveyor_node(conveyor_node), m_conveyor_state(conveyor_state), m_time(time) {
    
}

void TopConveyorAction::ActionInit() {
    m_timer.Start();
}

AutonAction::actionStatus TopConveyorAction::Action() {
    if (m_time <= 0) {
        m_conveyor_node->setConveyorState(m_conveyor_state);
        return END;
    } else {
        // Run until elapsed time is reached
        if (m_timer.Get() < m_time) {
            m_conveyor_node->setConveyorState(m_conveyor_state);
            return CONTINUE;
        } else {
            m_conveyor_node->setConveyorState(ConveyorNode::STOPPED);
            return END;
        }
    }
}

void TopConveyorAction::ActionEnd() {
    
}