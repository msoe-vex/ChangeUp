#include "auton/auton_actions/UpdateConveyorStateAction.h"

UpdateConveyorStateAction::UpdateConveyorStateAction(ConveyorNode* conveyor_node, 
        ConveyorNode::ConveyorState conveyor_state, double time) : 
        m_conveyor_node(conveyor_node), 
        m_conveyor_state(conveyor_state), 
        m_time(time) {
    
}

void UpdateConveyorStateAction::ActionInit() {
    m_timer.Start();
}

AutonAction::actionStatus UpdateConveyorStateAction::Action() {
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

void UpdateConveyorStateAction::ActionEnd() {
    
}